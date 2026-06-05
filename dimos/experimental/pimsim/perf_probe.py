# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
from itertools import pairwise
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time
from typing import Any
from urllib.parse import urlsplit, urlunsplit
from urllib.request import urlopen

import websockets

DEFAULT_URL = "http://127.0.0.1:8091/"
DEFAULT_CHROME = "/usr/bin/google-chrome"
DEFAULT_CDP_PORT = 9361

_FPS_EXPR = """
new Promise((resolve) => {
  const sampleMs = __SAMPLE_MS__;
  const start = performance.now();
  let last = start;
  const intervals = [];
  function tick(now) {
    intervals.push(now - last);
    last = now;
    if (now - start < sampleMs) {
      requestAnimationFrame(tick);
      return;
    }
    intervals.shift();
    intervals.sort((a, b) => a - b);
    const pick = (q) => intervals[Math.min(intervals.length - 1, Math.floor(q * (intervals.length - 1)))] || 0;
    resolve({
      frames: intervals.length,
      fps: intervals.length / (sampleMs / 1000),
      dt_ms_min: intervals[0] || 0,
      dt_ms_med: pick(0.5),
      dt_ms_p95: pick(0.95),
      dt_ms_p99: pick(0.99),
      dt_ms_max: intervals[intervals.length - 1] || 0,
      over_33ms: intervals.filter((x) => x > 33.3).length,
      over_50ms: intervals.filter((x) => x > 50).length,
      over_100ms: intervals.filter((x) => x > 100).length,
    });
  }
  requestAnimationFrame(tick);
})
"""


@dataclass(frozen=True)
class ProbeArgs:
    url: str
    chrome: str
    cdp_port: int
    load_wait: float
    sample: float
    websocket_sample: float
    output: Path | None


async def _cdp_call(
    websocket: websockets.ClientConnection,
    method: str,
    params: dict[str, Any] | None = None,
    *,
    counter: list[int],
) -> dict[str, Any]:
    counter[0] += 1
    message_id = counter[0]
    await websocket.send(json.dumps({"id": message_id, "method": method, "params": params or {}}))
    while True:
        message = json.loads(await websocket.recv())
        if message.get("id") != message_id:
            continue
        if "error" in message:
            raise RuntimeError(message["error"])
        return message.get("result") or {}


def _fetch_json(url: str, timeout: float = 1.0) -> Any:
    with urlopen(url, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


async def _sample_websocket(url: str, duration_s: float) -> dict[str, Any]:
    parsed = urlsplit(url)
    scheme = "wss" if parsed.scheme == "https" else "ws"
    websocket_url = urlunsplit((scheme, parsed.netloc, "/ws", "", ""))
    state_times: list[float] = []
    state_lags: list[float] = []
    state_sizes: list[int] = []
    binary_sizes: list[int] = []
    start = time.time()

    async with websockets.connect(websocket_url, max_size=None) as websocket:
        while time.time() - start < duration_s:
            message = await websocket.recv()
            now = time.time()
            if isinstance(message, str):
                state_sizes.append(len(message))
                payload = json.loads(message)
                if payload.get("type") == "state":
                    state_times.append(now)
                    if "time" in payload:
                        state_lags.append(now - float(payload["time"]))
            else:
                binary_sizes.append(len(message))

    intervals = [b - a for a, b in pairwise(state_times)]
    elapsed = max(time.time() - start, 1e-9)
    return {
        "state_count": len(state_times),
        "state_hz": len(state_times) / elapsed,
        "state_size": _summary(state_sizes),
        "state_interval_ms": _summary([1000.0 * value for value in intervals]),
        "state_lag_ms": _summary([1000.0 * value for value in state_lags]),
        "binary_count": len(binary_sizes),
        "binary_size": _summary(binary_sizes),
        "binary_total_mb": sum(binary_sizes) / 1_000_000,
    }


def _summary(values: list[float] | list[int]) -> dict[str, float] | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)

    def pick(q: float) -> float:
        return ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1)))]

    return {
        "min": ordered[0],
        "median": pick(0.5),
        "p95": pick(0.95),
        "max": ordered[-1],
    }


async def _probe_browser(args: ProbeArgs) -> dict[str, Any]:
    profile_dir = tempfile.mkdtemp(prefix="dimos-pimsim-chrome-")
    command = [
        args.chrome,
        "--headless=new",
        "--no-sandbox",
        "--disable-dev-shm-usage",
        "--enable-webgl",
        "--ignore-gpu-blocklist",
        f"--remote-debugging-port={args.cdp_port}",
        f"--user-data-dir={profile_dir}",
        "--no-first-run",
        "--disable-background-networking",
        args.url,
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    try:
        await _wait_for_chrome(args.cdp_port)
        pages = _fetch_json(f"http://127.0.0.1:{args.cdp_port}/json/list")
        page = next(item for item in pages if item.get("type") == "page")
        counter = [0]
        async with websockets.connect(page["webSocketDebuggerUrl"], max_size=None) as websocket:
            await _cdp_call(websocket, "Runtime.enable", counter=counter)
            await _cdp_call(websocket, "Performance.enable", counter=counter)
            await asyncio.sleep(args.load_wait)
            scene = await _runtime_value(websocket, _scene_info_expr(), counter)
            fps = await _runtime_value(
                websocket,
                _FPS_EXPR.replace("__SAMPLE_MS__", str(int(args.sample * 1000))),
                counter,
                await_promise=True,
            )
            perf = await _cdp_call(websocket, "Performance.getMetrics", counter=counter)
            metrics = {metric["name"]: metric["value"] for metric in perf.get("metrics", [])}
            return {
                "url": args.url,
                "scene": scene,
                "fps": fps,
                "performance": {
                    key: metrics.get(key)
                    for key in [
                        "JSHeapUsedSize",
                        "JSHeapTotalSize",
                        "Nodes",
                        "TaskDuration",
                        "ScriptDuration",
                    ]
                },
            }
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
        shutil.rmtree(profile_dir, ignore_errors=True)


async def _wait_for_chrome(port: int) -> None:
    deadline = time.time() + 15.0
    while time.time() < deadline:
        try:
            _fetch_json(f"http://127.0.0.1:{port}/json/version", timeout=0.2)
            return
        except Exception:
            await asyncio.sleep(0.1)
    raise RuntimeError("Chrome did not expose a DevTools endpoint")


async def _runtime_value(
    websocket: websockets.ClientConnection,
    expression: str,
    counter: list[int],
    *,
    await_promise: bool = False,
) -> Any:
    result = await _cdp_call(
        websocket,
        "Runtime.evaluate",
        {
            "expression": expression,
            "returnByValue": True,
            "awaitPromise": await_promise,
        },
        counter=counter,
    )
    return result["result"].get("value")


def _scene_info_expr() -> str:
    return """
(() => {
  const scene = BABYLON?.EngineStore?.LastCreatedScene;
  if (!scene) return null;
  const sceneMeshes = scene.meshes.filter((mesh) => mesh.metadata && mesh.metadata.dimosSceneMesh);
  return {
    status: document.getElementById("status")?.textContent || null,
    meshes: scene.meshes.length,
    sceneMeshes: sceneMeshes.length,
    materials: scene.materials.length,
    textures: scene.textures.length,
    activeMeshes: scene.getActiveMeshes().length,
    totalVertices: scene.meshes.reduce((count, mesh) => count + (mesh.getTotalVertices ? mesh.getTotalVertices() : 0), 0),
    sceneVertices: sceneMeshes.reduce((count, mesh) => count + (mesh.getTotalVertices ? mesh.getTotalVertices() : 0), 0),
    heap: performance.memory ? {
      used: performance.memory.usedJSHeapSize,
      total: performance.memory.totalJSHeapSize,
      limit: performance.memory.jsHeapSizeLimit,
    } : null,
  };
})()
"""


async def _main(args: ProbeArgs) -> None:
    result = {
        "browser": await _probe_browser(args),
        "websocket": await _sample_websocket(args.url, args.websocket_sample),
    }
    text = json.dumps(result, indent=2)
    print(text)
    if args.output is not None:
        args.output.write_text(f"{text}\n")


def _parse_args() -> ProbeArgs:
    parser = argparse.ArgumentParser(description="Probe a running pimsim Babylon viewer.")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--chrome", default=DEFAULT_CHROME)
    parser.add_argument("--cdp-port", type=int, default=DEFAULT_CDP_PORT)
    parser.add_argument("--load-wait", type=float, default=16.0)
    parser.add_argument("--sample", type=float, default=8.0)
    parser.add_argument("--websocket-sample", type=float, default=8.0)
    parser.add_argument("--output", type=Path)
    parsed = parser.parse_args()
    return ProbeArgs(
        url=parsed.url,
        chrome=parsed.chrome,
        cdp_port=parsed.cdp_port,
        load_wait=parsed.load_wait,
        sample=parsed.sample,
        websocket_sample=parsed.websocket_sample,
        output=parsed.output,
    )


if __name__ == "__main__":
    asyncio.run(_main(_parse_args()))
