import test from "node:test";
import assert from "node:assert/strict";
import { Readable, Writable } from "node:stream";
import { MAX_LINE_BYTES, parseFrame } from "../src/protocol.js";
import { runAdapter, type SessionAdapter } from "../src/main.js";

function sink(): { stream: Writable; lines: () => Record<string, unknown>[] } {
  const values: string[] = [];
  return { stream: new Writable({ write(chunk, _encoding, callback) { values.push(String(chunk)); callback(); } }), lines: () => values.join("").trim().split("\n").filter(Boolean).map((line) => JSON.parse(line) as Record<string, unknown>) };
}

test("parses protocol v1 run start and rejects malformed/oversized frames", () => {
  assert.equal(parseFrame('{"version":1,"type":"run_start","id":"r","prompt":"go","budget":{"maxTurns":1,"maxToolCalls":1,"timeoutMs":1000},"config":{"promptMode":"visualization_forbidden","answerType":"boolean","modelId":"gpt-5.6-luna","thinkingLevel":"medium","implementationDigests":{"adapter":"adapter@sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","scorer":"scorer@sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","protocol":"protocol@sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"}}}').type, "run_start");
  assert.equal(parseFrame('{"version":1,"type":"run_start","id":"r","prompt":"go","budget":{"maxTurns":1,"maxToolCalls":1,"timeoutMs":1000},"config":{"promptMode":"visualization_forbidden","answerType":"integer","modelId":"gpt-5.6-luna","thinkingLevel":"medium","implementationDigests":{"adapter":"adapter@sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","scorer":"scorer@sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","protocol":"protocol@sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"}}}').type, "run_start");
  assert.throws(() => parseFrame("{}"));
  assert.throws(() => parseFrame("x".repeat(MAX_LINE_BYTES + 1)));
  assert.throws(() => parseFrame('{"version":1,"type":"tool_reply","id":"x","ok":true,"extra":1}'));
  assert.throws(() => parseFrame('{"version":1,"type":"run_start","id":"r","prompt":"go","budget":{"maxTurns":1,"maxToolCalls":1,"timeoutMs":1000},"config":{"promptMode":"visualization_forbidden","answerType":"unknown","modelId":"gpt-5.6-luna","thinkingLevel":"medium","implementationDigests":{"adapter":"adapter@sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","scorer":"scorer@sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","protocol":"protocol@sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"}}}'));
  assert.throws(() => parseFrame('{"version":1,"type":"run_start","id":"r","prompt":"go","budget":{"maxTurns":1,"maxToolCalls":1,"timeoutMs":1000},"config":{"promptMode":"visualization_forbidden","answerType":"boolean","modelId":"gpt-5.6-luna","thinkingLevel":"high","implementationDigests":{"adapter":"adapter@sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","scorer":"scorer@sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","protocol":"protocol@sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"}}}'));
});

test("runs a fresh injected session and completes a tool roundtrip", async () => {
  const output = sink();
  let disposed = false;
  const factory = async (broker: Parameters<NonNullable<Parameters<typeof runAdapter>[3]>>[0]) : Promise<SessionAdapter> => ({
    subscribe: () => undefined,
    prompt: async () => { await broker.request("read_generated_image", { path: "plots/top.png" }); },
    dispose: () => { disposed = true; },
  });
  const start = JSON.stringify({ version: 1, type: "run_start", id: "r", prompt: "go", budget: { maxTurns: 1, maxToolCalls: 1, timeoutMs: 1000 }, config: { promptMode: "visualization_encouraged", answerType: "boolean", modelId: "gpt-5.6-luna", thinkingLevel: "medium", implementationDigests: { adapter: "adapter@sha256:" + "a".repeat(64), scorer: "scorer@sha256:" + "b".repeat(64), protocol: "protocol@sha256:" + "c".repeat(64) } } });
  const reply = JSON.stringify({ version: 1, type: "tool_reply", id: "tool-1", ok: true, result: { mime: "image/png", data: "iVBORw0KGgo=" } });
  await runAdapter(Readable.from([`${start}\n`, `${reply}\n`]), output.stream, new Writable({ write(_chunk, _encoding, callback) { callback(); } }), factory);
  const frames = output.lines();
  assert.equal(frames[0]?.type, "run_started");
  assert.deepEqual(frames[0]?.tools, ["sandbox_exec", "read_generated_image", "submit_answer"]);
  assert.equal(frames.some((frame) => frame.type === "tool_call" && frame.tool === "read_generated_image"), true);
  assert.equal(frames.at(-1)?.type, "run_complete");
  assert.equal(disposed, true);
});

test("fails closed on duplicate run and unknown tool replies", async () => {
  const output = sink();
  const start = JSON.stringify({ version: 1, type: "run_start", id: "r", prompt: "go", budget: { maxTurns: 1, maxToolCalls: 1, timeoutMs: 1000 }, config: { promptMode: "visualization_forbidden", answerType: "boolean", modelId: "gpt-5.6-luna", thinkingLevel: "medium", implementationDigests: { adapter: "adapter@sha256:" + "a".repeat(64), scorer: "scorer@sha256:" + "b".repeat(64), protocol: "protocol@sha256:" + "c".repeat(64) } } });
  await runAdapter(Readable.from([`${start}\n`, `${start}\n`]), output.stream, new Writable({ write(_chunk, _encoding, callback) { callback(); } }), async () => ({ subscribe: () => undefined, prompt: async () => undefined, dispose: () => undefined }));
  assert.equal(output.lines().some((frame) => frame.type === "protocol_error"), true);

  const unknownOutput = sink();
  const reply = JSON.stringify({ version: 1, type: "tool_reply", id: "not-pending", ok: true, result: { text: "no" } });
  await runAdapter(Readable.from([`${start}\n`, `${reply}\n`]), unknownOutput.stream, new Writable({ write(_chunk, _encoding, callback) { callback(); } }), async () => ({ subscribe: () => undefined, prompt: async () => new Promise(() => undefined), dispose: () => undefined }));
  assert.equal(unknownOutput.lines().some((frame) => frame.type === "protocol_error"), true);
});
