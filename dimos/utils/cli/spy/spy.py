# Copyright 2026 Dimensional Inc.
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

"""Transport-agnostic traffic spy.

Both LCM and Zenoh expose the same raw ``AllPubSub[Topic, bytes]`` interface,
so a single spy can monitor either backend. We subscribe to a wildcard on the
RAW base pubsub (no encoder) and measure each message's wire size.

We use ``subscribe(wildcard)`` rather than ``subscribe_all()``: Zenoh's
``subscribe_all`` is intentionally lossy (latest-per-topic drain, for rerun),
which would undercount freq/bandwidth. The wildcard subscribe fires for every
message on every topic.
"""

from __future__ import annotations

import re
import threading
from typing import TYPE_CHECKING

from dimos.core.global_config import global_config
from dimos.protocol.pubsub.impl.lcmpubsub import LCMPubSubBase, Topic
from dimos.utils.cli.lcmspy.lcmspy import GraphTopic

if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.protocol.pubsub.spec import AllPubSub


def make_spy_source(
    transport: str | None = None,
    key: str | None = None,
    connect: list[str] | None = None,
    iface: str | None = None,
) -> tuple[AllPubSub[Topic, bytes], Topic]:
    """Return ``(raw pubsub base, wildcard Topic)`` for the active transport.

    ``key`` overrides the wildcard: an LCM channel regex (e.g. ``/odom.*``) or a
    Zenoh key expression (e.g. ``dimos/**``). Defaults catch everything: ``.*``
    for LCM and ``**`` for Zenoh (Zenoh ``**`` also surfaces non-dimos keys,
    e.g. an external C++ publisher).

    ``connect`` (Zenoh only) is a list of endpoints to dial, e.g.
    ``["tcp/10.21.31.106:7447"]``. Required to reach a peer that has scouting
    (multicast/gossip) disabled, since it won't be auto-discovered.

    ``iface`` (Zenoh only) pins the multicast scout NIC, e.g. ``"eth0"``. None
    falls back to ``global_config.zenoh_iface`` / ``DIMOS_ZENOH_IFACE``.
    """
    transport = transport or global_config.transport
    if transport == "zenoh":
        from dimos.protocol.pubsub.impl.zenohpubsub import ZenohPubSubBase

        kwargs: dict[str, object] = {}
        if connect:
            kwargs["connect"] = list(connect)
        if iface:
            kwargs["multicast_iface"] = iface
        return ZenohPubSubBase(**kwargs), Topic(key or "**")
    return LCMPubSubBase(), Topic(re.compile(key) if key else re.compile(".*"))


class Spy:
    """Tracks per-topic freq/bandwidth/total traffic over any transport.

    Exposes the same ``topic`` dict / ``start`` / ``stop`` surface that the
    Textual UI in ``run_spy`` consumes.
    """

    def __init__(
        self,
        transport: str | None = None,
        key: str | None = None,
        connect: list[str] | None = None,
        iface: str | None = None,
        graph_log_window: float = 0.5,
        history_window: float = 60.0,
    ) -> None:
        self.pubsub, self._wildcard = make_spy_source(transport, key, connect, iface)
        self.history_window = history_window
        self.graph_log_window = graph_log_window
        self.topic: dict[str, GraphTopic] = {}
        self._topic_lock = threading.Lock()
        self._unsub: Callable[[], None] | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if hasattr(self.pubsub, "start"):
            self.pubsub.start()  # type: ignore[attr-defined]
        self._unsub = self.pubsub.subscribe(self._wildcard, self._on_msg)
        self._thread = threading.Thread(target=self._graph_log, name="spy-graph", daemon=True)
        self._thread.start()

    def _on_msg(self, data: bytes, topic: Topic) -> None:
        name = str(topic)
        with self._topic_lock:
            t = self.topic.get(name)
            if t is None:
                t = GraphTopic(name, history_window=self.history_window)
                self.topic[name] = t
        t.msg(data)

    def _graph_log(self) -> None:
        while not self._stop.is_set():
            with self._topic_lock:
                topics = list(self.topic.values())
            for t in topics:
                t.update_graphs(self.graph_log_window)
            self._stop.wait(self.graph_log_window)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._unsub is not None:
            self._unsub()
        if hasattr(self.pubsub, "stop"):
            self.pubsub.stop()  # type: ignore[attr-defined]


if __name__ == "__main__":
    import time

    spy = Spy()
    spy.start()
    try:
        while True:
            time.sleep(1.0)
            for name, t in sorted(spy.topic.items()):
                print(f"{name:50s} {t.freq(5.0):6.1f} Hz  {t.kbps_hr(5.0)}")
    except KeyboardInterrupt:
        spy.stop()
        print("Spy stopped.")
