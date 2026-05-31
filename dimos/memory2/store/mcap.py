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

"""Read-only memory2 store backed by an mcap file.

Generic and codec-injected — it knows nothing about any robot. The caller
supplies ``codecs`` (DDS/wire topic -> codec that decodes a message's stored
bytes) and an optional ``streams`` map (friendly stream name -> topic). See
``dimos.robot.unitree.go2dds.store.Go2McapStore`` for the Go2 wiring.

Read-only: no append, blobs, vectors, or embeddings. Payloads decode lazily on
``obs.data``; ts and counts are cheap (counts come from the mcap index).
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any, Protocol, runtime_checkable

from mcap.reader import make_reader

from dimos.memory2.backend import Backend
from dimos.memory2.codecs.base import codec_for
from dimos.memory2.notifier.subject import SubjectNotifier
from dimos.memory2.observationstore.base import ObservationStore, ObservationStoreConfig
from dimos.memory2.store.base import Store, StoreConfig
from dimos.memory2.type.observation import Observation


@runtime_checkable
class StreamCodec(Protocol):
    """What the store needs to turn a channel's stored bytes into a payload."""

    payload_type: type

    def decode(self, data: bytes) -> Any: ...


class McapObservationStoreConfig(ObservationStoreConfig):
    name: str = "<mcap>"


class McapObservationStore(ObservationStore[Any]):
    """Read-only metadata/query over one mcap channel. Payloads load lazily."""

    config: McapObservationStoreConfig

    def __init__(self, *, name: str, path: str, topic: str, codec: StreamCodec, count: int) -> None:
        super().__init__(name=name)
        self._path = path
        self._topic = topic
        self._codec = codec
        self._count = count

    @property
    def name(self) -> str:
        return self.config.name

    def _iter(self) -> Iterator[Observation[Any]]:
        decode, dtype = self._codec.decode, self._codec.payload_type
        with open(self._path, "rb") as f:
            for i, (_s, _c, m) in enumerate(make_reader(f).iter_messages(topics=[self._topic])):
                data = m.data
                yield Observation(
                    id=i,
                    ts=m.log_time / 1e9,
                    data_type=dtype,
                    _loader=(lambda d=data: decode(d)),
                )

    def query(self, q: Any) -> Iterator[Observation[Any]]:
        return q.apply(self._iter())

    def count(self, q: Any) -> int:
        if not q.filters and q.search_text is None and q.search_vec is None:
            n = self._count
            if q.offset_val:
                n = max(0, n - q.offset_val)
            if q.limit_val is not None:
                n = min(n, q.limit_val)
            return n
        return sum(1 for _ in self.query(q))

    def fetch_by_ids(self, ids: list[int]) -> list[Observation[Any]]:
        want = set(ids)
        return [o for o in self._iter() if o.id in want]

    def insert(self, obs: Observation[Any]) -> int:
        raise NotImplementedError("McapStore is read-only")


class McapStoreConfig(StoreConfig):
    path: str = ""


class McapStore(Store):
    """A memory2 store backed by an mcap file (read-only).

    ``codecs`` maps topic -> :class:`StreamCodec`. ``streams`` maps a friendly
    stream name -> topic; defaults to using the topic as the name.
    """

    config: McapStoreConfig

    def __init__(
        self,
        *,
        codecs: dict[str, StreamCodec],
        streams: dict[str, str] | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._codecs = codecs
        self._stream_topic = streams if streams is not None else {t: t for t in codecs}
        with open(self.config.path, "rb") as f:
            summary = make_reader(f).get_summary()
        counts: dict[str, int] = {}
        if summary is not None and summary.statistics is not None:
            by_topic = {ch.topic: cid for cid, ch in summary.channels.items()}
            for name, topic in self._stream_topic.items():
                cid = by_topic.get(topic)
                if cid is not None and topic in self._codecs:
                    counts[name] = summary.statistics.channel_message_counts.get(cid, 0)
        self._available = counts  # stream name -> count, present & decodable channels only

    def list_streams(self) -> list[str]:
        return sorted(set(self._available) | set(self._streams))

    def _create_backend(
        self, name: str, payload_type: type | None = None, **config: Any
    ) -> Backend[Any]:
        if name not in self._available:
            raise KeyError(f"No stream {name!r}. Available: {sorted(self._available)}")
        topic = self._stream_topic[name]
        codec = self._codecs[topic]
        ptype = codec.payload_type
        obs = McapObservationStore(
            name=name, path=self.config.path, topic=topic, codec=codec, count=self._available[name]
        )
        return Backend(
            metadata_store=obs,
            codec=codec_for(ptype),  # storage codec, unused (blob_store=None)
            data_type=ptype,
            blob_store=None,
            vector_store=None,
            notifier=SubjectNotifier(),
        )
