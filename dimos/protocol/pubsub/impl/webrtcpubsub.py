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

"""WebRTC DataChannel pubsub transport.

Two layers:

* ``DataChannelProvider`` — protocol for managing WebRTC DataChannels.
  Implementations handle signaling, PeerConnection lifecycle, and
  DataChannel creation for a specific SFU backend (Cloudflare Realtime,
  LiveKit, etc.).

* ``WebRTCPubSub`` — pubsub implementation inheriting from
  :class:`~dimos.protocol.pubsub.spec.AllPubSub`. Delegates to a
  provider and conforms to the standard DimOS pubsub interface
  (publish/subscribe/subscribe_all).

Providers are in ``dimos/protocol/pubsub/impl/webrtc_providers/``.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Protocol, runtime_checkable

from dimos.protocol.pubsub.spec import AllPubSub
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@runtime_checkable
class DataChannelProvider(Protocol):
    """Protocol for WebRTC DataChannel backends.

    A provider manages the WebRTC PeerConnection(s) and exposes
    publish/subscribe semantics over named DataChannels. Implementations
    handle signaling, ICE, DTLS, and channel lifecycle for their specific
    SFU (Cloudflare Realtime, LiveKit, Janus, etc.).

    DataChannels may be unidirectional (CF) or bidirectional (LiveKit).
    The provider handles this transparently.
    """

    def start(self) -> None:
        """Connect to the SFU and establish transport."""
        ...

    def stop(self) -> None:
        """Disconnect and release resources."""
        ...

    def publish(self, topic: str, data: bytes) -> None:
        """Send bytes on a named topic/channel."""
        ...

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        """Subscribe to bytes on a named topic. Returns unsubscribe callable."""
        ...

    @property
    def is_connected(self) -> bool:
        """Whether the provider is connected and ready."""
        ...


class WebRTCPubSub(AllPubSub[str, bytes]):
    """Bytes-on-the-wire pubsub over WebRTC DataChannels.

    Inherits from :class:`AllPubSub[str, bytes]`:
      - TopicT = str (DataChannel name / multiplexed topic)
      - MsgT = bytes (raw bytes, LCM-encoded or otherwise)

    Delegates to a :class:`DataChannelProvider` implementation.
    Satisfies the standard pubsub grid tests in ``test_spec.py``.

    WebRTC DataChannels are inherently "receive all" — messages arrive
    on a shared multiplexed channel and are demuxed by topic/fingerprint.
    This matches LCM multicast semantics, hence AllPubSub.
    """

    def __init__(self, provider: DataChannelProvider) -> None:
        self._provider = provider
        self._started = False
        self._all_callbacks: list[Callable[[bytes, str], Any]] = []

    @property
    def provider(self) -> DataChannelProvider:
        return self._provider

    def start(self) -> None:
        if self._started:
            return
        self._provider.start()
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        self._provider.stop()
        self._started = False

    def publish(self, topic: str, message: bytes) -> None:
        """Publish raw bytes to a named DataChannel/topic."""
        if not self._started:
            self.start()
        self._provider.publish(topic, message)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        """Subscribe to raw bytes on a topic. Callback is (data, topic)."""
        if not self._started:
            self.start()

        # Wrap callback to also fan-out to subscribe_all listeners
        def _wrapped(data: bytes, t: str) -> None:
            callback(data, t)
            for all_cb in self._all_callbacks:
                try:
                    all_cb(data, t)
                except Exception:
                    logger.exception("subscribe_all callback error")

        return self._provider.subscribe(topic, _wrapped)

    def subscribe_all(self, callback: Callable[[bytes, str], Any]) -> Callable[[], None]:
        """Subscribe to all messages on all topics.

        Messages received on any subscribed topic are delivered to
        subscribe_all listeners. This mirrors LCM multicast semantics
        where the underlying channel delivers everything.
        """
        self._all_callbacks.append(callback)

        def _unsub() -> None:
            try:
                self._all_callbacks.remove(callback)
            except ValueError:
                pass

        return _unsub


# Re-export provider availability flag
try:
    from dimos.protocol.pubsub.impl.webrtc_providers.cloudflare import (
        CLOUDFLARE_AVAILABLE,
        CloudflareProvider,
    )

    WEBRTC_AVAILABLE = CLOUDFLARE_AVAILABLE
except ImportError:
    WEBRTC_AVAILABLE = False
    CloudflareProvider = None  # type: ignore[assignment,misc]
    CLOUDFLARE_AVAILABLE = False

try:
    from dimos.protocol.pubsub.impl.webrtc_providers.broker import (
        BROKER_AVAILABLE,
        BrokerProvider,
    )
except ImportError:
    BROKER_AVAILABLE = False  # type: ignore[assignment]
    BrokerProvider = None  # type: ignore[assignment,misc]


__all__ = [
    "BROKER_AVAILABLE",
    "WEBRTC_AVAILABLE",
    "BrokerProvider",
    "CloudflareProvider",
    "DataChannelProvider",
    "WebRTCPubSub",
]
