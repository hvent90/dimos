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

import time
from types import MappingProxyType

import pytest

from dimos.core.coordination.blueprints import BlueprintAtom, StreamRef, autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out

_BUILD_WITHOUT_RERUN = MappingProxyType({"g": {"viewer": "none"}})


class Msg:
    pass


# An ordinary module: no declared_streams override, streams come from annotations.
class PlainModule(Module):
    reading: Out[Msg]


# A module whose ports come entirely from config, with zero stream annotations.
class ConfigConsumer(Module):
    @classmethod
    def declared_streams(cls, kwargs):
        return (StreamRef(name="chatter", type=str, direction="in"),)

    @rpc
    def start(self) -> None:
        self._received: list[str] = []
        super().start()

    async def handle_chatter(self, msg: str) -> None:
        self._received.append(msg)

    @rpc
    def get_received(self) -> list[str]:
        return list(getattr(self, "_received", []))


class ChatterProducer(Module):
    chatter: Out[str]

    @rpc
    def emit(self, msg: str) -> None:
        self.chatter.publish(msg)


# A module that mixes an annotation stream with a config-derived one.
class MixedModule(Module):
    reading: Out[Msg]

    @classmethod
    def declared_streams(cls, kwargs):
        return (StreamRef(name="command", type=str, direction="in"),)


class FanoutConfig(ModuleConfig):
    n: int = 0


# declared_streams keyed off config: the same kwargs must always yield the same refs.
class DynamicFanout(Module):
    config: FanoutConfig

    @classmethod
    def declared_streams(cls, kwargs):
        n = kwargs.get("n", 0)
        return tuple(StreamRef(name=f"out_{i}", type=str, direction="out") for i in range(n))


@pytest.fixture
def build_coordinator():
    coordinators = []

    def _build(blueprint):
        c = ModuleCoordinator.build(blueprint, _BUILD_WITHOUT_RERUN.copy())
        coordinators.append(c)
        return c

    yield _build

    for c in reversed(coordinators):
        c.stop()


@pytest.fixture
def dynamic_coordinator():
    mc = ModuleCoordinator(g=GlobalConfig(n_workers=0, viewer="none"))
    mc.start()
    yield mc
    mc.stop()


def _poll(fn, predicate, timeout=5.0):
    deadline = time.time() + timeout
    result = fn()
    while not predicate(result) and time.time() < deadline:
        time.sleep(0.02)
        result = fn()
    return result


def test_default_hook_is_empty() -> None:
    assert PlainModule.declared_streams({}) == ()


def test_ordinary_module_atom_unchanged() -> None:
    """An ordinary module's atom is exactly its annotation streams — the hook adds nothing."""
    atom = BlueprintAtom.create(PlainModule, kwargs={"k": "v"})
    assert atom == BlueprintAtom(
        module=PlainModule,
        streams=(StreamRef(name="reading", type=Msg, direction="out"),),
        module_refs=(),
        kwargs={"k": "v"},
    )


def test_atom_includes_config_derived_stream() -> None:
    """Config-derived streams are appended after annotation streams."""
    atom = BlueprintAtom.create(MixedModule, kwargs={})
    assert atom.streams == (
        StreamRef(name="reading", type=Msg, direction="out"),
        StreamRef(name="command", type=str, direction="in"),
    )


def test_atom_streams_scale_with_config() -> None:
    atom = BlueprintAtom.create(DynamicFanout, kwargs={"n": 3})
    assert atom.streams == (
        StreamRef(name="out_0", type=str, direction="out"),
        StreamRef(name="out_1", type=str, direction="out"),
        StreamRef(name="out_2", type=str, direction="out"),
    )
    assert BlueprintAtom.create(DynamicFanout, kwargs={"n": 1}).streams == (
        StreamRef(name="out_0", type=str, direction="out"),
    )


def test_instance_creates_config_derived_ports() -> None:
    """Module.__init__ builds live In/Out objects for config-derived streams."""
    inst = DynamicFanout(n=2)
    try:
        assert set(inst.outputs) == {"out_0", "out_1"}
        assert isinstance(inst.out_0, Out)
        assert inst.out_0.type is str
        assert inst.out_0.owner is inst
    finally:
        inst.stop()


def test_config_derived_ports_hidden_from_class_view_shown_on_instance() -> None:
    """Documented cosmetic limit: class-level module_info reads annotations only;
    instance-level scans the live instance and so shows config-derived ports."""
    class_info = DynamicFanout.module_info()
    assert not any(s.name.startswith("out_") for s in class_info.outputs)

    inst = DynamicFanout(n=2)
    try:
        instance_info = inst.module_info()
        assert {"out_0", "out_1"} <= {s.name for s in instance_info.outputs}
    finally:
        inst.stop()


def test_host_and_worker_derive_identical_streams() -> None:
    """The refs the host wires (atom) match the ports the worker instantiates."""
    kwargs = {"n": 2}
    atom = BlueprintAtom.create(DynamicFanout, kwargs=kwargs)
    inst = DynamicFanout(**kwargs)
    try:
        atom_out = {s.name for s in atom.streams if s.direction == "out"}
        assert atom_out == set(inst.outputs)
    finally:
        inst.stop()


def test_declared_streams_is_deterministic() -> None:
    """Equal kwargs must yield equal refs — the purity contract host/worker/restart rely on."""
    first = DynamicFanout.declared_streams({"n": 3})
    second = DynamicFanout.declared_streams({"n": 3})
    assert first == second
    assert BlueprintAtom.create(DynamicFanout, kwargs={"n": 3}) == BlueprintAtom.create(
        DynamicFanout, kwargs={"n": 3}
    )


def test_atom_rejects_collision_with_annotation() -> None:
    class Collide(Module):
        chatter: In[str]

        @classmethod
        def declared_streams(cls, kwargs):
            return (StreamRef(name="chatter", type=str, direction="out"),)

    with pytest.raises(ValueError, match="collides"):
        BlueprintAtom.create(Collide, kwargs={})


def test_atom_rejects_invalid_identifier() -> None:
    class BadName(Module):
        @classmethod
        def declared_streams(cls, kwargs):
            return (StreamRef(name="not an identifier", type=str, direction="in"),)

    with pytest.raises(ValueError, match="identifier"):
        BlueprintAtom.create(BadName, kwargs={})


def test_atom_rejects_invalid_direction() -> None:
    class BadDirection(Module):
        @classmethod
        def declared_streams(cls, kwargs):
            return (StreamRef(name="stream", type=str, direction="sideways"),)

    with pytest.raises(ValueError, match="direction"):
        BlueprintAtom.create(BadDirection, kwargs={})


def test_instance_rejects_collision_with_attribute() -> None:
    """A config-derived stream that shadows an existing attribute is rejected at init."""

    class ShadowStart(Module):
        @classmethod
        def declared_streams(cls, kwargs):
            return (StreamRef(name="start", type=str, direction="in"),)

    with pytest.raises(ValueError, match="collides"):
        ShadowStart()


def test_end_to_end_config_derived_consumer_receives_message(build_coordinator) -> None:
    """A zero-annotation consumer, wired via autoconnect to an annotated producer,
    receives a message on a config-derived port through an auto-bound handle_."""
    coordinator = build_coordinator(
        autoconnect(ChatterProducer.blueprint(), ConfigConsumer.blueprint())
    )
    producer = coordinator.get_instance(ChatterProducer)
    consumer = coordinator.get_instance(ConfigConsumer)
    assert producer is not None
    assert consumer is not None

    # The config-derived In is wired to the same transport as the producer's Out.
    assert consumer.chatter.transport.topic == producer.chatter.transport.topic

    producer.emit("world")

    received = _poll(consumer.get_received, lambda r: "world" in r)
    assert "world" in received


def test_restart_rewires_config_derived_stream(dynamic_coordinator) -> None:
    """restart_module re-derives the config-derived stream (via module.blueprint(**kwargs))
    and reconnects it to the existing transport — this only works if declared_streams
    is pure across host, worker, and restart."""
    dynamic_coordinator.load_module(ConfigConsumer)

    c = dynamic_coordinator.get_instance(ConfigConsumer)
    assert c is not None
    topic_before = c.chatter.transport.topic
    registry_before = dynamic_coordinator._transport_registry[("chatter", str)]

    dynamic_coordinator.restart_module(ConfigConsumer, reload_source=False)

    assert dynamic_coordinator._transport_registry[("chatter", str)] is registry_before

    c_after = dynamic_coordinator.get_instance(ConfigConsumer)
    assert c_after is not None
    assert c_after is not c
    assert c_after.chatter.transport.topic == topic_before
