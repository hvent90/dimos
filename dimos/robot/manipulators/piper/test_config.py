# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from dimos.core.global_config import global_config
from dimos.robot.manipulators.piper.config import PIPER_SIM_PATH, piper_hardware


def test_piper_defaults_to_mock_without_can_port(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "")

    for can_port in (None, ""):
        monkeypatch.setattr(global_config, "can_port", can_port)
        hardware = piper_hardware()

        assert hardware.adapter_type == "mock"
        assert hardware.address is None


def test_piper_uses_configured_can_port(monkeypatch) -> None:
    can_port = "can7"
    monkeypatch.setattr(global_config, "simulation", "")
    monkeypatch.setattr(global_config, "can_port", can_port)

    hardware = piper_hardware()

    assert hardware.adapter_type == "piper"
    assert hardware.address == can_port


def test_piper_simulation_selection_is_unchanged(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")
    monkeypatch.setattr(global_config, "can_port", "can7")

    hardware = piper_hardware()

    assert hardware.adapter_type == "sim_mujoco"
    assert hardware.address == str(PIPER_SIM_PATH)
