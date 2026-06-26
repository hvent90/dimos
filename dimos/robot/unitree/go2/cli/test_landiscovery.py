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

from unittest.mock import MagicMock

import pytest

from dimos.robot.unitree.go2.cli import landiscovery as lan
from dimos.robot.unitree.go2.cli.landiscovery import Go2Device

# All tests stub the network layer (_candidate_ifaces / _tcp_open / _resolve_mac
# / _probe_iface) so nothing touches a real socket — the logic is exercised in
# isolation and the suite is deterministic and offline.


@pytest.fixture
def one_iface(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pretend the host has a single interface eth0 on the 10.0.0.0/24 LAN."""
    monkeypatch.setattr(lan, "_candidate_ifaces", lambda: [("eth0", "10.0.0.5")])


def test_discover_probe_finds_hosts_with_open_signaling_port(
    one_iface: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Only .79 answers on the signaling port -> only .79 is a Go2.
    monkeypatch.setattr(lan, "_tcp_open", lambda ip, port, timeout: ip == "10.0.0.79")
    monkeypatch.setattr(lan, "_resolve_mac", lambda ip: "C8:FE:0F:F7:F8:BB")

    devices = lan.discover_probe()

    assert len(devices) == 1
    dev = devices[0]
    assert dev.ip == "10.0.0.79"
    assert dev.mac == "C8:FE:0F:F7:F8:BB"
    assert dev.iface == "eth0"
    # Serial is never inferred from the network in the probe path.
    assert dev.serial == ""


def test_discover_probe_returns_empty_when_no_host_answers(
    one_iface: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(lan, "_tcp_open", lambda ip, port, timeout: False)
    monkeypatch.setattr(lan, "_resolve_mac", lambda ip: None)

    assert lan.discover_probe() == []


def test_discover_probe_uses_the_configured_port(
    one_iface: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    seen_ports: set[int] = set()

    def fake_open(ip: str, port: int, timeout: float) -> bool:
        seen_ports.add(port)
        return False

    monkeypatch.setattr(lan, "_tcp_open", fake_open)
    lan.discover_probe()

    # The probe targets the Go2 signaling port, not some hard-coded literal.
    assert seen_ports == {lan.GO2_SIGNALING_PORT}


def test_discover_falls_back_to_probe_when_multicast_is_silent(
    one_iface: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Multicast finds nothing (the IGMP-snooping case).
    monkeypatch.setattr(lan, "_probe_iface", lambda iface_ip, timeout: iter(()))
    probe = MagicMock(return_value=[Go2Device(serial="", ip="10.0.0.79", iface="eth0")])
    monkeypatch.setattr(lan, "discover_probe", probe)

    result = lan.discover()

    probe.assert_called_once()
    assert [d.ip for d in result] == ["10.0.0.79"]


def test_discover_skips_probe_when_multicast_finds_a_device(
    one_iface: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    # Multicast returns a real, serial-bearing device.
    monkeypatch.setattr(
        lan,
        "_probe_iface",
        lambda iface_ip, timeout: iter([Go2Device(serial="SN123", ip="10.0.0.9", iface="")]),
    )
    monkeypatch.setattr(lan, "_resolve_mac", lambda ip: "78:22:88:AA:AA:AA")
    probe = MagicMock()
    monkeypatch.setattr(lan, "discover_probe", probe)

    result = lan.discover()

    # The fallback must NOT run when multicast already succeeded.
    probe.assert_not_called()
    assert len(result) == 1
    assert result[0].serial == "SN123"
    assert result[0].mac == "78:22:88:AA:AA:AA"


def test_discover_does_not_fall_back_when_disabled(
    one_iface: None, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(lan, "_probe_iface", lambda iface_ip, timeout: iter(()))
    probe = MagicMock()
    monkeypatch.setattr(lan, "discover_probe", probe)

    result = lan.discover(probe_fallback=False)

    probe.assert_not_called()
    assert result == []
