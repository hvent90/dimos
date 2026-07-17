import pytest

from dimos.benchmark.spatial.pi_baseline.proxy_evidence import (
    PackageIndexProxyConfig,
    ProxyConfigurationError,
    ProxyEvidence,
    audit_post_run_output,
)


def config() -> PackageIndexProxyConfig:
    return PackageIndexProxyConfig(
        "https://proxy.example/simple",
        ("https://pypi.example/simple",),
        ("proxy.example", "pypi.example"),
    )


def test_normalizes_allowlisted_proxy_evidence() -> None:
    evidence = ProxyEvidence(config(), ("https://pypi.example/simple/",))
    assert evidence.normalized()["requested_urls"] == ("https://pypi.example/simple",)


def test_rejects_unallowlisted_index_and_observation() -> None:
    with pytest.raises(ProxyConfigurationError):
        PackageIndexProxyConfig("https://proxy.example", ("https://evil.example",), ("proxy.example",))
    with pytest.raises(ProxyConfigurationError):
        ProxyEvidence(config(), ("https://evil.example/package",)).normalized()


def test_post_run_audit_flags_network_and_overrides_but_is_not_proof() -> None:
    audit = audit_post_run_output(
        "uv pip install --index-url https://evil.example --proxy http://x"
    )
    assert {"package-network", "proxy-override", "index-override"} <= set(audit.findings)
    assert "cannot prove absence" in audit.limitation
