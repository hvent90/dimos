import json
from pathlib import Path

import pytest

from dimos.benchmark.spatial.pi_baseline.cli import main
from dimos.benchmark.spatial.test_pi_baseline_config import valid_payload


def test_validate_command_is_explicit_and_side_effect_free(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    auth = tmp_path / "oauth.json"
    auth.write_text("{}", encoding="utf-8")
    config = tmp_path / "config.json"
    config.write_text(json.dumps(valid_payload(auth)), encoding="utf-8")
    assert main(["validate", str(config)]) == 0
    assert "configuration is valid" in capsys.readouterr().out
