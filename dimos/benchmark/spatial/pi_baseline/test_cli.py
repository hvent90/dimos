import pytest

from dimos.benchmark.spatial.pi_baseline.cli import main


@pytest.mark.parametrize("command", ["review", "report"])
def test_phase3_experiment_commands_remain_disabled(
    command: str, capsys: pytest.CaptureFixture[str]
) -> None:
    arguments = ["experiment", command, "experiment-1"]
    if command == "review":
        arguments.extend(
            ["--private-root", "private", "--reviewer", "reviewer", "--decision", "approved"]
        )
    else:
        arguments.extend(["--private-root", "private", "--review-decision", "decision.json"])
    assert main(arguments) == 1
    assert capsys.readouterr().err == "pi-baseline: operation unavailable\n"


def test_phase2_commands_require_explicit_bindings() -> None:
    with pytest.raises(SystemExit) as error:
        main(["experiment", "run", "experiment-1"])
    assert error.value.code == 2


def test_run_paired_is_not_a_production_command() -> None:
    with pytest.raises(SystemExit):
        main(["run-paired", "experiment-1"])
