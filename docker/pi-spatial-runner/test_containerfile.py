from pathlib import Path

CONTAINERFILE = Path(__file__).with_name("Containerfile")


def test_runner_containerfile_is_rootless_and_data_free() -> None:
    text = CONTAINERFILE.read_text()

    assert "python:3.12" in text
    assert "uv==" in text
    assert "USER runner" in text
    assert "WORKDIR /work" in text
    assert "ARG RUNNER_UID" in text
    assert "ARG RUNNER_GID" in text
    assert "COPY " not in text
    assert "SECRET" not in text.upper()
    assert "/var/run/docker.sock" not in text
