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

"""Debug logging helpers for the camera calibration CLI."""

from __future__ import annotations

import logging
from pathlib import Path
import tempfile
import time


def setup_debug_logger(debug: bool = False) -> tuple[logging.Logger, Path | None]:
    """Create a per-run debug logger, or a no-op logger when debug is disabled."""
    logger = logging.getLogger(f"dimos.cameracalibrate.{time.time_ns()}")
    logger.propagate = False

    if not debug:
        logger.addHandler(logging.NullHandler())
        return logger, None

    log_dir = Path(tempfile.gettempdir()) / "dimos-cameracalibrate"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"webcam-{time.strftime('%Y%m%d-%H%M%S')}-{time.time_ns()}.log"

    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler(log_path, encoding="utf-8")
    handler.setLevel(logging.DEBUG)
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(handler)
    return logger, log_path
