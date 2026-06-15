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

from __future__ import annotations

import importlib
from types import ModuleType
from typing import Protocol, cast
import webbrowser

from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

VISER_INSTALL_HINT = (
    "Viser manipulation visualization requires Viser. Install it with: uv sync --extra manipulation"
)
VISER_URDF_INSTALL_HINT = (
    "Viser URDF support requires yourdfpy. Install it with: uv sync --extra manipulation"
)


class _ViserModule(Protocol):
    def ViserServer(self, *, host: str, port: int) -> object: ...


class _ViserExtrasModule(Protocol):
    ViserUrdf: object


class _Stoppable(Protocol):
    def stop(self) -> None: ...


def import_viser() -> ModuleType:
    """Import Viser with a feature-specific install hint."""
    try:
        return importlib.import_module("viser")
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(VISER_INSTALL_HINT) from e


def import_viser_urdf() -> object:
    """Import ViserUrdf with a feature-specific install hint."""
    try:
        viser_extras = importlib.import_module("viser.extras")
    except (ImportError, ModuleNotFoundError) as e:
        raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e
    try:
        return cast("_ViserExtrasModule", viser_extras).ViserUrdf
    except AttributeError as e:
        raise ModuleNotFoundError(VISER_URDF_INSTALL_HINT) from e


class ViserRuntime:
    """Owns the Viser server lifecycle."""

    def __init__(self, config: ViserVisualizationConfig) -> None:
        self.config = config
        self.server: object | None = None

    @property
    def url(self) -> str | None:
        if self.server is None:
            return None
        return f"http://{self.config.host}:{self.config.port}"

    def start(self) -> object:
        if self.server is None:
            viser = cast("_ViserModule", import_viser())
            self.server = viser.ViserServer(host=self.config.host, port=self.config.port)
            _apply_appearance(self.server)
            if self.config.open_browser and self.url:
                webbrowser.open_new_tab(self.url)
        return self.server

    def close(self) -> None:
        server = self.server
        self.server = None
        if server is not None and hasattr(server, "stop"):
            cast("_Stoppable", server).stop()


# Brand color sampled from the dimensional logo; used as the viser UI accent.
_BRAND_COLOR = (96, 200, 220)


def _branded_background_image() -> object | None:
    """A dark vertical gradient with the dimensional logo centered, used instead of
    viser's flat white background. Returns an HxWx3 uint8 array, or None on failure."""
    try:
        import numpy as np

        w, h = 1280, 720
        top = np.array([26.0, 30.0, 42.0])
        bot = np.array([8.0, 10.0, 15.0])
        t = np.linspace(0.0, 1.0, h)[:, None, None]
        grad = (top * (1.0 - t) + bot * t).astype(np.uint8)  # (h, 1, 3)
        bg = np.ascontiguousarray(np.broadcast_to(grad, (h, w, 3)))

        try:
            from PIL import Image

            from dimos.constants import DIMOS_PROJECT_ROOT

            logo_path = (
                DIMOS_PROJECT_ROOT / "docs" / "assets" / "dimensional-logo-master-transparent.png"
            )
            if logo_path.exists():
                logo = Image.open(logo_path).convert("RGBA")
                scale = (w * 0.55) / logo.width
                logo = logo.resize(
                    (max(1, int(logo.width * scale)), max(1, int(logo.height * scale)))
                )
                logo.putalpha(logo.split()[3].point(lambda v: int(v * 0.85)))  # slightly soften
                canvas = Image.fromarray(bg, "RGB").convert("RGBA")
                canvas.alpha_composite(
                    logo, ((w - logo.width) // 2, (h - logo.height) // 2)
                )
                bg = np.asarray(canvas.convert("RGB"))
        except Exception as exc:  # noqa: BLE001 - logo is optional; keep the gradient
            logger.debug(f"viser: dimensional logo overlay skipped: {exc}")

        return bg
    except Exception as exc:  # noqa: BLE001 - background is cosmetic, never fatal
        logger.debug(f"viser: branded background skipped: {exc}")
        return None


def _apply_appearance(server: object) -> None:
    """Give the viser scene a branded look (dark gradient + dimensional logo background,
    a ground grid, and a dark UI theme) instead of a flat white page. Every step is
    best-effort so a viser API change never breaks the visualization."""
    scene = getattr(server, "scene", None)
    gui = getattr(server, "gui", None)

    img = _branded_background_image()
    if img is not None and scene is not None and hasattr(scene, "set_background_image"):
        try:
            scene.set_background_image(img)
        except Exception as exc:  # noqa: BLE001
            logger.debug(f"viser: set_background_image skipped: {exc}")

    if scene is not None and hasattr(scene, "add_grid"):
        try:
            scene.add_grid(
                "/ground",
                width=8.0,
                height=8.0,
                cell_size=0.25,
                section_size=1.0,
                plane="xy",  # floor in the z-up world
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug(f"viser: add_grid skipped: {exc}")

    if gui is not None and hasattr(gui, "configure_theme"):
        try:
            gui.configure_theme(
                dark_mode=True, brand_color=_BRAND_COLOR, control_layout="collapsible"
            )
        except Exception as exc:  # noqa: BLE001
            logger.debug(f"viser: configure_theme skipped: {exc}")
