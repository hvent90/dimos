"""Native MuJoCo assets and generation helpers for the Galaxea A1Z."""

from pathlib import Path

from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule

A1Z_SCENE_PATH = Path(__file__).resolve().parent / "a1z_tabletop.xml"
A1Z_SIM_HOME = (0.0, 0.7, -1.2, 0.5, 0.0, 0.0)


class _A1ZMujocoSimModule(MujocoSimModule):
    """A1Z simulator isolated from the coordinator worker."""

    dedicated_worker = True
