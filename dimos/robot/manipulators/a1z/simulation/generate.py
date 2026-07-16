"""Generate the repository-local A1Z MuJoCo scene from the vendor URDF.

The generator deliberately uses only the Python standard library and native
MuJoCo XML.  It can therefore be rerun after hydrating the LFS description:

    python -m dimos.robot.manipulators.a1z.simulation.generate

The generated XML keeps mesh paths relative to the XML file, rather than
depending on ROS package lookup or the current working directory.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import tarfile
import tempfile
import xml.etree.ElementTree as ET

PACKAGE_DIR = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_DIR.parents[4]
DESCRIPTION_TAR = REPO_ROOT / "data/.lfs/a1z_description.tar.gz"
A1Z_SCENE_PATH = PACKAGE_DIR / "a1z_tabletop.xml"
ASSET_DIR = PACKAGE_DIR / "assets"
MAX_GRIPPER_TRAVEL = 0.015
A1Z_SIM_HOME = (0.0, 0.7, -1.2, 0.5, 0.0, 0.0)
ARM_POSITION_KP = 500.0
ARM_POSITION_KV = 45.0
LEFT_GRIPPER_AXIS = "0 -0.811 0.584"
RIGHT_GRIPPER_AXIS = "0 0.811 -0.584"


def _patch_urdf(path: Path) -> None:
    root = ET.parse(path).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    left = joints["gripper_finger_left_joint"]
    right = joints["gripper_finger_rIght_joint"]
    left.set("type", "prismatic")
    right.set("type", "prismatic")
    for joint, axis in ((left, LEFT_GRIPPER_AXIS), (right, RIGHT_GRIPPER_AXIS)):
        axis_element = joint.find("axis")
        if axis_element is None:
            axis_element = ET.SubElement(joint, "axis")
        axis_element.set("xyz", axis)
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        limit.attrib.update(lower="0", upper=str(MAX_GRIPPER_TRAVEL), effort="20", velocity="0.2")
    mimic = right.find("mimic")
    if mimic is None:
        mimic = ET.SubElement(right, "mimic")
    mimic.attrib.update(joint="gripper_finger_left_joint", multiplier="1", offset="0")
    ET.indent(root, space="  ")
    path.write_text(ET.tostring(root, encoding="unicode") + "\n")


def patch_description(tar_path: Path = DESCRIPTION_TAR) -> None:
    """Patch and deterministically repack the local vendor description."""
    with tempfile.TemporaryDirectory() as temporary:
        unpacked = Path(temporary)
        with tarfile.open(tar_path, "r:gz") as archive:
            archive.extractall(unpacked)
        _patch_urdf(unpacked / "a1z_description/A1Z_G1Z/urdf/A1Z_G1Z.urdf")
        temporary_tar = tar_path.with_suffix(".tmp.tar.gz")
        try:
            with tarfile.open(temporary_tar, "w:gz", compresslevel=9) as archive:
                for path in sorted(unpacked.rglob("*")):
                    archive.add(path, arcname=path.relative_to(unpacked))
            temporary_tar.replace(tar_path)
        finally:
            temporary_tar.unlink(missing_ok=True)


def _mesh_name(filename: str) -> str:
    return Path(filename).name


def _copy_meshes(unpacked: Path, asset_dir: Path) -> None:
    source = unpacked / "a1z_description/A1Z_G1Z/meshes"
    asset_dir.mkdir(parents=True, exist_ok=True)
    for mesh in source.glob("*.STL"):
        shutil.copyfile(mesh, asset_dir / mesh.name)


def _body(link_name: str, joint: ET.Element | None, link: ET.Element, children: dict[str, list[tuple[ET.Element, ET.Element]]]) -> ET.Element:
    body = ET.Element("body", name=link_name)
    if joint is not None:
        origin = joint.find("origin")
        if origin is not None:
            body.set("pos", origin.attrib.get("xyz", "0 0 0"))
            if "rpy" in origin.attrib:
                body.set("euler", origin.attrib["rpy"])
        if joint.attrib["type"] != "fixed":
            mj_type = "slide" if joint.attrib["type"] == "prismatic" else "hinge"
            joint_element = ET.SubElement(body, "joint", name=joint.attrib["name"], type=mj_type, damping="1")
            axis = joint.find("axis")
            if axis is not None:
                joint_element.set("axis", axis.attrib.get("xyz", "0 0 1"))
            limit = joint.find("limit")
            if limit is not None:
                joint_element.set("range", f"{limit.attrib.get('lower', '-3.14')} {limit.attrib.get('upper', '3.14')}")
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is not None:
            filename = _mesh_name(mesh.attrib["filename"])
            ET.SubElement(body, "geom", name=f"{link_name}_visual", type="mesh", mesh=filename, contype="0", conaffinity="0", mass="0.1")
    for child_joint, child_link in children.get(link_name, []):
        body.append(_body(child_link.attrib["name"], child_joint, child_link, children))
    return body


def _scene_xml(urdf_path: Path) -> ET.Element:
    urdf = ET.parse(urdf_path).getroot()
    links = {link.attrib["name"]: link for link in urdf.findall("link")}
    children: dict[str, list[tuple[ET.Element, ET.Element]]] = {}
    child_names: set[str] = set()
    for joint in urdf.findall("joint"):
        parent_element = joint.find("parent")
        child_element = joint.find("child")
        if parent_element is None or child_element is None:
            raise ValueError(f"joint {joint.attrib.get('name', '<unnamed>')} has no parent/child")
        parent = parent_element.attrib["link"]
        child = child_element.attrib["link"]
        children.setdefault(parent, []).append((joint, links[child]))
        child_names.add(child)
    base = next(name for name in links if name not in child_names)
    root = ET.Element("mujoco", model="a1z_tabletop")
    ET.SubElement(root, "compiler", angle="radian", meshdir="assets")
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 -9.81", integrator="implicitfast")
    asset = ET.SubElement(root, "asset")
    for mesh in sorted(ASSET_DIR.glob("*.STL")):
        ET.SubElement(asset, "mesh", name=mesh.name, file=mesh.name)
    ET.SubElement(asset, "texture", name="cube_texture", type="2d", builtin="flat", width="32", height="32", rgb1="0.9 0.2 0.05", rgb2="0.9 0.2 0.05")
    ET.SubElement(asset, "material", name="cube_material", texture="cube_texture")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="key_light", pos="1 -1 2.5", dir="-0.3 0.3 -1", directional="true")
    table = ET.SubElement(world, "body", name="table", pos="0 0 0.38")
    ET.SubElement(table, "geom", name="tabletop", type="box", size="0.65 0.5 0.03", pos="0 0 0", friction="0.8 0.1 0.1")
    ET.SubElement(table, "geom", name="table_leg", type="box", size="0.05 0.05 0.38", pos="-0.5 0  -0.38")
    arm = ET.SubElement(world, "body", name="a1z", pos="0.42 0 0.41")
    arm.append(_body(base, None, links[base], children))
    cube = ET.SubElement(world, "body", name="cube", pos="0.08 -0.05 0.46")
    ET.SubElement(cube, "freejoint", name="cube_free")
    ET.SubElement(cube, "geom", name="cube_geom", type="box", size="0.025 0.025 0.025", mass="0.1", material="cube_material", friction="0.8 0.1 0.1")
    # Pads are intentionally primitive and separate from the visual finger meshes.
    for finger_name, pad_name in (("gripper_finger_left_link", "left_pad"), ("gripper_finger_rIght_link", "right_pad")):
        finger_body = next(element for element in arm.iter("body") if element.attrib.get("name") == finger_name)
        ET.SubElement(finger_body, "geom", name=pad_name, type="box", size="0.025 0.004 0.012", pos="0.105 0 0", friction="0.8 0.1 0.1")
    sensor_body = next(element for element in arm.iter("body") if element.attrib.get("name") == "arm_link6")
    ET.SubElement(sensor_body, "camera", name="wrist_camera", pos="0.11 0 -0.02", euler="0 1.57 1.57", mode="fixed", fovy="65")
    actuator = ET.SubElement(root, "actuator")
    for joint_name in [f"arm_joint{i}" for i in range(1, 7)]:
        ET.SubElement(
            actuator,
            "position",
            name=f"{joint_name}_motor",
            joint=joint_name,
            kp=str(ARM_POSITION_KP),
            kv=str(ARM_POSITION_KV),
        )
    ET.SubElement(actuator, "position", name="gripper_motor", joint="gripper_finger_left_joint", kp="10", kv="5", ctrlrange="0 0.015")
    equality = ET.SubElement(root, "equality")
    ET.SubElement(equality, "joint", name="gripper_mimic", joint1="gripper_finger_rIght_joint", joint2="gripper_finger_left_joint", polycoef="0 1 0 0 0")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="640", offheight="480")
    custom = ET.SubElement(root, "custom")
    ET.SubElement(custom, "numeric", name="wrist_camera_fps", data="30")
    keyframe = ET.SubElement(root, "keyframe")
    home_qpos = (*A1Z_SIM_HOME, 0.0, 0.0, 0.08, -0.05, 0.46, 1.0, 0.0, 0.0, 0.0)
    ET.SubElement(keyframe, "key", name="home", qpos=" ".join(map(str, home_qpos)))
    ET.indent(root, space="  ")
    return root


def generate_scene(output: Path = A1Z_SCENE_PATH, description_tar: Path = DESCRIPTION_TAR) -> Path:
    """Generate the final scene and its relative mesh assets."""
    with tempfile.TemporaryDirectory() as temporary:
        unpacked = Path(temporary)
        with tarfile.open(description_tar, "r:gz") as archive:
            archive.extractall(unpacked)
        _copy_meshes(unpacked, ASSET_DIR)
        root = _scene_xml(unpacked / "a1z_description/A1Z_G1Z/urdf/A1Z_G1Z.urdf")
    output.write_text(ET.tostring(root, encoding="unicode") + "\n")
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repack-description", action="store_true")
    parser.add_argument("--output", type=Path, default=A1Z_SCENE_PATH)
    args = parser.parse_args()
    if args.repack_description:
        patch_description()
    generate_scene(args.output)


if __name__ == "__main__":
    main()
