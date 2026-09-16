#!/usr/bin/env python3
"""Combine the Marvin M6S Lite CCS-680 left/right URDFs in one centered frame."""

from __future__ import annotations

import copy
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


HERE = Path(__file__).resolve().parent
LEFT_DIR = HERE / "Marvin M6S-Lite-L-CCS-680-V1.0 urdf"
RIGHT_DIR = HERE / "Marvin M6S-Lite-R-CCS-680-V1.0 urdf"
LEFT_URDF = LEFT_DIR / "urdf/Marvin M6S-Lite-L-CCS-680-V1.0 urdf.urdf"
RIGHT_URDF = RIGHT_DIR / "urdf/Marvin M6S-Lite-R-CCS-680-V1.0 urdf.urdf"
OUTPUT = HERE / "marvin_m6s_lite_dual_ccs_680.xml"
URDF_OUTPUT = HERE / "marvin_m6s_lite_dual_ccs_680.urdf"
GRIPPER_SOURCE_DIR = HERE.parent / "FingerController_urdf"
GRIPPER_SOURCE_URDF = GRIPPER_SOURCE_DIR / "urdf/FingerController_urdf.urdf"
GRIPPER_DIR = HERE / "gripper"
GRIPPER_URDF = GRIPPER_DIR / "finger_controller_simplified.urdf"


def append_model(output_root: ET.Element, source_path: Path, source_dir: Path) -> None:
    source_root = ET.parse(source_path).getroot()
    for child in source_root:
        cloned = copy.deepcopy(child)
        for mesh in cloned.iter("mesh"):
            filename = mesh.get("filename")
            if filename and filename.startswith("package://"):
                # The package prefix is the directory name; make it relative to this XML.
                relative_mesh = filename.removeprefix("package://").split("/meshes/", 1)[1]
                mesh.set("filename", f"{source_dir.name}/meshes/{relative_mesh}")
        # The vendor TCP marker has zero mass and a near-zero-volume collision STL.
        # Keep its visual geometry, but omit that collision so MuJoCo can compile it.
        if cloned.tag == "link" and cloned.get("name", "").startswith("TCP_Link_"):
            collision = cloned.find("collision")
            if collision is not None:
                cloned.remove(collision)
        output_root.append(cloned)


def make_simplified_gripper() -> None:
    """Re-root the gripper at fflan_Link and retain one commanded finger DOF."""
    source = ET.parse(GRIPPER_SOURCE_URDF).getroot()

    # fflan_Link is the mounting interface. Reverse the original fixed joint so
    # the rest of the housing remains in exactly the same physical location.
    flange_joint = source.find("joint[@name='fflan']")
    if flange_joint is None:
        raise ValueError("gripper joint 'fflan' not found")
    flange_joint.set("name", "fflan_to_base")
    flange_joint.find("parent").set("link", "fflan_Link")
    flange_joint.find("child").set("link", "base_link")
    flange_joint.find("origin").set(
        "xyz", "0.0708615418463922 0.0812149317453119 -0.0000482480870888924"
    )
    flange_joint.find("origin").set("rpy", "-1.5707963267948966 0 0")

    left_joint = source.find("joint[@name='fleft']")
    right_joint = source.find("joint[@name='fright']")
    if left_joint is None or right_joint is None:
        raise ValueError("gripper finger joints not found")
    left_joint.find("limit").set("lower", "0")
    left_joint.find("limit").set("upper", "1")
    right_joint.find("limit").set("lower", "-1")
    right_joint.find("limit").set("upper", "0")
    mimic = right_joint.find("mimic")
    if mimic is None:
        mimic = ET.SubElement(right_joint, "mimic")
    mimic.attrib.update({"joint": "fleft", "multiplier": "-1", "offset": "0"})

    for mesh in source.iter("mesh"):
        mesh.set("filename", f"meshes/{Path(mesh.get('filename')).name}")

    GRIPPER_DIR.mkdir(parents=True, exist_ok=True)
    mesh_output = GRIPPER_DIR / "meshes"
    mesh_output.mkdir(parents=True, exist_ok=True)
    for mesh_path in (GRIPPER_SOURCE_DIR / "meshes").glob("*.STL"):
        shutil.copy2(mesh_path, mesh_output / mesh_path.name)

    ET.indent(source, space="  ")
    ET.ElementTree(source).write(GRIPPER_URDF, encoding="utf-8", xml_declaration=True)


def append_gripper(output_root: ET.Element, prefix: str) -> None:
    """Append a namespaced copy of the simplified gripper."""
    source_root = ET.parse(GRIPPER_URDF).getroot()
    link_names = {element.get("name") for element in source_root.findall("link")}
    joint_names = {element.get("name") for element in source_root.findall("joint")}

    for child in source_root:
        cloned = copy.deepcopy(child)
        name = cloned.get("name")
        if cloned.tag == "link" and name in link_names:
            cloned.set("name", f"{prefix}{name}")
        elif cloned.tag == "joint" and name in joint_names:
            cloned.set("name", f"{prefix}{name}")
            cloned.find("parent").set("link", f"{prefix}{cloned.find('parent').get('link')}")
            cloned.find("child").set("link", f"{prefix}{cloned.find('child').get('link')}")
            mimic = cloned.find("mimic")
            if mimic is not None:
                mimic.set("joint", f"{prefix}{mimic.get('joint')}")

        for mesh in cloned.iter("mesh"):
            mesh.set("filename", f"gripper/meshes/{Path(mesh.get('filename')).name}")
        output_root.append(cloned)


def fixed_mount(name: str, child: str, xyz: str, rpy: str) -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": "dual_origin"})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def fixed_joint(name: str, parent: str, child: str, xyz: str, rpy: str) -> ET.Element:
    joint = ET.Element("joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    return joint


def box_link(name: str, size: str, rgba: str) -> ET.Element:
    link = ET.Element("link", {"name": name})
    for tag in ("visual", "collision"):
        element = ET.SubElement(link, tag)
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "box", {"size": size})
        if tag == "visual":
            material = ET.SubElement(element, "material", {"name": f"{name}_material"})
            ET.SubElement(material, "color", {"rgba": rgba})
    return link


def add_mujoco_scene(mjcf: ET.Element) -> None:
    """Restore rendering, floor, lighting, and the fixed overview camera."""
    compiler = mjcf.find("compiler")
    insert_at = list(mjcf).index(compiler) + 1 if compiler is not None else 0

    option = ET.Element(
        "option",
        {"timestep": "0.002", "gravity": "0 0 -9.81", "integrator": "implicitfast"},
    )
    statistic = ET.Element("statistic", {"center": "0 0 -0.35", "extent": "1.8"})
    visual = ET.Element("visual")
    ET.SubElement(
        visual,
        "global",
        {"offwidth": "1280", "offheight": "720", "azimuth": "135", "elevation": "-25"},
    )
    ET.SubElement(visual, "quality", {"shadowsize": "4096", "offsamples": "4"})
    ET.SubElement(
        visual,
        "headlight",
        {"ambient": "0.35 0.35 0.35", "diffuse": "0.7 0.7 0.7", "specular": "0.2 0.2 0.2"},
    )
    ET.SubElement(visual, "rgba", {"haze": "0.15 0.25 0.35 1"})
    for element in (option, statistic, visual):
        mjcf.insert(insert_at, element)
        insert_at += 1

    asset = mjcf.find("asset")
    if asset is None:
        asset = ET.SubElement(mjcf, "asset")
    asset.insert(
        0,
        ET.Element(
            "texture",
            {
                "name": "skybox",
                "type": "skybox",
                "builtin": "gradient",
                "rgb1": "0.32 0.46 0.62",
                "rgb2": "0.04 0.06 0.09",
                "width": "512",
                "height": "3072",
            },
        ),
    )
    asset.insert(
        1,
        ET.Element(
            "texture",
            {
                "name": "floor_grid",
                "type": "2d",
                "builtin": "checker",
                "rgb1": "0.22 0.24 0.27",
                "rgb2": "0.13 0.15 0.17",
                "width": "512",
                "height": "512",
            },
        ),
    )
    asset.insert(
        2,
        ET.Element(
            "material",
            {
                "name": "floor_material",
                "texture": "floor_grid",
                "texrepeat": "8 8",
                "reflectance": "0.12",
                "shininess": "0.15",
            },
        ),
    )

    worldbody = mjcf.find("worldbody")
    if worldbody is None:
        raise ValueError("converted MJCF has no worldbody")
    # Requested camera: position (-0.2, 0.2, 1.5), looking diagonally down
    # toward +X while retaining y=0.2. Target used here is (0.35, 0.2, -0.25).
    worldbody.insert(
        0,
        ET.Element(
            "camera",
            {
                "name": "overview",
                "pos": "-0.2 0.2 1.5",
                "xyaxes": "0 -1 0 0.953583 0 0.301131",
                "fovy": "45",
            },
        ),
    )
    worldbody.insert(
        1,
        ET.Element(
            "light",
            {
                "name": "key_light",
                "pos": "-1.5 1.2 2.5",
                "dir": "0.45 -0.3 -1",
                "directional": "true",
                "castshadow": "true",
                "diffuse": "0.85 0.85 0.8",
            },
        ),
    )
    worldbody.insert(
        2,
        ET.Element(
            "light",
            {
                "name": "fill_light",
                "pos": "1 -1 1.2",
                "dir": "-0.5 0.4 -0.7",
                "directional": "true",
                "diffuse": "0.35 0.4 0.5",
                "specular": "0.05 0.05 0.05",
            },
        ),
    )
    worldbody.insert(
        3,
        ET.Element(
            "geom",
            {
                "name": "floor",
                "type": "plane",
                "pos": "0 0 -1.225",
                "size": "3 3 0.1",
                "material": "floor_material",
                "condim": "3",
            },
        ),
    )

    # Vendor link colors are fully saturated (1,0,0 / 0,1,0 / 0,0,1), which
    # look overexposed under MuJoCo lighting. Preserve the color coding while
    # reducing mesh brightness.
    for geom in worldbody.iter("geom"):
        if geom.get("type") != "mesh" or not geom.get("rgba"):
            continue
        rgba = [float(value) for value in geom.get("rgba").split()]
        rgba[:3] = [channel * 0.42 for channel in rgba[:3]]
        geom.set("rgba", " ".join(f"{value:.6g}" for value in rgba))


def main() -> None:
    make_simplified_gripper()
    robot = ET.Element("robot", {"name": "marvin_m6s_lite_dual_ccs_680"})
    robot.append(ET.Element("link", {"name": "dual_origin"}))

    # Central pedestal: arm midpoint is z=0, and the support extends downward.
    robot.append(box_link("support_column", "0.12 0.12 1.2", "0.28 0.31 0.35 1"))
    robot.append(fixed_mount("dual_to_support_column", "support_column", "0 0 -0.6", "0 0 0"))
    robot.append(box_link("base_plate", "0.6 0.6 0.02", "0.18 0.20 0.23 1"))
    robot.append(fixed_mount("dual_to_base_plate", "base_plate", "0 0 -1.21", "0 0 0"))

    append_model(robot, LEFT_URDF, LEFT_DIR)
    append_model(robot, RIGHT_URDF, RIGHT_DIR)

    # Overall frame convention:
    #   Xo = +XL = +XR
    #   Zo = +YR = -YL
    #   Yo completes a right-handed frame, so Yo = +ZL = -ZR.
    # Base origins are 120 mm apart and centered at dual_origin. Following the
    # back-to-back installation drawing, each base +Z points outward.
    robot.append(fixed_mount("dual_to_Base_L", "Base_L", "0 0.06 -0.1", "-1.5707963267948966 0 0"))
    robot.append(fixed_mount("dual_to_Base_R", "Base_R", "0 -0.06 -0.1", "1.5707963267948966 0 0"))

    append_gripper(robot, "L_")
    append_gripper(robot, "R_")

    # Gripper mount frame is fflan_Link. Rotation columns express gripper axes
    # in each arm flange frame.
    # Left:  Xg=+Za, Yg=-Ya, Zg=+Xa.
    # Right: Xg=+Za, Yg=+Ya, Zg=-Xa.
    robot.append(
        fixed_joint(
            "TCP_L_to_gripper",
            "TCP_Link_L",
            "L_fflan_Link",
            "0 0 0",
            "3.141592653589793 -1.5707963267948966 0",
        )
    )
    robot.append(
        fixed_joint(
            "TCP_R_to_gripper",
            "TCP_Link_R",
            "R_fflan_Link",
            "0 0 0",
            "0 -1.5707963267948966 0",
        )
    )

    ET.indent(robot, space="  ")
    tree = ET.ElementTree(robot)
    tree.write(URDF_OUTPUT, encoding="utf-8", xml_declaration=True)
    print(URDF_OUTPUT)

    # MuJoCo does not import URDF <mimic>. Convert the combined URDF to native
    # MJCF and express each opposing finger pair with an equality constraint.
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(URDF_OUTPUT))
    mujoco.mj_saveLastXML(str(OUTPUT), model)
    mjcf = ET.parse(OUTPUT).getroot()
    add_mujoco_scene(mjcf)
    equality = mjcf.find("equality")
    if equality is None:
        equality = ET.SubElement(mjcf, "equality")
    ET.SubElement(
        equality,
        "joint",
        {
            "name": "L_gripper_coupling",
            "joint1": "L_fright",
            "joint2": "L_fleft",
            "polycoef": "0 -1 0 0 0",
        },
    )
    ET.SubElement(
        equality,
        "joint",
        {
            "name": "R_gripper_coupling",
            "joint1": "R_fright",
            "joint2": "R_fleft",
            "polycoef": "0 -1 0 0 0",
        },
    )
    ET.indent(mjcf, space="  ")
    ET.ElementTree(mjcf).write(OUTPUT, encoding="utf-8", xml_declaration=True)
    print(OUTPUT)


if __name__ == "__main__":
    main()
