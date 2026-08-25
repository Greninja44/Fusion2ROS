"""Orchestration layer: RobotModel + mesh files -> generated ROS 2 package.

Deliberately Fusion-symbol-free (no `adsk` import) so it's testable without
Fusion, per ARCHITECTURE.md's "don't put extraction/generation logic
directly inside UI callbacks" rule -- fusion_addin/ui/command.py (the actual
Fusion command handler) is a thin adapter that gathers adsk-side inputs
(the live Design, a robot name, an output directory) and calls straight into
this module; every decision of substance lives here where it can be unit
tested with a fake FusionDesignReader and plain paths.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional

from robot_model import Robot, JointType

from .extraction.converter import build_robot_model
from .extraction.interface import FusionDesignReader
from .generators.package import generate_package
from .generators.urdf import generate_urdf_xacro


class PipelineError(Exception):
    """Raised for a clear, actionable, pre-generation problem (e.g. a joint
    missing the motor limits URDF requires) -- distinct from a bug, this is
    expected user-facing input the caller should display, not a traceback."""


def build_robot_from_reader(reader: FusionDesignReader, robot_name: str) -> Robot:
    """Thin, named pass-through to converter.build_robot_model -- exists so
    callers (fusion_addin/ui/command.py) that need the Robot *before* mesh
    export (mesh export needs the live Design between this step and
    generate_ros_package) don't have to import converter directly."""
    return build_robot_model(reader, robot_name)


def check_missing_actuator_limits(robot: Robot) -> List[str]:
    """Fusion's CAD Joint object carries no motor/velocity/effort data (see
    fusion_adapter.py's _revolute_info/_slider_info), so every revolute or
    prismatic joint extracted straight from Fusion is missing the
    velocity_limit/effort_limit that URDF's <limit> tag requires --
    generate_urdf_xacro refuses to invent those values and raises ValueError
    instead. Surfacing that as one clear, itemized list up front (rather than
    a bare ValueError naming only the first offending joint) is what lets a
    caller show the user exactly what's missing before attempting generation.
    """
    problems = []
    for joint in robot.joints:
        if joint.type not in (JointType.REVOLUTE, JointType.PRISMATIC):
            continue
        missing = [
            attr
            for attr, val in (("velocity_limit", joint.velocity_limit), ("effort_limit", joint.effort_limit))
            if val is None
        ]
        if missing:
            problems.append(f"Joint {joint.name!r} ({joint.type.value}) is missing: {', '.join(missing)}")
    return problems


def attach_mesh_references(robot: Robot, mesh_files: Dict[str, Path]) -> Robot:
    """Sets visual_geometry/collision_geometry (both -- same mesh for MVP; a
    separate simplified collision mesh is a documented future enhancement,
    not attempted here) on each link named in mesh_files, using a
    package://<robot.name>/meshes/<filename> reference. Mutates and returns
    `robot`. Links not present in mesh_files are left untouched (no visual --
    e.g. a purely structural/reference link with no exportable body)."""
    from robot_model import Geometry

    for link in robot.links:
        mesh_path = mesh_files.get(link.name)
        if mesh_path is None:
            continue
        filename = Path(mesh_path).name
        geometry = Geometry(kind="mesh", mesh_path=f"package://{robot.name}/meshes/{filename}")
        link.visual_geometry = geometry
        link.collision_geometry = geometry
    return robot


def splice_xml_fragments(urdf_xacro: str, fragments: List[str]) -> str:
    """Inserts additional top-level XML elements (e.g. ros2_control.py's
    <ros2_control> element, or gazebo.py's <gazebo_fragment>-wrapped blocks)
    into an already-generated <robot>...</robot> URDF/xacro document, just
    before the closing tag.

    A <gazebo_fragment> wrapper (gazebo.py's documented splice-point
    convention, since it emits more than one sibling <gazebo> element) is
    unwrapped -- its children are inserted directly, since URDF itself has
    no <gazebo_fragment> tag. Any other fragment is inserted as a single
    element as-is (e.g. ros2_control.py's <ros2_control> root).
    """
    pieces = []
    for frag in fragments:
        root = ET.fromstring(frag)
        if root.tag == "gazebo_fragment":
            pieces.extend(ET.tostring(child, encoding="unicode") for child in root)
        else:
            pieces.append(ET.tostring(root, encoding="unicode"))

    text = urdf_xacro.rstrip()
    if not text.endswith("</robot>"):
        raise ValueError("urdf_xacro must end with </robot> to splice fragments into it")
    return text[: -len("</robot>")] + "".join(pieces) + "</robot>\n"


def generate_ros_package(
    robot: Robot,
    mesh_files: Dict[str, Path],
    output_dir: Path,
    include_ros2_control: bool = False,
    include_gazebo: bool = False,
    include_moveit: bool = False,
    include_nav2: bool = False,
    moveit_group_name: str = "arm",
) -> Path:
    """robot -> URDF/Xacro text -> full ROS 2 package tree under
    output_dir/<robot.name>/. Raises PipelineError (not a bare ValueError)
    with the full itemized list if any joint is missing required limits, or
    if a requested optional output (MoveIt/Nav2) isn't suitable for `robot`.

    The four `include_*` flags mirror the Fusion UI's planned output
    checkboxes (URDF/Xacro and the base ROS 2 package are always produced;
    ros2_control/Gazebo/MoveIt 2/Nav2 are each opt-in since not every robot
    needs or supports all four -- an arm has no business with Nav2, and a
    mobile base with no `metadata["drivetrain"]` has no business with
    MoveIt's arm-planning groups).
    """
    problems = check_missing_actuator_limits(robot)
    if problems:
        raise PipelineError(
            "Cannot generate URDF -- the following joints are missing motor limits "
            "required by the URDF <limit> tag (set them via an Actuator, or edit the "
            "Robot before generating):\n  " + "\n  ".join(problems)
        )

    attach_mesh_references(robot, mesh_files)
    urdf_xacro = generate_urdf_xacro(robot)

    fragments: List[str] = []
    extra_files: Dict[str, str] = {}

    if include_ros2_control:
        from .generators.ros2_control import (
            generate_control_launch,
            generate_controllers_yaml,
            generate_ros2_control_xml,
        )

        fragments.append(generate_ros2_control_xml(robot))
        extra_files["config/controllers.yaml"] = generate_controllers_yaml(robot)
        extra_files["launch/control.launch.py"] = generate_control_launch(robot)

    if include_gazebo:
        from .generators.gazebo import generate_gazebo_xml, generate_spawn_launch, generate_world_sdf

        fragments.append(generate_gazebo_xml(robot))
        extra_files["worlds/empty.sdf"] = generate_world_sdf()
        extra_files["launch/gazebo.launch.py"] = generate_spawn_launch(robot)

    if include_moveit:
        from .generators.moveit import (
            detect_moveit_suitability,
            generate_joint_limits_yaml,
            generate_kinematics_yaml,
            generate_moveit_controllers_yaml,
            generate_moveit_demo_launch,
            generate_srdf,
        )

        problems = detect_moveit_suitability(robot)
        if problems:
            raise PipelineError(
                "Cannot generate MoveIt 2 config -- robot is not suitable:\n  " + "\n  ".join(problems)
            )
        extra_files[f"config/{robot.name}.srdf"] = generate_srdf(robot, group_name=moveit_group_name)
        extra_files["config/joint_limits.yaml"] = generate_joint_limits_yaml(robot, moveit_group_name)
        extra_files["config/kinematics.yaml"] = generate_kinematics_yaml(robot, moveit_group_name)
        extra_files["config/moveit_controllers.yaml"] = generate_moveit_controllers_yaml(robot, moveit_group_name)
        extra_files["launch/moveit_demo.launch.py"] = generate_moveit_demo_launch(robot, moveit_group_name)

    if include_nav2:
        from .generators.nav2 import (
            detect_nav2_suitability,
            generate_map_yaml_stub,
            generate_nav2_bringup_launch,
            generate_nav2_params_yaml,
        )

        problems = detect_nav2_suitability(robot)
        if problems:
            raise PipelineError("Cannot generate Nav2 config -- robot is not suitable:\n  " + "\n  ".join(problems))
        extra_files["config/nav2_params.yaml"] = generate_nav2_params_yaml(robot)
        extra_files["launch/nav2_bringup.launch.py"] = generate_nav2_bringup_launch(robot)
        extra_files["config/map.yaml"] = generate_map_yaml_stub(robot)

    if fragments:
        urdf_xacro = splice_xml_fragments(urdf_xacro, fragments)

    # mesh_files here is keyed by link name (export_link_meshes' contract);
    # generate_package wants it keyed by the mesh's relative filename inside
    # the package (its own, independent contract) -- re-key at this boundary
    # rather than making either side guess the other's convention.
    package_mesh_files = {Path(path).name: path for path in mesh_files.values()}
    return generate_package(robot, urdf_xacro, package_mesh_files, output_dir, extra_files=extra_files)


def run_pipeline(
    reader: FusionDesignReader,
    robot_name: str,
    output_dir: Path,
    mesh_files: Optional[Dict[str, Path]] = None,
    **generate_kwargs,
) -> tuple:
    """Full pipeline: FusionDesignReader -> Robot -> generated package.
    Returns (robot, package_dir). `mesh_files` is supplied by the caller
    (typically fusion_addin/generators/mesh.export_link_meshes, run against
    the live Design before this is called -- kept as a separate step here
    since mesh export needs the live adsk Design/Occurrence objects that
    FusionDesignReader's plain-data abstraction deliberately doesn't carry).
    `generate_kwargs` forwards the include_ros2_control/include_gazebo/
    include_moveit/include_nav2/moveit_group_name flags to generate_ros_package.
    """
    robot = build_robot_from_reader(reader, robot_name)
    package_dir = generate_ros_package(robot, mesh_files or {}, Path(output_dir), **generate_kwargs)
    return robot, package_dir
