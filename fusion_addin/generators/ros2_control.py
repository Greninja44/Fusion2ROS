"""RobotModel -> ros2_control XML fragment + controller_manager YAML + a
control launch file.

Pure function(s) of a `robot_model.Robot`: no Fusion API, no ROS/rclpy
imports, no filesystem access, no network -- same constraint
`fusion_addin/generators/urdf.py` and `robot_model/schema.py` are held to, so
this module is importable and testable from plain WSL `python3 -m pytest`
with neither Fusion nor a ROS install present.

This module does NOT produce a full `<robot>` URDF document -- only a
`<ros2_control>` element as a string (`generate_ros2_control_xml`), meant to
be spliced into the existing URDF produced by `generators/urdf.py` by a
later, separate integration step. It is deliberately decoupled from
`urdf.py`/`app.py`/`package.py`: it only reads `robot_model.Robot` and never
imports those modules.

Differential-drive vs. arm/manipulator convention
--------------------------------------------------
`Robot` has no built-in notion of "this is a mobile base" -- it's a general
kinematic tree. Per the documented, shared convention (see the parallel
Nav2-generator work and the project brief), a differential-drive robot is
identified purely by `Robot.metadata["drivetrain"]`::

    robot.metadata["drivetrain"] = {
        "type": "differential_drive",
        "left_wheel_joint": "<joint name, CONTINUOUS or REVOLUTE>",
        "right_wheel_joint": "<joint name>",
        "wheel_separation": <float, meters>,
        "wheel_radius": <float, meters>,
    }

If that key is absent (or its "type" isn't "differential_drive"), every
non-fixed joint is treated as an individual arm/manipulator joint, driven by
a `joint_trajectory_controller`. If present, the two named wheel joints are
excluded from that per-joint treatment, get `velocity`-only command/state
interfaces, and are driven by a `diff_drive_controller` instead.

API surface verified against real ros2_control sources/docs
-------------------------------------------------------------
* `mock_components/GenericSystem` hardware plugin -- confirmed against
  ros2_control's own docs (control.ros.org "Mock Components" user doc) and
  against `ros2_control_demos`' `example_2/description/ros2_control/
  diffbot.ros2_control.xacro`, which uses exactly
  ``<hardware><plugin>mock_components/GenericSystem</plugin></hardware>``
  for its mock-hardware branch, with wheel joints getting a `velocity`
  command interface plus `position`+`velocity` state interfaces (no
  `effort`) -- exactly the shape this module emits for drivetrain wheels.
* `joint_state_broadcaster/JointStateBroadcaster`,
  `joint_trajectory_controller/JointTrajectoryController`, and
  `diff_drive_controller/DiffDriveController` type strings, plus the
  `left_wheel_names`/`right_wheel_names`/`wheel_separation`/`wheel_radius`
  diff-drive parameter names -- confirmed against
  `ros2_control_demos`' `example_2/bringup/config/diffbot_controllers.yaml`
  and `ros2_controllers`' `diff_drive_controller` user doc.
* The `controller_manager: ros__parameters: {update_rate, <controller_name>:
  {type: ...}}` plus a separate top-level `<controller_name>: ros__parameters:
  {...}` block per controller is the long-standing, widely used
  ros2_control YAML convention (Articulated Robotics' ros2_control
  tutorials, gz_ros2_control demos, ros2_control_demos' multi-controller
  examples) for auto-loadable controllers -- this is what's implemented
  here per the project brief's explicit instruction to shape the YAML this
  way.
* Spawning controllers via `ros2 run controller_manager spawner
  <controller_name>` (expressed as a launch `Node`) and starting
  `controller_manager/ros2_control_node` parameterized by both
  `robot_description` and a controllers YAML file are both drawn directly
  from `ros2_control_demos`' `example_1/bringup/launch/rrbot.launch.py`
  lineage (older/stable form: `parameters=[robot_description,
  controllers_yaml_path]` for the control node; newer `example_2` moved to
  `--param-file` on the spawner instead of a Python-object description --
  both are real, documented patterns; this module follows the former since
  that's the shape the project brief explicitly asked for).

Design notes (mirrors urdf.py's philosophy)
--------------------------------------------
* Deterministic output: joint ordering follows `Robot.joints` list order,
  never a dict/set iteration order; XML attribute order is the literal
  insertion order.
* "Don't invent" philosophy: a non-fixed, non-wheel joint with no matching
  `Actuator` raises `ValueError` naming the joint rather than guessing an
  interface. An `Actuator.interface` outside {"position", "velocity",
  "effort"} likewise raises `ValueError` rather than silently passing
  through an invalid ros2_control interface string.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Dict, List, Optional

from robot_model import Actuator, Joint, JointType, Robot

_VALID_INTERFACES = ("position", "velocity", "effort")
_WHEEL_JOINT_TYPES = (JointType.CONTINUOUS, JointType.REVOLUTE)

_JOINT_STATE_BROADCASTER_TYPE = "joint_state_broadcaster/JointStateBroadcaster"
_JOINT_TRAJECTORY_CONTROLLER_TYPE = "joint_trajectory_controller/JointTrajectoryController"
_DIFF_DRIVE_CONTROLLER_TYPE = "diff_drive_controller/DiffDriveController"


# --- shared helpers ---------------------------------------------------------


def _drivetrain_info(robot: Robot) -> Optional[Dict[str, object]]:
    """Return `robot.metadata["drivetrain"]` if it declares a
    differential_drive drivetrain, else None. Validates the two referenced
    wheel joints exist and are of a type continuous/revolute joints can
    sensibly spin (per the documented convention) -- raises ValueError with
    a clear message otherwise, rather than silently emitting nonsense.
    """
    drivetrain = (robot.metadata or {}).get("drivetrain")
    if not drivetrain:
        return None
    if drivetrain.get("type") != "differential_drive":
        return None

    for key in ("left_wheel_joint", "right_wheel_joint", "wheel_separation", "wheel_radius"):
        if key not in drivetrain:
            raise ValueError(
                f"Robot.metadata['drivetrain'] is missing required key {key!r}. "
                "Expected: {'type': 'differential_drive', 'left_wheel_joint': ..., "
                "'right_wheel_joint': ..., 'wheel_separation': ..., 'wheel_radius': ...}"
            )

    for side in ("left_wheel_joint", "right_wheel_joint"):
        joint_name = drivetrain[side]
        joint = robot.joint(joint_name)
        if joint is None:
            raise ValueError(
                f"Robot.metadata['drivetrain'][{side!r}] names joint {joint_name!r}, "
                "which does not exist in robot.joints."
            )
        if joint.type not in _WHEEL_JOINT_TYPES:
            raise ValueError(
                f"Robot.metadata['drivetrain'][{side!r}] joint {joint_name!r} has type "
                f"{joint.type.value!r}; a drivetrain wheel joint must be "
                "JointType.CONTINUOUS or JointType.REVOLUTE."
            )

    return drivetrain


def _actuator_for_joint(robot: Robot, joint_name: str) -> Optional[Actuator]:
    for actuator in robot.actuators:
        if actuator.joint == joint_name:
            return actuator
    return None


def _wheel_joint_names(drivetrain: Optional[Dict[str, object]]) -> List[str]:
    if drivetrain is None:
        return []
    return [drivetrain["left_wheel_joint"], drivetrain["right_wheel_joint"]]


def _fmt_number(value: float) -> str:
    """Deterministic, non-scientific-notation float formatting for YAML,
    matching urdf.py's `_fmt_float` convention so numbers generated by this
    module and by urdf.py look the same across the generated package."""
    s = f"{float(value):.8f}"
    if "." in s:
        s = s.rstrip("0")
        if s.endswith("."):
            s += "0"
    return s


# --- ros2_control XML --------------------------------------------------


def generate_ros2_control_xml(robot: Robot) -> str:
    """Render a `<ros2_control name="<robot.name>" type="system">` XML
    fragment for `robot` (NOT a full `<robot>` document -- meant to be
    spliced into the existing URDF by a later integration step).

    Raises:
        robot_model.ValidationError: if `robot.validate()` finds problems.
            Propagated unmodified.
        ValueError: if a non-fixed, non-wheel joint has no matching
            `Actuator`, if an `Actuator.interface` isn't one of
            "position"/"velocity"/"effort", or if `Robot.metadata
            ["drivetrain"]` is malformed (missing keys, or a wheel joint
            that doesn't exist / isn't continuous or revolute).
    """
    robot.validate()  # raises ValidationError on problems; let it propagate

    drivetrain = _drivetrain_info(robot)
    wheel_joint_names = set(_wheel_joint_names(drivetrain))

    root = ET.Element("ros2_control", {"name": robot.name, "type": "system"})
    hardware = ET.SubElement(root, "hardware")
    ET.SubElement(hardware, "plugin").text = "mock_components/GenericSystem"

    for joint in robot.joints:
        if joint.type == JointType.FIXED:
            continue

        if joint.name in wheel_joint_names:
            joint_elem = ET.SubElement(root, "joint", {"name": joint.name})
            ET.SubElement(joint_elem, "command_interface", {"name": "velocity"})
            ET.SubElement(joint_elem, "state_interface", {"name": "position"})
            ET.SubElement(joint_elem, "state_interface", {"name": "velocity"})
            continue

        actuator = _actuator_for_joint(robot, joint.name)
        if actuator is None:
            raise ValueError(
                f"Joint {joint.name!r} (type={joint.type.value}) has no matching "
                "Actuator -- ros2_control needs an Actuator.interface "
                "(position|velocity|effort) to know which <command_interface> to "
                "emit for this joint, and this generator will not invent one. Add "
                "an Actuator with `joint=" + repr(joint.name) + "` before generating "
                "ros2_control XML."
            )
        if actuator.interface not in _VALID_INTERFACES:
            raise ValueError(
                f"Actuator {actuator.name!r} (joint={joint.name!r}) has interface "
                f"{actuator.interface!r}; ros2_control command interfaces must be one "
                f"of {_VALID_INTERFACES}."
            )

        joint_elem = ET.SubElement(root, "joint", {"name": joint.name})
        ET.SubElement(joint_elem, "command_interface", {"name": actuator.interface})
        for state_if in _VALID_INTERFACES:
            ET.SubElement(joint_elem, "state_interface", {"name": state_if})

    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


# --- controllers.yaml ----------------------------------------------------


def generate_controllers_yaml(robot: Robot) -> str:
    """Render `controller_manager`/controller YAML text for `robot`.

    Always includes `joint_state_broadcaster`. Adds a
    `joint_trajectory_controller` covering every non-fixed, non-wheel joint
    for an arm/manipulator robot, or a `diff_drive_controller` configured
    from `Robot.metadata["drivetrain"]` for a differential-drive robot --
    never both.

    Raises the same ValueError/ValidationError conditions as
    `generate_ros2_control_xml` (it performs the same joint/actuator/
    drivetrain validation, since the controller's `command_interfaces` must
    agree with what was emitted there).
    """
    robot.validate()

    drivetrain = _drivetrain_info(robot)
    wheel_joint_names = set(_wheel_joint_names(drivetrain))

    arm_joint_names: List[str] = []
    command_interfaces: List[str] = []  # de-duplicated, first-seen order
    seen_interfaces = set()

    for joint in robot.joints:
        if joint.type == JointType.FIXED or joint.name in wheel_joint_names:
            continue
        actuator = _actuator_for_joint(robot, joint.name)
        if actuator is None:
            raise ValueError(
                f"Joint {joint.name!r} (type={joint.type.value}) has no matching "
                "Actuator -- cannot determine its ros2_control command interface "
                "for the joint_trajectory_controller config."
            )
        if actuator.interface not in _VALID_INTERFACES:
            raise ValueError(
                f"Actuator {actuator.name!r} (joint={joint.name!r}) has interface "
                f"{actuator.interface!r}; ros2_control command interfaces must be one "
                f"of {_VALID_INTERFACES}."
            )
        arm_joint_names.append(joint.name)
        if actuator.interface not in seen_interfaces:
            seen_interfaces.add(actuator.interface)
            command_interfaces.append(actuator.interface)

    lines: List[str] = []
    lines.append("controller_manager:")
    lines.append("  ros__parameters:")
    lines.append("    update_rate: 100")
    lines.append("")
    lines.append("    joint_state_broadcaster:")
    lines.append(f"      type: {_JOINT_STATE_BROADCASTER_TYPE}")
    lines.append("")

    if drivetrain is not None:
        lines.append("    diff_drive_controller:")
        lines.append(f"      type: {_DIFF_DRIVE_CONTROLLER_TYPE}")
        lines.append("")
        lines.append("joint_state_broadcaster:")
        lines.append("  ros__parameters:")
        lines.append(f"    type: {_JOINT_STATE_BROADCASTER_TYPE}")
        lines.append("")
        lines.append("diff_drive_controller:")
        lines.append("  ros__parameters:")
        lines.append(f'    left_wheel_names: ["{drivetrain["left_wheel_joint"]}"]')
        lines.append(f'    right_wheel_names: ["{drivetrain["right_wheel_joint"]}"]')
        lines.append(f'    wheel_separation: {_fmt_number(drivetrain["wheel_separation"])}')
        lines.append(f'    wheel_radius: {_fmt_number(drivetrain["wheel_radius"])}')
    else:
        lines.append("    joint_trajectory_controller:")
        lines.append(f"      type: {_JOINT_TRAJECTORY_CONTROLLER_TYPE}")
        lines.append("")
        lines.append("joint_state_broadcaster:")
        lines.append("  ros__parameters:")
        lines.append(f"    type: {_JOINT_STATE_BROADCASTER_TYPE}")
        lines.append("")
        lines.append("joint_trajectory_controller:")
        lines.append("  ros__parameters:")
        lines.append("    joints:")
        for name in arm_joint_names:
            lines.append(f"      - {name}")
        lines.append("    command_interfaces:")
        for interface in command_interfaces:
            lines.append(f"      - {interface}")
        lines.append("    state_interfaces:")
        for interface in _VALID_INTERFACES:
            lines.append(f"      - {interface}")

    return "\n".join(lines) + "\n"


# --- control launch file -------------------------------------------------


def generate_control_launch(robot: Robot) -> str:
    """Render a `launch`/`launch_ros` Python launch file that starts
    `controller_manager`'s `ros2_control_node` (parameterized by
    `robot_description` plus this module's generated `controllers.yaml`)
    and spawns `joint_state_broadcaster` plus the arm/diff-drive controller
    via `ros2 run controller_manager spawner <name>` (expressed as launch
    `Node` actions, matching `ros2_control_demos`' launch file pattern).

    Assumes the surrounding package (built by a later integration step)
    follows the same `urdf/<robot.name>.urdf.xacro` layout as
    `fusion_addin/generators/package.py`, plus a `config/controllers.yaml`
    holding this module's `generate_controllers_yaml` output -- both are
    just the conventional ros2_control_demos-style package layout, not
    something this module writes to disk itself.

    Raises the same ValueError/ValidationError conditions as
    `generate_ros2_control_xml` (used here only to pick the second
    controller's name).
    """
    drivetrain = _drivetrain_info(robot)
    second_controller = "diff_drive_controller" if drivetrain is not None else "joint_trajectory_controller"

    return f'''"""Control launch file for the "{robot.name}" robot.

Auto-generated by Fusion2ROS (fusion_addin/generators/ros2_control.py).
Regenerate rather than editing by hand.

Starts controller_manager/ros2_control_node (parameterized by
robot_description + controllers.yaml) and spawns joint_state_broadcaster
plus {second_controller} via controller_manager's spawner executable.
"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "{robot.name}"
URDF_XACRO_FILE = "{robot.name}.urdf.xacro"
CONTROLLERS_YAML_FILE = "controllers.yaml"

SECOND_CONTROLLER = "{second_controller}"


def generate_launch_description():
    pkg_share = FindPackageShare(PACKAGE_NAME)

    urdf_xacro_path = PathJoinSubstitution([pkg_share, "urdf", URDF_XACRO_FILE])
    controllers_yaml_path = PathJoinSubstitution([pkg_share, "config", CONTROLLERS_YAML_FILE])

    # Processed via the `xacro` command-line filter at launch time rather
    # than the `xacro` Python module, so this launch file has no import-time
    # dependency on xacro being importable in the launching interpreter.
    robot_description = {{
        "robot_description": Command(["xacro ", urdf_xacro_path])
    }}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[robot_description, controllers_yaml_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", controllers_yaml_path],
        output="screen",
    )

    second_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[SECOND_CONTROLLER, "--param-file", controllers_yaml_path],
        output="screen",
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        second_controller_spawner,
    ])
'''
