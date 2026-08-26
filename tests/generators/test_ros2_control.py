"""Tests for fusion_addin.generators.ros2_control.

Must run with plain `python3 -m pytest` -- no Fusion, no live ROS needed.
`ros2_control`/`controller_manager` et al. are not installed in this
environment; see the bottom of this file for the (skip-if-unavailable) real
integration check that only activates once they are.
"""

import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.generators.ros2_control import (
    generate_control_launch,
    generate_controllers_yaml,
    generate_ros2_control_xml,
)
from robot_model import (
    Actuator,
    Geometry,
    Inertial,
    Joint,
    JointType,
    Link,
    Material,
    Pose,
    Robot,
    ValidationError,
)

try:
    import yaml as _yaml
except ImportError:  # pragma: no cover - yaml is a very common transitive dep
    _yaml = None


def _minimal_load_yaml(text: str) -> dict:
    """Extremely small fallback YAML-subset parser, used only if PyYAML is
    genuinely unavailable. Handles the flat 2-space-indented
    `key:`/`key: scalar`/`- item` shape this module actually emits -- not a
    general YAML parser."""
    root: dict = {}
    stack = [(-1, root)]
    lines = [ln for ln in text.splitlines() if ln.strip() != ""]
    for raw_line in lines:
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        content = raw_line.strip()
        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]
        if content.startswith("- "):
            item = content[2:].strip()
            if not isinstance(parent, list):
                raise ValueError("fallback YAML parser: list item under non-list parent")
            parent.append(_scalar(item))
            continue
        key, _, rest = content.partition(":")
        key = key.strip()
        rest = rest.strip()
        if rest == "":
            # Could be a dict or a list; peek ahead to decide.
            child: object = {}
            parent[key] = child
            stack.append((indent, child))
        else:
            parent[key] = _scalar(rest)
    return root


def _scalar(text: str):
    if text.startswith('"') and text.endswith('"'):
        return text[1:-1]
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        pass
    return text


def load_yaml(text: str) -> dict:
    if _yaml is not None:
        return _yaml.safe_load(text)
    return _minimal_load_yaml(text)


# --- fixtures --------------------------------------------------------------


def make_arm_robot() -> Robot:
    """Two actuated joints (deliberately different interfaces, to exercise
    per-joint command_interface selection and de-duplication in the
    controller config) plus a fixed-joint sensor mount to confirm fixed
    joints are excluded from ros2_control entirely."""
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    link1 = Link(name="link1", parent="base_link", inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001))
    link2 = Link(name="link2", parent="link1", inertial=Inertial(mass=0.3, ixx=0.001, iyy=0.001, izz=0.001))
    sensor_mount = Link(name="sensor_mount", parent="link2")

    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.57,
        upper_limit=1.57,
    )
    joint2 = Joint(
        name="joint2",
        type=JointType.PRISMATIC,
        parent="link1",
        child="link2",
        axis=(0.0, 0.0, 1.0),
        lower_limit=0.0,
        upper_limit=0.2,
    )
    fixed_joint = Joint(name="sensor_mount_joint", type=JointType.FIXED, parent="link2", child="sensor_mount")

    actuator1 = Actuator(name="joint1_motor", type="electric_motor", joint="joint1", interface="velocity")
    actuator2 = Actuator(name="joint2_motor", type="electric_motor", joint="joint2", interface="effort")

    robot = Robot(
        name="test_arm",
        links=[base, link1, link2, sensor_mount],
        joints=[joint1, joint2, fixed_joint],
        actuators=[actuator1, actuator2],
    )
    robot.validate()
    return robot


def make_diff_drive_robot() -> Robot:
    base = Link(name="base_link", inertial=Inertial(mass=5.0, ixx=0.05, iyy=0.05, izz=0.05))
    left_wheel = Link(name="left_wheel", parent="base_link", inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001))
    right_wheel = Link(name="right_wheel", parent="base_link", inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001))
    caster = Link(name="caster", parent="base_link")

    left_joint = Joint(
        name="left_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="left_wheel",
        origin=Pose(xyz=(0.0, 0.15, 0.0)),
        axis=(0.0, 1.0, 0.0),
    )
    right_joint = Joint(
        name="right_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="right_wheel",
        origin=Pose(xyz=(0.0, -0.15, 0.0)),
        axis=(0.0, 1.0, 0.0),
    )
    caster_joint = Joint(name="caster_joint", type=JointType.FIXED, parent="base_link", child="caster")

    robot = Robot(
        name="test_diffbot",
        links=[base, left_wheel, right_wheel, caster],
        joints=[left_joint, right_joint, caster_joint],
        metadata={
            "drivetrain": {
                "type": "differential_drive",
                "left_wheel_joint": "left_wheel_joint",
                "right_wheel_joint": "right_wheel_joint",
                "wheel_separation": 0.3,
                "wheel_radius": 0.05,
            }
        },
    )
    robot.validate()
    return robot


def make_mecanum_robot() -> Robot:
    """Four mecanum wheel joints (all CONTINUOUS). Given full visual/
    collision geometry (unlike the other fixtures above) so this same
    fixture doubles as the input for the real colcon-build/launch
    integration check at the bottom of this file, which needs a URDF
    `generate_urdf_xacro` can render completely -- see
    `examples/sample_rover.py` for the equivalent diff-drive fixture this
    mirrors (not reused directly: this module may only touch
    ros2_control.py and this test file)."""

    def _wheel_link(name: str) -> Link:
        return Link(
            name=name,
            parent="base_link",
            origin=Pose(rpy=(1.5707963, 0.0, 0.0)),
            visual_geometry=Geometry(kind="cylinder", radius=0.08, length=0.04),
            collision_geometry=Geometry(kind="cylinder", radius=0.08, length=0.04),
            material=Material(name="wheel_black", rgba=(0.1, 0.1, 0.1, 1.0)),
            inertial=Inertial(mass=0.4, ixx=0.0006, iyy=0.0006, izz=0.001),
        )

    def _wheel_joint(name: str, child: str, x: float, y: float) -> Joint:
        return Joint(
            name=name,
            type=JointType.CONTINUOUS,
            parent="base_link",
            child=child,
            origin=Pose(xyz=(x, y, -0.05)),
            axis=(0.0, 1.0, 0.0),
        )

    base = Link(
        name="base_link",
        origin=Pose.IDENTITY,
        visual_geometry=Geometry(kind="box", size=(0.5, 0.4, 0.15)),
        collision_geometry=Geometry(kind="box", size=(0.5, 0.4, 0.15)),
        material=Material(name="chassis_blue", rgba=(0.2, 0.2, 0.8, 1.0)),
        inertial=Inertial(mass=8.0, ixx=0.1, iyy=0.12, izz=0.15),
    )
    front_left = _wheel_link("front_left_wheel")
    front_right = _wheel_link("front_right_wheel")
    back_left = _wheel_link("back_left_wheel")
    back_right = _wheel_link("back_right_wheel")

    fl_joint = _wheel_joint("front_left_wheel_joint", "front_left_wheel", 0.2, 0.15)
    fr_joint = _wheel_joint("front_right_wheel_joint", "front_right_wheel", 0.2, -0.15)
    bl_joint = _wheel_joint("back_left_wheel_joint", "back_left_wheel", -0.2, 0.15)
    br_joint = _wheel_joint("back_right_wheel_joint", "back_right_wheel", -0.2, -0.15)

    robot = Robot(
        name="test_mecanum",
        links=[base, front_left, front_right, back_left, back_right],
        joints=[fl_joint, fr_joint, bl_joint, br_joint],
        metadata={
            "drivetrain": {
                "type": "mecanum_drive",
                "front_left_wheel_joint": "front_left_wheel_joint",
                "front_right_wheel_joint": "front_right_wheel_joint",
                "back_left_wheel_joint": "back_left_wheel_joint",
                "back_right_wheel_joint": "back_right_wheel_joint",
                "wheel_radius": 0.08,
                # lx + ly: half wheelbase (0.2) + half track width (0.15).
                "sum_of_robot_center_projection_on_x_y_axis": 0.35,
            }
        },
    )
    robot.validate()
    return robot


# --- generate_ros2_control_xml: arm -----------------------------------


def test_arm_xml_parses_and_has_hardware_plugin():
    robot = make_arm_robot()
    xml_text = generate_ros2_control_xml(robot)
    root = ET.fromstring(xml_text)
    assert root.tag == "ros2_control"
    assert root.attrib["name"] == "test_arm"
    assert root.attrib["type"] == "system"
    plugin = root.find("hardware/plugin")
    assert plugin is not None
    assert plugin.text == "mock_components/GenericSystem"


def test_arm_xml_excludes_fixed_joint_and_has_two_actuated_joints():
    robot = make_arm_robot()
    root = ET.fromstring(generate_ros2_control_xml(robot))
    joints = root.findall("joint")
    assert {j.attrib["name"] for j in joints} == {"joint1", "joint2"}


def test_arm_xml_command_interface_matches_actuator():
    robot = make_arm_robot()
    root = ET.fromstring(generate_ros2_control_xml(robot))
    joints_by_name = {j.attrib["name"]: j for j in root.findall("joint")}

    joint1 = joints_by_name["joint1"]
    cmd1 = joint1.find("command_interface")
    assert cmd1.attrib["name"] == "velocity"
    state_names1 = {s.attrib["name"] for s in joint1.findall("state_interface")}
    assert state_names1 == {"position", "velocity", "effort"}

    joint2 = joints_by_name["joint2"]
    cmd2 = joint2.find("command_interface")
    assert cmd2.attrib["name"] == "effort"
    state_names2 = {s.attrib["name"] for s in joint2.findall("state_interface")}
    assert state_names2 == {"position", "velocity", "effort"}


def test_arm_joint_missing_actuator_raises_value_error():
    robot = make_arm_robot()
    robot.actuators = [a for a in robot.actuators if a.joint != "joint2"]
    with pytest.raises(ValueError, match="joint2"):
        generate_ros2_control_xml(robot)


def test_arm_actuator_invalid_interface_raises_value_error():
    robot = make_arm_robot()
    robot.actuators[0].interface = "torque_boost"
    with pytest.raises(ValueError, match="torque_boost"):
        generate_ros2_control_xml(robot)


def test_invalid_robot_raises_validation_error():
    base = Link(name="base_link")
    orphan = Link(name="orphan", parent="ghost")
    robot = Robot(name="broken", links=[base, orphan], joints=[])
    with pytest.raises(ValidationError):
        generate_ros2_control_xml(robot)


# --- generate_ros2_control_xml: differential drive ----------------------


def test_diff_drive_xml_wheel_joints_get_velocity_only():
    robot = make_diff_drive_robot()
    root = ET.fromstring(generate_ros2_control_xml(robot))
    joints = root.findall("joint")
    assert {j.attrib["name"] for j in joints} == {"left_wheel_joint", "right_wheel_joint"}

    for joint in joints:
        cmd_ifaces = joint.findall("command_interface")
        assert len(cmd_ifaces) == 1
        assert cmd_ifaces[0].attrib["name"] == "velocity"
        state_names = {s.attrib["name"] for s in joint.findall("state_interface")}
        assert state_names == {"position", "velocity"}
        assert "effort" not in state_names


def test_diff_drive_wheel_joints_need_no_actuator():
    # Drivetrain wheels get velocity interfaces purely from the metadata
    # convention -- no Actuator object required.
    robot = make_diff_drive_robot()
    assert robot.actuators == []
    generate_ros2_control_xml(robot)  # must not raise


def test_diff_drive_wrong_wheel_joint_type_raises():
    robot = make_diff_drive_robot()
    # Repoint the drivetrain at the fixed caster joint -- not a valid wheel.
    robot.metadata["drivetrain"]["left_wheel_joint"] = "caster_joint"
    with pytest.raises(ValueError, match="caster_joint"):
        generate_ros2_control_xml(robot)


def test_diff_drive_unknown_wheel_joint_raises():
    robot = make_diff_drive_robot()
    robot.metadata["drivetrain"]["left_wheel_joint"] = "no_such_joint"
    with pytest.raises(ValueError, match="no_such_joint"):
        generate_ros2_control_xml(robot)


def test_diff_drive_missing_metadata_key_raises():
    robot = make_diff_drive_robot()
    del robot.metadata["drivetrain"]["wheel_radius"]
    with pytest.raises(ValueError, match="wheel_radius"):
        generate_ros2_control_xml(robot)


def test_metadata_without_drivetrain_key_is_treated_as_arm():
    robot = make_arm_robot()
    assert "drivetrain" not in robot.metadata
    root = ET.fromstring(generate_ros2_control_xml(robot))
    assert {j.attrib["name"] for j in root.findall("joint")} == {"joint1", "joint2"}


# --- generate_ros2_control_xml: mecanum drive ---------------------------


def test_mecanum_xml_wheel_joints_get_velocity_only():
    robot = make_mecanum_robot()
    root = ET.fromstring(generate_ros2_control_xml(robot))
    joints = root.findall("joint")
    assert {j.attrib["name"] for j in joints} == {
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "back_left_wheel_joint",
        "back_right_wheel_joint",
    }

    for joint in joints:
        cmd_ifaces = joint.findall("command_interface")
        assert len(cmd_ifaces) == 1
        assert cmd_ifaces[0].attrib["name"] == "velocity"
        state_names = {s.attrib["name"] for s in joint.findall("state_interface")}
        assert state_names == {"position", "velocity"}
        assert "effort" not in state_names


def test_mecanum_wheel_joints_need_no_actuator():
    robot = make_mecanum_robot()
    assert robot.actuators == []
    generate_ros2_control_xml(robot)  # must not raise


def test_mecanum_wrong_wheel_joint_type_raises():
    robot = make_mecanum_robot()
    # Repoint a wheel joint at base_link's own (nonexistent) fixed joint by
    # instead flipping one wheel joint's type to FIXED directly -- not a
    # valid wheel type for a drivetrain.
    robot.joints[0].type = JointType.FIXED
    with pytest.raises(ValueError, match="front_left_wheel_joint"):
        generate_ros2_control_xml(robot)


def test_mecanum_unknown_wheel_joint_raises():
    robot = make_mecanum_robot()
    robot.metadata["drivetrain"]["front_left_wheel_joint"] = "no_such_joint"
    with pytest.raises(ValueError, match="no_such_joint"):
        generate_ros2_control_xml(robot)


def test_mecanum_missing_metadata_key_raises():
    robot = make_mecanum_robot()
    del robot.metadata["drivetrain"]["sum_of_robot_center_projection_on_x_y_axis"]
    with pytest.raises(ValueError, match="sum_of_robot_center_projection_on_x_y_axis"):
        generate_ros2_control_xml(robot)


def test_mecanum_missing_wheel_radius_raises():
    robot = make_mecanum_robot()
    del robot.metadata["drivetrain"]["wheel_radius"]
    with pytest.raises(ValueError, match="wheel_radius"):
        generate_ros2_control_xml(robot)


# --- unrecognized drivetrain type: regression test for the silent- --------
# --- fallthrough bug (an unknown "type" used to be treated as "no ---------
# --- drivetrain at all" instead of raising) --------------------------------


def test_unrecognized_drivetrain_type_raises_value_error_naming_it():
    robot = make_diff_drive_robot()
    robot.metadata["drivetrain"]["type"] = "omni_drive"
    with pytest.raises(ValueError, match="omni_drive"):
        generate_ros2_control_xml(robot)


def test_unrecognized_drivetrain_type_raises_for_controllers_yaml_too():
    robot = make_diff_drive_robot()
    robot.metadata["drivetrain"]["type"] = "omni_drive"
    with pytest.raises(ValueError, match="omni_drive"):
        generate_controllers_yaml(robot)


def test_unrecognized_drivetrain_type_raises_for_control_launch_too():
    robot = make_diff_drive_robot()
    robot.metadata["drivetrain"]["type"] = "omni_drive"
    with pytest.raises(ValueError, match="omni_drive"):
        generate_control_launch(robot)


def test_typo_drivetrain_type_raises_value_error_naming_it():
    # A plausible real-world typo, not just an arbitrary made-up type.
    robot = make_mecanum_robot()
    robot.metadata["drivetrain"]["type"] = "mecanum_drve"
    with pytest.raises(ValueError, match="mecanum_drve"):
        generate_ros2_control_xml(robot)


# --- generate_controllers_yaml: arm --------------------------------------


def test_arm_controllers_yaml_parses_and_has_expected_shape():
    robot = make_arm_robot()
    text = generate_controllers_yaml(robot)
    data = load_yaml(text)

    cm_params = data["controller_manager"]["ros__parameters"]
    assert cm_params["update_rate"] == 100
    assert cm_params["joint_state_broadcaster"]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
    assert cm_params["joint_trajectory_controller"]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController"
    )
    assert "diff_drive_controller" not in cm_params

    jtc_params = data["joint_trajectory_controller"]["ros__parameters"]
    assert jtc_params["joints"] == ["joint1", "joint2"]
    assert set(jtc_params["command_interfaces"]) == {"velocity", "effort"}
    # NOT "effort" -- joint_trajectory_controller's own state_interfaces param
    # only accepts {position, velocity, acceleration}; confirmed for real
    # against a live controller_manager (see ros2_control.py's
    # _JTC_STATE_INTERFACES comment for the exact rejection error).
    assert set(jtc_params["state_interfaces"]) == {"position", "velocity"}

    assert "diff_drive_controller" not in data


def test_arm_controllers_yaml_missing_actuator_raises():
    robot = make_arm_robot()
    robot.actuators = []
    with pytest.raises(ValueError, match="joint1"):
        generate_controllers_yaml(robot)


# --- generate_controllers_yaml: differential drive -----------------------


def test_diff_drive_controllers_yaml_parses_and_has_expected_shape():
    robot = make_diff_drive_robot()
    text = generate_controllers_yaml(robot)
    data = load_yaml(text)

    cm_params = data["controller_manager"]["ros__parameters"]
    assert cm_params["update_rate"] == 100
    assert cm_params["diff_drive_controller"]["type"] == "diff_drive_controller/DiffDriveController"
    assert "joint_trajectory_controller" not in cm_params

    ddc_params = data["diff_drive_controller"]["ros__parameters"]
    assert ddc_params["left_wheel_names"] == ["left_wheel_joint"]
    assert ddc_params["right_wheel_names"] == ["right_wheel_joint"]
    assert float(ddc_params["wheel_separation"]) == pytest.approx(0.3)
    assert float(ddc_params["wheel_radius"]) == pytest.approx(0.05)

    assert "joint_trajectory_controller" not in data


# --- generate_controllers_yaml: mecanum drive -----------------------------


def test_mecanum_controllers_yaml_parses_and_has_expected_shape():
    robot = make_mecanum_robot()
    text = generate_controllers_yaml(robot)
    data = load_yaml(text)

    cm_params = data["controller_manager"]["ros__parameters"]
    assert cm_params["update_rate"] == 100
    assert cm_params["mecanum_drive_controller"]["type"] == (
        "mecanum_drive_controller/MecanumDriveController"
    )
    assert "diff_drive_controller" not in cm_params
    assert "joint_trajectory_controller" not in cm_params

    mdc_params = data["mecanum_drive_controller"]["ros__parameters"]
    # Real mecanum_drive_controller parameter names (confirmed against this
    # machine's installed mecanum_drive_controller_parameters.hpp -- see
    # ros2_control.py's module docstring). Note "rear_*", not "back_*",
    # matching the controller's own declared parameter names even though our
    # metadata convention uses "back_*".
    assert mdc_params["front_left_wheel_command_joint_name"] == "front_left_wheel_joint"
    assert mdc_params["front_right_wheel_command_joint_name"] == "front_right_wheel_joint"
    assert mdc_params["rear_left_wheel_command_joint_name"] == "back_left_wheel_joint"
    assert mdc_params["rear_right_wheel_command_joint_name"] == "back_right_wheel_joint"

    kinematics = mdc_params["kinematics"]
    assert float(kinematics["wheels_radius"]) == pytest.approx(0.08)
    assert float(kinematics["sum_of_robot_center_projection_on_X_Y_axis"]) == pytest.approx(0.35)

    assert "diff_drive_controller" not in data
    assert "joint_trajectory_controller" not in data


# --- determinism -----------------------------------------------------------


def test_xml_generation_is_deterministic():
    robot = make_arm_robot()
    assert generate_ros2_control_xml(robot) == generate_ros2_control_xml(robot)
    diff_robot = make_diff_drive_robot()
    assert generate_ros2_control_xml(diff_robot) == generate_ros2_control_xml(diff_robot)
    mecanum_robot = make_mecanum_robot()
    assert generate_ros2_control_xml(mecanum_robot) == generate_ros2_control_xml(mecanum_robot)


def test_yaml_generation_is_deterministic():
    robot = make_arm_robot()
    assert generate_controllers_yaml(robot) == generate_controllers_yaml(robot)
    diff_robot = make_diff_drive_robot()
    assert generate_controllers_yaml(diff_robot) == generate_controllers_yaml(diff_robot)
    mecanum_robot = make_mecanum_robot()
    assert generate_controllers_yaml(mecanum_robot) == generate_controllers_yaml(mecanum_robot)


# --- generate_control_launch ---------------------------------------------


def test_arm_launch_file_is_syntactically_valid_python():
    robot = make_arm_robot()
    text = generate_control_launch(robot)
    compile(text, "<generated arm launch>", "exec")
    assert "joint_trajectory_controller" in text
    assert "ros2_control_node" in text
    assert "def generate_launch_description" in text
    # Regression: this ros2_control version requires each spawner to receive
    # --param-file directly (passing --params-file only to ros2_control_node
    # at startup is NOT enough -- confirmed for real against a live
    # controller_manager: joint_trajectory_controller failed to initialize
    # with "Length of parameter 'joints' is '0'" without this).
    assert text.count("--param-file") == 2
    # Regression: ros2_control_node subscribes to /robot_description as a
    # topic and hangs forever without something publishing it -- confirmed
    # for real (see generate_control_launch's docstring for the exact log
    # messages this produced before robot_state_publisher was added here).
    assert "robot_state_publisher" in text


def test_diff_drive_launch_file_is_syntactically_valid_python():
    robot = make_diff_drive_robot()
    text = generate_control_launch(robot)
    compile(text, "<generated diffdrive launch>", "exec")
    assert "diff_drive_controller" in text
    assert "ros2_control_node" in text


def test_mecanum_launch_file_is_syntactically_valid_python():
    robot = make_mecanum_robot()
    text = generate_control_launch(robot)
    compile(text, "<generated mecanum launch>", "exec")
    assert "mecanum_drive_controller" in text
    assert "diff_drive_controller" not in text
    assert "joint_trajectory_controller" not in text
    assert "ros2_control_node" in text
    assert text.count("--param-file") == 2
    assert "robot_state_publisher" in text


# --- real ros2_control integration (only runs if it's actually installed) --

_CONTROLLER_MANAGER_INSTALLED = shutil.which("ros2") is not None and (
    subprocess.run(
        ["ros2", "pkg", "prefix", "controller_manager"],
        capture_output=True,
        timeout=30,
    ).returncode
    == 0
)


@pytest.mark.skipif(
    not _CONTROLLER_MANAGER_INSTALLED,
    reason="controller_manager (ros2_control) is not installed on this machine",
)
def test_colcon_build_and_optional_launch_smoke(tmp_path):
    """Real integration check, gated on ros2_control actually being
    installed (it is not, as of this module's authoring session -- `ros2 pkg
    list` shows no ros2_control/controller_manager/diff_drive_controller/
    joint_state_broadcaster packages here). If it ever is:

    1. Build a tiny package with our generated ros2_control.xml spliced into
       its URDF (inline string splice here -- there is no integration step
       to depend on yet) via fusion_addin.generators.package.generate_package.
    2. Drop our generated controllers.yaml and control launch file into that
       package (under config/ and launch/ respectively -- generate_package
       itself doesn't know about either, that splice is this test's job).
    3. Copy the package into a throwaway colcon workspace under tmp_path
       (never ~/ros2_ws) and `colcon build` it.
    4. If `ros2_control_node`/`spawner` binaries are on PATH, try a
       timeout-bounded `ros2 launch` smoke test; skip that step gracefully
       otherwise.
    """
    import sys as _sys

    _sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from examples.sample_arm import build_sample_arm
    from fusion_addin.generators.package import generate_package
    from fusion_addin.generators.urdf import generate_urdf_xacro

    if shutil.which("colcon") is None:
        pytest.skip("colcon not found on PATH")

    robot = build_sample_arm()
    urdf_text = generate_urdf_xacro(robot)
    ros2_control_xml = generate_ros2_control_xml(robot)
    controllers_yaml = generate_controllers_yaml(robot)
    control_launch = generate_control_launch(robot)

    # Inline splice: insert the <ros2_control> fragment just before </robot>.
    assert urdf_text.rstrip().endswith("</robot>")
    spliced_urdf = urdf_text.rstrip()[: -len("</robot>")] + ros2_control_xml + "</robot>\n"

    gen_dir = tmp_path / "generated"
    pkg_dir = generate_package(robot, spliced_urdf, {}, gen_dir)
    (pkg_dir / "config").mkdir(exist_ok=True)
    (pkg_dir / "config" / "controllers.yaml").write_text(controllers_yaml, encoding="utf-8")
    (pkg_dir / "launch" / "control.launch.py").write_text(control_launch, encoding="utf-8")

    ws_src = tmp_path / "ws" / "src"
    ws_src.mkdir(parents=True)
    shutil.copytree(pkg_dir, ws_src / robot.name)

    build_result = subprocess.run(
        ["colcon", "build", "--packages-select", robot.name],
        cwd=tmp_path / "ws",
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert build_result.returncode == 0, (
        f"colcon build failed.\nstdout:\n{build_result.stdout}\nstderr:\n{build_result.stderr}"
    )

    ros2_control_node = shutil.which("ros2_control_node")
    spawner = shutil.which("spawner")
    if ros2_control_node is None or spawner is None:
        pytest.skip("ros2_control_node/spawner executables not found on PATH; build-only check passed")

    setup_bash = tmp_path / "ws" / "install" / "setup.bash"
    if not setup_bash.exists():
        pytest.skip("colcon install/setup.bash not produced; build-only check passed")

    launch_cmd = (
        f"source {setup_bash} && "
        f"timeout 10 ros2 launch {robot.name} control.launch.py"
    )
    launch_result = subprocess.run(
        ["bash", "-c", launch_cmd],
        capture_output=True,
        text=True,
        timeout=30,
    )
    # `timeout 10` sends SIGTERM after 10s to a long-running launch that
    # started successfully -- exit code 124 (timed out while still running)
    # or 0/-15 all indicate it came up and ran rather than failing outright.
    assert launch_result.returncode in (0, 124, -15), (
        f"ros2 launch smoke test failed unexpectedly.\n"
        f"stdout:\n{launch_result.stdout}\nstderr:\n{launch_result.stderr}"
    )


@pytest.mark.skipif(
    not _CONTROLLER_MANAGER_INSTALLED,
    reason="controller_manager (ros2_control) is not installed on this machine",
)
def test_mecanum_colcon_build_and_optional_launch_smoke(tmp_path):
    """Same real-integration shape as test_colcon_build_and_optional_launch_smoke
    above, but for a hand-built 4-wheel mecanum Robot (make_mecanum_robot())
    instead of examples.sample_arm -- exercises the mecanum_drive_controller
    path end-to-end: colcon build, then (if ros2_control_node/spawner are on
    PATH) a real `ros2 launch` plus polling `ros2 control list_controllers`
    to confirm mecanum_drive_controller actually reaches the `active` state,
    not just that the launch process didn't crash outright.
    """
    import sys as _sys

    _sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from fusion_addin.generators.package import generate_package
    from fusion_addin.generators.urdf import generate_urdf_xacro

    if shutil.which("colcon") is None:
        pytest.skip("colcon not found on PATH")

    robot = make_mecanum_robot()
    urdf_text = generate_urdf_xacro(robot)
    ros2_control_xml = generate_ros2_control_xml(robot)
    controllers_yaml = generate_controllers_yaml(robot)
    control_launch = generate_control_launch(robot)

    assert urdf_text.rstrip().endswith("</robot>")
    spliced_urdf = urdf_text.rstrip()[: -len("</robot>")] + ros2_control_xml + "</robot>\n"

    gen_dir = tmp_path / "generated"
    pkg_dir = generate_package(robot, spliced_urdf, {}, gen_dir)
    (pkg_dir / "config").mkdir(exist_ok=True)
    (pkg_dir / "config" / "controllers.yaml").write_text(controllers_yaml, encoding="utf-8")
    (pkg_dir / "launch" / "control.launch.py").write_text(control_launch, encoding="utf-8")

    ws_src = tmp_path / "ws" / "src"
    ws_src.mkdir(parents=True)
    shutil.copytree(pkg_dir, ws_src / robot.name)

    build_result = subprocess.run(
        ["colcon", "build", "--packages-select", robot.name],
        cwd=tmp_path / "ws",
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert build_result.returncode == 0, (
        f"colcon build failed.\nstdout:\n{build_result.stdout}\nstderr:\n{build_result.stderr}"
    )

    ros2_control_node = shutil.which("ros2_control_node")
    spawner = shutil.which("spawner")
    if ros2_control_node is None or spawner is None:
        pytest.skip("ros2_control_node/spawner executables not found on PATH; build-only check passed")

    setup_bash = tmp_path / "ws" / "install" / "setup.bash"
    if not setup_bash.exists():
        pytest.skip("colcon install/setup.bash not produced; build-only check passed")

    launch_proc = subprocess.Popen(
        ["bash", "-c", f"source {setup_bash} && exec ros2 launch {robot.name} control.launch.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        list_result = None
        # Poll for controllers to reach 'active' rather than a fixed sleep --
        # ros2_control_node/spawners take a variable amount of time to come
        # up (xacro processing + robot_state_publisher + spawner sequencing).
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            time.sleep(2)
            list_result = subprocess.run(
                ["bash", "-c", f"source {setup_bash} && ros2 control list_controllers"],
                capture_output=True,
                text=True,
                timeout=15,
            )
            if "mecanum_drive_controller" in list_result.stdout and "active" in list_result.stdout:
                break

        assert list_result is not None
        mdc_lines = [
            line for line in list_result.stdout.splitlines() if "mecanum_drive_controller" in line
        ]
        assert mdc_lines, (
            "mecanum_drive_controller never appeared in `ros2 control "
            f"list_controllers` output.\nstdout:\n{list_result.stdout}\n"
            f"stderr:\n{list_result.stderr}"
        )
        assert "active" in mdc_lines[0], (
            f"mecanum_drive_controller did not reach 'active' state.\n"
            f"list_controllers line: {mdc_lines[0]}\n"
            f"full list_controllers stdout:\n{list_result.stdout}"
        )
    finally:
        launch_proc.terminate()
        try:
            launch_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            launch_proc.kill()
            launch_proc.wait(timeout=10)
