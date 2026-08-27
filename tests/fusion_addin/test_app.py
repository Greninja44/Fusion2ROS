import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.app import (
    GenerationCancelled,
    PipelineError,
    _sanitize_ros_package_name,
    attach_collision_proxies,
    attach_mesh_references,
    build_robot_from_reader,
    check_missing_actuator_limits,
    format_robot_summary,
    generate_ros_package,
    robot_summary_as_dict,
    run_pipeline,
)
from robot_model import Actuator, Geometry, Inertial, Joint, JointType, Link, Robot

from tests.fusion_addin.test_extraction import FakeFusionDesignReader
from fusion_addin.extraction.interface import FusionInertia, FusionJointInfo, FusionOccurrence, FusionPose


def make_simple_robot(with_limits=True):
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    arm = Link(name="arm", parent="base_link", inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001))
    joint = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="arm",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        velocity_limit=2.0 if with_limits else None,
        effort_limit=5.0 if with_limits else None,
    )
    return Robot(name="app_test_robot", links=[base, arm], joints=[joint])


def test_check_missing_actuator_limits_clean_robot():
    robot = make_simple_robot(with_limits=True)
    assert check_missing_actuator_limits(robot) == []


def test_check_missing_actuator_limits_reports_joint():
    robot = make_simple_robot(with_limits=False)
    problems = check_missing_actuator_limits(robot)
    assert len(problems) == 1
    assert "joint1" in problems[0]
    assert "velocity_limit" in problems[0]
    assert "effort_limit" in problems[0]


def test_format_robot_summary_lists_links_and_joints():
    robot = make_simple_robot(with_limits=True)
    summary = format_robot_summary(robot)

    assert "Links (2):" in summary
    assert "- base_link" in summary
    assert "(root)" in summary
    assert "- arm" in summary
    assert "(parent: base_link)" in summary

    assert "Joints (1):" in summary
    assert "- joint1" in summary
    assert "[revolute]" in summary
    assert "base_link -> arm" in summary


def test_format_robot_summary_no_joints():
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    robot = Robot(name="lone_link_robot", links=[base], joints=[])

    summary = format_robot_summary(robot)

    assert "Links (1):" in summary
    assert "Joints (0):" in summary
    assert "(none)" in summary


def test_robot_summary_as_dict_lists_links_and_joints():
    robot = make_simple_robot(with_limits=True)
    summary = robot_summary_as_dict(robot)

    assert summary == {
        "links": [
            {"name": "base_link", "parent": None, "is_root": True},
            {"name": "arm", "parent": "base_link", "is_root": False},
        ],
        "joints": [
            {"name": "joint1", "type": "revolute", "parent": "base_link", "child": "arm"},
        ],
    }


def test_robot_summary_as_dict_no_joints():
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    robot = Robot(name="lone_link_robot", links=[base], joints=[])

    summary = robot_summary_as_dict(robot)

    assert summary == {
        "links": [{"name": "base_link", "parent": None, "is_root": True}],
        "joints": [],
    }


def test_robot_summary_as_dict_matches_format_robot_summary_content():
    # Not the same shape, but the same underlying facts -- every link/joint
    # name mentioned in the plain-text summary must also appear in the
    # structured one, and vice versa.
    robot = make_simple_robot(with_limits=True)
    text_summary = format_robot_summary(robot)
    dict_summary = robot_summary_as_dict(robot)

    for link in dict_summary["links"]:
        assert link["name"] in text_summary
    for joint in dict_summary["joints"]:
        assert joint["name"] in text_summary
        assert joint["type"] in text_summary


def test_attach_mesh_references(tmp_path):
    robot = make_simple_robot()
    fake_mesh = tmp_path / "base_link.stl"
    fake_mesh.write_bytes(b"fake stl")
    attach_mesh_references(robot, {"base_link": fake_mesh})

    base = robot.link("base_link")
    assert base.visual_geometry.kind == "mesh"
    assert base.visual_geometry.mesh_path == "package://app_test_robot/meshes/base_link.stl"
    assert base.collision_geometry.mesh_path == base.visual_geometry.mesh_path

    arm = robot.link("arm")
    assert arm.visual_geometry is None  # not in mesh_files -> left alone


def test_generate_ros_package_raises_pipeline_error_for_missing_limits(tmp_path):
    robot = make_simple_robot(with_limits=False)
    with pytest.raises(PipelineError, match="joint1"):
        generate_ros_package(robot, {}, tmp_path)


def test_generate_ros_package_succeeds(tmp_path):
    robot = make_simple_robot(with_limits=True)
    package_dir = generate_ros_package(robot, {}, tmp_path)
    assert package_dir == tmp_path / "app_test_robot"
    assert (package_dir / "package.xml").exists()
    assert (package_dir / "urdf" / "app_test_robot.urdf.xacro").exists()


def test_generate_ros_package_with_meshes_lands_under_meshes_dir(tmp_path):
    # Exercises the mesh_files re-keying boundary in generate_ros_package:
    # export_link_meshes-shaped input (keyed by LINK NAME) must end up
    # correctly placed under the package's meshes/ dir (keyed by FILENAME),
    # and the URDF's mesh reference must point at that same filename.
    robot = make_simple_robot(with_limits=True)
    fake_mesh = tmp_path / "exported" / "base_link.stl"
    fake_mesh.parent.mkdir()
    fake_mesh.write_bytes(b"fake stl content")

    package_dir = generate_ros_package(robot, {"base_link": fake_mesh}, tmp_path)

    copied = package_dir / "meshes" / "base_link.stl"
    assert copied.exists()
    assert copied.read_bytes() == b"fake stl content"

    urdf_text = (package_dir / "urdf" / "app_test_robot.urdf.xacro").read_text()
    assert "package://app_test_robot/meshes/base_link.stl" in urdf_text


def test_run_pipeline_end_to_end(tmp_path):
    base = FusionOccurrence(
        name="base_link:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=100, iyy=100, izz=100, ixy=0, ixz=0, iyz=0),
    )
    arm = FusionOccurrence(
        name="arm:1",
        pose=FusionPose(xyz=(5.0, 0.0, 0.0)),
        inertia=FusionInertia(mass=1.0, center_of_mass=(5, 0, 0), ixx=100, iyy=135, izz=135, ixy=0, ixz=0, iyz=0),
    )
    joint = FusionJointInfo(
        name="joint1",
        joint_type="RevoluteJointType",
        occurrence_one="base_link:1",
        occurrence_two="arm:1",
        origin=FusionPose(xyz=(5.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
    )
    reader = FakeFusionDesignReader([base, arm], [joint])

    # Extraction alone (no motor limits from Fusion) should raise a clear
    # PipelineError, not a bare ValueError -- this is the real-world shape
    # every Fusion-sourced robot hits until actuators are assigned.
    with pytest.raises(PipelineError):
        run_pipeline(reader, "e2e_robot", tmp_path)

    # Once a caller sets the limits an extractor can't provide, the full
    # pipeline runs through to a real package on disk.
    robot, package_dir = None, None
    from fusion_addin.extraction.converter import build_robot_model

    robot = build_robot_model(reader, "e2e_robot")
    robot.joint("joint1").velocity_limit = 2.0
    robot.joint("joint1").effort_limit = 10.0
    package_dir = generate_ros_package(robot, {}, tmp_path)
    assert (package_dir / "urdf" / "e2e_robot.urdf.xacro").exists()


# ---------------------------------------------------------------------------
# _sanitize_ros_package_name / build_robot_from_reader -- real bug found
# live: Fusion's default root component name is literally "Main Assembly",
# which the "Robot name" UI field defaults to verbatim (command.py), and
# every generator (package.py, ros2_control.py, gazebo.py, nav2.py) uses
# robot.name as-is for the package directory, package.xml <name>, and
# CMakeLists.txt project(). colcon's real package-name validation
# (catkin_pkg.package.Package.validate: requires
# `^[a-zA-Z0-9][a-zA-Z0-9_-]*$`) hard-rejects a name containing a space, so
# colcon silently drops the whole package ("ignoring unknown package") and
# 0 packages get built -- confirmed live with a real generated package
# named "Main Assembly".
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "raw_name, expected",
    [
        ("Main Assembly", "main_assembly"),
        ("already_valid", "already_valid"),
        ("R6A2-Arm!!", "r6a2_arm"),
        ("  leading and trailing  ", "leading_and_trailing"),
        ("123_starts_with_digit", "robot_123_starts_with_digit"),
        ("", "robot"),
        ("!!!", "robot"),
    ],
)
def test_sanitize_ros_package_name(raw_name, expected):
    assert _sanitize_ros_package_name(raw_name) == expected


def test_build_robot_from_reader_sanitizes_robot_name():
    base = FusionOccurrence(
        name="base_link:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=100, iyy=100, izz=100, ixy=0, ixz=0, iyz=0),
    )
    reader = FakeFusionDesignReader([base], [])

    robot = build_robot_from_reader(reader, "Main Assembly")

    assert robot.name == "main_assembly"


@pytest.mark.skipif(
    shutil.which("colcon") is None or not Path("/opt/ros/lyrical/setup.bash").is_file(),
    reason="colcon and/or /opt/ros/lyrical/setup.bash not available in this environment",
)
def test_run_pipeline_with_fusion_default_name_builds_with_colcon(tmp_path):
    """End-to-end reproduction of the real bug: run_pipeline with Fusion's
    actual default robot name ("Main Assembly") must still produce a
    package colcon can build, not one it silently ignores."""
    base = FusionOccurrence(
        name="base_link:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=100, iyy=100, izz=100, ixy=0, ixz=0, iyz=0),
    )
    arm = FusionOccurrence(
        name="arm:1",
        pose=FusionPose(xyz=(5.0, 0.0, 0.0)),
        inertia=FusionInertia(mass=1.0, center_of_mass=(5, 0, 0), ixx=100, iyy=135, izz=135, ixy=0, ixz=0, iyz=0),
    )
    joint = FusionJointInfo(
        name="joint1",
        joint_type="RevoluteJointType",
        occurrence_one="base_link:1",
        occurrence_two="arm:1",
        origin=FusionPose(xyz=(5.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
    )
    reader = FakeFusionDesignReader([base, arm], [joint])

    from fusion_addin.extraction.converter import build_robot_model

    robot = build_robot_model(reader, _sanitize_ros_package_name("Main Assembly"))
    robot.joint("joint1").velocity_limit = 2.0
    robot.joint("joint1").effort_limit = 10.0
    package_dir = generate_ros_package(robot, {}, tmp_path)
    assert robot.name == "main_assembly"

    # NEVER touch ~/ros2_ws -- build in a fully throwaway workspace.
    ws_dir = tmp_path / "colcon_ws"
    src_dir = ws_dir / "src" / robot.name
    src_dir.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(package_dir, src_dir)

    cmd = f"source /opt/ros/lyrical/setup.bash && cd {ws_dir} && colcon build --packages-select {robot.name}"
    result = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=300)

    assert result.returncode == 0, (
        f"colcon build failed (rc={result.returncode})\n"
        f"--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}"
    )


# ---------------------------------------------------------------------------
# Integration of the ros2_control / Gazebo / MoveIt 2 / Nav2 generators into
# generate_ros_package's include_* flags.
# ---------------------------------------------------------------------------


def make_diff_drive_robot():
    from robot_model import Geometry

    base = Link(
        name="base_link",
        collision_geometry=Geometry(kind="box", size=(0.4, 0.3, 0.15)),
        inertial=Inertial(mass=5.0, ixx=0.1, iyy=0.1, izz=0.1),
    )
    left_wheel = Link(
        name="left_wheel",
        parent="base_link",
        collision_geometry=Geometry(kind="cylinder", radius=0.1, length=0.04),
        inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001),
    )
    right_wheel = Link(
        name="right_wheel",
        parent="base_link",
        collision_geometry=Geometry(kind="cylinder", radius=0.1, length=0.04),
        inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001),
    )
    left_joint = Joint(
        name="left_wheel_joint", type=JointType.CONTINUOUS, parent="base_link", child="left_wheel", axis=(0, 1, 0)
    )
    right_joint = Joint(
        name="right_wheel_joint", type=JointType.CONTINUOUS, parent="base_link", child="right_wheel", axis=(0, 1, 0)
    )
    return Robot(
        name="app_test_rover",
        links=[base, left_wheel, right_wheel],
        joints=[left_joint, right_joint],
        metadata={
            "drivetrain": {
                "type": "differential_drive",
                "left_wheel_joint": "left_wheel_joint",
                "right_wheel_joint": "right_wheel_joint",
                "wheel_separation": 0.4,
                "wheel_radius": 0.1,
            }
        },
    )


def test_generate_ros_package_with_ros2_control_arm(tmp_path):
    robot = make_simple_robot(with_limits=True)
    robot.actuators.append(Actuator(name="joint1_motor", type="electric_motor", joint="joint1", interface="position"))

    package_dir = generate_ros_package(robot, {}, tmp_path, include_ros2_control=True)

    assert (package_dir / "config" / "controllers.yaml").exists()
    assert (package_dir / "launch" / "control.launch.py").exists()
    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    assert "<ros2_control" in urdf_text
    assert "</robot>" in urdf_text.strip().splitlines()[-1] or urdf_text.rstrip().endswith("</robot>")


def test_generate_ros_package_with_gazebo(tmp_path):
    robot = make_simple_robot(with_limits=True)

    package_dir = generate_ros_package(robot, {}, tmp_path, include_gazebo=True)

    assert (package_dir / "worlds" / "empty.sdf").exists()
    assert (package_dir / "launch" / "gazebo.launch.py").exists()
    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    assert "<gazebo" in urdf_text
    assert "gazebo_fragment" not in urdf_text  # wrapper must be unwrapped, not leaked into the URDF

    # Real gap found while adding Gazebo support: package.xml never declared
    # ros_gz_sim/ros_gz_bridge/gz_ros2_control regardless of include_gazebo.
    import xml.etree.ElementTree as ET

    depends = {el.text for el in ET.parse(package_dir / "package.xml").getroot().findall("depend")}
    assert {"ros_gz_sim", "ros_gz_bridge", "gz_ros2_control"} <= depends

    # joint1 is a non-fixed (revolute) joint -> JointStatePublisher plugin +
    # a joint_states bridge entry, independent of drivetrain type.
    assert "JointStatePublisher" in urdf_text
    bridge_yaml = (package_dir / "config" / "ros_gz_bridge.yaml").read_text()
    assert "joint_states" in bridge_yaml


@pytest.mark.skipif(
    shutil.which("gz") is None or not Path("/opt/ros/lyrical/setup.bash").is_file(),
    reason="colcon/gz-sim and/or /opt/ros/lyrical/setup.bash not available in this environment",
)
def test_generate_ros_package_diff_drive_gazebo_builds_and_spawns_without_gz_ros2_control_crash(tmp_path):
    """End-to-end reproduction of the real fix: a differential-drive robot's
    Gazebo package must use the native DiffDrive plugin (not the
    gz_ros2_control plugin, confirmed to SIGSEGV on this machine's gz-sim
    10.4.0 -- docs/ARCHITECTURE.md's "Gazebo" section) -- build the full
    generated package with colcon and spawn it for real, headlessly."""
    robot = make_diff_drive_robot()
    package_dir = generate_ros_package(robot, {}, tmp_path, include_gazebo=True)

    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    assert "gz-sim-diff-drive-system" in urdf_text
    assert "libgz_ros2_control-system.so" not in urdf_text

    ws_dir = tmp_path / "colcon_ws"
    src_dir = ws_dir / "src" / robot.name
    src_dir.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(package_dir, src_dir)

    build_cmd = f"source /opt/ros/lyrical/setup.bash && cd {ws_dir} && colcon build --packages-select {robot.name}"
    build_result = subprocess.run(["bash", "-lc", build_cmd], capture_output=True, text=True, timeout=300)
    assert build_result.returncode == 0, (
        f"colcon build failed (rc={build_result.returncode})\n"
        f"--- stdout ---\n{build_result.stdout}\n--- stderr ---\n{build_result.stderr}"
    )

    launch_cmd = (
        f"source /opt/ros/lyrical/setup.bash && source {ws_dir}/install/setup.bash && "
        f"timeout 25 ros2 launch {robot.name} gazebo.launch.py"
    )
    launch_result = subprocess.run(["bash", "-lc", launch_cmd], capture_output=True, text=True, timeout=60)
    combined_output = launch_result.stdout + launch_result.stderr
    # `timeout 25` kills the (still-running, GUI-less-headless-or-not) launch
    # after 25s -- SIGTERM (rc 124 from `timeout`, or the negative signal
    # number if a child propagates it) is the expected "it was still running
    # and got killed" outcome, not a real failure; only a crash-shaped early
    # exit or a segfault message means the fix regressed.
    assert "Segmentation fault" not in combined_output
    assert "GazeboSimROS2ControlPlugin" not in combined_output
    assert "Entity creation successful" in combined_output, combined_output


def test_generate_ros_package_with_gazebo_and_sensors(tmp_path):
    from robot_model import Sensor

    robot = make_simple_robot(with_limits=True)
    robot.sensors.append(Sensor(name="head_camera", type="camera", parent_link="arm"))
    robot.sensors.append(Sensor(name="base_imu", type="imu", parent_link="base_link"))

    package_dir = generate_ros_package(robot, {}, tmp_path, include_gazebo=True)

    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    assert "<sensor name=\"head_camera\" type=\"camera\">" in urdf_text
    assert "<sensor name=\"base_imu\" type=\"imu\">" in urdf_text
    assert "gazebo_fragment" not in urdf_text  # wrapper must be unwrapped, not leaked into the URDF

    import xml.etree.ElementTree as ET

    ET.fromstring(urdf_text)  # must still be well-formed XML with all fragments spliced in

    bridge_yaml_path = package_dir / "config" / "ros_gz_bridge.yaml"
    assert bridge_yaml_path.exists()
    import yaml

    entries = yaml.safe_load(bridge_yaml_path.read_text())
    ros_topics = {e["ros_topic_name"] for e in entries}
    assert "/head_camera/image" in ros_topics
    assert "/base_imu" in ros_topics


def test_generate_ros_package_with_gazebo_no_sensors_skips_sensor_gazebo_xml(tmp_path):
    # include_gazebo=True but robot.sensors is empty -> no sensor-specific
    # <gazebo> XML generated at all -- matches the "robot.sensors non-empty"
    # gate in generate_ros_package's include_gazebo block. The bridge config
    # DOES still get written, for the unrelated joint_states bridge every
    # gazebo-generated robot with a non-fixed joint gets (see gazebo.py's
    # generate_gazebo_ros_bridge_yaml) -- sensors and joint_states are
    # independent reasons to need a bridge file.
    robot = make_simple_robot(with_limits=True)
    assert robot.sensors == []

    package_dir = generate_ros_package(robot, {}, tmp_path, include_gazebo=True)

    bridge_yaml_path = package_dir / "config" / "ros_gz_bridge.yaml"
    assert bridge_yaml_path.exists()
    import yaml

    entries = yaml.safe_load(bridge_yaml_path.read_text())
    assert [e["ros_topic_name"] for e in entries] == ["joint_states"]


def test_generate_ros_package_with_ros2_control_and_gazebo_both_splice(tmp_path):
    robot = make_simple_robot(with_limits=True)
    robot.actuators.append(Actuator(name="joint1_motor", type="electric_motor", joint="joint1", interface="position"))

    package_dir = generate_ros_package(robot, {}, tmp_path, include_ros2_control=True, include_gazebo=True)

    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    assert "<ros2_control" in urdf_text
    assert "<gazebo" in urdf_text
    import xml.etree.ElementTree as ET

    ET.fromstring(urdf_text)  # must still be well-formed XML with both fragments spliced in


def test_generate_ros_package_with_moveit_suitable(tmp_path):
    robot = make_simple_robot(with_limits=True)

    package_dir = generate_ros_package(robot, {}, tmp_path, include_moveit=True, moveit_group_name="arm")

    assert (package_dir / "config" / f"{robot.name}.srdf").exists()
    assert (package_dir / "config" / "joint_limits.yaml").exists()
    assert (package_dir / "config" / "kinematics.yaml").exists()
    assert (package_dir / "config" / "moveit_controllers.yaml").exists()
    # Regression: move_group won't start without this (see
    # generate_ompl_planning_yaml's docstring) -- confirmed for real.
    assert (package_dir / "config" / "ompl_planning.yaml").exists()
    launch_text = (package_dir / "launch" / "moveit_demo.launch.py").read_text()
    assert f'MOVEIT_CONFIG_PACKAGE = "{robot.name}"' in launch_text
    assert (package_dir / "launch" / "moveit_demo.launch.py").exists()


def test_generate_ros_package_with_moveit_unsuitable_raises(tmp_path):
    robot = make_diff_drive_robot()  # a drivetrain robot is not MoveIt-suitable
    with pytest.raises(PipelineError):
        generate_ros_package(robot, {}, tmp_path, include_moveit=True)


def test_generate_ros_package_with_nav2_suitable(tmp_path):
    robot = make_diff_drive_robot()

    package_dir = generate_ros_package(robot, {}, tmp_path, include_nav2=True)

    assert (package_dir / "config" / "nav2_params.yaml").exists()
    assert (package_dir / "launch" / "nav2_bringup.launch.py").exists()
    assert (package_dir / "config" / "map.yaml").exists()


def test_generate_ros_package_with_nav2_unsuitable_raises(tmp_path):
    robot = make_simple_robot(with_limits=True)  # an arm has no drivetrain metadata
    with pytest.raises(PipelineError):
        generate_ros_package(robot, {}, tmp_path, include_nav2=True)


def test_generate_ros_package_all_four_together_on_diff_drive(tmp_path):
    # A mobile base with wheel actuators requesting every optional output at
    # once (ros2_control + gazebo + nav2; NOT moveit, since a drivetrain
    # robot is correctly refused by detect_moveit_suitability).
    robot = make_diff_drive_robot()
    robot.actuators.append(
        Actuator(name="left_wheel_motor", type="electric_motor", joint="left_wheel_joint", interface="velocity")
    )
    robot.actuators.append(
        Actuator(name="right_wheel_motor", type="electric_motor", joint="right_wheel_joint", interface="velocity")
    )

    package_dir = generate_ros_package(
        robot, {}, tmp_path, include_ros2_control=True, include_gazebo=True, include_nav2=True
    )

    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    import xml.etree.ElementTree as ET

    ET.fromstring(urdf_text)
    assert (package_dir / "config" / "controllers.yaml").exists()
    assert (package_dir / "worlds" / "empty.sdf").exists()
    assert (package_dir / "config" / "nav2_params.yaml").exists()
# attach_collision_proxies
# ---------------------------------------------------------------------------


def make_robot_with_bounding_box():
    base = Link(
        name="base_link",
        inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01),
        metadata={"bounding_box_size": (0.2, 0.3, 0.1)},
    )
    # "arm" deliberately has NO bounding_box_size -- e.g. a hand-authored
    # link, or a Fusion occurrence Fusion reported no bounding box for.
    arm = Link(name="arm", parent="base_link", inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001))
    joint = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="arm",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        velocity_limit=2.0,
        effort_limit=5.0,
    )
    return Robot(name="bbox_test_robot", links=[base, arm], joints=[joint])


def test_attach_collision_proxies_disabled_by_default_leaves_geometry_untouched():
    robot = make_robot_with_bounding_box()
    mesh = Geometry(kind="mesh", mesh_path="package://bbox_test_robot/meshes/base_link.stl")
    robot.link("base_link").visual_geometry = mesh
    robot.link("base_link").collision_geometry = mesh

    result = attach_collision_proxies(robot)  # use_bounding_box_collision defaults to False
    assert result is robot
    base = robot.link("base_link")
    assert base.collision_geometry is mesh
    assert base.visual_geometry is mesh


def test_attach_collision_proxies_replaces_collision_only_for_links_with_bbox():
    robot = make_robot_with_bounding_box()
    mesh = Geometry(kind="mesh", mesh_path="package://bbox_test_robot/meshes/base_link.stl")
    robot.link("base_link").visual_geometry = mesh
    robot.link("base_link").collision_geometry = mesh
    arm_mesh = Geometry(kind="mesh", mesh_path="package://bbox_test_robot/meshes/arm.stl")
    robot.link("arm").visual_geometry = arm_mesh
    robot.link("arm").collision_geometry = arm_mesh

    attach_collision_proxies(robot, use_bounding_box_collision=True)

    base = robot.link("base_link")
    assert base.collision_geometry.kind == "box"
    assert base.collision_geometry.size == pytest.approx((0.2, 0.3, 0.1))
    assert base.visual_geometry is mesh  # visual untouched

    # "arm" has no bounding_box_size metadata -> left completely alone.
    arm = robot.link("arm")
    assert arm.collision_geometry is arm_mesh
    assert arm.visual_geometry is arm_mesh


def test_attach_collision_proxies_no_metadata_at_all_is_a_noop():
    # A hand-authored Robot (e.g. examples/sample_arm.py) has no
    # bounding_box_size metadata on any link -- must be left untouched even
    # with use_bounding_box_collision=True.
    robot = make_simple_robot()
    mesh = Geometry(kind="mesh", mesh_path="package://app_test_robot/meshes/base_link.stl")
    robot.link("base_link").collision_geometry = mesh

    attach_collision_proxies(robot, use_bounding_box_collision=True)
    assert robot.link("base_link").collision_geometry is mesh


def test_generate_ros_package_with_bounding_box_collision_opt_in(tmp_path):
    robot = make_robot_with_bounding_box()
    package_dir = generate_ros_package(robot, {}, tmp_path, use_bounding_box_collision=True)
    assert (package_dir / "urdf" / "bbox_test_robot.urdf.xacro").exists()
    base = robot.link("base_link")
    assert base.collision_geometry.kind == "box"
    assert base.collision_geometry.size == pytest.approx((0.2, 0.3, 0.1))
    # "arm" had no bounding box metadata -> left with no collision geometry
    # at all (attach_mesh_references never set one either, since no mesh
    # was supplied for it in this test).
    assert robot.link("arm").collision_geometry is None


# ---------------------------------------------------------------------------
# Backward compatibility: leaving the new parameter unset must be byte-for-
# byte identical to this change never having happened.
# ---------------------------------------------------------------------------


def _package_file_bytes(package_dir: Path) -> dict:
    return {
        str(p.relative_to(package_dir)): p.read_bytes()
        for p in sorted(package_dir.rglob("*"))
        if p.is_file()
    }


def test_generate_ros_package_default_output_unchanged_with_bbox_metadata(tmp_path):
    # Same robot, same mesh_files, one run with the new parameter explicitly
    # False/unset and one with it explicitly True -- the False/unset run must
    # be identical to pre-change behavior (full mesh collision), proving the
    # opt-in default doesn't alter existing output even when bounding-box
    # metadata IS present and available to use.
    robot_default = make_robot_with_bounding_box()
    fake_mesh = tmp_path / "base_link.stl"
    fake_mesh.write_bytes(b"fake stl content")

    out_default = tmp_path / "default"
    out_default.mkdir()
    package_default = generate_ros_package(robot_default, {"base_link": fake_mesh}, out_default)
    assert robot_default.link("base_link").collision_geometry.kind == "mesh"

    robot_explicit_false = make_robot_with_bounding_box()
    out_explicit = tmp_path / "explicit_false"
    out_explicit.mkdir()
    package_explicit = generate_ros_package(
        robot_explicit_false, {"base_link": fake_mesh}, out_explicit, use_bounding_box_collision=False
    )

    assert _package_file_bytes(package_default) == _package_file_bytes(package_explicit)


def test_run_pipeline_default_matches_pre_change_behavior(tmp_path):
    base = FusionOccurrence(
        name="base_link:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=100, iyy=100, izz=100, ixy=0, ixz=0, iyz=0),
        bounding_box=((0.0, 0.0, 0.0), (10.0, 10.0, 10.0)),
    )
    arm = FusionOccurrence(
        name="arm:1",
        pose=FusionPose(xyz=(5.0, 0.0, 0.0)),
        inertia=FusionInertia(mass=1.0, center_of_mass=(5, 0, 0), ixx=100, iyy=135, izz=135, ixy=0, ixz=0, iyz=0),
        bounding_box=((0.0, 0.0, 0.0), (4.0, 4.0, 4.0)),
    )
    joint = FusionJointInfo(
        name="joint1",
        joint_type="RevoluteJointType",
        occurrence_one="base_link:1",
        occurrence_two="arm:1",
        origin=FusionPose(xyz=(5.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        velocity_limit=2.0,
        effort_limit=10.0,
    )

    from fusion_addin.extraction.converter import build_robot_model

    reader = FakeFusionDesignReader([base, arm], [joint])
    robot_default = build_robot_model(reader, "pipeline_robot")
    out_default = tmp_path / "default"
    out_default.mkdir()
    _, package_default = run_pipeline(reader, "pipeline_robot", out_default)

    reader2 = FakeFusionDesignReader([base, arm], [joint])
    out_explicit = tmp_path / "explicit"
    out_explicit.mkdir()
    _, package_explicit = run_pipeline(
        reader2, "pipeline_robot", out_explicit, use_bounding_box_collision=False
    )

    assert _package_file_bytes(package_default) == _package_file_bytes(package_explicit)


# ---------------------------------------------------------------------------
# progress_callback
# ---------------------------------------------------------------------------


def test_progress_callback_reports_minimal_stages_for_urdf_only(tmp_path):
    robot = make_simple_robot(with_limits=True)
    calls = []

    generate_ros_package(robot, {}, tmp_path, progress_callback=lambda *args: calls.append(args))

    # 3 fixed stages when no include_* flag is set: check+mesh, URDF, write.
    assert len(calls) == 3
    for i, (description, step, total) in enumerate(calls, start=1):
        assert isinstance(description, str) and description
        assert step == i
        assert total == 3


def test_progress_callback_total_grows_with_include_flags(tmp_path):
    robot = make_simple_robot(with_limits=True)
    robot.actuators.append(Actuator(name="joint1_motor", type="electric_motor", joint="joint1", interface="position"))
    calls = []

    generate_ros_package(
        robot,
        {},
        tmp_path,
        include_ros2_control=True,
        include_moveit=True,
        progress_callback=lambda *args: calls.append(args),
    )

    # 3 fixed stages + ros2_control + moveit = 5, and every call must agree
    # on the same total (it's fixed for the whole call, computed up front).
    assert len(calls) == 5
    totals = {total for _, _, total in calls}
    assert totals == {5}
    steps = [step for _, step, _ in calls]
    assert steps == [1, 2, 3, 4, 5]


def test_progress_callback_includes_sensor_stage_only_when_sensors_present(tmp_path):
    from robot_model import Sensor

    robot = make_simple_robot(with_limits=True)
    robot.sensors.append(Sensor(name="cam1", type="camera", parent_link="arm"))
    calls = []

    generate_ros_package(robot, {}, tmp_path, include_gazebo=True, progress_callback=lambda *args: calls.append(args))

    # 3 fixed stages + gazebo + sensors = 5.
    assert len(calls) == 5
    descriptions = [d for d, _, _ in calls]
    assert any("sensor" in d.lower() for d in descriptions)


def test_progress_callback_not_called_past_the_failure_point(tmp_path):
    # A PipelineError (missing actuator limits) is raised inside the very
    # first stage -- the callback sees that one stage announced (it reports
    # "starting", not "succeeded") and no more; later stages (URDF
    # generation, write-to-disk) must never be reported since they never run.
    robot = make_simple_robot(with_limits=False)
    calls = []

    with pytest.raises(PipelineError):
        generate_ros_package(robot, {}, tmp_path, progress_callback=lambda *args: calls.append(args))

    assert len(calls) == 1
    assert calls[0][1:] == (1, 3)


# ---------------------------------------------------------------------------
# should_cancel
# ---------------------------------------------------------------------------


def test_should_cancel_stops_generation_immediately(tmp_path):
    robot = make_simple_robot(with_limits=True)
    calls = []

    with pytest.raises(GenerationCancelled):
        generate_ros_package(
            robot,
            {},
            tmp_path,
            progress_callback=lambda *args: calls.append(args),
            should_cancel=lambda: True,
        )

    # Cancelled right after the very first stage is reported -- no package
    # should have been written at all.
    assert len(calls) == 1
    assert not (tmp_path / robot.name).exists()


def test_should_cancel_partway_through_stops_before_later_stages(tmp_path):
    robot = make_simple_robot(with_limits=True)
    robot.actuators.append(Actuator(name="joint1_motor", type="electric_motor", joint="joint1", interface="position"))
    calls = []

    # should_cancel is checked immediately after progress_callback at every
    # stage checkpoint, so it sees the just-appended call for that same
    # stage -- returning True once 2 stages have been reported cancels right
    # after "Generating URDF/Xacro" (the 2nd stage), before the
    # ros2_control-specific stage (or writing to disk) ever runs.
    def cancel_after_two():
        return len(calls) >= 2

    with pytest.raises(GenerationCancelled):
        generate_ros_package(
            robot,
            {},
            tmp_path,
            include_ros2_control=True,
            progress_callback=lambda *args: calls.append(args),
            should_cancel=cancel_after_two,
        )

    assert len(calls) == 2
    assert calls[0][0] == "Checking actuator limits and attaching mesh/collision geometry"
    assert calls[1][0] == "Generating URDF/Xacro"
    assert not (tmp_path / robot.name).exists()


def test_should_cancel_never_called_when_absent_default_behavior_unchanged(tmp_path):
    # Omitting should_cancel entirely must behave exactly as before this
    # parameter existed -- generation runs to completion.
    robot = make_simple_robot(with_limits=True)
    package_dir = generate_ros_package(robot, {}, tmp_path)
    assert package_dir == tmp_path / "app_test_robot"
    assert (package_dir / "package.xml").exists()


def test_should_cancel_false_lets_generation_complete(tmp_path):
    robot = make_simple_robot(with_limits=True)
    package_dir = generate_ros_package(robot, {}, tmp_path, should_cancel=lambda: False)
    assert (package_dir / "package.xml").exists()


