"""Tests for fusion_addin.generators.gazebo.

Must run with plain `python3 -m pytest` -- no Fusion, no live ROS/Gazebo
needed for the core tests. Two extra tests give real signal when the tools
happen to be installed and skip cleanly otherwise:

- `test_gz_ros2_control_plugin_block_if_installed`: if `gz_ros2_control` is
  installed, does one extra check that the plugin .so this module names is
  the same one gz_ros2_control actually ships. Not installed as of writing
  this (`ros2 pkg list | grep -i gz_ros2_control` and
  `dpkg -l | grep gz-ros2-control` both came back empty in this sandbox),
  so it is expected to skip here -- included so it does something useful the
  day that changes.
- `test_generated_world_sdf_loads_headless_in_gz_sim`: writes the generated
  world SDF to a tmp_path file and runs it for real via
  `gz sim -s -r --iterations N <file>` (headless server-only, short
  iteration count, short timeout), confirming it doesn't immediately crash.
  `gz` (gz-sim 10.4.0) IS installed in this environment, so this one
  actually runs rather than skipping.
"""

import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.generators.gazebo import (
    GZ_DIFF_DRIVE_PLUGIN_FILENAME,
    GZ_DIFF_DRIVE_PLUGIN_NAME,
    GZ_JOINT_STATE_PUBLISHER_PLUGIN_FILENAME,
    GZ_JOINT_STATE_PUBLISHER_PLUGIN_NAME,
    GZ_ROS2_CONTROL_PLUGIN_FILENAME,
    GZ_ROS2_CONTROL_PLUGIN_NAME,
    generate_gazebo_ros_bridge_yaml,
    generate_gazebo_xml,
    generate_spawn_launch,
    generate_world_sdf,
)
from robot_model import Geometry, Inertial, Joint, JointType, Link, Material, Pose, Robot


# --- fixtures ---------------------------------------------------------------


def make_two_link_arm() -> Robot:
    """Same shape as tests/generators/test_urdf.py's fixture: a base_link
    with a colored material, and a second link with NO material at all, so
    generate_gazebo_xml's per-link skip logic (no material / no rgba -> no
    <gazebo reference=...> block) is exercised alongside the case that does
    get one."""
    base = Link(
        name="base_link",
        visual_geometry=Geometry(kind="box", size=(0.2, 0.2, 0.05)),
        collision_geometry=Geometry(kind="box", size=(0.2, 0.2, 0.05)),
        material=Material(name="grey", rgba=(0.5, 0.25, 0.125, 1.0)),
        inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01),
    )
    link1 = Link(
        name="link1",
        parent="base_link",
        inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001),
    )
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        origin=Pose(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.57,
        upper_limit=1.57,
        velocity_limit=2.0,
        effort_limit=10.0,
    )
    return Robot(name="two_link_arm", links=[base, link1], joints=[joint1])


def make_robot_with_material_but_no_rgba() -> Robot:
    base = Link(name="base_link", material=Material(name="unnamed_no_color", rgba=None))
    return Robot(name="colorless_bot", links=[base])


# --- generate_gazebo_xml -----------------------------------------------


def test_gazebo_xml_parses_and_has_expected_root():
    robot = make_two_link_arm()
    xml_text = generate_gazebo_xml(robot)
    root = ET.fromstring(xml_text)
    assert root.tag == "gazebo_fragment"


def test_material_block_emitted_only_for_link_with_rgba():
    robot = make_two_link_arm()
    root = ET.fromstring(generate_gazebo_xml(robot))
    referenced = [g.attrib.get("reference") for g in root.findall("gazebo") if "reference" in g.attrib]
    assert referenced == ["base_link"]  # link1 has no material -> no block


def test_material_block_uses_gz_sim_color_tags_not_classic_gazebo_string():
    robot = make_two_link_arm()
    root = ET.fromstring(generate_gazebo_xml(robot))
    base_gazebo = next(g for g in root.findall("gazebo") if g.attrib.get("reference") == "base_link")
    visual = base_gazebo.find("visual")
    assert visual is not None
    material = visual.find("material")
    assert material is not None
    # gz-sim convention: color child elements, not a "Gazebo/Blue"-style
    # material-script string directly under <material>.
    assert material.text is None or not material.text.strip()
    ambient = material.find("ambient")
    diffuse = material.find("diffuse")
    specular = material.find("specular")
    assert ambient is not None and ambient.text == "0.5 0.25 0.125 1.0"
    assert diffuse is not None and diffuse.text == "0.5 0.25 0.125 1.0"
    assert specular is not None and specular.text == "0.5 0.25 0.125 1.0"


def test_link_with_material_but_no_rgba_gets_no_block():
    robot = make_robot_with_material_but_no_rgba()
    root = ET.fromstring(generate_gazebo_xml(robot))
    referenced = [g for g in root.findall("gazebo") if "reference" in g.attrib]
    assert referenced == []


def test_ros2_control_plugin_block_present_and_robot_level():
    # make_two_link_arm() has no metadata["drivetrain"], so it falls back to
    # the (still-crashing-on-real-gz-sim, but unchanged/non-regressed)
    # gz_ros2_control plugin -- alongside the now-unconditional
    # JointStatePublisher block (any non-fixed joint gets one, independent
    # of drivetrain type), so there are two un-referenced <gazebo> blocks
    # now, not one; filter to the actuation one specifically.
    robot = make_two_link_arm()
    root = ET.fromstring(generate_gazebo_xml(robot))
    plugin_gazebos = [g for g in root.findall("gazebo") if "reference" not in g.attrib]
    assert len(plugin_gazebos) == 2
    ros2_control_gazebos = [
        g for g in plugin_gazebos if g.find("plugin").attrib["filename"] == GZ_ROS2_CONTROL_PLUGIN_FILENAME
    ]
    assert len(ros2_control_gazebos) == 1
    plugin = ros2_control_gazebos[0].find("plugin")
    assert plugin.attrib["filename"] == "libgz_ros2_control-system.so" == GZ_ROS2_CONTROL_PLUGIN_FILENAME
    assert (
        plugin.attrib["name"] == "gz_ros2_control::GazeboSimROS2ControlPlugin" == GZ_ROS2_CONTROL_PLUGIN_NAME
    )
    parameters = plugin.find("parameters")
    assert parameters is not None
    assert parameters.text == "$(find two_link_arm)/config/controllers.yaml"


def test_joint_state_publisher_plugin_present_for_any_non_fixed_joint():
    robot = make_two_link_arm()
    root = ET.fromstring(generate_gazebo_xml(robot))
    plugin_gazebos = [g for g in root.findall("gazebo") if "reference" not in g.attrib]
    jsp_gazebos = [
        g
        for g in plugin_gazebos
        if g.find("plugin").attrib["filename"] == GZ_JOINT_STATE_PUBLISHER_PLUGIN_FILENAME
    ]
    assert len(jsp_gazebos) == 1
    plugin = jsp_gazebos[0].find("plugin")
    assert plugin.attrib["name"] == GZ_JOINT_STATE_PUBLISHER_PLUGIN_NAME
    assert plugin.find("topic").text == "joint_states"
    joint_names = [e.text for e in plugin.findall("joint_name")]
    assert joint_names == [j.name for j in robot.joints if j.type != JointType.FIXED]


def make_diff_drive_rover():
    base = Link(name="base_link", inertial=Inertial(mass=5.0, ixx=0.1, iyy=0.1, izz=0.1))
    left_wheel = Link(name="left_wheel", parent="base_link", inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001))
    right_wheel = Link(name="right_wheel", parent="base_link", inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001))
    left_joint = Joint(
        name="left_wheel_joint", type=JointType.CONTINUOUS, parent="base_link", child="left_wheel", axis=(0, 1, 0)
    )
    right_joint = Joint(
        name="right_wheel_joint", type=JointType.CONTINUOUS, parent="base_link", child="right_wheel", axis=(0, 1, 0)
    )
    return Robot(
        name="diff_drive_rover",
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


def test_differential_drive_robot_gets_native_diff_drive_plugin_not_ros2_control():
    # Real bug this replaces: gz_ros2_control SIGSEGVs on load against this
    # machine's gz-sim 10.4.0 (confirmed upstream, see
    # docs/ARCHITECTURE.md's "Gazebo" section and gz-sim-diff-drive-system's
    # module-level comment in gazebo.py) -- a "differential_drive" robot
    # must get the native DiffDrive plugin INSTEAD, not alongside it.
    robot = make_diff_drive_rover()
    root = ET.fromstring(generate_gazebo_xml(robot))
    plugin_gazebos = [g for g in root.findall("gazebo") if "reference" not in g.attrib]
    filenames = {g.find("plugin").attrib["filename"] for g in plugin_gazebos}
    assert GZ_DIFF_DRIVE_PLUGIN_FILENAME in filenames
    assert GZ_ROS2_CONTROL_PLUGIN_FILENAME not in filenames

    diff_drive_gazebo = next(
        g for g in plugin_gazebos if g.find("plugin").attrib["filename"] == GZ_DIFF_DRIVE_PLUGIN_FILENAME
    )
    plugin = diff_drive_gazebo.find("plugin")
    assert plugin.attrib["name"] == GZ_DIFF_DRIVE_PLUGIN_NAME
    assert plugin.find("left_joint").text == "left_wheel_joint"
    assert plugin.find("right_joint").text == "right_wheel_joint"
    assert plugin.find("wheel_separation").text == "0.4"
    assert plugin.find("wheel_radius").text == "0.1"
    assert plugin.find("topic").text == "cmd_vel"
    assert plugin.find("odom_topic").text == "odom"
    assert plugin.find("frame_id").text == "odom"
    assert plugin.find("child_frame_id").text == "base_link"


def test_gazebo_ros_bridge_yaml_for_differential_drive_has_cmd_vel_odom_tf_and_joint_states():
    robot = make_diff_drive_rover()
    text = generate_gazebo_ros_bridge_yaml(robot)
    entries = yaml.safe_load(text)
    topics = {e["ros_topic_name"]: e for e in entries}
    assert set(topics) == {"joint_states", "cmd_vel", "odom", "tf"}
    assert topics["cmd_vel"]["direction"] == "ROS_TO_GZ"
    assert topics["cmd_vel"]["ros_type_name"] == "geometry_msgs/msg/TwistStamped"
    assert topics["cmd_vel"]["gz_type_name"] == "gz.msgs.Twist"
    assert topics["odom"]["direction"] == "GZ_TO_ROS"
    assert topics["odom"]["ros_type_name"] == "nav_msgs/msg/Odometry"
    assert topics["joint_states"]["direction"] == "GZ_TO_ROS"
    assert topics["joint_states"]["gz_type_name"] == "gz.msgs.Model"


def test_gazebo_ros_bridge_yaml_empty_for_robot_with_no_joints_or_drivetrain():
    robot = Robot(name="single_link", links=[Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))])
    assert generate_gazebo_ros_bridge_yaml(robot) == ""


def test_spawn_launch_starts_parameter_bridge_only_when_include_bridge_true():
    robot = make_diff_drive_rover()
    with_bridge = generate_spawn_launch(robot, include_bridge=True)
    without_bridge = generate_spawn_launch(robot, include_bridge=False)
    assert "ros_gz_bridge" in with_bridge
    assert "parameter_bridge" in with_bridge
    assert "bridge_node" in with_bridge
    assert "ros_gz_bridge" not in without_bridge
    compile(with_bridge, "gazebo.launch.py", "exec")
    compile(without_bridge, "gazebo.launch.py", "exec")


@pytest.mark.skipif(
    shutil.which("gz") is None,
    reason="gz-sim not installed in this environment",
)
def test_diff_drive_plugin_loads_without_crashing_in_real_gz_sim(tmp_path):
    """Real, live verification that gz-sim-diff-drive-system does NOT hit
    the confirmed gz_ros2_control SIGSEGV (docs/ARCHITECTURE.md's "Gazebo"
    section) -- spawns a real model with this exact plugin block into a
    real headless gz-sim (`gz sim -s -r --iterations N <file>.sdf`) and
    confirms the process exits 0, the same style
    `test_generated_world_sdf_loads_headless_in_gz_sim` below already uses.
    """
    robot = make_diff_drive_rover()
    root = ET.fromstring(generate_gazebo_xml(robot))
    diff_drive_gazebo = next(
        g
        for g in root.findall("gazebo")
        if "reference" not in g.attrib and g.find("plugin").attrib["filename"] == GZ_DIFF_DRIVE_PLUGIN_FILENAME
    )
    plugin_sdf = ET.tostring(diff_drive_gazebo.find("plugin"), encoding="unicode")

    world_sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="diff_drive_smoke_test">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <model name="diff_drive_rover">
      <link name="base_link">
        <inertial><mass>5.0</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>
      </link>
      {plugin_sdf}
    </model>
  </world>
</sdf>
"""
    world_path = tmp_path / "diff_drive_smoke_test.sdf"
    world_path.write_text(world_sdf, encoding="utf-8")

    result = subprocess.run(
        ["gz", "sim", "-s", "-r", "--iterations", "20", str(world_path)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f"gz sim exited {result.returncode} (a non-zero/crash here would mean DiffDrive "
        f"regressed to the same crash gz_ros2_control hits)\n"
        f"--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}"
    )


def test_gazebo_xml_is_deterministic():
    robot = make_two_link_arm()
    assert generate_gazebo_xml(robot) == generate_gazebo_xml(robot)


# --- generate_world_sdf --------------------------------------------------


def test_world_sdf_parses_and_has_world_root():
    sdf_text = generate_world_sdf("test_world")
    root = ET.fromstring(sdf_text)
    assert root.tag == "sdf"
    world = root.find("world")
    assert world is not None
    assert world.attrib["name"] == "test_world"


def test_world_sdf_default_name_is_empty():
    sdf_text = generate_world_sdf()
    world = ET.fromstring(sdf_text).find("world")
    assert world.attrib["name"] == "empty"


def test_world_sdf_has_ground_plane_and_light():
    world = ET.fromstring(generate_world_sdf()).find("world")
    models = world.findall("model")
    assert any(m.attrib.get("name") == "ground_plane" for m in models)
    lights = world.findall("light")
    assert len(lights) == 1
    assert lights[0].attrib["type"] == "directional"


def test_world_sdf_has_required_gz_sim_system_plugins():
    """gz-sim (unlike classic Gazebo) doesn't auto-load Physics/
    UserCommands/SceneBroadcaster -- a world missing these won't step
    physics or expose the spawn service. Confirmed against this machine's
    installed ros_gz_sim_demos/worlds/default.sdf (see gazebo.py docstring)."""
    world = ET.fromstring(generate_world_sdf()).find("world")
    plugin_filenames = {p.attrib["filename"] for p in world.findall("plugin")}
    assert "gz-sim-physics-system" in plugin_filenames
    assert "gz-sim-user-commands-system" in plugin_filenames
    assert "gz-sim-scene-broadcaster-system" in plugin_filenames


def test_world_sdf_has_sensor_system_plugins():
    """Regression: fusion_addin/generators/sensors.py's generated <sensor>
    elements (camera/lidar/imu) produce no live Gazebo Transport topics at
    all without gz-sim-sensors-system loaded in the world, and an IMU
    sensor specifically also needs gz-sim-imu-system -- confirmed via a
    real before/after gz-sim headless topic-list comparison. Both plugins'
    exact filename/name and the Sensors plugin's <render_engine> child are
    copied verbatim from this machine's installed
    ros_gz_sim_demos/worlds/default.sdf, same as the other system plugins."""
    world = ET.fromstring(generate_world_sdf()).find("world")
    plugins = {p.attrib["filename"]: p for p in world.findall("plugin")}
    assert "gz-sim-sensors-system" in plugins
    assert plugins["gz-sim-sensors-system"].attrib["name"] == "gz::sim::systems::Sensors"
    assert plugins["gz-sim-sensors-system"].findtext("render_engine") == "ogre2"
    assert "gz-sim-imu-system" in plugins
    assert plugins["gz-sim-imu-system"].attrib["name"] == "gz::sim::systems::Imu"


# --- generate_spawn_launch -----------------------------------------------


def test_spawn_launch_is_syntactically_valid_python():
    robot = make_two_link_arm()
    text = generate_spawn_launch(robot)
    compile(text, "<generated spawn launch>", "exec")


def test_spawn_launch_references_expected_ros_gz_sim_names():
    robot = make_two_link_arm()
    text = generate_spawn_launch(robot)
    assert "ros_gz_sim" in text
    assert "gz_sim.launch.py" in text
    assert '"create"' in text
    assert "/robot_description" in text
    assert "robot_state_publisher" in text
    assert "two_link_arm" in text  # robot.name threaded through
    assert "def generate_launch_description():" in text


def test_spawn_launch_is_deterministic():
    robot = make_two_link_arm()
    assert generate_spawn_launch(robot) == generate_spawn_launch(robot)


# --- real-tool integration checks (skip cleanly if unavailable) ------------


def _gz_ros2_control_installed() -> bool:
    if shutil.which("gz_ros2_control") is not None:
        return True
    try:
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "gz_ros2_control"],
            capture_output=True,
            text=True,
            timeout=15,
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


@pytest.mark.skipif(not _gz_ros2_control_installed(), reason="gz_ros2_control not installed on this machine")
def test_gz_ros2_control_plugin_block_if_installed():
    """Only runs if gz_ros2_control shows up in `ros2 pkg prefix`. As of
    writing this test it does not (confirmed via both `ros2 pkg list` and
    `dpkg -l`), so this is expected to skip in this environment -- it's here
    so the plugin filename/name pair gets a real cross-check the day the
    package is installed, rather than resting solely on the fetched doc
    example cited in gazebo.py."""
    result = subprocess.run(["ros2", "pkg", "prefix", "gz_ros2_control"], capture_output=True, text=True, timeout=15)
    prefix = Path(result.stdout.strip())
    so_matches = list(prefix.rglob(f"*{GZ_ROS2_CONTROL_PLUGIN_FILENAME.removeprefix('lib')}"))
    assert so_matches, f"{GZ_ROS2_CONTROL_PLUGIN_FILENAME} not found under installed gz_ros2_control prefix {prefix}"


GZ_BIN = shutil.which("gz") or "/opt/ros/lyrical/opt/gz_tools_vendor/bin/gz"


@pytest.mark.skipif(
    shutil.which("gz") is None and not Path("/opt/ros/lyrical/opt/gz_tools_vendor/bin/gz").exists(),
    reason="gz (gz-sim) binary not available on this machine",
)
def test_generated_world_sdf_loads_headless_in_gz_sim(tmp_path):
    world_text = generate_world_sdf("pytest_world")
    world_path = tmp_path / "pytest_world.sdf"
    world_path.write_text(world_text)

    proc = subprocess.run(
        [GZ_BIN, "sim", "-s", "-r", "--iterations", "5", str(world_path)],
        capture_output=True,
        text=True,
        timeout=40,
    )
    assert proc.returncode == 0, (
        f"gz sim exited {proc.returncode} on generated world.\n"
        f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    )
