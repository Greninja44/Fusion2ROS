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

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.generators.gazebo import (
    GZ_ROS2_CONTROL_PLUGIN_FILENAME,
    GZ_ROS2_CONTROL_PLUGIN_NAME,
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
    robot = make_two_link_arm()
    root = ET.fromstring(generate_gazebo_xml(robot))
    plugin_gazebos = [g for g in root.findall("gazebo") if "reference" not in g.attrib]
    assert len(plugin_gazebos) == 1
    plugin = plugin_gazebos[0].find("plugin")
    assert plugin is not None
    assert plugin.attrib["filename"] == "libgz_ros2_control-system.so" == GZ_ROS2_CONTROL_PLUGIN_FILENAME
    assert (
        plugin.attrib["name"] == "gz_ros2_control::GazeboSimROS2ControlPlugin" == GZ_ROS2_CONTROL_PLUGIN_NAME
    )
    parameters = plugin.find("parameters")
    assert parameters is not None
    assert parameters.text == "$(find two_link_arm)/config/controllers.yaml"


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
