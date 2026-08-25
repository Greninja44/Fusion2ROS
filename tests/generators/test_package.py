"""Tests for fusion_addin.generators.package.generate_package.

Most of this file runs with plain `python3 -m pytest` (no Fusion, no ROS).
One test is an integration check that shells out to `colcon build` in a
throwaway workspace and is skipped automatically when colcon or the ROS 2
"lyrical" setup script aren't available.
"""

import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from robot_model import Inertial, Joint, JointType, Link, Robot
from fusion_addin.generators.package import generate_package

ROS_SETUP = Path("/opt/ros/lyrical/setup.bash")

TRIVIAL_URDF_XACRO = """<?xml version="1.0"?>
<robot name="demo_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="2"/>
  </joint>
</robot>
"""


def make_demo_robot(name: str = "demo_arm") -> Robot:
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    link1 = Link(name="link1", parent="base_link", inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001))
    joint1 = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="link1",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.57,
        upper_limit=1.57,
        velocity_limit=2.0,
        effort_limit=10.0,
    )
    robot = Robot(name=name, links=[base, link1], joints=[joint1])
    robot.validate()
    return robot


FAKE_MESH_BYTES = b"solid base_link\nfacet normal 0 0 1\nendsolid base_link\n"


def test_generate_package_writes_expected_tree(tmp_path):
    robot = make_demo_robot()
    mesh_files = {"base_link.stl": FAKE_MESH_BYTES}

    pkg_dir = generate_package(robot, TRIVIAL_URDF_XACRO, mesh_files, tmp_path)

    assert pkg_dir == tmp_path / robot.name
    assert pkg_dir.is_dir()

    expected_files = [
        pkg_dir / "package.xml",
        pkg_dir / "CMakeLists.txt",
        pkg_dir / "urdf" / f"{robot.name}.urdf.xacro",
        pkg_dir / "meshes" / "base_link.stl",
        pkg_dir / "launch" / "display.launch.py",
        pkg_dir / "rviz" / f"{robot.name}.rviz",
    ]
    for f in expected_files:
        assert f.is_file(), f"missing expected file: {f}"

    # URDF/xacro written verbatim.
    assert (pkg_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text() == TRIVIAL_URDF_XACRO

    # Mesh bytes written verbatim.
    assert (pkg_dir / "meshes" / "base_link.stl").read_bytes() == FAKE_MESH_BYTES

    # CMakeLists references the four install dirs and ament_cmake.
    cmake_text = (pkg_dir / "CMakeLists.txt").read_text()
    assert "ament_cmake" in cmake_text
    assert "urdf" in cmake_text and "meshes" in cmake_text and "launch" in cmake_text and "rviz" in cmake_text
    assert f"project({robot.name})" in cmake_text

    # launch file references robot_state_publisher, joint_state_publisher_gui, rviz2, xacro.
    launch_text = (pkg_dir / "launch" / "display.launch.py").read_text()
    assert "robot_state_publisher" in launch_text
    assert "joint_state_publisher_gui" in launch_text
    assert "rviz2" in launch_text
    assert "xacro" in launch_text
    assert "generate_launch_description" in launch_text

    # rviz config has the root link as Fixed Frame and a RobotModel display.
    rviz_text = (pkg_dir / "rviz" / f"{robot.name}.rviz").read_text()
    assert f"Fixed Frame: {robot.root_link().name}" in rviz_text
    assert "RobotModel" in rviz_text


def test_package_xml_is_well_formed_and_has_required_fields(tmp_path):
    robot = make_demo_robot()
    pkg_dir = generate_package(robot, TRIVIAL_URDF_XACRO, {}, tmp_path)

    package_xml_path = pkg_dir / "package.xml"
    tree = ET.parse(package_xml_path)
    root = tree.getroot()

    assert root.tag == "package"
    assert root.get("format") == "3"
    assert root.findtext("name") == robot.name

    buildtool_depends = [el.text for el in root.findall("buildtool_depend")]
    assert "ament_cmake" in buildtool_depends

    depends = {el.text for el in root.findall("depend")}
    required_depends = {
        "urdf",
        "xacro",
        "robot_state_publisher",
        "joint_state_publisher",
        "joint_state_publisher_gui",
        "rviz2",
        "launch",
        "launch_ros",
    }
    assert required_depends <= depends

    # Must not hardcode a specific ROS distro anywhere in package.xml.
    raw_text = package_xml_path.read_text()
    assert "lyrical" not in raw_text.lower()


def test_generate_package_is_idempotent_and_clears_stale_meshes(tmp_path):
    robot = make_demo_robot()

    first_pkg_dir = generate_package(
        robot, TRIVIAL_URDF_XACRO, {"old_mesh.stl": FAKE_MESH_BYTES}, tmp_path
    )
    assert (first_pkg_dir / "meshes" / "old_mesh.stl").is_file()

    second_pkg_dir = generate_package(
        robot, TRIVIAL_URDF_XACRO, {"new_mesh.stl": FAKE_MESH_BYTES}, tmp_path
    )

    assert second_pkg_dir == first_pkg_dir
    assert (second_pkg_dir / "meshes" / "new_mesh.stl").is_file()
    assert not (second_pkg_dir / "meshes" / "old_mesh.stl").exists()

    mesh_dir_contents = sorted(p.name for p in (second_pkg_dir / "meshes").iterdir())
    assert mesh_dir_contents == ["new_mesh.stl"]


def test_generate_package_regenerating_with_same_inputs_is_stable(tmp_path):
    robot = make_demo_robot()
    mesh_files = {"base_link.stl": FAKE_MESH_BYTES}

    pkg_dir_1 = generate_package(robot, TRIVIAL_URDF_XACRO, mesh_files, tmp_path)
    contents_1 = {
        p.relative_to(pkg_dir_1): p.read_bytes() for p in pkg_dir_1.rglob("*") if p.is_file()
    }

    pkg_dir_2 = generate_package(robot, TRIVIAL_URDF_XACRO, mesh_files, tmp_path)
    contents_2 = {
        p.relative_to(pkg_dir_2): p.read_bytes() for p in pkg_dir_2.rglob("*") if p.is_file()
    }

    assert contents_1 == contents_2


def test_mesh_files_accepts_path_source(tmp_path):
    robot = make_demo_robot()

    source_dir = tmp_path / "source_meshes"
    source_dir.mkdir()
    source_mesh = source_dir / "external.stl"
    source_mesh.write_bytes(FAKE_MESH_BYTES)

    out_dir = tmp_path / "out"
    pkg_dir = generate_package(robot, TRIVIAL_URDF_XACRO, {"external.stl": source_mesh}, out_dir)

    copied = pkg_dir / "meshes" / "external.stl"
    assert copied.is_file()
    assert copied.read_bytes() == FAKE_MESH_BYTES
    # Confirm it was copied, not linked/moved: source still exists.
    assert source_mesh.is_file()


def test_mesh_files_rejects_unsupported_type(tmp_path):
    robot = make_demo_robot()
    with pytest.raises(TypeError):
        generate_package(robot, TRIVIAL_URDF_XACRO, {"bad.stl": "not bytes or Path"}, tmp_path)


def test_extra_files_written_and_installed(tmp_path):
    # This is the integration point ros2_control/gazebo/moveit/nav2's
    # generated config/launch text lands through -- confirm it actually
    # reaches disk under the package dir, in a config/ dir the CMakeLists
    # installs, alongside str-vs-bytes handling like mesh_files has.
    robot = make_demo_robot()
    pkg_dir = generate_package(
        robot,
        TRIVIAL_URDF_XACRO,
        {},
        tmp_path,
        extra_files={
            "config/controllers.yaml": "controller_manager:\n  ros__parameters: {}\n",
            "launch/control.launch.py": "def generate_launch_description():\n    pass\n",
            "config/binary.bin": b"\x00\x01\x02",
        },
    )

    assert (pkg_dir / "config" / "controllers.yaml").read_text() == "controller_manager:\n  ros__parameters: {}\n"
    assert (pkg_dir / "launch" / "control.launch.py").is_file()
    assert (pkg_dir / "config" / "binary.bin").read_bytes() == b"\x00\x01\x02"
    assert "config" in (pkg_dir / "CMakeLists.txt").read_text()


def test_extra_files_rejects_unsupported_type(tmp_path):
    robot = make_demo_robot()
    with pytest.raises(TypeError):
        generate_package(
            robot, TRIVIAL_URDF_XACRO, {}, tmp_path, extra_files={"config/bad.yaml": 12345}
        )


@pytest.mark.skipif(
    shutil.which("colcon") is None or not ROS_SETUP.is_file(),
    reason="colcon and/or /opt/ros/lyrical/setup.bash not available in this environment",
)
def test_generated_package_builds_with_colcon(tmp_path):
    robot = make_demo_robot(name="colcon_build_demo")
    mesh_files = {"base_link.stl": FAKE_MESH_BYTES}

    out_dir = tmp_path / "generated"
    pkg_dir = generate_package(robot, TRIVIAL_URDF_XACRO, mesh_files, out_dir)

    # NEVER touch ~/ros2_ws — build in a fully throwaway workspace under tmp_path.
    ws_dir = tmp_path / "colcon_ws"
    src_dir = ws_dir / "src" / robot.name
    src_dir.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(pkg_dir, src_dir)

    cmd = (
        f"source {ROS_SETUP} && "
        f"cd {ws_dir} && "
        f"colcon build --packages-select {robot.name}"
    )
    result = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        timeout=300,
    )

    assert result.returncode == 0, (
        f"colcon build failed (rc={result.returncode})\n"
        f"--- stdout ---\n{result.stdout}\n--- stderr ---\n{result.stderr}"
    )
