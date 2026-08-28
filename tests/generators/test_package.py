"""Tests for fusion_addin.generators.package.generate_package.

Most of this file runs with plain `python3 -m pytest` (no Fusion, no ROS).
One test is an integration check that shells out to `colcon build` in a
throwaway workspace and is skipped automatically when colcon or the ROS 2
"lyrical" setup script aren't available.
"""

import ast
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from robot_model import Inertial, Joint, JointType, Link, Robot
from fusion_addin.generators.package import PackageManifest, generate_package

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


def test_extra_depends_are_added_without_duplicating_base_set(tmp_path):
    # Real gap found while adding Gazebo support: package.xml never declared
    # ros_gz_sim/ros_gz_bridge regardless of include_gazebo, since the
    # <depend> list was entirely hardcoded -- extra_depends closes that.
    robot = make_demo_robot()
    pkg_dir = generate_package(
        robot,
        TRIVIAL_URDF_XACRO,
        {},
        tmp_path,
        extra_depends=["ros_gz_sim", "ros_gz_bridge", "xacro"],  # "xacro" duplicates the base set
    )

    depends = [el.text for el in ET.parse(pkg_dir / "package.xml").getroot().findall("depend")]
    assert depends.count("xacro") == 1
    assert "ros_gz_sim" in depends
    assert "ros_gz_bridge" in depends


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


def test_extra_files_introducing_a_new_top_level_dir_is_installed(tmp_path):
    # Regression: gazebo.py's generated worlds/empty.sdf sat in the package
    # but was never installed by CMakeLists (which hardcoded a fixed
    # urdf/meshes/launch/rviz/config list) -- `gz sim` reported "Unable to
    # find or download file ... worlds/empty.sdf" at launch despite a
    # successful colcon build. install_dirs must be computed from what's
    # actually on disk, not a fixed list, so any generator's new top-level
    # directory (not just the ones known about today) gets installed too.
    robot = make_demo_robot()
    pkg_dir = generate_package(
        robot, TRIVIAL_URDF_XACRO, {}, tmp_path, extra_files={"worlds/empty.sdf": "<sdf version='1.8'></sdf>\n"}
    )

    assert (pkg_dir / "worlds" / "empty.sdf").is_file()
    cmake_text = (pkg_dir / "CMakeLists.txt").read_text()
    assert "worlds" in cmake_text.split("DIRECTORY")[1].split("DESTINATION")[0]


def test_extra_files_rejects_unsupported_type(tmp_path):
    robot = make_demo_robot()
    with pytest.raises(TypeError):
        generate_package(
            robot, TRIVIAL_URDF_XACRO, {}, tmp_path, extra_files={"config/bad.yaml": 12345}
        )


# --- metadata/name edge cases -------------------------------------------


def test_package_xml_escapes_special_characters_in_metadata(tmp_path):
    # Real bug: package.xml's fields were built with raw f-string
    # interpolation of free-form metadata text (a Fusion project
    # description, a maintainer's name, ...). A description like
    # "Arm & Gripper <v2>" or a maintainer name containing a `"` produced
    # package.xml that isn't well-formed XML at all.
    robot = make_demo_robot()
    robot.metadata = {
        "description": "Arm & Gripper <v2>",
        "maintainer_name": 'O\'Brien "Bob"',
        "maintainer_email": "a@b.com",
        "license": "BSD & MIT",
    }
    pkg_dir = generate_package(robot, TRIVIAL_URDF_XACRO, {}, tmp_path)

    package_xml_path = pkg_dir / "package.xml"
    tree = ET.parse(package_xml_path)  # raises if not well-formed
    root = tree.getroot()
    assert root.findtext("description") == "Arm & Gripper <v2>"
    assert root.findtext("maintainer") == 'O\'Brien "Bob"'
    assert root.find("maintainer").get("email") == "a@b.com"
    assert root.findtext("license") == "BSD & MIT"


def test_launch_file_is_valid_python_when_robot_name_contains_quotes(tmp_path):
    # Real bug: PACKAGE_NAME/URDF_XACRO_FILE/RVIZ_CONFIG_FILE were built by
    # wrapping robot.name in literal double quotes via an f-string, so a
    # robot named e.g. 'weird"name' produced a display.launch.py that failed
    # to even parse (unterminated string literal).
    robot = make_demo_robot(name='weird"name')
    pkg_dir = generate_package(robot, TRIVIAL_URDF_XACRO, {}, tmp_path)

    launch_text = (pkg_dir / "launch" / "display.launch.py").read_text()
    ast.parse(launch_text)  # raises SyntaxError if malformed
    assert 'weird"name' in launch_text


def test_generate_package_rejects_robot_name_that_escapes_output_dir(tmp_path):
    # Real bug: pathlib's `/` silently discards the left side when the right
    # side is absolute, and a ".." component walks back out of output_dir --
    # either way, output_dir / robot.name could resolve outside output_dir,
    # and generate_package unconditionally shutil.rmtree's whatever's there.
    robot = make_demo_robot(name="../escaped_pkg")
    with pytest.raises(ValueError, match="Robot.name"):
        generate_package(robot, TRIVIAL_URDF_XACRO, {}, tmp_path)


def test_generate_package_rejects_mesh_files_key_that_escapes_meshes_dir(tmp_path):
    robot = make_demo_robot()
    with pytest.raises(ValueError, match="mesh_files"):
        generate_package(robot, TRIVIAL_URDF_XACRO, {"../escaped.stl": FAKE_MESH_BYTES}, tmp_path)


def test_generate_package_rejects_extra_files_key_that_escapes_pkg_dir(tmp_path):
    robot = make_demo_robot()
    with pytest.raises(ValueError, match="extra_files"):
        generate_package(
            robot, TRIVIAL_URDF_XACRO, {}, tmp_path, extra_files={"../escaped.yaml": "data: 1\n"}
        )


# --- dry_run -------------------------------------------------------------


def test_dry_run_writes_nothing(tmp_path):
    robot = make_demo_robot()
    mesh_files = {"base_link.stl": FAKE_MESH_BYTES}

    result = generate_package(
        robot,
        TRIVIAL_URDF_XACRO,
        mesh_files,
        tmp_path,
        extra_files={"config/controllers.yaml": "controller_manager: {}\n"},
        dry_run=True,
    )

    assert isinstance(result, PackageManifest)
    # Not even the package directory itself is created -- a real run's
    # first move is `pkg_dir.mkdir(...)` (after any pre-existing dir is
    # rmtree'd); dry_run must short-circuit before any of that.
    assert not tmp_path.exists() or not any(tmp_path.iterdir())


def test_dry_run_manifest_lists_every_file_a_real_run_would_write(tmp_path):
    robot = make_demo_robot()
    mesh_files = {"base_link.stl": FAKE_MESH_BYTES}
    extra_files = {
        "config/controllers.yaml": "controller_manager: {}\n",
        "worlds/empty.sdf": "<sdf version='1.8'></sdf>\n",
    }

    manifest = generate_package(
        robot, TRIVIAL_URDF_XACRO, mesh_files, tmp_path, extra_files=extra_files, dry_run=True
    )
    real_pkg_dir = generate_package(
        robot, TRIVIAL_URDF_XACRO, mesh_files, tmp_path, extra_files=extra_files
    )

    real_files = {str(p.relative_to(real_pkg_dir)) for p in real_pkg_dir.rglob("*") if p.is_file()}
    assert set(manifest.paths) == real_files
    assert manifest.pkg_dir == real_pkg_dir
    # worlds/ is a new top-level dir introduced by extra_files -- must show
    # up in install_dirs same as a real run's CMakeLists.txt would.
    assert "worlds" in manifest.install_dirs
    real_cmake_dirs = (real_pkg_dir / "CMakeLists.txt").read_text()
    assert "worlds" in real_cmake_dirs


def test_dry_run_still_rejects_robot_name_that_escapes_output_dir(tmp_path):
    robot = make_demo_robot(name="../escaped_pkg")
    with pytest.raises(ValueError, match="Robot.name"):
        generate_package(robot, TRIVIAL_URDF_XACRO, {}, tmp_path, dry_run=True)


def test_dry_run_still_rejects_mesh_files_key_that_escapes_meshes_dir(tmp_path):
    robot = make_demo_robot()
    with pytest.raises(ValueError, match="mesh_files"):
        generate_package(
            robot, TRIVIAL_URDF_XACRO, {"../escaped.stl": FAKE_MESH_BYTES}, tmp_path, dry_run=True
        )


def test_dry_run_still_rejects_extra_files_key_that_escapes_pkg_dir(tmp_path):
    robot = make_demo_robot()
    with pytest.raises(ValueError, match="extra_files"):
        generate_package(
            robot, TRIVIAL_URDF_XACRO, {}, tmp_path, extra_files={"../escaped.yaml": "data: 1\n"}, dry_run=True
        )


def test_dry_run_still_rejects_bad_mesh_file_type(tmp_path):
    robot = make_demo_robot()
    with pytest.raises(TypeError):
        generate_package(robot, TRIVIAL_URDF_XACRO, {"bad.stl": "not bytes or Path"}, tmp_path, dry_run=True)


def test_dry_run_does_not_disturb_an_existing_package_directory(tmp_path):
    # A real run is idempotent via `shutil.rmtree` + rebuild; dry_run must
    # NOT do that -- an existing generated package (or, worse, anything
    # else already at that path) must be left completely untouched.
    robot = make_demo_robot()
    generate_package(robot, TRIVIAL_URDF_XACRO, {"base_link.stl": FAKE_MESH_BYTES}, tmp_path)
    pkg_dir = tmp_path / robot.name
    before = {p: p.read_bytes() for p in pkg_dir.rglob("*") if p.is_file()}

    generate_package(robot, TRIVIAL_URDF_XACRO, {"different.stl": FAKE_MESH_BYTES}, tmp_path, dry_run=True)

    after = {p: p.read_bytes() for p in pkg_dir.rglob("*") if p.is_file()}
    assert before == after


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
