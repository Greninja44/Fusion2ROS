"""Tests for ros2_tools.validate. Must run with plain `python3 -m pytest` —
these hit the pure-stdlib validation logic directly. The one test that
shells out to `check_urdf` skips itself gracefully if the binary isn't on
PATH, since ros2_tools.validate must also work on a box with no ROS 2
installed at all.
"""

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_tools.validate import validate_package_structure, validate_urdf_file


VALID_URDF = """<?xml version="1.0"?>
<robot name="valid_bot">
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="2.0"/>
  </joint>
</robot>
"""

DANGLING_CHILD_URDF = """<?xml version="1.0"?>
<robot name="dangling_bot">
  <link name="base_link"/>
  <joint name="joint1" type="fixed">
    <parent link="base_link"/>
    <child link="nonexistent"/>
  </joint>
</robot>
"""

CYCLE_URDF = """<?xml version="1.0"?>
<robot name="cyclic_bot">
  <link name="a"/>
  <link name="b"/>
  <link name="c"/>
  <joint name="j1" type="fixed">
    <parent link="a"/>
    <child link="b"/>
  </joint>
  <joint name="j2" type="fixed">
    <parent link="b"/>
    <child link="c"/>
  </joint>
  <joint name="j3" type="fixed">
    <parent link="c"/>
    <child link="a"/>
  </joint>
</robot>
"""

DUPLICATE_NAMES_URDF = """<?xml version="1.0"?>
<robot name="dup_bot">
  <link name="base_link"/>
  <link name="base_link"/>
  <link name="link1"/>
  <joint name="joint1" type="fixed">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
  <joint name="joint1" type="fixed">
    <parent link="base_link"/>
    <child link="link1"/>
  </joint>
</robot>
"""

MISSING_MESH_URDF = """<?xml version="1.0"?>
<robot name="mesh_bot">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://fake_pkg/meshes/foo.stl"/>
      </geometry>
    </visual>
  </link>
</robot>
"""

MINIMAL_PACKAGE_XML = """<?xml version="1.0"?>
<package format="3">
  <name>fake_pkg</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
"""


def test_valid_urdf_has_no_problems(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(VALID_URDF)

    assert validate_urdf_file(urdf_path) == []


def test_malformed_xml_reported(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text("<robot><link name=\"a\"></robot>")  # mismatched tag

    problems = validate_urdf_file(urdf_path)
    assert problems
    assert any("well-formed" in p for p in problems)


def test_wrong_root_tag_reported(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text('<not_a_robot name="x"/>')

    problems = validate_urdf_file(urdf_path)
    assert any("expected <robot>" in p for p in problems)


def test_dangling_child_reference_reported(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(DANGLING_CHILD_URDF)

    problems = validate_urdf_file(urdf_path)
    assert any(
        "unknown child link 'nonexistent'" in p for p in problems
    ), problems


def test_cycle_reported(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(CYCLE_URDF)

    problems = validate_urdf_file(urdf_path)
    assert any("cycle detected" in p for p in problems), problems


def test_duplicate_names_reported(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(DUPLICATE_NAMES_URDF)

    problems = validate_urdf_file(urdf_path)
    assert any("duplicate link name: 'base_link'" in p for p in problems), problems
    assert any("duplicate joint name: 'joint1'" in p for p in problems), problems


def test_missing_mesh_file_reported(tmp_path):
    pkg_dir = tmp_path / "fake_pkg"
    urdf_dir = pkg_dir / "urdf"
    urdf_dir.mkdir(parents=True)
    (pkg_dir / "package.xml").write_text(MINIMAL_PACKAGE_XML)
    urdf_path = urdf_dir / "robot.urdf"
    urdf_path.write_text(MISSING_MESH_URDF)
    # Deliberately do NOT create meshes/foo.stl.

    problems = validate_urdf_file(urdf_path)
    assert any(
        "mesh file not found" in p and "foo.stl" in p for p in problems
    ), problems


def test_mesh_check_skipped_without_package_root(tmp_path):
    # No sibling package.xml anywhere above this file -> the mesh check
    # should be skipped rather than erroring the whole validation.
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(MISSING_MESH_URDF)

    problems = validate_urdf_file(urdf_path)
    assert not any("mesh file not found" in p for p in problems), problems


@pytest.mark.skipif(
    shutil.which("check_urdf") is None, reason="check_urdf not on PATH"
)
def test_check_urdf_agrees_on_valid_fixture(tmp_path):
    urdf_path = tmp_path / "robot.urdf"
    urdf_path.write_text(VALID_URDF)

    # Our XML-only checks report nothing wrong...
    assert validate_urdf_file(urdf_path) == []

    # ...and check_urdf independently accepts the same file.
    result = subprocess.run(
        [shutil.which("check_urdf"), str(urdf_path)],
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stderr


# --- validate_package_structure ---


def _write_valid_package(pkg_dir: Path, pkg_name: str = None) -> None:
    pkg_dir.mkdir(parents=True)
    (pkg_dir / "urdf").mkdir()
    (pkg_dir / "meshes").mkdir()

    name = pkg_name if pkg_name is not None else pkg_dir.name
    package_xml = f"""<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
"""
    (pkg_dir / "package.xml").write_text(package_xml)
    (pkg_dir / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.8)\n")

    robot_urdf = f"""<?xml version="1.0"?>
<robot name="{name}">
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="package://{name}/meshes/base.stl"/>
      </geometry>
    </visual>
  </link>
</robot>
"""
    (pkg_dir / "urdf" / "robot.urdf").write_text(robot_urdf)
    (pkg_dir / "meshes" / "base.stl").write_text("solid base endsolid base\n")


def test_valid_package_structure_has_no_problems(tmp_path):
    pkg_dir = tmp_path / "my_robot"
    _write_valid_package(pkg_dir)

    assert validate_package_structure(pkg_dir) == []


def test_missing_package_xml_reported(tmp_path):
    pkg_dir = tmp_path / "my_robot"
    _write_valid_package(pkg_dir)
    (pkg_dir / "package.xml").unlink()

    problems = validate_package_structure(pkg_dir)
    assert any("package.xml is missing" in p for p in problems), problems


def test_name_mismatch_reported(tmp_path):
    pkg_dir = tmp_path / "my_robot"
    _write_valid_package(pkg_dir, pkg_name="a_totally_different_name")

    problems = validate_package_structure(pkg_dir)
    assert any(
        "does not match package directory name" in p for p in problems
    ), problems


def test_missing_urdf_dir_reported(tmp_path):
    pkg_dir = tmp_path / "my_robot"
    _write_valid_package(pkg_dir)
    shutil.rmtree(pkg_dir / "urdf")

    problems = validate_package_structure(pkg_dir)
    assert any("urdf/ directory is missing" in p for p in problems), problems


def test_package_missing_mesh_reported(tmp_path):
    pkg_dir = tmp_path / "my_robot"
    _write_valid_package(pkg_dir)
    (pkg_dir / "meshes" / "base.stl").unlink()

    problems = validate_package_structure(pkg_dir)
    assert any(
        "mesh file not found" in p and "base.stl" in p for p in problems
    ), problems


def test_missing_buildtool_depend_reported(tmp_path):
    pkg_dir = tmp_path / "my_robot"
    _write_valid_package(pkg_dir)
    package_xml = """<?xml version="1.0"?>
<package format="3">
  <name>my_robot</name>
</package>
"""
    (pkg_dir / "package.xml").write_text(package_xml)

    problems = validate_package_structure(pkg_dir)
    assert any("buildtool_depend" in p for p in problems), problems
