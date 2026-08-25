"""Tests for bridge.wsl_side (build.py). Covers ONLY the WSL-side half of
the bridge -- bridge.windows is untestable from inside WSL (see its module
docstrings) and deliberately has no tests here.

Must run with plain `python3 -m pytest` like the rest of tests/. The
colcon_build integration tests are real (no mocking): they build a genuine
throwaway `ament_cmake` package under `tmp_path` and check the actual
result. They never touch ~/ros2_ws -- every workspace here is created fresh
under pytest's `tmp_path` fixture and thrown away afterward.
"""

import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from bridge.wsl_side.build import BuildResult, colcon_build, copy_package_to_workspace

COLCON_AVAILABLE = shutil.which("colcon") is not None
ROS_SETUP = Path("/opt/ros/lyrical/setup.bash")
ROS_SETUP_AVAILABLE = ROS_SETUP.exists()

requires_colcon = pytest.mark.skipif(
    not (COLCON_AVAILABLE and ROS_SETUP_AVAILABLE),
    reason="colcon and/or /opt/ros/lyrical/setup.bash not found on this machine",
)


# --- copy_package_to_workspace -------------------------------------------


def make_fake_package(root: Path, name: str = "fake_robot") -> Path:
    """Hand-build a minimal fake generated ROS 2 package dir -- deliberately
    not depending on fusion_addin/generators/package.py (another agent's
    code); just enough files to exercise copy behavior."""
    pkg = root / name
    pkg.mkdir(parents=True)
    (pkg / "package.xml").write_text(
        '<?xml version="1.0"?>\n<package format="3"><name>%s</name></package>\n' % name
    )
    (pkg / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.8)\nproject(%s)\n" % name)
    (pkg / "urdf").mkdir()
    (pkg / "urdf" / "robot.urdf").write_text("<robot name=\"%s\"></robot>\n" % name)
    return pkg


def test_copy_package_to_workspace_copies_files(tmp_path):
    src_root = tmp_path / "output"
    ros_ws_src = tmp_path / "ros_ws" / "src"
    package_dir = make_fake_package(src_root)

    dest = copy_package_to_workspace(package_dir, ros_ws_src)

    assert dest == ros_ws_src / "fake_robot"
    assert dest.is_dir()
    assert (dest / "package.xml").read_text() == (package_dir / "package.xml").read_text()
    assert (dest / "CMakeLists.txt").exists()
    assert (dest / "urdf" / "robot.urdf").exists()
    # Source is untouched.
    assert package_dir.exists()


def test_copy_package_to_workspace_replaces_stale_files(tmp_path):
    src_root = tmp_path / "output"
    ros_ws_src = tmp_path / "ros_ws" / "src"
    package_dir = make_fake_package(src_root)

    dest = copy_package_to_workspace(package_dir, ros_ws_src)
    assert (dest / "urdf" / "robot.urdf").exists()

    # Simulate a re-generation of the package where a file was removed and
    # another added.
    (package_dir / "urdf" / "robot.urdf").unlink()
    (package_dir / "meshes").mkdir()
    (package_dir / "meshes" / "base_link.stl").write_text("fake stl contents")

    dest2 = copy_package_to_workspace(package_dir, ros_ws_src)

    assert dest2 == dest
    assert not (dest2 / "urdf" / "robot.urdf").exists(), "stale file from previous copy leaked into destination"
    assert (dest2 / "meshes" / "base_link.stl").read_text() == "fake stl contents"


def test_copy_package_to_workspace_preserves_symlinks_without_following(tmp_path):
    src_root = tmp_path / "output"
    ros_ws_src = tmp_path / "ros_ws" / "src"
    package_dir = make_fake_package(src_root)

    # A symlink inside the package pointing at something outside it -- must
    # be copied as a symlink, not dereferenced and its target's content
    # copied in.
    outside_file = tmp_path / "outside_secret.txt"
    outside_file.write_text("outside content")
    (package_dir / "linked.txt").symlink_to(outside_file)

    dest = copy_package_to_workspace(package_dir, ros_ws_src)

    copied_link = dest / "linked.txt"
    assert copied_link.is_symlink()
    assert Path(shutil.os.readlink(copied_link)) == outside_file


def test_copy_package_to_workspace_rejects_missing_source(tmp_path):
    with pytest.raises(NotADirectoryError):
        copy_package_to_workspace(tmp_path / "does_not_exist", tmp_path / "ros_ws" / "src")


def test_copy_package_to_workspace_only_touches_its_own_destination_dir(tmp_path):
    src_root = tmp_path / "output"
    ros_ws_src = tmp_path / "ros_ws" / "src"
    ros_ws_src.mkdir(parents=True)
    unrelated = ros_ws_src / "unrelated_package"
    unrelated.mkdir()
    (unrelated / "marker.txt").write_text("do not touch me")

    package_dir = make_fake_package(src_root)
    copy_package_to_workspace(package_dir, ros_ws_src)

    assert (unrelated / "marker.txt").read_text() == "do not touch me"


# --- colcon_build ----------------------------------------------------------


def make_ament_cmake_package(ros_ws: Path, name: str, valid: bool) -> None:
    pkg_dir = ros_ws / "src" / name
    pkg_dir.mkdir(parents=True)
    (pkg_dir / "package.xml").write_text(
        f"""<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.0.1</version>
  <description>Throwaway test package</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
</package>
"""
    )
    if valid:
        cmake = f"""cmake_minimum_required(VERSION 3.8)
project({name})
find_package(ament_cmake REQUIRED)
ament_package()
"""
    else:
        # Deliberately broken: an unknown CMake command instead of the
        # required ament_package() call.
        cmake = f"""cmake_minimum_required(VERSION 3.8)
project({name})
find_package(ament_cmake REQUIRED)
this_is_not_a_real_cmake_command(oops)
"""
    (pkg_dir / "CMakeLists.txt").write_text(cmake)


@requires_colcon
def test_colcon_build_succeeds_on_valid_package(tmp_path):
    ros_ws = tmp_path / "throwaway_ws"
    make_ament_cmake_package(ros_ws, "fake_pkg_ok", valid=True)

    result = colcon_build(ros_ws, "fake_pkg_ok", ros_setup=ROS_SETUP, timeout=120)

    assert isinstance(result, BuildResult)
    assert result.returncode == 0
    assert result.success is True


@requires_colcon
def test_colcon_build_fails_on_broken_package(tmp_path):
    ros_ws = tmp_path / "throwaway_ws"
    make_ament_cmake_package(ros_ws, "fake_pkg_bad", valid=False)

    result = colcon_build(ros_ws, "fake_pkg_bad", ros_setup=ROS_SETUP, timeout=120)

    assert isinstance(result, BuildResult)
    assert result.success is False
    assert result.returncode != 0
    assert result.stderr.strip() != ""


@requires_colcon
def test_colcon_build_uses_packages_select_scoping(tmp_path):
    """A second, unrelated (and broken) package must not affect the
    outcome for the one we actually asked to build -- proves
    --packages-select is really scoping the build."""
    ros_ws = tmp_path / "throwaway_ws"
    make_ament_cmake_package(ros_ws, "fake_pkg_ok", valid=True)
    make_ament_cmake_package(ros_ws, "fake_pkg_bad", valid=False)

    result = colcon_build(ros_ws, "fake_pkg_ok", ros_setup=ROS_SETUP, timeout=120)

    assert result.success is True
    assert result.returncode == 0
