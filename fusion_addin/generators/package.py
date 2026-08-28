"""RobotModel + URDF + meshes -> a complete, portable ROS 2 package tree.

This module writes a ROS 2 (ament_cmake) package to disk given:

- a validated ``robot_model.Robot`` (used only for its name and root link —
  the actual URDF authoring is done by ``fusion_addin/generators/urdf.py``),
- the already-generated URDF/Xacro text,
- a mapping of mesh files to embed,

and produces ``<output_dir>/<robot.name>/`` containing ``package.xml``,
``CMakeLists.txt``, ``urdf/``, ``meshes/``, ``launch/`` and ``rviz/``.

Pure stdlib. No Fusion API calls, no ROS/rclpy imports — this must be
importable and testable under plain WSL/CI ``python3`` just like
``robot_model``, even though it lives under ``fusion_addin/`` and is only
*run* from inside Fusion's embedded interpreter in production.
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union
from xml.sax.saxutils import escape, quoteattr

from robot_model import Robot

MeshSource = Union[bytes, Path]
FileContent = Union[str, bytes]


@dataclass(frozen=True)
class ManifestEntry:
    """One file `generate_package(..., dry_run=True)` would write.

    `path` is package-relative (forward slashes, e.g. "urdf/rover.urdf.xacro"
    -- never an absolute filesystem path, so a manifest is stable/comparable
    across machines and output directories)."""

    path: str
    description: str


@dataclass(frozen=True)
class PackageManifest:
    """What `generate_package(..., dry_run=True)` returns instead of writing
    anything: the package directory a real run would create (which may not
    exist yet -- dry-run never creates it), the files that run would write
    (see ManifestEntry), and the top-level directories CMakeLists.txt would
    install (install_dirs -- see generate_package's own docstring for why
    that list is derived rather than fixed).

    Deliberately a different return type than a real run's `Path` (rather
    than, say, returning a Path that happens to not exist) -- code that
    forgets to branch on `dry_run` and tries to treat the result as a path
    fails immediately and obviously instead of silently operating on a
    directory that was never created.
    """

    pkg_dir: Path
    entries: Tuple[ManifestEntry, ...]
    install_dirs: Tuple[str, ...]

    @property
    def paths(self) -> Tuple[str, ...]:
        return tuple(entry.path for entry in self.entries)


def _ensure_within(base_dir: Path, candidate: Path, label: str) -> None:
    """Raise ValueError if `candidate` would land outside `base_dir`.

    `pathlib`'s `/` operator silently discards the left-hand side when the
    right-hand side is itself absolute (`Path("/out") / "/etc/passwd" ==
    Path("/etc/passwd")`), and a plain `..` component walks back out of
    `base_dir` just as it would in a shell -- either way, a robot name or a
    mesh_files/extra_files key that isn't a clean relative path can silently
    write (or, since `generate_package` calls `shutil.rmtree` on a
    pre-existing `pkg_dir`, delete) something far outside the package
    directory this function promises to confine itself to.
    """
    base_resolved = base_dir.resolve()
    candidate_resolved = candidate.resolve()
    try:
        candidate_resolved.relative_to(base_resolved)
    except ValueError:
        raise ValueError(
            f"{label} must resolve to a path inside {base_dir}, got {candidate!r} "
            f"(resolves to {candidate_resolved})"
        ) from None


def generate_package(
    robot: Robot,
    urdf_xacro: str,
    mesh_files: Dict[str, MeshSource],
    output_dir: Path,
    extra_files: Optional[Dict[str, FileContent]] = None,
    extra_depends: Optional[List[str]] = None,
    dry_run: bool = False,
) -> Union[Path, PackageManifest]:
    """Write a complete ROS 2 package tree for ``robot`` under ``output_dir``.

    Returns the path to the generated package directory
    (``output_dir / robot.name``).

    Idempotent: if the target package directory already exists, it is
    removed and rebuilt from scratch so no file from a previous generation
    (e.g. a mesh that is no longer referenced) lingers.

    ``extra_files`` writes additional package-relative files (e.g.
    ``"config/controllers.yaml"``, ``"launch/control.launch.py"``,
    ``"config/<robot>.srdf"``) beyond the standard urdf/meshes/launch/rviz
    set -- str values are written as text, bytes as binary. This is the
    integration point the ros2_control/Gazebo/MoveIt/Nav2 generators plug
    into: each of those produces plain XML/YAML/launch text, and this is
    where it actually lands on disk as part of one coherent package.

    ``extra_depends`` (default None) adds extra ``<depend>`` lines to
    ``package.xml`` beyond the fixed base set (``urdf``, ``xacro``,
    ``robot_state_publisher``, etc.) -- real gap found while adding Gazebo
    support: this list was previously hardcoded regardless of which
    ``include_*`` flags were used, so a Gazebo/Nav2/MoveIt-enabled package's
    ``package.xml`` never declared ``ros_gz_sim``/``ros_gz_bridge``/Nav2's
    own packages as dependencies at all -- wrong metadata for anything that
    resolves dependencies from it (e.g. ``rosdep``), even though colcon
    itself doesn't enforce this for a plain ``ament_cmake`` package with no
    compiled code. Duplicates against the base set are dropped, preserving
    ``extra_depends``' order for anything new.

    ``dry_run`` (default False): when True, no directory is created and no
    file is written -- ``output_dir``/``pkg_dir`` are never touched at all,
    not even ``mkdir``'d. Every other computation that a real run performs
    before it starts writing still happens: the same ``Robot.name`` ->
    ``pkg_dir`` path-escape guard, and the same mesh_files/extra_files
    key-escape and value-type checks that ``_write_meshes``/
    ``_write_extra_files`` perform (via the exact same ``_ensure_within``
    calls and dest-path helpers a real run uses -- not a separate,
    reimplemented check that could silently drift out of sync with them).
    Returns a `PackageManifest` describing what a real run (same arguments,
    ``dry_run=False``) would write, instead of the `Path` a real run
    returns -- see `PackageManifest`'s docstring for why that's a distinct
    type rather than a `Path` that happens not to exist yet.
    """
    output_dir = Path(output_dir)
    pkg_dir = output_dir / robot.name
    _ensure_within(output_dir, pkg_dir, f"Robot.name {robot.name!r}")

    mesh_files = mesh_files or {}
    extra_files = extra_files or {}

    if dry_run:
        return _plan_package(robot, pkg_dir, mesh_files, extra_files)

    if pkg_dir.exists():
        shutil.rmtree(pkg_dir)

    urdf_dir = pkg_dir / "urdf"
    meshes_dir = pkg_dir / "meshes"
    launch_dir = pkg_dir / "launch"
    rviz_dir = pkg_dir / "rviz"
    config_dir = pkg_dir / "config"
    for d in (pkg_dir, urdf_dir, meshes_dir, launch_dir, rviz_dir, config_dir):
        d.mkdir(parents=True, exist_ok=True)

    _write_package_xml(robot, pkg_dir, extra_depends or [])
    _write_urdf_xacro(robot, urdf_dir, urdf_xacro)
    _write_meshes(mesh_files, meshes_dir)
    _write_launch(robot, launch_dir)
    _write_rviz(robot, rviz_dir)
    _write_extra_files(extra_files, pkg_dir)

    # Written last: extra_files can introduce entirely new top-level dirs
    # (e.g. gazebo.py's "worlds/") beyond the fixed urdf/meshes/launch/rviz/
    # config set -- discovering what's actually on disk here, rather than
    # hardcoding a directory list, is what makes CMakeLists actually install
    # them. (A real bug this exact gap caused: a generated worlds/empty.sdf
    # sat in the package but was never installed, so `gz sim` reported
    # "Unable to find or download file ... worlds/empty.sdf" at launch even
    # though colcon build had reported success.)
    install_dirs = sorted(p.name for p in pkg_dir.iterdir() if p.is_dir())
    _write_cmakelists(robot, pkg_dir, install_dirs)

    return pkg_dir


# Fixed top-level directories every real run creates (and therefore installs
# via CMakeLists) regardless of content -- see generate_package's `for d in
# (...)` mkdir loop. Shared with _plan_package so a dry run's install_dirs
# matches a real run's without hardcoding this list a second time.
_FIXED_INSTALL_DIRS = ("config", "launch", "meshes", "rviz", "urdf")


def _plan_package(
    robot: Robot,
    pkg_dir: Path,
    mesh_files: Dict[str, MeshSource],
    extra_files: Dict[str, FileContent],
) -> PackageManifest:
    """dry_run=True's implementation: validates exactly what a real run
    would validate (via the same _mesh_file_dest/_extra_file_dest helpers
    _write_meshes/_write_extra_files themselves call), then describes what
    would be written without writing it. Never creates or modifies
    anything on disk."""
    entries: List[ManifestEntry] = [
        ManifestEntry("package.xml", "ROS 2 package manifest (name, dependencies, maintainer)"),
        ManifestEntry(f"urdf/{robot.name}.urdf.xacro", "Robot description (URDF/Xacro)"),
    ]

    meshes_dir = pkg_dir / "meshes"
    for rel_name, source in mesh_files.items():
        _mesh_file_dest(meshes_dir, rel_name)  # validates escape; result unused, dry run copies nothing
        if not isinstance(source, (Path, bytes, bytearray)):
            raise TypeError(
                f"mesh_files[{rel_name!r}] must be bytes or a pathlib.Path, got {type(source).__name__}"
            )
        entries.append(ManifestEntry(f"meshes/{rel_name}", "Mesh file"))

    entries.append(ManifestEntry("launch/display.launch.py", "RViz display launch file"))
    entries.append(ManifestEntry(f"rviz/{robot.name}.rviz", "RViz configuration"))

    extra_top_dirs = set()
    for rel_path, content in extra_files.items():
        _extra_file_dest(pkg_dir, rel_path)  # validates escape; result unused, dry run writes nothing
        if not isinstance(content, (str, bytes, bytearray)):
            raise TypeError(f"extra_files[{rel_path!r}] must be str or bytes, got {type(content).__name__}")
        parts = Path(rel_path).parts
        if len(parts) > 1:
            extra_top_dirs.add(parts[0])
        kind = "text" if isinstance(content, str) else "binary"
        entries.append(ManifestEntry(rel_path.replace("\\", "/"), f"Generated output ({kind})"))

    install_dirs = tuple(sorted(set(_FIXED_INSTALL_DIRS) | extra_top_dirs))
    entries.append(
        ManifestEntry("CMakeLists.txt", f"Build/install rules (installs: {', '.join(install_dirs)})")
    )

    return PackageManifest(pkg_dir=pkg_dir, entries=tuple(entries), install_dirs=install_dirs)


def _extra_file_dest(pkg_dir: Path, rel_path: str) -> Path:
    """Computes (and, via `_ensure_within`, validates) where `extra_files[rel_path]`
    lands -- factored out of `_write_extra_files` so `_plan_package`'s dry-run
    path performs the exact same escape check against the exact same
    destination a real write would use, rather than a separate check that
    could drift out of sync with it."""
    dest = pkg_dir / rel_path
    _ensure_within(pkg_dir, dest, f"extra_files key {rel_path!r}")
    return dest


def _write_extra_files(extra_files: Dict[str, FileContent], pkg_dir: Path) -> None:
    for rel_path, content in extra_files.items():
        dest = _extra_file_dest(pkg_dir, rel_path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(content, str):
            dest.write_text(content, encoding="utf-8")
        elif isinstance(content, (bytes, bytearray)):
            dest.write_bytes(bytes(content))
        else:
            raise TypeError(f"extra_files[{rel_path!r}] must be str or bytes, got {type(content).__name__}")


def _write_package_xml(robot: Robot, pkg_dir: Path, extra_depends: List[str]) -> None:
    meta = robot.metadata or {}
    version = meta.get("version", "0.0.1")
    description = meta.get(
        "description", f"ROS 2 package for the {robot.name} robot, generated by Fusion2ROS."
    )
    maintainer_name = meta.get("maintainer_name", "TODO")
    maintainer_email = meta.get("maintainer_email", "TODO@TODO.TODO")
    license_name = meta.get("license", "TODO: License declaration")

    depends = [
        "urdf",
        "xacro",
        "robot_state_publisher",
        "joint_state_publisher",
        "joint_state_publisher_gui",
        "rviz2",
        "launch",
        "launch_ros",
    ]
    for dep in extra_depends:
        if dep not in depends:
            depends.append(dep)
    # robot.name/version/description/maintainer_*/license_name are free-form
    # user/metadata text (a Fusion project description, an author's name,
    # ...), not markup -- XML-escape every value landing in element text or
    # an attribute so a name like "Arm & Gripper" or a maintainer "O'Brien"
    # can't produce malformed XML (or, worse, inject a sibling element).
    depend_lines = "\n".join(f"  <depend>{escape(d)}</depend>" for d in depends)

    content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{escape(str(robot.name))}</name>
  <version>{escape(str(version))}</version>
  <description>{escape(str(description))}</description>
  <maintainer email={quoteattr(str(maintainer_email))}>{escape(str(maintainer_name))}</maintainer>
  <license>{escape(str(license_name))}</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

{depend_lines}

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
    (pkg_dir / "package.xml").write_text(content, encoding="utf-8")


def _write_cmakelists(robot: Robot, pkg_dir: Path, install_dirs) -> None:
    dir_lines = "\n".join(f"    {d}" for d in install_dirs)
    content = f"""cmake_minimum_required(VERSION 3.10)
project({robot.name})

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
{dir_lines}
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()
"""
    (pkg_dir / "CMakeLists.txt").write_text(content, encoding="utf-8")


def _write_urdf_xacro(robot: Robot, urdf_dir: Path, urdf_xacro: str) -> None:
    (urdf_dir / f"{robot.name}.urdf.xacro").write_text(urdf_xacro, encoding="utf-8")


def _mesh_file_dest(meshes_dir: Path, rel_name: str) -> Path:
    """Computes (and, via `_ensure_within`, validates) where `mesh_files[rel_name]`
    lands -- see `_extra_file_dest`'s docstring for why this is factored out
    the same way."""
    dest = meshes_dir / rel_name
    _ensure_within(meshes_dir, dest, f"mesh_files key {rel_name!r}")
    return dest


def _write_meshes(mesh_files: Dict[str, MeshSource], meshes_dir: Path) -> None:
    for rel_name, source in mesh_files.items():
        dest = _mesh_file_dest(meshes_dir, rel_name)
        dest.parent.mkdir(parents=True, exist_ok=True)
        if isinstance(source, Path):
            shutil.copyfile(source, dest)
        elif isinstance(source, (bytes, bytearray)):
            dest.write_bytes(bytes(source))
        else:
            raise TypeError(
                f"mesh_files[{rel_name!r}] must be bytes or a pathlib.Path, got {type(source).__name__}"
            )


def _write_launch(robot: Robot, launch_dir: Path) -> None:
    content = f'''"""Launch file for displaying the "{robot.name}" robot in RViz2.

Auto-generated by Fusion2ROS (fusion_addin/generators/package.py).
Regenerate from Fusion rather than editing by hand.
"""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# robot.name is embedded via repr() rather than naive f-string quoting: it is
# free-form text (no character restrictions in robot_model.schema), and a
# name containing a `"` -- e.g. `weird"name` -- inside a naively-quoted
# f-string produces a Python file that fails to parse at all. repr() always
# emits a syntactically valid string literal for any str value.
PACKAGE_NAME = {robot.name!r}
URDF_XACRO_FILE = {(robot.name + ".urdf.xacro")!r}
RVIZ_CONFIG_FILE = {(robot.name + ".rviz")!r}


def generate_launch_description():
    pkg_share = FindPackageShare(PACKAGE_NAME)

    urdf_xacro_path = PathJoinSubstitution([pkg_share, "urdf", URDF_XACRO_FILE])
    rviz_config_path = PathJoinSubstitution([pkg_share, "rviz", RVIZ_CONFIG_FILE])

    # Processed via the `xacro` command-line filter at launch time rather
    # than the `xacro` Python module, so this launch file has no import-time
    # dependency on xacro being importable in the launching interpreter.
    robot_description = {{
        "robot_description": Command(["xacro ", urdf_xacro_path])
    }}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
'''
    (launch_dir / "display.launch.py").write_text(content, encoding="utf-8")


def _write_rviz(robot: Robot, rviz_dir: Path) -> None:
    root = robot.root_link()
    fixed_frame = root.name if root is not None else "base_link"

    content = f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {fixed_frame}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Pitch: 0.5
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Height: 800
  Width: 1200
  X: 0
  Y: 0
"""
    (rviz_dir / f"{robot.name}.rviz").write_text(content, encoding="utf-8")
