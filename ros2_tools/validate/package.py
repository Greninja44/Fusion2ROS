"""ROS 2 package tree structural validation.

Pure Python standard library only — see ros2_tools/__init__.py. Checks the
on-disk shape of a generated package (package.xml, CMakeLists.txt, urdf/,
meshes/) without needing a live ROS 2 install or a colcon workspace.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List

from .urdf import extract_package_mesh_uris, resolve_mesh_uri

# ".urdf.xacro" is matched via the ".xacro" suffix check below.
_URDF_LIKE_SUFFIXES = (".urdf", ".xacro")


def _is_urdf_like(p: Path) -> bool:
    return p.is_file() and p.suffix in _URDF_LIKE_SUFFIXES


def validate_package_structure(package_dir: Path) -> List[str]:
    """Validate the structure of a generated ROS 2 package directory.
    Returns a list of problem strings (empty = valid). Never raises."""
    package_dir = Path(package_dir)
    problems: List[str] = []

    if not package_dir.is_dir():
        return [f"package directory does not exist: '{package_dir}'"]

    # --- package.xml ---
    package_xml = package_dir / "package.xml"
    pkg_name = None
    if not package_xml.is_file():
        problems.append("package.xml is missing")
    else:
        pkg_root = None
        try:
            pkg_root = ET.fromstring(package_xml.read_text())
        except OSError as exc:
            problems.append(f"could not read package.xml: {exc}")
        except ET.ParseError as exc:
            problems.append(f"package.xml is not well-formed XML: {exc}")

        if pkg_root is not None:
            name_el = pkg_root.find("name")
            if name_el is None or not (name_el.text or "").strip():
                problems.append("package.xml has no <name> element")
            else:
                pkg_name = name_el.text.strip()

            if not pkg_root.findall("buildtool_depend"):
                problems.append("package.xml has no <buildtool_depend> element")

    # --- CMakeLists.txt ---
    if not (package_dir / "CMakeLists.txt").is_file():
        problems.append("CMakeLists.txt is missing")

    # --- urdf/ directory with at least one .urdf or .xacro file ---
    urdf_dir = package_dir / "urdf"
    urdf_files: List[Path] = []
    if not urdf_dir.is_dir():
        problems.append("urdf/ directory is missing")
    else:
        urdf_files = sorted(p for p in urdf_dir.iterdir() if _is_urdf_like(p))
        if not urdf_files:
            problems.append(
                "urdf/ directory contains no .urdf or .xacro/.urdf.xacro file"
            )

    # --- every mesh referenced by those urdf files exists under meshes/ ---
    for urdf_file in urdf_files:
        try:
            urdf_root = ET.fromstring(urdf_file.read_text())
        except (OSError, ET.ParseError):
            problems.append(f"{urdf_file.name}: could not be parsed as XML")
            continue
        for uri in extract_package_mesh_uris(urdf_root):
            resolved = resolve_mesh_uri(uri, package_dir)
            if not resolved.is_file():
                problems.append(
                    f"{urdf_file.name}: mesh file not found: '{uri}' "
                    f"(resolved to '{resolved}')"
                )

    # --- package.xml <name> matches the directory basename ---
    if pkg_name is not None and pkg_name != package_dir.name:
        problems.append(
            f"package.xml <name> ('{pkg_name}') does not match "
            f"package directory name ('{package_dir.name}')"
        )

    return problems
