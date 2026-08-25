"""URDF file validation.

Pure Python standard library only (xml.etree, pathlib, shutil, subprocess —
all stdlib). Must stay importable on a plain Linux box with no ROS 2
installed at all: the only ROS-touching bit, the optional `check_urdf`
cross-check, detects the binary's absence via `shutil.which` and skips
itself rather than raising.

`validate_urdf_file` never raises for a malformed/invalid URDF — problems
are reported, not thrown. It only accumulates as many independent findings
as it can rather than stopping at the first one, in the same spirit as
robot_model.errors.ValidationError.
"""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional

_PACKAGE_URI_PREFIX = "package://"


def find_package_root(start_dir: Path) -> Optional[Path]:
    """Walk up from `start_dir` looking for a sibling `package.xml`.

    Returns the directory containing it (the package root), or None if none
    is found before reaching the filesystem root.
    """
    current = start_dir.resolve()
    for candidate in (current, *current.parents):
        if (candidate / "package.xml").is_file():
            return candidate
    return None


def extract_package_mesh_uris(root: ET.Element) -> List[str]:
    """Return every `package://...` URI found in a `<mesh filename="...">`
    under this (already-parsed) URDF/Xacro element tree."""
    uris = []
    for mesh_el in root.iter("mesh"):
        filename = mesh_el.get("filename")
        if filename and filename.startswith(_PACKAGE_URI_PREFIX):
            uris.append(filename)
    return uris


def resolve_mesh_uri(uri: str, package_root: Path) -> Path:
    """Resolve a `package://<pkgname>/<rest>` URI against `package_root`.

    We have no standalone way to locate a package other than the one that
    owns the URDF file itself (no ROS package index is available without a
    live ROS environment), so `<pkgname>` is trusted to mean "this package"
    and only `<rest>` is joined onto `package_root`.
    """
    without_prefix = uri[len(_PACKAGE_URI_PREFIX):]
    _pkgname, _sep, rest = without_prefix.partition("/")
    return package_root / rest


def _find_cycle(nodes: List[str], children_of: Dict[str, List[str]]) -> Optional[List[str]]:
    """Directed-graph cycle detection via DFS with white/gray/black coloring.

    Returns the cycle as a list of link names (first repeated at the end),
    or None if the graph is acyclic. Works regardless of how many/few root
    candidates the graph has, since it iterates every node as a potential
    DFS start.
    """
    WHITE, GRAY, BLACK = 0, 1, 2
    color = {n: WHITE for n in nodes}
    path: List[str] = []

    def visit(node: str) -> Optional[List[str]]:
        color[node] = GRAY
        path.append(node)
        for child in children_of.get(node, []):
            state = color.get(child, WHITE)
            if state == GRAY:
                idx = path.index(child)
                return path[idx:] + [child]
            if state == WHITE:
                found = visit(child)
                if found is not None:
                    return found
        path.pop()
        color[node] = BLACK
        return None

    for node in nodes:
        if color[node] == WHITE:
            found = visit(node)
            if found is not None:
                return found
    return None


def _run_check_urdf(path: Path) -> List[str]:
    """Shell out to `check_urdf` if it's on PATH; fold any error into a
    list of problem strings. Returns [] if the binary is missing, if it
    ran and found nothing wrong, or if it couldn't be run at all (in which
    case we don't want to fail validation over a tooling hiccup — the
    XML-only checks already ran)."""
    binary = shutil.which("check_urdf")
    if binary is None:
        return []
    try:
        result = subprocess.run(
            [binary, str(path)],
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.SubprocessError):
        return []
    if result.returncode == 0:
        return []
    message = (result.stderr or result.stdout or "").strip()
    if not message:
        message = f"check_urdf exited with status {result.returncode}"
    return [f"check_urdf: {message}"]


def validate_urdf_file(path: Path) -> List[str]:
    """Validate a URDF file, returning a list of problem strings (empty =
    valid). Never raises for a bad/invalid URDF."""
    path = Path(path)
    problems: List[str] = []

    try:
        text = path.read_text()
    except OSError as exc:
        return [f"could not read '{path}': {exc}"]

    try:
        root = ET.fromstring(text)
    except ET.ParseError as exc:
        return [f"not well-formed XML: {exc}"]

    if root.tag != "robot":
        problems.append(f"root element is <{root.tag}>, expected <robot>")

    # --- links: names, duplicates ---
    link_names: List[str] = []
    seen_links = set()
    for link_el in root.findall("link"):
        name = link_el.get("name")
        if not name:
            problems.append("a <link> element has no name attribute")
            continue
        if name in seen_links:
            problems.append(f"duplicate link name: '{name}'")
            continue
        seen_links.add(name)
        link_names.append(name)

    # --- joints: names, duplicates, parent/child references ---
    seen_joints = set()
    edges = []  # (parent_name, child_name)
    for joint_el in root.findall("joint"):
        jname = joint_el.get("name")
        label = jname if jname else "<unnamed joint>"

        if not jname:
            problems.append("a <joint> element has no name attribute")
        elif jname in seen_joints:
            problems.append(f"duplicate joint name: '{jname}'")
        else:
            seen_joints.add(jname)

        parent_el = joint_el.find("parent")
        child_el = joint_el.find("child")
        parent_name = parent_el.get("link") if parent_el is not None else None
        child_name = child_el.get("link") if child_el is not None else None

        if not parent_name:
            problems.append(f"joint '{label}' is missing a <parent link=\"...\"/> reference")
        elif parent_name not in seen_links:
            problems.append(
                f"joint '{label}' references unknown parent link '{parent_name}'"
            )

        if not child_name:
            problems.append(f"joint '{label}' is missing a <child link=\"...\"/> reference")
        elif child_name not in seen_links:
            problems.append(
                f"joint '{label}' references unknown child link '{child_name}'"
            )

        if parent_name and child_name:
            edges.append((parent_name, child_name))

    # --- root link: exactly one link never appears as a <child> ---
    child_names = {c for _, c in edges}
    roots = [name for name in link_names if name not in child_names]
    if len(roots) == 0:
        problems.append(
            "no root link found: every link is referenced as a <child> by some joint"
        )
    elif len(roots) > 1:
        problems.append(
            "multiple root links found (not referenced as a <child> by any joint): "
            + ", ".join(sorted(roots))
        )

    # --- cycle detection over the parent -> child graph ---
    children_of: Dict[str, List[str]] = {}
    for parent_name, child_name in edges:
        children_of.setdefault(parent_name, []).append(child_name)
    cycle = _find_cycle(link_names, children_of)
    if cycle is not None:
        problems.append("cycle detected in link tree: " + " -> ".join(cycle))

    # --- mesh package:// references resolve to real files ---
    mesh_uris = extract_package_mesh_uris(root)
    if mesh_uris:
        package_root = find_package_root(path.parent)
        if package_root is not None:
            for uri in mesh_uris:
                resolved = resolve_mesh_uri(uri, package_root)
                if not resolved.is_file():
                    problems.append(
                        f"mesh file not found: '{uri}' (resolved to '{resolved}')"
                    )
        # else: can't determine the owning package's root from here; skip
        # this specific check rather than erroring the whole validation.

    # --- optional check_urdf cross-check ---
    problems.extend(_run_check_urdf(path))

    return problems
