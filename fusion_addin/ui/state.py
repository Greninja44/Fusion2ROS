"""Tiny, Fusion-symbol-free shared session state for the Fusion UI commands.

Not business logic (nothing here belongs in app.py) -- just a place for
"Generate ROS 2 Package" to remember the most-recently-generated package
directory / robot name so "Validate", "Build in WSL", and "Launch RViz"
(each a separate command definition, registered from separate sibling
modules in this package -- see command.py, validate_command.py,
build_command.py, launch_command.py) can default to it instead of forcing
the user to retype/re-browse a path every time. Deliberately process-local
(module-level globals, reset when the add-in reloads) -- there is no
requirement anywhere in the project brief for this to survive across a
Fusion session restart.

No `adsk` import anywhere in this file, so -- unlike the rest of
fusion_addin/ui/ -- it's fully unit-testable without a running Fusion
process.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

_last_package_dir: Optional[Path] = None
_last_robot_name: Optional[str] = None
_last_wsl_ros_ws_src: Optional[str] = None


def set_last_generated(package_dir: Path, robot_name: str) -> None:
    """Called by command.py's GenerateCommandExecuteHandler after a
    successful `Generate ROS 2 Package` run."""
    global _last_package_dir, _last_robot_name
    _last_package_dir = Path(package_dir)
    _last_robot_name = robot_name


def get_last_package_dir() -> Optional[Path]:
    return _last_package_dir


def get_last_robot_name() -> Optional[str]:
    return _last_robot_name


def set_last_wsl_ros_ws_src(wsl_ros_ws_src: str) -> None:
    """Called by build_command.py after a "Build in WSL" attempt, so
    launch_command.py's "Launch RViz" (which needs the same workspace path
    to source its install/setup.bash) can default to the same value."""
    global _last_wsl_ros_ws_src
    _last_wsl_ros_ws_src = wsl_ros_ws_src


def get_last_wsl_ros_ws_src() -> Optional[str]:
    return _last_wsl_ros_ws_src
