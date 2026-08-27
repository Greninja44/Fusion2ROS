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

import json
from pathlib import Path
from typing import Dict, Optional

_last_package_dir: Optional[Path] = None
_last_robot_name: Optional[str] = None
_last_wsl_ros_ws_src: Optional[str] = None

# Real gap found live: reopening "Generate ROS 2 Package" -- even within the
# same Fusion session, let alone after a restart -- lost every checkbox and
# drivetrain field the user had typed in, since only robot_name/output_dir
# had any memory at all (via the design's own default name / a fixed
# default path). Persisted to a small JSON file (not just an in-process
# global, unlike the rest of this module) specifically so it survives an
# actual Fusion restart too -- keyed by robot name, since that's the one
# identifier a user reopening the same design would naturally expect their
# settings to come back under.
_DIALOG_STATE_PATH = Path.home() / "Fusion2ROS" / ".generate_dialog_state.json"


def save_generate_dialog_state(robot_name: str, fields: Dict[str, object]) -> None:
    """Persist `fields` (a flat {input_id: value} dict, JSON-serializable
    values only) under `robot_name`, merging into whatever's already on
    disk for OTHER robot names (never clobbers a different robot's saved
    state). Best-effort: swallows any I/O/JSON error rather than failing
    the generation that already succeeded by the time this is called."""
    try:
        all_state = _read_dialog_state_file()
        all_state[robot_name] = fields
        _DIALOG_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
        _DIALOG_STATE_PATH.write_text(json.dumps(all_state, indent=2), encoding="utf-8")
    except OSError:
        pass


def load_generate_dialog_state(robot_name: str) -> Dict[str, object]:
    """Returns the {input_id: value} dict previously saved for
    `robot_name`, or {} if none was ever saved (including if the file
    itself doesn't exist yet, or is corrupt) -- callers apply this as
    defaults on top of the dialog's normal built-in defaults, so an empty
    dict here is always a safe, harmless no-op."""
    return _read_dialog_state_file().get(robot_name, {})


def _read_dialog_state_file() -> Dict[str, Dict[str, object]]:
    try:
        return json.loads(_DIALOG_STATE_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


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
