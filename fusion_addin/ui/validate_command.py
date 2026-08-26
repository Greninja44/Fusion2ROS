"""Fusion 360 command handler for "Validate ROS 2 Package".

*** UNVERIFIED against a live Fusion process *** -- same caveat as
command.py; see that file's docstring for the general pattern and citation
style. This is the one command in this package that deliberately crosses
docs/ARCHITECTURE.md's stated "nothing under fusion_addin/ imports anything
under ros2_tools/" layering rule -- that rule predates this task and was
written for the extraction/generation pipeline (fusion_addin/extraction,
fusion_addin/generators), which does stay strictly Fusion-only. "Validate"
is explicitly meant to run the WSL/Linux-side validation tooling
(ros2_tools.validate) from the Windows-side UI, which only works at all
because ros2_tools.validate.{package,urdf} are pure-stdlib Python with zero
adsk and zero live-ROS-environment dependency (see their own module
docstrings) -- they run identically under Windows Python and WSL Python.
Flagging this explicitly rather than silently breaking the documented rule.

Thin by design: gathers one input (a package directory, defaulting to the
most recently generated package -- see ui/state.py), calls straight into
ros2_tools.validate, and formats the result into a message box. No logic of
substance lives here.
"""

from __future__ import annotations

import traceback
from pathlib import Path

import adsk.core

from ros2_tools.validate.package import validate_package_structure
from ros2_tools.validate.urdf import validate_urdf_file
from . import state

CMD_ID = "fusion2ros_validate"
CMD_NAME = "Validate ROS 2 Package"
CMD_DESCRIPTION = "Validate a generated ROS 2 package's structure and URDF against ros2_tools.validate."
PANEL_ID = "SolidCreatePanel"

_handlers = []


def register(ui: "adsk.core.UserInterface") -> None:
    existing = ui.commandDefinitions.itemById(CMD_ID)
    if existing:
        existing.deleteMe()

    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESCRIPTION)
    on_created = ValidateCommandCreatedHandler()
    cmd_def.commandCreated.add(on_created)
    _handlers.append(on_created)

    panel = ui.allToolbarPanels.itemById(PANEL_ID)
    if panel and not panel.controls.itemById(CMD_ID):
        panel.controls.addCommand(cmd_def)


def unregister(ui: "adsk.core.UserInterface") -> None:
    panel = ui.allToolbarPanels.itemById(PANEL_ID)
    if panel:
        control = panel.controls.itemById(CMD_ID)
        if control:
            control.deleteMe()
    cmd_def = ui.commandDefinitions.itemById(CMD_ID)
    if cmd_def:
        cmd_def.deleteMe()
    _handlers.clear()


def _find_urdf_like_files(package_dir: Path):
    urdf_dir = package_dir / "urdf"
    if not urdf_dir.is_dir():
        return []
    return sorted(p for p in urdf_dir.iterdir() if p.is_file() and p.suffix in (".urdf", ".xacro"))


class ValidateCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args: "adsk.core.CommandCreatedEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            last_dir = state.get_last_package_dir()
            default_dir = str(last_dir) if last_dir else str(Path.home() / "Fusion2ROS" / "output")

            inputs.addStringValueInput(
                "package_dir",
                "Package directory",
                default_dir,
            )

            on_execute = ValidateCommandExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
        except Exception:
            ui.messageBox(f"Fusion2ROS: failed to create command:\n{traceback.format_exc()}")


class ValidateCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args: "adsk.core.CommandEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            package_dir = Path(inputs.itemById("package_dir").value.strip())

            problems = validate_package_structure(package_dir)

            for urdf_file in _find_urdf_like_files(package_dir):
                problems.extend(f"{urdf_file.name}: {p}" for p in validate_urdf_file(urdf_file))

            if problems:
                ui.messageBox(
                    "Fusion2ROS: validation found problems in\n"
                    f"{package_dir}:\n\n" + "\n".join(f"- {p}" for p in problems)
                )
            else:
                ui.messageBox(f"Fusion2ROS: validation passed -- no problems found in\n{package_dir}")
        except Exception:
            ui.messageBox(f"Fusion2ROS: validation failed:\n{traceback.format_exc()}")
