"""Fusion 360 command handler for "Check WSL Environment".

*** UNVERIFIED against a live Fusion process *** -- same caveat as
command.py/build_command.py; see command.py's docstring for the general
pattern and citation style.

Motivation: real bugs found live this session (see
bridge/windows/doctor.py's module docstring) only ever surfaced as a
cryptic mid-build `colcon build failed` message box from "Build in WSL" (or
would surface the same way from the one-click "Also build in WSL" option on
"Generate ROS 2 Package"), with no indication of WHICH part of the
environment was actually broken. This command runs the same
bridge.windows.doctor checks standalone, any time, so a user can check
"is my WSL/ROS 2 setup even usable" before attempting a build at all --
`command.py`'s own execute handler also runs these same checks
automatically before its optional "Also build in WSL" step, so this command
mainly exists for on-demand troubleshooting.

No RESOURCE_FOLDER of its own exists yet (no new icon assets were created
for this task) -- reuses validate_command.py's icon set as a reasonable
stand-in ("check something and report back" is the same visual idea);
swap in a dedicated set later if desired.
"""

from __future__ import annotations

import traceback
from pathlib import Path

import adsk.core

from bridge.windows.doctor import format_report, run_environment_checks
from bridge.windows.invoke import DEFAULT_DISTRO, DEFAULT_ROS_SETUP, DEFAULT_WSL_ROS_WS_SRC
from . import state

CMD_ID = "fusion2ros_doctor"
CMD_NAME = "Check WSL Environment"
CMD_DESCRIPTION = "Run pre-flight checks against the WSL/ROS 2 environment (distro, colcon, catkin_pkg, workspace)."
PANEL_ID = "SolidCreatePanel"
RESOURCE_FOLDER = str(Path(__file__).parent / "resources" / "validate")

_handlers = []


def register(ui: "adsk.core.UserInterface") -> None:
    existing = ui.commandDefinitions.itemById(CMD_ID)
    if existing:
        existing.deleteMe()

    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESCRIPTION, RESOURCE_FOLDER)
    on_created = DoctorCommandCreatedHandler()
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


class DoctorCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args: "adsk.core.CommandCreatedEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            inputs = cmd.commandInputs
            default_ws_src = state.get_last_wsl_ros_ws_src() or DEFAULT_WSL_ROS_WS_SRC
            inputs.addStringValueInput("wsl_ros_ws_src", "WSL colcon workspace src/ (e.g. ~/ros2_ws/src)", default_ws_src)
            inputs.addBoolValueInput("check_gazebo", "Also check for Gazebo (gz)", True, "", False)

            on_execute = DoctorCommandExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
        except Exception:
            ui.messageBox(f"Fusion2ROS: failed to create command:\n{traceback.format_exc()}")


class DoctorCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args: "adsk.core.CommandEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            wsl_ros_ws_src = inputs.itemById("wsl_ros_ws_src").value.strip() or DEFAULT_WSL_ROS_WS_SRC
            check_gazebo = inputs.itemById("check_gazebo").value

            progress_dialog = ui.createProgressDialog()
            progress_dialog.isCancelButtonShown = False
            progress_dialog.show(CMD_NAME, "Running checks...", 0, 1, 0)
            try:
                checks = run_environment_checks(
                    distro=DEFAULT_DISTRO,
                    ros_setup=DEFAULT_ROS_SETUP,
                    wsl_ros_ws_src=wsl_ros_ws_src,
                    check_gazebo=check_gazebo,
                )
            finally:
                progress_dialog.hide()

            ui.messageBox(f"Fusion2ROS: WSL environment check\n\n{format_report(checks)}")
        except Exception:
            ui.messageBox(f"Fusion2ROS: environment check failed:\n{traceback.format_exc()}")
