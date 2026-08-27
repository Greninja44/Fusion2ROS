"""Fusion 360 command handler for "Build in WSL".

*** UNVERIFIED against a live Fusion process, and against a live wsl.exe ***
-- this command's entire job is to call bridge.windows.invoke.build_package_in_wsl,
which is itself explicitly flagged "THIS COMPOSITION HAS NOT BEEN RUN END TO
END" in its own docstring (see bridge/windows/invoke.py). Nothing new is
invented here beyond that already-unverified piece; this file only gathers
two path inputs and formats the resulting WslResult into a message box.

Per docs/ARCHITECTURE.md's "Bridge workflow": "The Windows add-in never
requires WSL to be present to extract a model, build RobotModel, or generate
URDF/package files -- only the final 'Build in WSL' / 'Launch RViz' actions
need it. Bridge detects WSL availability up front and disables those actions
gracefully if absent." This command checks bridge.windows.detect.is_wsl_available()
both when the toolbar control is created (setting the command definition's
controlDefinition.isEnabled -- confirmed via CommandDefinition_controlDefinition.htm
that this property exists; Autodesk Community threads note it does not
always visually grey out a button reliably in every Fusion version, hence
the belt-and-suspenders re-check below) and again at execute time as a
defensive fallback that always works regardless of that greying-out quirk.
"""

from __future__ import annotations

import traceback
from pathlib import Path

import adsk.core

from bridge.windows.detect import is_wsl_available
from bridge.windows.invoke import (
    DEFAULT_DISTRO,
    DEFAULT_ROS_SETUP,
    DEFAULT_WSL_ROS_WS_SRC,
    build_package_in_wsl,
)
from . import state

CMD_ID = "fusion2ros_build_wsl"
CMD_NAME = "Build in WSL"
CMD_DESCRIPTION = "Copy a generated ROS 2 package into the WSL colcon workspace and build it."
PANEL_ID = "SolidCreatePanel"
# See command.py's RESOURCE_FOLDER comment for the confirmed
# addButtonDefinition/resourceFolder icon-naming convention this relies on.
RESOURCE_FOLDER = str(Path(__file__).parent / "resources" / "build")

_handlers = []


def register(ui: "adsk.core.UserInterface") -> None:
    existing = ui.commandDefinitions.itemById(CMD_ID)
    if existing:
        existing.deleteMe()

    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESCRIPTION, RESOURCE_FOLDER)
    on_created = BuildCommandCreatedHandler()
    cmd_def.commandCreated.add(on_created)
    _handlers.append(on_created)

    try:
        cmd_def.controlDefinition.isEnabled = is_wsl_available()
    except Exception:
        pass  # best-effort greying-out only; execute-time check below is the real gate.

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


class BuildCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args: "adsk.core.CommandCreatedEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            last_dir = state.get_last_package_dir()
            default_package_path = str(last_dir) if last_dir else ""
            default_ws_src = state.get_last_wsl_ros_ws_src() or DEFAULT_WSL_ROS_WS_SRC

            inputs.addStringValueInput(
                "windows_package_path",
                "Generated package path (Windows-side)",
                default_package_path,
            )
            inputs.addStringValueInput(
                "wsl_ros_ws_src",
                "WSL colcon workspace src/ (e.g. ~/ros2_ws/src)",
                default_ws_src,
            )

            on_execute = BuildCommandExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
        except Exception:
            ui.messageBox(f"Fusion2ROS: failed to create command:\n{traceback.format_exc()}")


class BuildCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args: "adsk.core.CommandEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            if not is_wsl_available():
                ui.messageBox(
                    "Fusion2ROS: WSL is not available on this machine (or has no registered "
                    "distro) -- cannot build. Install WSL and a distro, then try again."
                )
                return

            inputs = args.command.commandInputs
            windows_package_path = inputs.itemById("windows_package_path").value.strip()
            wsl_ros_ws_src = inputs.itemById("wsl_ros_ws_src").value.strip() or DEFAULT_WSL_ROS_WS_SRC

            if not windows_package_path:
                ui.messageBox("Fusion2ROS: package path must not be empty.")
                return

            # bridge.windows.invoke.build_package_in_wsl's on_output_line
            # parameter is now REAL-VERIFIED (see
            # tests/bridge/test_windows_invoke.py) -- a colcon build relayed
            # through wsl.exe takes real wall-clock time, so this shows live
            # build output in a progress dialog instead of a frozen-looking
            # command while it runs. Same ProgressDialog API confirmed and
            # used by command.py's GenerateCommandExecuteHandler.
            progress_dialog = ui.createProgressDialog()
            progress_dialog.isCancelButtonShown = False  # colcon build isn't cooperatively cancellable mid-flight
            progress_dialog.show("Build in WSL", "Starting build...", 0, 0, 0)

            def _on_output_line(line: str) -> None:
                # maximumValue stays 0 (an indeterminate/marquee-style bar,
                # per ProgressDialog.htm) since colcon gives no total-step
                # count to report against -- the live message text is the
                # actual feedback here, not the bar's fill level.
                progress_dialog.message = line[-200:] if line else progress_dialog.message

            try:
                result = build_package_in_wsl(
                    windows_package_path,
                    wsl_ros_ws_src,
                    distro=DEFAULT_DISTRO,
                    ros_setup=DEFAULT_ROS_SETUP,
                    on_output_line=_on_output_line,
                )
            finally:
                progress_dialog.hide()

            if result.success:
                state.set_last_wsl_ros_ws_src(wsl_ros_ws_src)
                ui.messageBox(f"Fusion2ROS: BUILD SUCCESS\n\n{result.stdout}")
            else:
                ui.messageBox(f"Fusion2ROS: BUILD FAILED (exit {result.returncode})\n\n{result.stderr}")
        except Exception:
            ui.messageBox(f"Fusion2ROS: build failed:\n{traceback.format_exc()}")
