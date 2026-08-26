"""Fusion 360 command handler for "Launch RViz".

*** UNVERIFIED against a live Fusion process *** -- the adsk.core side of
this file has never run inside a real Fusion 360 session (no adsk.core in
this sandbox), same as every other file in fusion_addin/ui/. The
`bridge.windows.invoke.launch_ros2_in_wsl` call it makes is, however, now
PARTIALLY real-verified: its immediate-failure-detection path (a launch
that dies within a few seconds -- bad package/launch-file name, etc.) has
been run for real against a genuine `wsl.exe` (see
tests/bridge/test_windows_invoke.py and that function's own docstring for
exactly what's confirmed vs. not -- the GUI-stays-up success path remains
unverified, since confirming an RViz window actually renders hit the WSLg
screenshot limitation documented elsewhere in this project).

Intended flow (per the task brief): run this after "Build in WSL" has
succeeded, so the workspace's `install/setup.bash` actually exists to
source. This command does not itself re-run or require a build -- it will
simply fail with whatever `source .../install/setup.bash` reports if the
workspace hasn't been built yet, which is a clear enough signal without
adding a redundant "have you built?" check here.
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
    launch_ros2_in_wsl,
)
from . import state

CMD_ID = "fusion2ros_launch_rviz"
CMD_NAME = "Launch RViz"
CMD_DESCRIPTION = "Run 'ros2 launch <package> display.launch.py' inside WSL for a built package."
PANEL_ID = "SolidCreatePanel"
DEFAULT_LAUNCH_FILE = "display.launch.py"
# See command.py's RESOURCE_FOLDER comment for the confirmed
# addButtonDefinition/resourceFolder icon-naming convention this relies on.
RESOURCE_FOLDER = str(Path(__file__).parent / "resources" / "launch")

_handlers = []


def register(ui: "adsk.core.UserInterface") -> None:
    existing = ui.commandDefinitions.itemById(CMD_ID)
    if existing:
        existing.deleteMe()

    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESCRIPTION, RESOURCE_FOLDER)
    on_created = LaunchCommandCreatedHandler()
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


class LaunchCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args: "adsk.core.CommandCreatedEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            default_package_name = state.get_last_robot_name() or ""
            default_ws_src = state.get_last_wsl_ros_ws_src() or DEFAULT_WSL_ROS_WS_SRC

            inputs.addStringValueInput("package_name", "Package (robot) name", default_package_name)
            inputs.addStringValueInput("launch_file", "Launch file", DEFAULT_LAUNCH_FILE)
            inputs.addStringValueInput(
                "wsl_ros_ws_src",
                "WSL colcon workspace src/ (e.g. ~/ros2_ws/src)",
                default_ws_src,
            )

            on_execute = LaunchCommandExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
        except Exception:
            ui.messageBox(f"Fusion2ROS: failed to create command:\n{traceback.format_exc()}")


class LaunchCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args: "adsk.core.CommandEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            if not is_wsl_available():
                ui.messageBox(
                    "Fusion2ROS: WSL is not available on this machine (or has no registered "
                    "distro) -- cannot launch. Install WSL and a distro, then try again."
                )
                return

            inputs = args.command.commandInputs
            package_name = inputs.itemById("package_name").value.strip()
            launch_file = inputs.itemById("launch_file").value.strip() or DEFAULT_LAUNCH_FILE
            wsl_ros_ws_src = inputs.itemById("wsl_ros_ws_src").value.strip() or DEFAULT_WSL_ROS_WS_SRC

            if not package_name:
                ui.messageBox("Fusion2ROS: package (robot) name must not be empty.")
                return

            result = launch_ros2_in_wsl(
                package_name,
                launch_file,
                wsl_ros_ws_src,
                distro=DEFAULT_DISTRO,
                ros_setup=DEFAULT_ROS_SETUP,
            )

            if result.success:
                ui.messageBox(f"Fusion2ROS: {result.stdout.strip() or 'launch started.'}")
            else:
                ui.messageBox(f"Fusion2ROS: launch failed (exit {result.returncode})\n\n{result.stderr}")
        except Exception:
            ui.messageBox(f"Fusion2ROS: launch failed:\n{traceback.format_exc()}")
