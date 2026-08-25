"""Fusion 360 command handlers for "Generate ROS 2 Package".

*** UNVERIFIED against a live Fusion process *** -- same caveat as
extraction/fusion_adapter.py and generators/mesh.py. Written against the
standard adsk.core command/event-handler pattern used throughout Autodesk's
own API samples (CommandCreatedEventHandler -> commandInputs ->
CommandEventHandler on cmd.execute), but never executed, since this sandbox
has no adsk.core/adsk.fusion.

Deliberately thin, per ARCHITECTURE.md's "don't put extraction/generation
logic directly inside UI callbacks" rule: this file only gathers inputs from
Fusion (the active Design, two text fields) and a temp dir for mesh export,
then hands off to fusion_addin.app, which is fully unit tested without
Fusion. If you're debugging a wrong RESULT (wrong URDF, wrong package
layout, wrong mesh placement), the bug is almost certainly in app.py /
generators/ / extraction/converter.py, not here.
"""

from __future__ import annotations

import tempfile
import traceback
from pathlib import Path

import adsk.core
import adsk.fusion

from .. import app
from ..extraction.fusion_adapter import FusionDesignReaderAdapter
from ..generators.mesh import export_link_meshes

CMD_ID = "fusion2ros_generate"
CMD_NAME = "Generate ROS 2 Package"
CMD_DESCRIPTION = (
    "Extract this design as a RobotModel and generate a ROS 2 package "
    "(URDF/Xacro, meshes, launch files) under Fusion2ROS/output/."
)
# SolidCreatePanel is a standard, always-present Fusion panel id -- reusing
# it avoids inventing a new toolbar tab/panel id no one has confirmed exists.
PANEL_ID = "SolidCreatePanel"

_handlers = []  # Fusion requires handlers to be kept alive; module-level list per standard pattern.


def register(ui: "adsk.core.UserInterface") -> None:
    existing = ui.commandDefinitions.itemById(CMD_ID)
    if existing:
        existing.deleteMe()

    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESCRIPTION)
    on_created = GenerateCommandCreatedHandler()
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


class GenerateCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args: "adsk.core.CommandCreatedEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            cmd = args.command
            inputs = cmd.commandInputs

            fusion_app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(fusion_app.activeProduct)
            default_name = design.rootComponent.name if design else "robot"
            default_output = str(Path.home() / "Fusion2ROS" / "output")

            inputs.addStringValueInput("robot_name", "Robot name", default_name)
            inputs.addStringValueInput("output_dir", "Output folder", default_output)

            on_execute = GenerateCommandExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)
        except Exception:
            ui.messageBox(f"Fusion2ROS: failed to create command:\n{traceback.format_exc()}")


class GenerateCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args: "adsk.core.CommandEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            robot_name = inputs.itemById("robot_name").value.strip()
            output_dir = Path(inputs.itemById("output_dir").value.strip())

            fusion_app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(fusion_app.activeProduct)
            if design is None:
                ui.messageBox("Fusion2ROS: no active Design (open/activate a design first).")
                return
            if not robot_name:
                ui.messageBox("Fusion2ROS: robot name must not be empty.")
                return

            reader = FusionDesignReaderAdapter(design)
            robot = app.build_robot_from_reader(reader, robot_name)

            with tempfile.TemporaryDirectory(prefix="fusion2ros_mesh_") as tmp_mesh_dir:
                mesh_files = export_link_meshes(design, robot, Path(tmp_mesh_dir))
                package_dir = app.generate_ros_package(robot, mesh_files, output_dir)

            ui.messageBox(f"Fusion2ROS: generated ROS 2 package at:\n{package_dir}")
        except app.PipelineError as exc:
            ui.messageBox(f"Fusion2ROS: {exc}")
        except Exception:
            ui.messageBox(f"Fusion2ROS: generation failed:\n{traceback.format_exc()}")
