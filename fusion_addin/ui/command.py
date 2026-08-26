"""Fusion 360 command handler for "Generate ROS 2 Package".

*** UNVERIFIED against a live Fusion process *** -- same caveat as
extraction/fusion_adapter.py and generators/mesh.py. Written against the
standard adsk.core command/event-handler pattern used throughout Autodesk's
own API samples (CommandCreatedEventHandler -> commandInputs ->
CommandEventHandler on cmd.execute, InputChangedEventHandler on
cmd.inputChanged), but never executed, since this sandbox has no
adsk.core/adsk.fusion. Every adsk symbol used below was checked against
Autodesk's official Fusion 360 API documentation
(help.autodesk.com/cloudhelp/ENU/Fusion-360-API/...) during this change --
see the per-input comments for citations; nothing here is invented.

Deliberately thin, per ARCHITECTURE.md's "don't put extraction/generation
logic directly inside UI callbacks" rule: this file only gathers inputs from
Fusion (the active Design, an optional root-occurrence selection, a robot
name, an output folder, four output checkboxes, a MoveIt group name) and a
temp dir for mesh export, then hands off to fusion_addin.app, which is fully
unit tested without Fusion. If you're debugging a wrong RESULT (wrong URDF,
wrong package layout, wrong mesh placement, wrong include_* behavior), the
bug is almost certainly in app.py / generators/ / extraction/converter.py,
not here.

The three sibling commands ("Validate ROS 2 Package", "Build in WSL",
"Launch RViz") live in validate_command.py / build_command.py /
launch_command.py, following the same register()/unregister() pattern as
this file -- see those modules' docstrings.
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
from . import state

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


# ---------------------------------------------------------------------------
# Shared helpers (used by both the created-handler's initial readback and the
# input-changed-handler's live refresh).
# ---------------------------------------------------------------------------


def _selected_root_component(inputs: "adsk.core.CommandInputs"):
    """Returns the adsk.fusion.Component to scope extraction to, or None for
    "whole design" (the default, unchanged behavior). Confirmed via
    SelectionCommandInput.htm: `selectionCount` / `selection(index)` give the
    user's current selections; `Selection.entity` (Selection.htm: "Gets the
    selected entity") gives back the picked object -- here always an
    Occurrence, since the input below is restricted to the "Occurrences"
    selection filter. `Occurrence.component` (Occurrence_component.htm: "The
    component this occurrence references") is the value fusion_adapter.py's
    new `root_component` parameter expects.
    """
    sel_input = adsk.core.SelectionCommandInput.cast(inputs.itemById("root_occurrence"))
    if sel_input is None or sel_input.selectionCount == 0:
        return None
    occurrence = adsk.fusion.Occurrence.cast(sel_input.selection(0).entity)
    return occurrence.component if occurrence else None


def _refresh_detected_summary(inputs: "adsk.core.CommandInputs") -> None:
    """Rebuilds the read-only "Detected Links/Joints" text box by actually
    running extraction (FusionDesignReaderAdapter -> app.build_robot_from_reader)
    against the current robot-name/root-selection inputs, without generating
    anything. Mirrors the mockup's "Detected Links/Joints" readback checklist
    -- simplified to a plain read-only list (see app.format_robot_summary's
    docstring for why per-item checkboxes aren't attempted here). Any
    extraction problem (disconnected joint graph, duplicate names, etc.) is
    shown in the text box itself rather than a message box, since this runs
    on every keystroke/selection change and popping a dialog each time would
    be intrusive.
    """
    text_input = inputs.itemById("detected_summary")
    if text_input is None:
        return
    try:
        robot_name_input = inputs.itemById("robot_name")
        robot_name = (robot_name_input.value.strip() if robot_name_input else "") or "robot"

        fusion_app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(fusion_app.activeProduct)
        if design is None:
            text_input.text = "(no active Design -- open/activate a design first)"
            return

        root_component = _selected_root_component(inputs)
        reader = FusionDesignReaderAdapter(design, root_component=root_component)
        robot = app.build_robot_from_reader(reader, robot_name)
        text_input.text = app.format_robot_summary(robot)
    except Exception as exc:  # noqa: BLE001 -- deliberately broad, see docstring
        text_input.text = f"(could not detect links/joints yet: {exc})"


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

            # Root/occurrence selection (task item 1): optional, restricted to
            # occurrences via addSelectionFilter("Occurrences") -- confirmed
            # string per SelectionFilters_UM.htm ("Occurrences" -- "Select
            # Occurrence objects."). setSelectionLimits(0, 1): a minimum of 0
            # makes the selection optional (long-standing convention across
            # Autodesk's own samples for an optional single-pick input; the
            # exact minimum=0=optional semantics were not independently
            # re-confirmed on SelectionCommandInput.htm's own text beyond the
            # maximum=0=unlimited note it does give -- flagged here as the
            # one point in this input worth double-checking live).
            sel_input = inputs.addSelectionInput(
                "root_occurrence",
                "Root sub-assembly (optional)",
                "Select an occurrence to scope extraction to its sub-assembly; leave empty for the whole design.",
            )
            sel_input.addSelectionFilter("Occurrences")
            sel_input.setSelectionLimits(0, 1)

            inputs.addStringValueInput("robot_name", "Robot name", default_name)
            inputs.addStringValueInput("output_dir", "Output folder", default_output)

            # Output checkboxes (task item 2). addBoolValueInput signature
            # confirmed via CommandInputs_addBoolValueInput.htm:
            # (id, name, isCheckBox, resourceFolder="", initialValue=False).
            # URDF/Xacro + the base ROS 2 package are always produced (no
            # checkbox for them), matching app.generate_ros_package's
            # always-on base behavior plus its four opt-in include_* flags.
            inputs.addBoolValueInput("include_ros2_control", "ros2_control", True, "", False)
            inputs.addBoolValueInput("include_gazebo", "Gazebo", True, "", False)
            inputs.addBoolValueInput("include_moveit", "MoveIt 2", True, "", False)
            inputs.addBoolValueInput("include_nav2", "Nav2", True, "", False)
            inputs.addStringValueInput("moveit_group_name", "MoveIt planning group (if MoveIt 2 checked)", "arm")

            # Detected Links/Joints readback (task item 3). addTextBoxCommandInput
            # signature confirmed via help.autodesk.com (id, name,
            # formattedText, numRows, isReadOnly) -- Autodesk's docs flag this
            # method as superseded by addSimpleTextBoxCommandInput /
            # addFormattedTextBoxCommandInput for new code, but it remains a
            # real, documented, currently-working method, and is used here
            # for its simplicity (a single plain-text multi-line box is all
            # this readback needs).
            inputs.addTextBoxCommandInput(
                "detected_summary",
                "Detected Links/Joints",
                "(computing...)",
                10,
                True,
            )

            on_input_changed = GenerateInputChangedHandler()
            cmd.inputChanged.add(on_input_changed)
            _handlers.append(on_input_changed)

            on_execute = GenerateCommandExecuteHandler()
            cmd.execute.add(on_execute)
            _handlers.append(on_execute)

            # Populate the readback once up front, using the just-added
            # inputs' initial values, so the user doesn't have to touch
            # anything first to see it.
            _refresh_detected_summary(inputs)
        except Exception:
            ui.messageBox(f"Fusion2ROS: failed to create command:\n{traceback.format_exc()}")


class GenerateInputChangedHandler(adsk.core.InputChangedEventHandler):
    """Confirmed via Command_inputChanged.htm / InputChangedEventArgs.htm:
    fires whenever any command input changes, with `args.input` naming which
    one and `args.inputs` giving the full CommandInputs collection to react
    against -- exactly what `_refresh_detected_summary` needs."""

    def notify(self, args: "adsk.core.InputChangedEventArgs") -> None:
        if args.input.id in ("robot_name", "root_occurrence"):
            _refresh_detected_summary(args.inputs)


class GenerateCommandExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args: "adsk.core.CommandEventArgs") -> None:
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            robot_name = inputs.itemById("robot_name").value.strip()
            output_dir = Path(inputs.itemById("output_dir").value.strip())
            include_ros2_control = inputs.itemById("include_ros2_control").value
            include_gazebo = inputs.itemById("include_gazebo").value
            include_moveit = inputs.itemById("include_moveit").value
            include_nav2 = inputs.itemById("include_nav2").value
            moveit_group_name = inputs.itemById("moveit_group_name").value.strip() or "arm"

            fusion_app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(fusion_app.activeProduct)
            if design is None:
                ui.messageBox("Fusion2ROS: no active Design (open/activate a design first).")
                return
            if not robot_name:
                ui.messageBox("Fusion2ROS: robot name must not be empty.")
                return

            root_component = _selected_root_component(inputs)
            reader = FusionDesignReaderAdapter(design, root_component=root_component)
            robot = app.build_robot_from_reader(reader, robot_name)

            with tempfile.TemporaryDirectory(prefix="fusion2ros_mesh_") as tmp_mesh_dir:
                mesh_files = export_link_meshes(design, robot, Path(tmp_mesh_dir))
                package_dir = app.generate_ros_package(
                    robot,
                    mesh_files,
                    output_dir,
                    include_ros2_control=include_ros2_control,
                    include_gazebo=include_gazebo,
                    include_moveit=include_moveit,
                    include_nav2=include_nav2,
                    moveit_group_name=moveit_group_name,
                )

            state.set_last_generated(package_dir, robot_name)
            ui.messageBox(f"Fusion2ROS: generated ROS 2 package at:\n{package_dir}")
        except app.PipelineError as exc:
            # exc's text is already a clear, itemized explanation (see
            # app.py's PipelineError docstring/raise sites) -- shown in full,
            # not re-summarized, per the task brief.
            ui.messageBox(f"Fusion2ROS: {exc}")
        except Exception:
            ui.messageBox(f"Fusion2ROS: generation failed:\n{traceback.format_exc()}")
