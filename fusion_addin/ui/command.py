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

import json
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

# Toolbar icon folder. Confirmed via Autodesk's official docs
# (CommandDefinitions_addButtonDefinition.htm: resourceFolder is "the
# resource folder that contains the images used for the icon. Icons can be
# defined using either PNG or SVG files"; UserInterface_UM.htm's icon-naming
# convention: Fusion looks inside this folder for files literally named
# "16x16.png"/"32x32.png" -- the two sizes it actually uses for toolbar
# buttons -- with optional "16x16@2x.png"/"32x32@2x.png" HiDPI variants and
# optional "-<theme>" suffixes; a missing size is scaled from whichever one
# IS present rather than erroring. The generate/validate/build/launch
# resource folders here also ship 24x24.png/64x64.png (from the icon-asset
# commit) -- those aren't part of the documented naming convention above, so
# Fusion is not expected to pick them up on its own, but their presence
# alongside the two required sizes is harmless.
RESOURCE_FOLDER = str(Path(__file__).parent / "resources" / "generate")

# Palette (task item 3): a richer, real HTML/CSS/JS "Detected Links/Joints"
# tree, shown alongside this command's dialog. See detected_summary.html's
# own comments for the rendering side, and _get_or_create_palette /
# PaletteHTMLEventHandler below for the Python side. Confirmed against
# Autodesk's official Palettes_UM.htm and Palettes_add.htm (add() argument
# order below), and against the documented Palette Sample add-in's
# create-once-and-reuse-via-itemById pattern (help.autodesk.com
# PaletteSample_Sample.htm) -- see the per-call comments for exactly what
# was confirmed vs. carried over as long-standing convention.
PALETTE_ID = "fusion2ros_detected_summary_palette"
PALETTE_HTML_PATH = str(Path(__file__).parent / "resources" / "palette" / "detected_summary.html")

_handlers = []  # Fusion requires handlers to be kept alive; module-level list per standard pattern.

# Last-known structured summary (app.robot_summary_as_dict's shape), kept
# module-level so PaletteHTMLEventHandler can (re-)send it the moment the
# palette's HTML signals it has finished loading ("ready"), independent of
# whichever GenerateCommand* handler instance last computed it.
_last_summary_dict: dict = {"links": [], "joints": []}


def register(ui: "adsk.core.UserInterface") -> None:
    existing = ui.commandDefinitions.itemById(CMD_ID)
    if existing:
        existing.deleteMe()

    cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_DESCRIPTION, RESOURCE_FOLDER)
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

    # Definitively tear down the palette on add-in stop (not just hide it --
    # this is the actual end of its lifecycle), per PaletteSample_Sample.htm's
    # documented stop()-time cleanup (`palette.deleteMe()` after `itemById`).
    palette = ui.palettes.itemById(PALETTE_ID)
    if palette:
        palette.deleteMe()

    _handlers.clear()


# ---------------------------------------------------------------------------
# Palette lifecycle + Python<->HTML glue (task item 3).
# ---------------------------------------------------------------------------


def _get_or_create_palette(ui: "adsk.core.UserInterface"):
    """Create-once-and-reuse pattern, confirmed via Autodesk's own
    PaletteSample_Sample.htm: look the palette up by id first, and only
    call palettes.add(...) the first time; every later call just re-shows
    the existing one (isVisible = True) instead of recreating it.

    Palettes.add's argument order (id, name, htmlFileURL, isVisible,
    showCloseButton, isResizable, width, height[, useNewWebBrowser]) is
    confirmed via Palettes_add.htm. `useNewWebBrowser` is left at its
    documented default (True -- the modern Qt-based web view) rather than
    passed explicitly, since nothing here depends on the older CEF browser's
    synchronous adsk.fusionSendData behavior (detected_summary.html handles
    both, see its own comments).
    """
    palette = ui.palettes.itemById(PALETTE_ID)
    if palette is None:
        palette = ui.palettes.add(
            PALETTE_ID,
            "Fusion2ROS: Detected Links/Joints",
            PALETTE_HTML_PATH,
            True,  # isVisible
            True,  # showCloseButton
            True,  # isResizable
            360,  # width (px)
            480,  # height (px)
        )
        on_html_event = PaletteHTMLEventHandler()
        palette.incomingFromHTML.add(on_html_event)
        _handlers.append(on_html_event)
    else:
        palette.isVisible = True
    return palette


def _push_summary_to_palette(ui: "adsk.core.UserInterface") -> None:
    """Best-effort push of the current _last_summary_dict to the palette's
    HTML side via Palette.sendInfoToHTML(action, data) -- confirmed via
    Palette_sendInfoToHTML.htm; `data` is an arbitrary string, JSON here by
    this add-in's own convention (matching detected_summary.html's JS side).
    Never allowed to fail the surrounding command -- this is a UI nicety on
    top of the always-present detected_summary text box, not load-bearing.
    """
    try:
        palette = ui.palettes.itemById(PALETTE_ID)
        if palette:
            palette.sendInfoToHTML("update", json.dumps(_last_summary_dict))
    except Exception:
        pass


class PaletteHTMLEventHandler(adsk.core.HTMLEventHandler):
    """Confirmed via Palettes_UM.htm / Palette_sendInfoToHTML.htm: a
    sendInfoToHTML call made before the palette's own HTML/JS has finished
    loading (and registered its receive-side handler) has nowhere to land.
    detected_summary.html's JS sends a `{action: "ready"}` event via
    adsk.fusionSendData the moment it has finished loading; this handler
    (registered on Palette.incomingFromHTML, per PaletteSample_Sample.htm)
    reacts to exactly that by (re-)sending the latest known summary --
    removing any guesswork about load timing on the Python side.
    """

    def notify(self, args: "adsk.core.HTMLEventArgs") -> None:
        try:
            html_args = adsk.core.HTMLEventArgs.cast(args)
            if html_args.action == "ready":
                _push_summary_to_palette(adsk.core.Application.get().userInterface)
        except Exception:
            pass  # palette readback is a nicety, never worth crashing the command over.


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
    global _last_summary_dict

    text_input = inputs.itemById("detected_summary")
    if text_input is None:
        return

    ui = adsk.core.Application.get().userInterface
    try:
        robot_name_input = inputs.itemById("robot_name")
        robot_name = (robot_name_input.value.strip() if robot_name_input else "") or "robot"

        fusion_app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(fusion_app.activeProduct)
        if design is None:
            text_input.text = "(no active Design -- open/activate a design first)"
            _last_summary_dict = {"links": [], "joints": [], "error": "no active Design"}
            _push_summary_to_palette(ui)
            return

        root_component = _selected_root_component(inputs)
        reader = FusionDesignReaderAdapter(design, root_component=root_component)
        robot = app.build_robot_from_reader(reader, robot_name)
        text_input.text = app.format_robot_summary(robot)
        _last_summary_dict = app.robot_summary_as_dict(robot)
        _push_summary_to_palette(ui)
    except Exception as exc:  # noqa: BLE001 -- deliberately broad, see docstring
        text_input.text = f"(could not detect links/joints yet: {exc})"
        _last_summary_dict = {"links": [], "joints": [], "error": str(exc)}
        _push_summary_to_palette(ui)


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

            # Show the richer HTML/JS "Detected Links/Joints" tree (task item
            # 3) alongside this command's dialog, and hide it again once the
            # dialog closes (on_destroy below) -- the TextBoxCommandInput
            # readback above is kept as-is as a guaranteed-to-work fallback
            # (e.g. if a given Fusion install's web-view is unavailable),
            # so nothing regresses if the palette fails to show.
            _get_or_create_palette(ui)

            on_destroy = GenerateCommandDestroyHandler()
            cmd.destroy.add(on_destroy)
            _handlers.append(on_destroy)

            # Populate the readback once up front, using the just-added
            # inputs' initial values, so the user doesn't have to touch
            # anything first to see it.
            _refresh_detected_summary(inputs)
        except Exception:
            ui.messageBox(f"Fusion2ROS: failed to create command:\n{traceback.format_exc()}")


class GenerateCommandDestroyHandler(adsk.core.CommandEventHandler):
    """Confirmed via Command.htm's `destroy` event ("fired when the command
    is destroyed... can be cleaned up") -- hides (not deletes) the palette
    when this command's dialog closes, whether via execute or cancel, so it
    doesn't linger as a stray window. The palette itself is reused (not
    recreated) the next time "Generate ROS 2 Package" runs, per
    _get_or_create_palette's create-once pattern; unregister() above deletes
    it for real when the add-in stops.
    """

    def notify(self, args: "adsk.core.CommandEventArgs") -> None:
        try:
            ui = adsk.core.Application.get().userInterface
            palette = ui.palettes.itemById(PALETTE_ID)
            if palette:
                palette.isVisible = False
        except Exception:
            pass


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
        # Confirmed via ProgressDialog.htm / ProgressDialog_show.htm:
        # ui.createProgressDialog() makes the dialog; .show(title, message,
        # minimumValue, maximumValue, delay=0) displays it, where `message`
        # supports "%v"/"%m"/"%p" placeholders (current value/total/percent)
        # that Fusion substitutes itself; .progressValue and .message are
        # plain read/write properties; .maximumValue is also read/write
        # (confirmed on the same ProgressDialog.htm reference), used below to
        # widen the bar once generate_ros_package reports its real
        # total_steps (unknown until generation starts, since it depends on
        # which include_* flags are set); .wasCancelled reflects whether the
        # dialog's own Cancel button (shown here via isCancelButtonShown)
        # was clicked; .hide() tears it down.
        #
        # The dialog starts with a 1-step range and a generic message so
        # something reasonable is on screen for the (typically brief) window
        # before the first _report() callback fires and supplies the real
        # total.
        progress_dialog = ui.createProgressDialog()
        progress_dialog.isCancelButtonShown = True
        progress_dialog.show(CMD_NAME, "Preparing to generate...", 0, 1, 0)
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

            def _on_progress(stage_description: str, step: int, total_steps: int) -> None:
                if progress_dialog.maximumValue != total_steps:
                    progress_dialog.maximumValue = total_steps
                progress_dialog.message = f"(%v/%m) {stage_description}"
                progress_dialog.progressValue = step

            def _should_cancel() -> bool:
                return progress_dialog.wasCancelled

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
                    progress_callback=_on_progress,
                    should_cancel=_should_cancel,
                )

            state.set_last_generated(package_dir, robot_name)
            ui.messageBox(f"Fusion2ROS: generated ROS 2 package at:\n{package_dir}")
        except app.GenerationCancelled:
            # Deliberate, user-initiated stop (Cancel button on the progress
            # dialog) -- not an error, so no traceback / no PipelineError-style
            # wall of text, just a plain confirmation.
            ui.messageBox("Fusion2ROS: generation cancelled.")
        except app.PipelineError as exc:
            # exc's text is already a clear, itemized explanation (see
            # app.py's PipelineError docstring/raise sites) -- shown in full,
            # not re-summarized, per the task brief.
            ui.messageBox(f"Fusion2ROS: {exc}")
        except Exception:
            ui.messageBox(f"Fusion2ROS: generation failed:\n{traceback.format_exc()}")
        finally:
            progress_dialog.hide()
