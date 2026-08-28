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
from typing import Dict, Optional

import adsk.core
import adsk.fusion

from bridge.windows.doctor import all_critical_passed, format_report, run_environment_checks
from bridge.windows.invoke import (
    DEFAULT_DISTRO,
    DEFAULT_ROS_SETUP,
    DEFAULT_WSL_ROS_WS_SRC,
    build_package_in_wsl,
    launch_ros2_in_wsl,
)

# Crosses docs/ARCHITECTURE.md's "nothing under fusion_addin/ imports
# anything under ros2_tools/" layering rule -- validate_command.py's own
# docstring already documents the same deliberate exception (pure-stdlib,
# zero adsk/live-ROS dependency, so it runs fine under Windows Python too);
# this file now does the same to chain validation straight into Generate,
# instead of requiring a separate manual "Validate ROS 2 Package" run.
from robot_model import Sensor

from ros2_tools.validate.package import validate_package_structure
from ros2_tools.validate.urdf import validate_urdf_file

from .. import app
from ..extraction.drivetrain_detect import detect_drivetrain
from ..extraction.fusion_adapter import FusionDesignReaderAdapter
from ..generators.gazebo import uses_gz_ros2_control_fallback
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
# REAL BUG FOUND LIVE: Palettes.add()'s htmlFileURL argument needs an actual
# URI, not a bare OS filesystem path. Passing a raw Windows path (e.g.
# "C:\Users\...\detected_summary.html") made Fusion's internal browser
# malform it into "file:///C:/%5CUsers%5C...%5Cdetected_summary.html" --
# literally percent-encoding the backslashes instead of treating them as
# path separators -- so the palette showed a "This site can't be reached /
# ERR_INVALID_URL" page instead of the HTML. Path.as_uri() builds a correct
# "file:///C:/Users/.../detected_summary.html" URI regardless of OS
# separator, which is what's actually required here.
PALETTE_HTML_PATH = (Path(__file__).parent / "resources" / "palette" / "detected_summary.html").as_uri()

# Drivetrain dropdown item labels -- see the "drivetrain_type" input's own
# comment in GenerateCommandCreatedHandler for why this exists.
_DRIVETRAIN_NONE = "None (arm/manipulator)"
_DRIVETRAIN_DIFFERENTIAL = "Differential Drive"
_DRIVETRAIN_MECANUM = "Mecanum Drive"

# Input IDs shown only for their respective drivetrain_type selection --
# shared between _set_drivetrain_inputs_visibility and
# _build_drivetrain_metadata so the two can't drift out of sync.
_DIFFERENTIAL_INPUT_IDS = (
    "drivetrain_left_wheel_joint",
    "drivetrain_right_wheel_joint",
    "drivetrain_wheel_separation_m",
    "drivetrain_wheel_radius_m",
)
_MECANUM_INPUT_IDS = (
    "drivetrain_fl_wheel_joint",
    "drivetrain_fr_wheel_joint",
    "drivetrain_bl_wheel_joint",
    "drivetrain_br_wheel_joint",
    "drivetrain_mecanum_wheel_radius_m",
    "drivetrain_mecanum_center_sum_m",
)

# One-click "Generate -> Build -> Launch" (task: reduce manual steps between
# Fusion and WSL). "Don't launch" is the safe default -- launching starts a
# GUI (RViz/Gazebo) or a long-lived bringup, not something to fire
# automatically without the user opting in explicitly via this dropdown.
_LAUNCH_NONE = "Don't launch"
_LAUNCH_DISPLAY = "display.launch.py (RViz)"
_LAUNCH_GAZEBO = "gazebo.launch.py (Gazebo Sim)"
_LAUNCH_NAV2 = "nav2_bringup.launch.py (Nav2)"
_LAUNCH_BRINGUP = "bringup.launch.py (everything together)"
_LAUNCH_FILES = {
    _LAUNCH_DISPLAY: "display.launch.py",
    _LAUNCH_GAZEBO: "gazebo.launch.py",
    _LAUNCH_NAV2: "nav2_bringup.launch.py",
    _LAUNCH_BRINGUP: "bringup.launch.py",
}
_BUILD_CHAIN_INPUT_IDS = ("wsl_ros_ws_src_for_build", "launch_after_build")

# Sensor UI (real gap: fusion_addin/generators/sensors.py's camera/lidar/imu
# Gazebo XML + ros_gz_bridge generation already worked, but nothing in this
# dialog let a Fusion user actually attach a sensor to a link -- the only
# way was hand-editing a Robot object in a script. A fixed number of
# independent "slots" (not a dynamically-add-a-row UI, which Fusion's
# CommandInputs API has no established pattern for anywhere in this file)
# mirrors the drivetrain dropdown's own "None (arm/manipulator)" convention,
# extended to several independent choices -- 3 is enough for the common
# cases (e.g. one camera + one lidar + one imu) without cluttering the
# dialog; a user needing more can still hand-edit the generated URDF/robot
# script the way they always could.
_SENSOR_TYPE_NONE = "None"
_SENSOR_TYPE_CAMERA = "Camera"
_SENSOR_TYPE_LIDAR = "Lidar"
_SENSOR_TYPE_IMU = "IMU"
_SENSOR_TYPE_DEPTH_CAMERA = "Depth Camera"
_SENSOR_TYPE_GPS = "GPS / NavSat"
# "force_torque" is deliberately NOT offered here: gz-sim's force_torque
# sensor is joint-mounted (see generators/sensors.py's
# _gazebo_reference_for_sensor docstring) and needs a
# parameters["joint"] override this fixed type+parent_link+name+update_rate
# slot shape has no field for -- a user who wants one can still hand-edit
# the generated URDF/robot script, same as any sensor needing a pose offset
# already can (see _build_sensors' docstring).
_SENSOR_TYPE_LABEL_TO_MODEL_TYPE = {
    _SENSOR_TYPE_CAMERA: "camera",
    _SENSOR_TYPE_LIDAR: "lidar",
    _SENSOR_TYPE_IMU: "imu",
    _SENSOR_TYPE_DEPTH_CAMERA: "depth_camera",
    _SENSOR_TYPE_GPS: "gps",
}
_SENSOR_SLOT_COUNT = 3
_SENSOR_SLOT_IDS = tuple(range(1, _SENSOR_SLOT_COUNT + 1))


def _sensor_slot_input_ids(slot: int) -> Dict[str, str]:
    return {
        "type": f"sensor{slot}_type",
        "parent_link": f"sensor{slot}_parent_link",
        "name": f"sensor{slot}_name",
        "update_rate": f"sensor{slot}_update_rate_hz",
    }


_SENSOR_DEPENDENT_INPUT_IDS = tuple(
    _sensor_slot_input_ids(slot)[field] for slot in _SENSOR_SLOT_IDS for field in ("parent_link", "name", "update_rate")
)

# Persisted (to disk, via ui/state.py's JSON file, keyed by robot name) so
# reopening "Generate ROS 2 Package" -- even after a full Fusion restart --
# restores every checkbox/drivetrain/build-chain/sensor field instead of
# only robot_name/output_dir. Deliberately excludes "robot_name"/
# "output_dir" (already have their own defaulting via the design's name /
# a fixed default path) and "root_occurrence" (a live Design selection, not
# something JSON-serializable or meaningful to restore across designs).
_PERSISTED_BOOL_FIELD_IDS = (
    "include_ros2_control",
    "include_gazebo",
    "include_moveit",
    "include_nav2",
    "use_bounding_box_collision",
    "build_in_wsl_after_generate",
)
_PERSISTED_STRING_FIELD_IDS = (
    ("moveit_group_name", "wsl_ros_ws_src_for_build")
    + _DIFFERENTIAL_INPUT_IDS
    + _MECANUM_INPUT_IDS
    + tuple(
        _sensor_slot_input_ids(slot)[field]
        for slot in _SENSOR_SLOT_IDS
        for field in ("parent_link", "name", "update_rate")
    )
)
_PERSISTED_DROPDOWN_FIELD_IDS = ("drivetrain_type", "launch_after_build") + tuple(
    _sensor_slot_input_ids(slot)["type"] for slot in _SENSOR_SLOT_IDS
)

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


def _set_drivetrain_inputs_visibility(inputs: "adsk.core.CommandInputs") -> None:
    """Shows only the wheel-joint/numeric inputs relevant to the currently
    selected "drivetrain_type" dropdown value, hiding the rest. Called once
    at command creation (for the default "None" selection) and again from
    GenerateInputChangedHandler whenever the user changes the dropdown.
    CommandInput.isVisible is a plain read/write property, the standard way
    to show/hide an input without destroying/recreating it.
    """
    selected = inputs.itemById("drivetrain_type").selectedItem.name
    for input_id in _DIFFERENTIAL_INPUT_IDS:
        inputs.itemById(input_id).isVisible = selected == _DRIVETRAIN_DIFFERENTIAL
    for input_id in _MECANUM_INPUT_IDS:
        inputs.itemById(input_id).isVisible = selected == _DRIVETRAIN_MECANUM


def _set_build_chain_inputs_visibility(inputs: "adsk.core.CommandInputs") -> None:
    """Shows the WSL-workspace-path/launch-choice inputs only when "Also
    build in WSL after generating" is checked -- same show/hide-without-
    destroying pattern as _set_drivetrain_inputs_visibility."""
    build_checked = inputs.itemById("build_in_wsl_after_generate").value
    for input_id in _BUILD_CHAIN_INPUT_IDS:
        inputs.itemById(input_id).isVisible = build_checked


def _select_dropdown_item(inputs: "adsk.core.CommandInputs", input_id: str, label: str) -> None:
    """Programmatically selects `label` in the dropdown at `input_id` --
    DropDownCommandInput.listItems / ListItem.isSelected, the same
    real-confirmed API this file already uses to READ a selection
    (`.selectedItem.name`, see _set_drivetrain_inputs_visibility) and to
    ADD items with an initial isSelected value (`listItems.add(name,
    isSelected)`, see the drivetrain_type/launch_after_build dropdowns' own
    setup) -- setting isSelected on an existing ListItem is the same
    property used in reverse, a standard exclusive-selection dropdown
    pattern. A `label` that doesn't match any item is a silent no-op
    (defensive against stale persisted state naming an option this version
    of the dialog no longer has)."""
    dropdown = inputs.itemById(input_id)
    for item in dropdown.listItems:
        if item.name == label:
            item.isSelected = True
            return


def _select_drivetrain_type(inputs: "adsk.core.CommandInputs", label: str) -> None:
    _select_dropdown_item(inputs, "drivetrain_type", label)


def _collect_persistable_dialog_state(inputs: "adsk.core.CommandInputs") -> Dict[str, object]:
    """The flip side of _apply_persisted_dialog_state -- gathers every
    persisted field's CURRENT value right after a successful generation, for
    ui/state.py's save_generate_dialog_state to write to disk."""
    fields: Dict[str, object] = {}
    for input_id in _PERSISTED_BOOL_FIELD_IDS:
        fields[input_id] = inputs.itemById(input_id).value
    for input_id in _PERSISTED_STRING_FIELD_IDS:
        fields[input_id] = inputs.itemById(input_id).value
    for input_id in _PERSISTED_DROPDOWN_FIELD_IDS:
        fields[input_id] = inputs.itemById(input_id).selectedItem.name
    return fields


def _apply_persisted_dialog_state(inputs: "adsk.core.CommandInputs", saved: Dict[str, object]) -> None:
    """Restores every field `save_generate_dialog_state` previously saved
    for this robot name -- called once at command-creation time, BEFORE
    _refresh_detected_summary's own auto-detection runs, so a user's past
    explicit choice always outranks a fresh geometric guess (auto-detection
    already only fills fields it finds empty -- see
    _apply_auto_detected_drivetrain's docstring). Missing keys (e.g. a
    field added to the dialog after this robot's state was last saved) are
    simply left at their normal default -- never an error."""
    for input_id in _PERSISTED_BOOL_FIELD_IDS:
        if input_id in saved:
            inputs.itemById(input_id).value = bool(saved[input_id])
    for input_id in _PERSISTED_STRING_FIELD_IDS:
        if input_id in saved:
            inputs.itemById(input_id).value = str(saved[input_id])
    for input_id in _PERSISTED_DROPDOWN_FIELD_IDS:
        if input_id in saved:
            _select_dropdown_item(inputs, input_id, str(saved[input_id]))
    _set_drivetrain_inputs_visibility(inputs)
    _set_build_chain_inputs_visibility(inputs)
    # REAL BUG FIXED HERE: sensor{1,2,3}_type is itself a persisted dropdown
    # (see _PERSISTED_DROPDOWN_FIELD_IDS) and its parent_link/name/update_rate
    # fields are persisted strings too, so a saved "Camera"/"Lidar"/"IMU"
    # selection and its values ARE restored above -- but without this call
    # their inputs stayed hidden (still at the all-slots-"None" visibility
    # set once at command-creation time, before this restore ran), so a user
    # reopening "Generate ROS 2 Package" (or restarting Fusion) saw a sensor
    # slot silently reset back to "None" in the dropdown... no, worse: the
    # dropdown itself correctly showed "Camera" again, but its parent
    # link/name/rate fields stayed invisible, hiding data that was in fact
    # restored and would still be used on Generate. Same
    # show/hide-after-restore pattern as the two calls just above.
    _set_sensor_inputs_visibility(inputs)


def _apply_drivetrain_smart_default_checkboxes(inputs: "adsk.core.CommandInputs") -> None:
    """Nudges "Gazebo" and "Nav2" checked once a drivetrain is selected --
    the natural payoff of telling Fusion2ROS about a wheeled robot's
    drivetrain is simulating/navigating with it. Deliberately one-directional
    (only ever turns these ON, never OFF): switching drivetrain_type back to
    "None" must not silently uncheck Gazebo/Nav2 out from under a user who
    wants them for an arm too, or who unchecked drivetrain by mistake after
    already deciding they want Gazebo regardless."""
    if inputs.itemById("drivetrain_type").selectedItem.name == _DRIVETRAIN_NONE:
        return
    gazebo_input = inputs.itemById("include_gazebo")
    if not gazebo_input.value:
        gazebo_input.value = True
    nav2_input = inputs.itemById("include_nav2")
    if not nav2_input.value:
        nav2_input.value = True


def _apply_moveit_smart_default_checkbox(inputs: "adsk.core.CommandInputs", robot) -> None:
    """Nudges "MoveIt 2" checked when `robot` is a suitable single-chain arm
    -- reuses generators/moveit.py's own detect_moveit_suitability (already
    written for this exact question: a single unbranched chain from the
    root link to one leaf link, and no metadata["drivetrain"]) rather than
    a separate detector, since generate_srdf/etc. already auto-derive
    base_link/tip_link from the same "one clean chain" shape when none are
    passed explicitly -- there is nothing left to "detect" beyond
    suitability itself; the group's NAME is just a label ("arm" by
    default), not something geometry determines.

    Only checked when drivetrain_type is still "None" -- run AFTER
    _apply_auto_detected_drivetrain, so a robot the UI just suggested
    Differential/Mecanum Drive for (checked via the dropdown selection,
    not robot.metadata -- _apply_auto_detected_drivetrain fills UI fields
    only, it never mutates `robot` itself) is never also suggested as a
    MoveIt arm. One-directional like the drivetrain nudge: never unchecks
    "MoveIt 2" if the robot turns out unsuitable."""
    if inputs.itemById("drivetrain_type").selectedItem.name != _DRIVETRAIN_NONE:
        return
    moveit_input = inputs.itemById("include_moveit")
    if moveit_input.value:
        return
    from ..generators.moveit import detect_moveit_suitability

    if not detect_moveit_suitability(robot):
        moveit_input.value = True


def _set_sensor_inputs_visibility(inputs: "adsk.core.CommandInputs") -> None:
    """Shows each sensor slot's parent-link/name/update-rate inputs only
    when that slot's own type dropdown isn't "None" -- same pattern as
    _set_drivetrain_inputs_visibility, just per-slot instead of a single
    shared selection."""
    for slot in _SENSOR_SLOT_IDS:
        ids = _sensor_slot_input_ids(slot)
        slot_active = inputs.itemById(ids["type"]).selectedItem.name != _SENSOR_TYPE_NONE
        for field in ("parent_link", "name", "update_rate"):
            inputs.itemById(ids[field]).isVisible = slot_active


def _build_sensors(inputs: "adsk.core.CommandInputs") -> list:
    """Reads the sensor{1,2,3}_* inputs and returns the list of
    `robot_model.Sensor` objects to attach to the Robot before generation --
    empty list if every slot is "None". Raises ValueError (caught by
    GenerateCommandExecuteHandler the same way drivetrain validation errors
    are) naming the offending slot for a missing parent link or a
    non-numeric update rate.

    Origin is left at Pose.IDENTITY (Sensor's own default) -- exposing a
    6-field pose offset per slot was judged not worth the dialog clutter for
    a first version of this UI; a sensor mounted exactly at its parent
    link's own origin is a reasonable default users can already override by
    hand-editing the generated URDF/robot script if they need a precise
    offset. update_rate is the one parameter surfaced (the one
    `sensors.py`'s generators actually read from `Sensor.parameters` --
    resolution/FOV/range/etc. all keep their per-type defaults, see
    generators/sensors.py's own docstrings for exactly what those are).
    """
    sensors = []
    for slot in _SENSOR_SLOT_IDS:
        ids = _sensor_slot_input_ids(slot)
        type_label = inputs.itemById(ids["type"]).selectedItem.name
        if type_label == _SENSOR_TYPE_NONE:
            continue

        parent_link = inputs.itemById(ids["parent_link"]).value.strip()
        if not parent_link:
            raise ValueError(f"Sensor slot {slot} ({type_label}) requires a parent link name.")

        name = inputs.itemById(ids["name"]).value.strip() or f"{_SENSOR_TYPE_LABEL_TO_MODEL_TYPE[type_label]}_{slot}"

        parameters = {}
        rate_raw = inputs.itemById(ids["update_rate"]).value.strip()
        if rate_raw:
            try:
                parameters["update_rate"] = float(rate_raw)
            except ValueError:
                raise ValueError(
                    f"Sensor slot {slot} ({type_label}) update rate must be a number (got {rate_raw!r})."
                ) from None

        sensors.append(
            Sensor(
                name=name,
                type=_SENSOR_TYPE_LABEL_TO_MODEL_TYPE[type_label],
                parent_link=parent_link,
                parameters=parameters,
            )
        )
    return sensors


def _drivetrain_fields_are_empty(inputs: "adsk.core.CommandInputs") -> bool:
    return all(not inputs.itemById(input_id).value.strip() for input_id in _DIFFERENTIAL_INPUT_IDS)


def _apply_auto_detected_drivetrain(inputs: "adsk.core.CommandInputs", robot) -> None:
    """Pre-fills the Differential Drive drivetrain fields from
    drivetrain_detect.detect_drivetrain's best-effort geometric guess --
    ONLY when the user hasn't already picked a drivetrain type or typed
    anything into the differential-drive fields, so this never overwrites a
    manual choice (including a manual choice of "None", or Mecanum, or
    different differential joint names than the guess). Always an editable
    suggestion, never forced -- see detect_drivetrain's own docstring for
    the heuristic and its conservative return-None-rather-than-guess-wrong
    stance."""
    if inputs.itemById("drivetrain_type").selectedItem.name != _DRIVETRAIN_NONE:
        return
    if not _drivetrain_fields_are_empty(inputs):
        return

    drivetrain = detect_drivetrain(robot)
    if drivetrain is None:
        return

    inputs.itemById("drivetrain_left_wheel_joint").value = drivetrain["left_wheel_joint"]
    inputs.itemById("drivetrain_right_wheel_joint").value = drivetrain["right_wheel_joint"]
    inputs.itemById("drivetrain_wheel_separation_m").value = str(drivetrain["wheel_separation"])
    inputs.itemById("drivetrain_wheel_radius_m").value = str(drivetrain["wheel_radius"])
    _select_drivetrain_type(inputs, _DRIVETRAIN_DIFFERENTIAL)
    _set_drivetrain_inputs_visibility(inputs)
    _apply_drivetrain_smart_default_checkboxes(inputs)


def _parse_positive_float(inputs: "adsk.core.CommandInputs", input_id: str, field_label: str) -> float:
    raw = inputs.itemById(input_id).value.strip()
    try:
        value = float(raw)
    except ValueError:
        raise ValueError(f"{field_label} must be a number (got {raw!r}).") from None
    if value <= 0:
        raise ValueError(f"{field_label} must be a positive number (got {value}).")
    return value


def _build_drivetrain_metadata(inputs: "adsk.core.CommandInputs") -> Optional[Dict[str, object]]:
    """Reads the drivetrain_* inputs and returns the robot.metadata["drivetrain"]
    dict app.py/ros2_control.py/nav2.py expect (see ros2_control.py's module
    docstring for the exact shape), or None if "None (arm/manipulator)" is
    selected. Raises ValueError with a clear, field-specific message on
    bad/missing input -- caught by GenerateCommandExecuteHandler the same
    way any other input-validation error is.
    """
    selected = inputs.itemById("drivetrain_type").selectedItem.name

    if selected == _DRIVETRAIN_NONE:
        return None

    if selected == _DRIVETRAIN_DIFFERENTIAL:
        left = inputs.itemById("drivetrain_left_wheel_joint").value.strip()
        right = inputs.itemById("drivetrain_right_wheel_joint").value.strip()
        if not left or not right:
            raise ValueError("Differential Drive requires both a left and right wheel joint name.")
        return {
            "type": "differential_drive",
            "left_wheel_joint": left,
            "right_wheel_joint": right,
            "wheel_separation": _parse_positive_float(
                inputs, "drivetrain_wheel_separation_m", "Wheel separation (m)"
            ),
            "wheel_radius": _parse_positive_float(inputs, "drivetrain_wheel_radius_m", "Wheel radius (m)"),
        }

    if selected == _DRIVETRAIN_MECANUM:
        joint_fields = {
            "front_left_wheel_joint": "drivetrain_fl_wheel_joint",
            "front_right_wheel_joint": "drivetrain_fr_wheel_joint",
            "back_left_wheel_joint": "drivetrain_bl_wheel_joint",
            "back_right_wheel_joint": "drivetrain_br_wheel_joint",
        }
        drivetrain: Dict[str, object] = {"type": "mecanum_drive"}
        for key, input_id in joint_fields.items():
            value = inputs.itemById(input_id).value.strip()
            if not value:
                raise ValueError(f"Mecanum Drive requires a joint name for {key.replace('_', ' ')}.")
            drivetrain[key] = value
        drivetrain["wheel_radius"] = _parse_positive_float(
            inputs, "drivetrain_mecanum_wheel_radius_m", "Wheel radius (m)"
        )
        drivetrain["sum_of_robot_center_projection_on_x_y_axis"] = _parse_positive_float(
            inputs, "drivetrain_mecanum_center_sum_m", "Sum of robot-center projection on X/Y axis (m)"
        )
        return drivetrain

    raise ValueError(f"Unrecognized drivetrain selection {selected!r}.")


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
        _apply_auto_detected_drivetrain(inputs, robot)
        _apply_moveit_smart_default_checkbox(inputs, robot)
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

            # REAL GAP FOUND AND FIXED HERE: app.generate_ros_package's
            # use_bounding_box_collision flag (app.attach_collision_proxies)
            # was never exposed here at all, so it was always False from the
            # real add-in -- every Fusion-sourced link kept full-mesh
            # collision geometry unconditionally. Nav2's footprint-radius
            # computation (generators/nav2.py's compute_footprint_radius)
            # explicitly refuses to measure a mesh (it can't know a mesh's
            # true extent) and requires at least one link with a box/
            # cylinder/sphere primitive -- so Nav2 generation was UNREACHABLE
            # from the Fusion UI even after correctly declaring a drivetrain,
            # for every robot whose links are all meshes (i.e. every
            # Fusion-sourced robot, always). Defaulting this ON (unlike
            # generate_ros_package's own default of False, kept for
            # non-Fusion callers like the CLI/tests) since Nav2/Gazebo both
            # want cheap collision proxies anyway and a Fusion user has no
            # other way to attach primitive collision geometry today.
            inputs.addBoolValueInput(
                "use_bounding_box_collision",
                "Use bounding-box collision proxies (required for Nav2)",
                True,
                "",
                True,
            )

            # REAL GAP FOUND AND FIXED HERE: fusion_addin/generators/
            # sensors.py's camera/lidar/imu Gazebo XML + ros_gz_bridge
            # generation already worked, but nothing in this dialog let a
            # user actually attach a sensor to a link -- see
            # _build_sensors'/_set_sensor_inputs_visibility's own docstrings
            # for the fixed-3-slot design and what's/isn't configurable.
            for slot in _SENSOR_SLOT_IDS:
                ids = _sensor_slot_input_ids(slot)
                sensor_dropdown = inputs.addDropDownCommandInput(
                    ids["type"], f"Sensor {slot}", adsk.core.DropDownStyles.TextListDropDownStyle
                )
                sensor_dropdown.listItems.add(_SENSOR_TYPE_NONE, True)
                sensor_dropdown.listItems.add(_SENSOR_TYPE_CAMERA, False)
                sensor_dropdown.listItems.add(_SENSOR_TYPE_LIDAR, False)
                sensor_dropdown.listItems.add(_SENSOR_TYPE_IMU, False)
                sensor_dropdown.listItems.add(_SENSOR_TYPE_DEPTH_CAMERA, False)
                sensor_dropdown.listItems.add(_SENSOR_TYPE_GPS, False)
                inputs.addStringValueInput(
                    ids["parent_link"], f"  Sensor {slot} parent link name", ""
                )
                inputs.addStringValueInput(ids["name"], f"  Sensor {slot} name (optional)", "")
                inputs.addStringValueInput(ids["update_rate"], f"  Sensor {slot} update rate Hz (optional)", "")
            _set_sensor_inputs_visibility(inputs)

            # REAL GAP FOUND AND FIXED HERE: robot.metadata["drivetrain"] --
            # the convention ros2_control.py/nav2.py use to recognize a
            # differential-drive or mecanum-drive mobile base (see
            # ros2_control.py's module docstring for the exact shape) -- was
            # previously only ever set by hand in example scripts
            # (examples/sample_rover.py). A real wheeled robot built through
            # this add-in had NO way to enable Nav2/ros2_control's
            # drivetrain-specific controllers at all; "Nav2" would always
            # fail with "robot.metadata['drivetrain'] is not set" even when
            # the design's wheel joints were correctly detected. See
            # _build_drivetrain_metadata / _set_drivetrain_inputs_visibility
            # below for how these inputs are read and shown/hidden.
            #
            # DropDownStyles.TextListDropDownStyle and ListItems.add's
            # signature confirmed via DropDownStyles.htm / ListItems_add.htm.
            drivetrain_input = inputs.addDropDownCommandInput(
                "drivetrain_type",
                "Drivetrain (for Nav2 / ros2_control)",
                adsk.core.DropDownStyles.TextListDropDownStyle,
            )
            drivetrain_input.listItems.add(_DRIVETRAIN_NONE, True)
            drivetrain_input.listItems.add(_DRIVETRAIN_DIFFERENTIAL, False)
            drivetrain_input.listItems.add(_DRIVETRAIN_MECANUM, False)

            # Plain string inputs (type the exact joint name shown in
            # "Detected Links/Joints" below) rather than a dropdown of
            # detected joints -- consistent with every other free-text input
            # in this file, and avoids needing to rebuild a live-populated
            # dropdown's list items every time root_occurrence changes.
            inputs.addStringValueInput("drivetrain_left_wheel_joint", "Left wheel joint name", "")
            inputs.addStringValueInput("drivetrain_right_wheel_joint", "Right wheel joint name", "")
            inputs.addStringValueInput("drivetrain_wheel_separation_m", "Wheel separation (m)", "")
            inputs.addStringValueInput("drivetrain_wheel_radius_m", "Wheel radius (m)", "")

            inputs.addStringValueInput("drivetrain_fl_wheel_joint", "Front-left wheel joint name", "")
            inputs.addStringValueInput("drivetrain_fr_wheel_joint", "Front-right wheel joint name", "")
            inputs.addStringValueInput("drivetrain_bl_wheel_joint", "Back-left wheel joint name", "")
            inputs.addStringValueInput("drivetrain_br_wheel_joint", "Back-right wheel joint name", "")
            inputs.addStringValueInput("drivetrain_mecanum_wheel_radius_m", "Wheel radius (m)", "")
            inputs.addStringValueInput(
                "drivetrain_mecanum_center_sum_m",
                "Sum of robot-center projection on X/Y axis (m)",
                "",
            )

            _set_drivetrain_inputs_visibility(inputs)

            # One-click "Generate -> Build -> Launch" (opt-in, default off --
            # this adds real wall-clock time and needs a usable WSL/ROS 2
            # environment, checked automatically via bridge.windows.doctor
            # right before the build step in GenerateCommandExecuteHandler).
            inputs.addBoolValueInput(
                "build_in_wsl_after_generate", "Also build in WSL after generating", True, "", False
            )
            default_ws_src = state.get_last_wsl_ros_ws_src() or DEFAULT_WSL_ROS_WS_SRC
            inputs.addStringValueInput(
                "wsl_ros_ws_src_for_build", "WSL colcon workspace src/ (if building)", default_ws_src
            )
            launch_input = inputs.addDropDownCommandInput(
                "launch_after_build", "Launch after build", adsk.core.DropDownStyles.TextListDropDownStyle
            )
            launch_input.listItems.add(_LAUNCH_NONE, True)
            launch_input.listItems.add(_LAUNCH_DISPLAY, False)
            launch_input.listItems.add(_LAUNCH_GAZEBO, False)
            launch_input.listItems.add(_LAUNCH_NAV2, False)
            launch_input.listItems.add(_LAUNCH_BRINGUP, False)
            _set_build_chain_inputs_visibility(inputs)

            # Restore this robot's previously-saved dialog state (if any),
            # BEFORE _refresh_detected_summary's auto-detection runs below --
            # see _apply_persisted_dialog_state's docstring for why a past
            # explicit choice must outrank a fresh guess.
            saved_dialog_state = state.load_generate_dialog_state(default_name)
            if saved_dialog_state:
                _apply_persisted_dialog_state(inputs, saved_dialog_state)

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
    against -- exactly what `_refresh_detected_summary` needs.

    Reacts to "root_occurrence" (re-run extraction -- a root-occurrence
    change actually re-scopes what gets extracted) and to "drivetrain_type"
    (show/hide the relevant wheel-joint/numeric inputs). NOT to "robot_name"
    or any drivetrain_* text field: a robot's name has zero effect on which
    links/joints extraction finds (converter.py only ever uses it as the
    resulting Robot's .name label), and re-running full extraction against a
    live Fusion design on every single keystroke while typing a name (or a
    wheel joint name / wheel radius) would make this dialog feel sluggish on
    anything but a tiny assembly, for a readback that wouldn't even change.
    """

    def notify(self, args: "adsk.core.InputChangedEventArgs") -> None:
        if args.input.id == "root_occurrence":
            _refresh_detected_summary(args.inputs)
        elif args.input.id == "drivetrain_type":
            _set_drivetrain_inputs_visibility(args.inputs)
            _apply_drivetrain_smart_default_checkboxes(args.inputs)
        elif args.input.id == "build_in_wsl_after_generate":
            _set_build_chain_inputs_visibility(args.inputs)
        elif args.input.id in {_sensor_slot_input_ids(slot)["type"] for slot in _SENSOR_SLOT_IDS}:
            _set_sensor_inputs_visibility(args.inputs)


def _validate_generated_package(package_dir: Path) -> list:
    """Runs the same checks "Validate ROS 2 Package" (validate_command.py)
    does, against the package that was JUST generated -- so a structural or
    URDF problem is caught immediately, before the optional WSL build step
    spends real time building something that was doomed anyway. Returns a
    flat list of problem strings (empty == clean), same convention as
    ros2_tools.validate's own functions."""
    problems = validate_package_structure(package_dir)
    urdf_dir = package_dir / "urdf"
    if urdf_dir.is_dir():
        for urdf_file in sorted(p for p in urdf_dir.iterdir() if p.is_file() and p.suffix in (".urdf", ".xacro")):
            problems.extend(f"{urdf_file.name}: {p}" for p in validate_urdf_file(urdf_file))
    return problems


def _build_and_maybe_launch_in_wsl(
    progress_dialog: "adsk.core.ProgressDialog",
    package_dir: Path,
    robot_name: str,
    wsl_ros_ws_src: str,
    launch_choice: str,
    include_gazebo: bool,
) -> str:
    """The "Also build in WSL after generating" / "Launch after build"
    one-click chain: doctor-check the WSL/ROS 2 environment, build the
    just-generated package there, and (opt-in, per `launch_choice`) launch
    it -- all from the same "Generate ROS 2 Package" dialog, instead of
    requiring a separate "Build in WSL" command run afterwards with
    manually-retyped paths.

    Returns a plain-text report string for the caller to fold into its own
    messageBox -- never raises for an ordinary failure (unready
    environment, failed build, failed launch); each is reported as text,
    matching this file's existing PipelineError-style "show the real reason,
    not a traceback" convention. Reuses `progress_dialog` (already open from
    generation) rather than creating a second one, updating only its
    `.message` -- never touching `.minimumValue`/`.maximumValue` here (see
    build_command.py's own real-bug comment on why `.show(..., 0, 0, 0)`
    is invalid) since generation already left them at a valid, non-equal
    range.
    """
    progress_dialog.message = "Checking WSL environment..."
    checks = run_environment_checks(
        distro=DEFAULT_DISTRO, ros_setup=DEFAULT_ROS_SETUP, wsl_ros_ws_src=wsl_ros_ws_src, check_gazebo=include_gazebo
    )
    if not all_critical_passed(checks):
        return f"Package generated, but the WSL environment isn't ready to build it:\n\n{format_report(checks)}"

    progress_dialog.message = "Building in WSL..."

    def _on_output_line(line: str) -> None:
        progress_dialog.message = line[-200:] if line else progress_dialog.message

    build_result = build_package_in_wsl(
        str(package_dir),
        wsl_ros_ws_src,
        distro=DEFAULT_DISTRO,
        ros_setup=DEFAULT_ROS_SETUP,
        on_output_line=_on_output_line,
        timeout=600,
    )
    if not build_result.success:
        return f"Package generated, but the WSL build FAILED (exit {build_result.returncode}):\n\n{build_result.stderr}"

    state.set_last_wsl_ros_ws_src(wsl_ros_ws_src)

    if launch_choice == _LAUNCH_NONE:
        return f"Built successfully in the WSL workspace:\n{wsl_ros_ws_src}"

    launch_file = _LAUNCH_FILES[launch_choice]
    progress_dialog.message = f"Launching {launch_file}..."
    launch_result = launch_ros2_in_wsl(
        package_name=robot_name,
        launch_file=launch_file,
        wsl_ros_ws_src=wsl_ros_ws_src,
        distro=DEFAULT_DISTRO,
        ros_setup=DEFAULT_ROS_SETUP,
    )
    if launch_result.success:
        return f"Built successfully and launched {launch_file} in WSL.\n\n{launch_result.stdout}"
    return f"Built successfully, but launching {launch_file} FAILED:\n\n{launch_result.stderr}"


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
            use_bounding_box_collision = inputs.itemById("use_bounding_box_collision").value

            fusion_app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(fusion_app.activeProduct)
            if design is None:
                ui.messageBox("Fusion2ROS: no active Design (open/activate a design first).")
                return
            if not robot_name:
                ui.messageBox("Fusion2ROS: robot name must not be empty.")
                return

            try:
                drivetrain = _build_drivetrain_metadata(inputs)
                sensors = _build_sensors(inputs)
            except ValueError as exc:
                ui.messageBox(f"Fusion2ROS: {exc}")
                return

            root_component = _selected_root_component(inputs)
            reader = FusionDesignReaderAdapter(design, root_component=root_component)
            robot = app.build_robot_from_reader(reader, robot_name)
            if drivetrain is not None:
                # robot.metadata is a plain mutable dict even though Robot
                # itself is a frozen dataclass -- see
                # _build_drivetrain_metadata's docstring for why this exists
                # and ros2_control.py's module docstring for the shape.
                robot.metadata["drivetrain"] = drivetrain
            robot.sensors.extend(sensors)

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
                    use_bounding_box_collision=use_bounding_box_collision,
                    progress_callback=_on_progress,
                    should_cancel=_should_cancel,
                )

            state.set_last_generated(package_dir, robot_name)
            state.save_generate_dialog_state(robot_name, _collect_persistable_dialog_state(inputs))
            report_lines = [f"Fusion2ROS: generated ROS 2 package at:\n{package_dir}"]

            if include_gazebo and uses_gz_ros2_control_fallback(robot):
                report_lines.append(
                    "Warning: this robot has no differential-drive metadata, so the "
                    "generated Gazebo config uses the gz_ros2_control plugin rather than "
                    "gz-sim's native DiffDrive. gz_ros2_control has a confirmed upstream "
                    "crash (SIGSEGV on startup) on some gz-sim versions -- see "
                    "docs/ARCHITECTURE.md's \"Gazebo\" section before relying on gazebo.launch.py "
                    "for this robot."
                )

            progress_dialog.message = "Validating generated package..."
            validation_problems = _validate_generated_package(package_dir)
            if validation_problems:
                report_lines.append(
                    "Validation found problems (fix these before building/launching):\n"
                    + "\n".join(f"- {p}" for p in validation_problems)
                )

            if inputs.itemById("build_in_wsl_after_generate").value:
                if validation_problems:
                    report_lines.append("Skipped the WSL build step because validation found problems above.")
                else:
                    wsl_ros_ws_src = inputs.itemById("wsl_ros_ws_src_for_build").value.strip() or DEFAULT_WSL_ROS_WS_SRC
                    launch_choice = inputs.itemById("launch_after_build").selectedItem.name
                    report_lines.append(
                        _build_and_maybe_launch_in_wsl(progress_dialog, package_dir, robot_name, wsl_ros_ws_src, launch_choice, include_gazebo)
                    )

            ui.messageBox("\n\n".join(report_lines))
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
