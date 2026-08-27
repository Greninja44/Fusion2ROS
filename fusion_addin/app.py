"""Orchestration layer: RobotModel + mesh files -> generated ROS 2 package.

Deliberately Fusion-symbol-free (no `adsk` import) so it's testable without
Fusion, per ARCHITECTURE.md's "don't put extraction/generation logic
directly inside UI callbacks" rule -- fusion_addin/ui/command.py (the actual
Fusion command handler) is a thin adapter that gathers adsk-side inputs
(the live Design, a robot name, an output directory) and calls straight into
this module; every decision of substance lives here where it can be unit
tested with a fake FusionDesignReader and plain paths.
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Callable, Dict, List, Optional

ProgressCallback = Callable[[str, int, int], None]
CancelCheck = Callable[[], bool]

from robot_model import Robot, JointType

from .extraction.converter import build_robot_model
from .extraction.interface import FusionDesignReader
from .generators.package import generate_package
from .generators.urdf import generate_urdf_xacro

_INVALID_PACKAGE_NAME_CHARS = re.compile(r"[^a-z0-9_]+")


def _sanitize_ros_package_name(name: str) -> str:
    """Turn an arbitrary Fusion component/robot name into a valid ROS 2
    package identifier.

    Real bug found live: Fusion's default root component name is literally
    "Main Assembly", and the "Robot name" UI field in command.py defaults to
    that verbatim -- every generator in this project (package.py,
    ros2_control.py, gazebo.py, nav2.py, urdf.py's `package://` mesh URIs)
    then used the raw robot.name as-is for the package directory name,
    package.xml <name>, and CMakeLists.txt project(). colcon's actual
    package-name validation (catkin_pkg.package.Package.validate, confirmed
    by reading the installed package: requires
    `^[a-zA-Z0-9][a-zA-Z0-9_-]*$`) rejects a name containing a space
    outright -- not a warning, a hard "does not follow naming conventions"
    error -- so colcon silently drops the whole package
    ("ignoring unknown package") and 0 packages get built, no matter how
    correct everything else in the generated tree is. Confirmed live:
    `colcon build --packages-select "Main Assembly"` on a real generated
    package reproduces exactly that failure.

    Sanitizing once here, at the single point where a Fusion-supplied name
    becomes `robot.name`, means every downstream generator (which all read
    `robot.name` directly) gets a valid, consistent identifier for free,
    rather than requiring each call site to sanitize independently and risk
    a mismatch (e.g. a mesh's `package://` URI referencing a different
    string than the package actually installs as).
    """
    sanitized = _INVALID_PACKAGE_NAME_CHARS.sub("_", name.strip().lower())
    sanitized = re.sub("_+", "_", sanitized).strip("_")
    if not sanitized:
        return "robot"
    if not sanitized[0].isalpha():
        sanitized = f"robot_{sanitized}"
    return sanitized


class PipelineError(Exception):
    """Raised for a clear, actionable, pre-generation problem (e.g. a joint
    missing the motor limits URDF requires) -- distinct from a bug, this is
    expected user-facing input the caller should display, not a traceback."""


class GenerationCancelled(Exception):
    """Raised by generate_ros_package when `should_cancel()` returns True at
    one of its `_report()` checkpoints (see that parameter's docstring).
    Deliberately NOT a PipelineError: a PipelineError means "your input is
    wrong, fix it and try again"; this means "you asked to stop, and we did"
    -- a caller (ui/command.py's ProgressDialog integration) should treat it
    as a clean, silent abort, not show a traceback or an error dialog."""


def build_robot_from_reader(reader: FusionDesignReader, robot_name: str) -> Robot:
    """Thin, named pass-through to converter.build_robot_model -- exists so
    callers (fusion_addin/ui/command.py) that need the Robot *before* mesh
    export (mesh export needs the live Design between this step and
    generate_ros_package) don't have to import converter directly.

    Sanitizes `robot_name` into a valid ROS 2 package identifier first (see
    `_sanitize_ros_package_name`) -- this is the one place a raw Fusion
    component/UI name becomes `Robot.name`, which every generator downstream
    treats as the actual package name."""
    return build_robot_model(reader, _sanitize_ros_package_name(robot_name))


def check_missing_actuator_limits(robot: Robot) -> List[str]:
    """Fusion's CAD Joint object carries no motor/velocity/effort data (see
    fusion_adapter.py's _revolute_info/_slider_info), so every revolute or
    prismatic joint extracted straight from Fusion is missing the
    velocity_limit/effort_limit that URDF's <limit> tag requires --
    generate_urdf_xacro refuses to invent those values and raises ValueError
    instead. Surfacing that as one clear, itemized list up front (rather than
    a bare ValueError naming only the first offending joint) is what lets a
    caller show the user exactly what's missing before attempting generation.
    """
    problems = []
    for joint in robot.joints:
        if joint.type not in (JointType.REVOLUTE, JointType.PRISMATIC):
            continue
        missing = [
            attr
            for attr, val in (("velocity_limit", joint.velocity_limit), ("effort_limit", joint.effort_limit))
            if val is None
        ]
        if missing:
            problems.append(f"Joint {joint.name!r} ({joint.type.value}) is missing: {', '.join(missing)}")
    return problems


def attach_mesh_references(robot: Robot, mesh_files: Dict[str, Path]) -> Robot:
    """Sets visual_geometry/collision_geometry (both -- same full-resolution
    mesh, by default) on each link named in mesh_files, using a
    package://<robot.name>/meshes/<filename> reference. Mutates and returns
    `robot`. Links not present in mesh_files are left untouched (no visual --
    e.g. a purely structural/reference link with no exportable body).

    Callers that want a cheaper collision proxy instead of the full mesh
    should call attach_collision_proxies(robot, use_bounding_box_collision=True)
    afterward (generate_ros_package does this automatically when its own
    use_bounding_box_collision parameter is set)."""
    from robot_model import Geometry

    for link in robot.links:
        mesh_path = mesh_files.get(link.name)
        if mesh_path is None:
            continue
        filename = Path(mesh_path).name
        geometry = Geometry(kind="mesh", mesh_path=f"package://{robot.name}/meshes/{filename}")
        link.visual_geometry = geometry
        link.collision_geometry = geometry
    return robot


def splice_xml_fragments(urdf_xacro: str, fragments: List[str]) -> str:
    """Inserts additional top-level XML elements (e.g. ros2_control.py's
    <ros2_control> element, or gazebo.py's <gazebo_fragment>-wrapped blocks)
    into an already-generated <robot>...</robot> URDF/xacro document, just
    before the closing tag.

    A <gazebo_fragment> wrapper (gazebo.py's documented splice-point
    convention, since it emits more than one sibling <gazebo> element) is
    unwrapped -- its children are inserted directly, since URDF itself has
    no <gazebo_fragment> tag. Any other fragment is inserted as a single
    element as-is (e.g. ros2_control.py's <ros2_control> root).
    """
    pieces = []
    for frag in fragments:
        root = ET.fromstring(frag)
        if root.tag == "gazebo_fragment":
            pieces.extend(ET.tostring(child, encoding="unicode") for child in root)
        else:
            pieces.append(ET.tostring(root, encoding="unicode"))

    text = urdf_xacro.rstrip()
    if not text.endswith("</robot>"):
        raise ValueError("urdf_xacro must end with </robot> to splice fragments into it")
    return text[: -len("</robot>")] + "".join(pieces) + "</robot>\n"


def attach_collision_proxies(robot: Robot, use_bounding_box_collision: bool = False) -> Robot:
    """Optionally replaces each link's collision_geometry with a cheap
    axis-aligned box proxy derived from its Fusion bounding box, instead of
    reusing the full-resolution visual mesh (attach_mesh_references' default).
    Full-mesh collision is expensive for a physics engine (Gazebo/Bullet/ODE)
    and usually unnecessary -- a bounding-box proxy is standard, well-known
    real-world practice.

    Only touches links that have a "bounding_box_size" key in `metadata`
    (populated by fusion_addin/extraction/converter.py's build_robot_model
    for Fusion-sourced links only -- see FusionOccurrence.bounding_box). A
    hand-authored Robot (e.g. examples/sample_arm.py) has no such metadata
    and is left completely untouched, keeping whatever collision_geometry it
    already had. visual_geometry is never modified here.

    No-op (robot returned unchanged) when use_bounding_box_collision is
    False -- the default, so every existing caller keeps its current
    full-mesh-collision behavior unless it opts in.

    Simplification, documented same as converter.py's _bounding_box_size:
    the box is world-axis-aligned, not rotated to the link's own local frame.
    """
    if not use_bounding_box_collision:
        return robot

    from robot_model import Geometry

    for link in robot.links:
        size = link.metadata.get("bounding_box_size")
        if size is None:
            continue
        link.collision_geometry = Geometry(kind="box", size=size)
    return robot


def format_robot_summary(robot: Robot) -> str:
    """Human-readable readback of a built Robot's detected links and joints.

    Backs the Fusion UI's read-only "Detected Links/Joints" text box
    (ui/command.py's GenerateCommandCreatedHandler / InputChangedEventHandler)
    -- the mockup's "Detected Links [check] base_link [check] link1..."
    checklist, simplified here to a plain read-only listing rather than a
    per-item checkbox tree (Fusion's TextBoxCommandInput has no such control;
    building one would need a BrowserCommandInput/custom HTML palette, out of
    scope for this pass). Pure formatting of an already-built Robot, no adsk
    dependency -- unit tested directly with a hand-built Robot fixture.
    """
    lines: List[str] = [f"Links ({len(robot.links)}):"]
    for link in robot.links:
        suffix = f"  (parent: {link.parent})" if link.parent else "  (root)"
        lines.append(f"  - {link.name}{suffix}")

    lines.append("")
    lines.append(f"Joints ({len(robot.joints)}):")
    if not robot.joints:
        lines.append("  (none)")
    for joint in robot.joints:
        lines.append(f"  - {joint.name}  [{joint.type.value}]  {joint.parent} -> {joint.child}")

    return "\n".join(lines)


def robot_summary_as_dict(robot: Robot) -> dict:
    """Structured (not pre-formatted) equivalent of format_robot_summary,
    same underlying information, for UI surfaces that can render a real tree
    instead of a plain-text box -- specifically the Palette-based "Detected
    Links/Joints" view in ui/command.py (see that module's
    detected_summary.html), which renders this straight to HTML/JS via JSON
    over Palette.sendInfoToHTML. Kept as a separate function rather than
    having format_robot_summary build on it (or vice versa) so the existing
    plain-text CLI/log path (format_robot_summary) is untouched byte-for-byte.

    Fusion-symbol-free, pure formatting of an already-built Robot -- unit
    tested directly with a hand-built Robot fixture, same as
    format_robot_summary.

    Shape:
        {
            "links": [{"name": str, "parent": str | None, "is_root": bool}, ...],
            "joints": [{"name": str, "type": str, "parent": str, "child": str}, ...],
        }
    `type` is the joint's JointType.value string (e.g. "revolute"), matching
    format_robot_summary's `[{joint.type.value}]` rendering.
    """
    return {
        "links": [
            {"name": link.name, "parent": link.parent, "is_root": link.parent is None} for link in robot.links
        ],
        "joints": [
            {"name": joint.name, "type": joint.type.value, "parent": joint.parent, "child": joint.child}
            for joint in robot.joints
        ],
    }


def generate_ros_package(
    robot: Robot,
    mesh_files: Dict[str, Path],
    output_dir: Path,
    include_ros2_control: bool = False,
    include_gazebo: bool = False,
    include_moveit: bool = False,
    include_nav2: bool = False,
    moveit_group_name: str = "arm",
    use_bounding_box_collision: bool = False,
    progress_callback: Optional[ProgressCallback] = None,
    should_cancel: Optional[CancelCheck] = None,
) -> Path:
    """robot -> URDF/Xacro text -> full ROS 2 package tree under
    output_dir/<robot.name>/. Raises PipelineError (not a bare ValueError)
    with the full itemized list if any joint is missing required limits, or
    if a requested optional output (MoveIt/Nav2) isn't suitable for `robot`.

    The four `include_*` flags mirror the Fusion UI's planned output
    checkboxes (URDF/Xacro and the base ROS 2 package are always produced;
    ros2_control/Gazebo/MoveIt 2/Nav2 are each opt-in since not every robot
    needs or supports all four -- an arm has no business with Nav2, and a
    mobile base with no `metadata["drivetrain"]` has no business with
    MoveIt's arm-planning groups).

    use_bounding_box_collision (default False, opt-in): after mesh references
    are attached, replace collision_geometry with a simplified bounding-box
    proxy for links that have one available (see attach_collision_proxies).
    Left False, output is byte-identical to before this parameter existed.

    progress_callback (default None, opt-in): if given, called as
    `callback(stage_description, step_number, total_steps)` once per stage
    (1-indexed step_number, `total_steps` fixed for the whole call and
    computed up front from which include_* flags are set, so a caller can
    render a determinate progress bar rather than a spinner). Exists so the
    Fusion UI can drive a real adsk.core.ProgressDialog during generation
    instead of appearing to hang (generation of a complex robot with many
    optional outputs is not instantaneous), and so the standalone CLI
    (scripts/generate_from_json.py) can print progress for a large batch.
    Never called with a partially-failed stage -- if a stage raises, this
    function raises too, mid-callback-sequence, exactly where the error
    actually occurred.

    should_cancel (default None, opt-in): if given, called with no arguments
    at every stage checkpoint (right after progress_callback, before that
    stage's work begins) -- if it returns truthy, generation stops
    immediately by raising GenerationCancelled instead of proceeding.
    Cooperative, not preemptive: this only checks between stages (URDF
    generation, one generator's config, writing to disk), never mid-stage,
    since none of those individual steps are themselves interruptible. Exists
    so the Fusion UI can wire a real ProgressDialog's `wasCancelled` into
    an actual, working cancel -- passing this without also passing
    progress_callback is legal but pointless, since there'd be no UI moment
    for a user to have clicked cancel from.
    """
    total_steps = 3  # validate+mesh, URDF, write-to-disk -- always present
    if include_ros2_control:
        total_steps += 1
    if include_gazebo:
        total_steps += 1
        if robot.sensors:
            total_steps += 1
    if include_moveit:
        total_steps += 1
    if include_nav2:
        total_steps += 1

    step = 0

    def _report(stage_description: str) -> None:
        nonlocal step
        step += 1
        if progress_callback is not None:
            progress_callback(stage_description, step, total_steps)
        if should_cancel is not None and should_cancel():
            raise GenerationCancelled(
                f"Generation cancelled before stage {step}/{total_steps} ({stage_description!r})"
            )

    _report("Checking actuator limits and attaching mesh/collision geometry")
    problems = check_missing_actuator_limits(robot)
    if problems:
        raise PipelineError(
            "Cannot generate URDF -- the following joints are missing motor limits "
            "required by the URDF <limit> tag (set them via an Actuator, or edit the "
            "Robot before generating):\n  " + "\n  ".join(problems)
        )

    attach_mesh_references(robot, mesh_files)
    attach_collision_proxies(robot, use_bounding_box_collision)

    _report("Generating URDF/Xacro")
    urdf_xacro = generate_urdf_xacro(robot)

    fragments: List[str] = []
    extra_files: Dict[str, str] = {}
    extra_depends: List[str] = []

    if include_ros2_control:
        _report("Generating ros2_control configuration")
        from .generators.ros2_control import (
            generate_control_launch,
            generate_controllers_yaml,
            generate_ros2_control_xml,
        )

        fragments.append(generate_ros2_control_xml(robot))
        extra_files["config/controllers.yaml"] = generate_controllers_yaml(robot)
        extra_files["launch/control.launch.py"] = generate_control_launch(robot)

    if include_gazebo:
        _report("Generating Gazebo Sim configuration")
        # Real gap found while adding Gazebo support: package.xml never
        # declared ros_gz_sim/ros_gz_bridge/gz_ros2_control regardless of
        # include_gazebo -- see generate_package's extra_depends docstring.
        extra_depends.extend(["ros_gz_sim", "ros_gz_bridge", "gz_ros2_control"])
        from .generators.gazebo import (
            generate_gazebo_ros_bridge_yaml,
            generate_gazebo_xml,
            generate_spawn_launch,
            generate_world_sdf,
        )

        fragments.append(generate_gazebo_xml(robot))
        extra_files["worlds/empty.sdf"] = generate_world_sdf()

        # generate_gazebo_ros_bridge_yaml covers joint_states (any non-fixed
        # joint) and, for a differential-drive robot, the native DiffDrive
        # plugin's cmd_vel/odom/tf topics (see gazebo.py's module docstring
        # for why DiffDrive replaces gz_ros2_control for that case). Both
        # this and sensors.py's generate_ros_gz_bridge_yaml emit the same
        # flat "- ros_topic_name: ..." list style with no "[...]" wrapper,
        # so concatenating their text is one valid combined bridge config --
        # exactly what a single `ros_gz_bridge parameter_bridge` process
        # needs (it takes one config file, not one per source).
        bridge_yaml_parts = []
        gazebo_bridge_yaml = generate_gazebo_ros_bridge_yaml(robot)
        if gazebo_bridge_yaml:
            bridge_yaml_parts.append(gazebo_bridge_yaml)

        if robot.sensors:
            _report("Generating sensor configuration")
            from .generators.sensors import generate_ros_gz_bridge_yaml, generate_sensor_gazebo_xml

            fragments.append(generate_sensor_gazebo_xml(robot))
            bridge_yaml_parts.append(generate_ros_gz_bridge_yaml(robot))

        include_bridge = bool(bridge_yaml_parts)
        if include_bridge:
            extra_files["config/ros_gz_bridge.yaml"] = "".join(bridge_yaml_parts)

        # Written after bridge_yaml_parts is known, so the spawn launch only
        # starts `ros_gz_bridge`'s parameter_bridge node when there's an
        # actual config/ros_gz_bridge.yaml for it to read -- see
        # generate_spawn_launch's docstring for the real gap this closes
        # (the bridge config was previously written but never started).
        extra_files["launch/gazebo.launch.py"] = generate_spawn_launch(robot, include_bridge=include_bridge)

    if include_moveit:
        _report("Generating MoveIt 2 configuration")
        from .generators.moveit import (
            detect_moveit_suitability,
            generate_joint_limits_yaml,
            generate_kinematics_yaml,
            generate_moveit_controllers_yaml,
            generate_moveit_demo_launch,
            generate_ompl_planning_yaml,
            generate_srdf,
        )

        problems = detect_moveit_suitability(robot)
        if problems:
            raise PipelineError(
                "Cannot generate MoveIt 2 config -- robot is not suitable:\n  " + "\n  ".join(problems)
            )
        extra_files[f"config/{robot.name}.srdf"] = generate_srdf(robot, group_name=moveit_group_name)
        extra_files["config/joint_limits.yaml"] = generate_joint_limits_yaml(robot, moveit_group_name)
        extra_files["config/kinematics.yaml"] = generate_kinematics_yaml(robot, moveit_group_name)
        extra_files["config/moveit_controllers.yaml"] = generate_moveit_controllers_yaml(robot, moveit_group_name)
        # move_group won't start at all without a planning pipeline
        # registered -- see generate_ompl_planning_yaml's docstring.
        extra_files["config/ompl_planning.yaml"] = generate_ompl_planning_yaml()
        # moveit.py defaults to assuming a separate "<robot>_moveit_config"
        # package (the real-world moveit_setup_assistant convention) -- this
        # pipeline puts every generator's output into ONE combined package
        # instead, so override it to match where the SRDF/kinematics/etc.
        # files above actually land (found via a real move_group launch
        # that failed to find "sample_arm_moveit_config" until this was added).
        extra_files["launch/moveit_demo.launch.py"] = generate_moveit_demo_launch(
            robot, moveit_group_name, moveit_config_package=robot.name
        )

    if include_nav2:
        _report("Generating Nav2 configuration")
        from .generators.nav2 import (
            detect_nav2_suitability,
            generate_map_yaml_stub,
            generate_nav2_bringup_launch,
            generate_nav2_params_yaml,
        )

        problems = detect_nav2_suitability(robot)
        if problems:
            raise PipelineError("Cannot generate Nav2 config -- robot is not suitable:\n  " + "\n  ".join(problems))
        extra_files["config/nav2_params.yaml"] = generate_nav2_params_yaml(robot)
        extra_files["launch/nav2_bringup.launch.py"] = generate_nav2_bringup_launch(robot)
        extra_files["config/map.yaml"] = generate_map_yaml_stub(robot)

    if fragments:
        urdf_xacro = splice_xml_fragments(urdf_xacro, fragments)

    _report("Writing ROS 2 package to disk")
    # mesh_files here is keyed by link name (export_link_meshes' contract);
    # generate_package wants it keyed by the mesh's relative filename inside
    # the package (its own, independent contract) -- re-key at this boundary
    # rather than making either side guess the other's convention.
    package_mesh_files = {Path(path).name: path for path in mesh_files.values()}
    return generate_package(
        robot, urdf_xacro, package_mesh_files, output_dir, extra_files=extra_files, extra_depends=extra_depends
    )


def run_pipeline(
    reader: FusionDesignReader,
    robot_name: str,
    output_dir: Path,
    mesh_files: Optional[Dict[str, Path]] = None,
    **generate_kwargs,
) -> tuple:
    """Full pipeline: FusionDesignReader -> Robot -> generated package.
    Returns (robot, package_dir). `mesh_files` is supplied by the caller
    (typically fusion_addin/generators/mesh.export_link_meshes, run against
    the live Design before this is called -- kept as a separate step here
    since mesh export needs the live adsk Design/Occurrence objects that
    FusionDesignReader's plain-data abstraction deliberately doesn't carry).
    `generate_kwargs` forwards the include_ros2_control/include_gazebo/
    include_moveit/include_nav2/moveit_group_name/use_bounding_box_collision
    flags to generate_ros_package.
    """
    robot = build_robot_from_reader(reader, robot_name)
    package_dir = generate_ros_package(robot, mesh_files or {}, Path(output_dir), **generate_kwargs)
    return robot, package_dir
