"""RobotModel -> MoveIt 2 scaffolding: SRDF, joint limits, kinematics,
controller, and demo launch generation.

Pure function of a `robot_model.Robot`: no Fusion API, no ROS/rclpy, no
filesystem access, no network, and no third-party packages -- same
constraint `robot_model/schema.py` and `fusion_addin/generators/urdf.py`
are held to (see docs/ARCHITECTURE.md). Safe to import and unit-test with
plain `python3 -m pytest`, no MoveIt 2 installation required.

Scope
-----
This module produces SEPARATE, ADDITIONAL files (SRDF text, YAML config
text, a launch-file's Python source text) -- it does NOT assemble a full
`<robot>_moveit_config` ROS 2 package tree. That assembly (package.xml,
CMakeLists.txt, directory layout, writing these strings to disk) is
`fusion_addin/generators/package.py`'s job, owned separately. Every
function here is a pure `Robot -> str` (or `Robot -> List[str]`)
transform.

Single-chain assumption (branchy-robot handling: MVP, documented here)
------------------------------------------------------------------------
`detect_moveit_suitability` requires the robot's kinematic tree to be a
single, unbranched chain from the root link to one leaf link (a link that
is nobody's parent). If the tree branches anywhere (more than one link
has more than one child), this module reports a clear problem and refuses
to guess which branch is "the arm" -- it does NOT implement the
longest-chain-plus-secondary-group heuristic the project brief calls out
as a legitimate stretch goal (e.g. an arm with a gripper hanging off the
last link). That heuristic needs: picking the longest root->leaf path as
the primary group, treating the remaining sub-chain below the fork point
as a second small group, and SRDF/kinematics/controllers support for
multiple groups in one call. All of that is a real, bounded amount of
extra work that was consciously deferred to keep this an honestly-scoped,
correctly-working MVP rather than a half-working multi-group generator.
A caller with a legitimate branchy robot (e.g. arm + gripper) can still
use these generators today by calling `generate_srdf` (etc.) once per
branch with explicit `base_link`/`tip_link` -- only the automatic
base_link/tip_link derivation (and `detect_moveit_suitability`'s
suitability verdict) assumes a single chain.

Shared ros2_control naming convention (unverified against a sibling module)
-----------------------------------------------------------------------------
`fusion_addin/generators/ros2_control.py` (built by a parallel agent, may
not exist yet in this worktree) is expected to define one
`joint_trajectory_controller` per arm-type robot, controlling every
non-fixed joint via position/velocity/effort interfaces with the
`FollowJointTrajectory` action. `generate_moveit_controllers_yaml` below
assumes that controller is named `"<group_name>_controller"` and
references it by that name. This is a naming convention agreed in the
project brief, not something importable/verifiable from this file --
if the sibling module ends up using a different name, only the
controller *name* in the generated YAML needs to change to match.

MoveIt config values used below (kinematics defaults, YAML/SRDF shapes)
-----------------------------------------------------------------------
MoveIt 2 was not installed in this environment as of this session (see
docs/ARCHITECTURE.md), so none of this was checked against a live
`moveit_setup_assistant` run or `moveit_configs_utils` import. Confidence
levels, spelled out here rather than left implicit:
  * SRDF element shapes (`<group>`, `<chain>`, `<group_state>`,
    `<disable_collisions>`) -- high confidence; this is the stable,
    long-documented SRDF XML schema, unchanged across MoveIt/MoveIt 2.
  * `kdl_kinematics_plugin/KDLKinematicsPlugin` plugin id, and the
    `kinematics_solver_search_resolution: 0.005`,
    `kinematics_solver_timeout: 0.005` defaults -- high confidence; these
    are the long-standing values `moveit_setup_assistant` has emitted for
    years and appear throughout MoveIt's own tutorial/demo configs, but
    were not re-verified against a live install this session.
  * `moveit_simple_controller_manager` YAML shape (`controller_names`,
    `action_ns`, `type: FollowJointTrajectory`, `default`, `joints`) --
    high confidence; this is MoveIt 2's documented controller-manager
    plugin format.
  * The hand-rolled demo launch file -- see `generate_moveit_demo_launch`'s
    docstring for exactly what's simplified/omitted and why
    `moveit_configs_utils.MoveItConfigsBuilder` was not used.
"""

import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

from robot_model import Joint, JointType, Robot

# --- shared chain-walking helpers -------------------------------------------


def _joints_by_parent(robot: Robot) -> Dict[str, List[Joint]]:
    d: Dict[str, List[Joint]] = {}
    for j in robot.joints:
        d.setdefault(j.parent, []).append(j)
    return d


def _joints_by_child(robot: Robot) -> Dict[str, List[Joint]]:
    d: Dict[str, List[Joint]] = {}
    for j in robot.joints:
        d.setdefault(j.child, []).append(j)
    return d


def _leaf_link_names(robot: Robot) -> List[str]:
    """Links that are nobody's parent -- i.e. never appear as a Joint.parent."""
    parent_names = {j.parent for j in robot.joints}
    return [l.name for l in robot.links if l.name not in parent_names]


def _resolve_chain(robot: Robot, base_link: Optional[str], tip_link: Optional[str]) -> Tuple[str, str]:
    """Fill in base_link/tip_link when not given, assuming (and requiring)
    a single unbranched chain -- see module docstring's branchy-robot note."""
    if base_link is None:
        root = robot.root_link()
        if root is None:
            raise ValueError("Robot has no unique root link; pass base_link explicitly.")
        base_link = root.name
    if tip_link is None:
        leaves = _leaf_link_names(robot)
        if len(leaves) != 1:
            raise ValueError(
                f"Robot does not have exactly one leaf link (found {sorted(leaves)!r}); "
                "cannot auto-derive tip_link for a branchy robot -- pass tip_link "
                "explicitly to select one chain (see detect_moveit_suitability and "
                "this module's branchy-robot handling note)."
            )
        tip_link = leaves[0]
    return base_link, tip_link


def _chain_joints(robot: Robot, base_link: str, tip_link: str) -> List[Joint]:
    """All joints (including fixed ones), in base_link -> tip_link order,
    along the path from tip_link back up to base_link."""
    joints_by_child = _joints_by_child(robot)
    chain: List[Joint] = []
    current = tip_link
    visited = set()
    while current != base_link:
        if current in visited:
            raise ValueError(f"Cycle detected while walking the chain toward {base_link!r}.")
        visited.add(current)
        candidates = joints_by_child.get(current)
        if not candidates:
            raise ValueError(
                f"No path from base_link {base_link!r} to tip_link {tip_link!r}: "
                f"link {current!r} has no parent joint."
            )
        if len(candidates) > 1:
            raise ValueError(f"Link {current!r} is the child of multiple joints -- ambiguous chain.")
        joint = candidates[0]
        chain.append(joint)
        current = joint.parent
    chain.reverse()
    return chain


def _group_chain(robot: Robot, base_link: Optional[str] = None, tip_link: Optional[str] = None) -> Tuple[str, str, List[Joint]]:
    base_link, tip_link = _resolve_chain(robot, base_link, tip_link)
    return base_link, tip_link, _chain_joints(robot, base_link, tip_link)


def _non_fixed(joints: List[Joint]) -> List[Joint]:
    return [j for j in joints if j.type != JointType.FIXED]


def _fmt_float(value: float) -> str:
    """Deterministic, non-scientific-notation float formatting -- same
    scheme as fusion_addin/generators/urdf.py's `_fmt_float`, duplicated
    here rather than imported so this module has no dependency on another
    generator's private internals."""
    s = f"{float(value):.8f}"
    if "." in s:
        s = s.rstrip("0")
        if s.endswith("."):
            s += "0"
    return s


# --- 1. suitability detection ------------------------------------------------


def detect_moveit_suitability(robot: Robot) -> List[str]:
    """Return a list of problems that make `robot` unsuitable for the
    MoveIt scaffolding this module generates. Empty list == suitable.

    Checks:
      * `robot.validate()` passes (a structurally broken graph can't be
        reasoned about further, so other checks are skipped if it fails).
      * At least one non-fixed joint exists (otherwise there is nothing
        for MoveIt to plan motion for).
      * `robot.metadata` does not carry a `"drivetrain"` key -- a
        mobile/drivetrain robot needs Nav2-style navigation, not a MoveIt
        arm planning group, and silently generating a nonsense planning
        group for e.g. a differential-drive base would be worse than
        refusing.
      * The kinematic tree from the root link is a single, unbranched
        chain to exactly one leaf link. See the module docstring for why
        a branchy tree is refused outright rather than guessing which
        branch is "the arm" (documented MVP scope, not an oversight).
    """
    problems: List[str] = []

    validation_problems = robot.validate(raise_on_error=False)
    if validation_problems:
        problems.append(
            "Robot fails basic structural validation, which must be fixed before "
            "MoveIt suitability can be assessed: " + "; ".join(validation_problems)
        )
        return problems

    non_fixed_joints = [j for j in robot.joints if j.type != JointType.FIXED]
    if not non_fixed_joints:
        problems.append("Robot has no non-fixed joints -- there is nothing for MoveIt to plan motion for.")

    if "drivetrain" in robot.metadata:
        problems.append(
            "Robot is flagged as a mobile/drivetrain robot "
            f"(metadata['drivetrain'] = {robot.metadata['drivetrain']!r}). MoveIt's "
            "arm-planning-group model doesn't apply the same way to a mobile base -- "
            "generate Nav2 configuration for this robot instead of MoveIt scaffolding."
        )

    root = robot.root_link()
    if root is None:
        problems.append("Robot has no unique root link; cannot determine a kinematic chain.")
    else:
        joints_by_parent = _joints_by_parent(robot)
        branch_points = sorted(name for name, js in joints_by_parent.items() if len(js) > 1)
        leaves = sorted(_leaf_link_names(robot))
        if branch_points:
            problems.append(
                f"Robot's kinematic tree branches at link(s) {branch_points}. MoveIt "
                "needs a single, unambiguous chain from the root link to one tip link "
                "to form a planning group; this generator does not guess which branch "
                "is 'the arm' for a multi-branch robot (a longest-chain-plus-secondary-"
                "group heuristic for cases like an arm with a gripper is a documented, "
                "deliberately deferred stretch goal -- see this module's docstring). "
                f"Leaf links found: {leaves}. Pass explicit base_link/tip_link to "
                "generate_srdf (etc.) for each branch you want as its own group instead."
            )
        elif len(leaves) != 1:
            problems.append(f"Robot does not have exactly one leaf link (found {leaves}).")

    return problems


# --- 2. SRDF -----------------------------------------------------------------


def _home_value(joint: Joint) -> float:
    """0.0 if that's within [lower, upper], else the midpoint. Continuous
    joints (no position limits) and any joint missing limit data default
    to 0.0, which is always a valid position for them."""
    if joint.lower_limit is None or joint.upper_limit is None:
        return 0.0
    if joint.lower_limit <= 0.0 <= joint.upper_limit:
        return 0.0
    return (joint.lower_limit + joint.upper_limit) / 2.0


def generate_srdf(
    robot: Robot,
    group_name: str = "arm",
    base_link: Optional[str] = None,
    tip_link: Optional[str] = None,
) -> str:
    """Render `robot` to SRDF XML text for planning group `group_name`.

    If `base_link`/`tip_link` are omitted they're derived assuming a
    single unbranched chain (root link -> the one leaf link) -- see
    `detect_moveit_suitability` and the module docstring's branchy-robot
    note. Pass them explicitly to target one branch of a branchy robot.

    Emits:
      * one `<group>` containing a `<chain base_link=".." tip_link=".."/>`
      * one `<group_state name="home">` with every non-fixed joint in that
        chain set to 0.0 (or the limit midpoint if 0.0 is out of range)
      * `<disable_collisions>` for every directly-connected (parent/child)
        link pair in the WHOLE robot (not just this group's chain) --
        adjacent links always touch at their shared joint, so this is
        standard SRDF practice to avoid constant false-positive self
        collisions, independent of which single group is being emitted.
    """
    robot.validate()
    base_link, tip_link, chain_joints = _group_chain(robot, base_link, tip_link)
    non_fixed_chain_joints = _non_fixed(chain_joints)

    root = ET.Element("robot", {"name": robot.name})

    group = ET.SubElement(root, "group", {"name": group_name})
    ET.SubElement(group, "chain", {"base_link": base_link, "tip_link": tip_link})

    state = ET.SubElement(root, "group_state", {"name": "home", "group": group_name})
    for joint in non_fixed_chain_joints:
        ET.SubElement(state, "joint", {"name": joint.name, "value": _fmt_float(_home_value(joint))})

    for joint in robot.joints:
        ET.SubElement(root, "disable_collisions", {"link1": joint.parent, "link2": joint.child, "reason": "Adjacent"})

    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode")
    return f'<?xml version="1.0"?>\n{body}\n'


# --- 3. joint_limits.yaml -----------------------------------------------------


def generate_joint_limits_yaml(robot: Robot, group_name: str = "arm") -> str:
    """YAML with one `joint_limits.<joint_name>` entry per non-fixed joint
    in the (single, auto-derived) chain, `max_velocity` pulled straight
    from `Joint.velocity_limit`.

    Raises:
        ValueError: naming the joint, if any non-fixed joint in the chain
            has `velocity_limit is None` -- same "don't invent physical
            values" principle as `generate_urdf_xacro`'s limit check; a
            joint with no known velocity limit can't get a real
            joint_limits.yaml entry.
    """
    robot.validate()
    _, _, chain_joints = _group_chain(robot)
    non_fixed_chain_joints = _non_fixed(chain_joints)

    lines = [
        "# Generated by fusion_addin.generators.moveit.generate_joint_limits_yaml",
        f'# MoveIt joint_limits.yaml for planning group "{group_name}" of robot "{robot.name}".',
        "joint_limits:",
    ]
    for joint in non_fixed_chain_joints:
        if joint.velocity_limit is None:
            raise ValueError(
                f"Joint {joint.name!r} has no velocity_limit set. MoveIt's "
                "joint_limits.yaml requires max_velocity for every joint in the "
                "planning group, and this generator will not invent a physical "
                "value (same principle fusion_addin/generators/urdf.py applies to "
                "URDF's <limit> tag). Set Joint.velocity_limit before generating."
            )
        lines.append(f"  {joint.name}:")
        lines.append("    has_velocity_limits: true")
        lines.append(f"    max_velocity: {_fmt_float(joint.velocity_limit)}")
        lines.append("    has_acceleration_limits: false")

    return "\n".join(lines) + "\n"


# --- 4. kinematics.yaml -------------------------------------------------------


def generate_kinematics_yaml(robot: Robot, group_name: str = "arm") -> str:
    """YAML selecting MoveIt's standard KDL kinematics plugin for
    `group_name`, with MoveIt's own long-standing default solver
    parameters (search resolution and timeout of 0.005s). See the module
    docstring for confidence notes on these specific values -- KDL is the
    always-available, no-extra-dependency default; a specialized IK
    plugin (TRAC-IK/IKFast) is deliberately NOT assumed to be present.
    """
    robot.validate()

    lines = [
        "# Generated by fusion_addin.generators.moveit.generate_kinematics_yaml",
        f'# MoveIt kinematics.yaml for planning group "{group_name}" of robot "{robot.name}".',
        "# KDL: MoveIt's standard, always-available, no-extra-dependency default IK",
        "# plugin. A specialized solver (TRAC-IK/IKFast) is not assumed to be",
        "# available and is not used here.",
        f"{group_name}:",
        "  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin",
        "  kinematics_solver_search_resolution: 0.005",
        "  kinematics_solver_timeout: 0.005",
    ]
    return "\n".join(lines) + "\n"


# --- 5. moveit_controllers.yaml ------------------------------------------------


def generate_moveit_controllers_yaml(robot: Robot, group_name: str = "arm") -> str:
    """YAML in `moveit_simple_controller_manager` format, one controller
    entry named `"<group_name>_controller"` -- the ros2_control controller
    name convention this module assumes `fusion_addin/generators/
    ros2_control.py` (a parallel, possibly-not-yet-existing module) uses
    for arm-type robots. See module docstring for the full rationale.
    """
    robot.validate()
    _, _, chain_joints = _group_chain(robot)
    non_fixed_chain_joints = _non_fixed(chain_joints)
    controller_name = f"{group_name}_controller"

    lines = [
        "# Generated by fusion_addin.generators.moveit.generate_moveit_controllers_yaml",
        f'# MoveIt moveit_controllers.yaml for planning group "{group_name}" of robot "{robot.name}".',
        "#",
        f'# ASSUMED SHARED CONVENTION: "{controller_name}" is expected to be defined',
        "# as a joint_trajectory_controller by fusion_addin/generators/ros2_control.py",
        '# (controller name = "<group_name>_controller"), which was not importable',
        "# from this worktree at generation time -- verify the name matches if that",
        "# module's convention ever changes.",
        "moveit_simple_controller_manager:",
        "  controller_names:",
        f"    - {controller_name}",
        f"  {controller_name}:",
        "    action_ns: follow_joint_trajectory",
        "    type: FollowJointTrajectory",
        "    default: true",
        "    joints:",
    ]
    for joint in non_fixed_chain_joints:
        lines.append(f"      - {joint.name}")

    return "\n".join(lines) + "\n"


# --- 6. demo launch file -------------------------------------------------------


def generate_moveit_demo_launch(
    robot: Robot, group_name: str = "arm", moveit_config_package: Optional[str] = None
) -> str:
    """Return the Python source text of a `launch`/`launch_ros` launch
    file starting `move_group` and `rviz2` for `robot`.

    Deliberately hand-rolled rather than built on
    `moveit_configs_utils.MoveItConfigsBuilder` (the modern, "correct" way
    real `moveit_setup_assistant`-generated packages wire this up):
    MoveIt 2, and therefore `moveit_configs_utils`, was not installed in
    the environment this generator was developed/tested in (checked via
    `ros2 pkg list | grep -i moveit` and `dpkg -l | grep moveit`, both
    empty), so its API/availability could not be verified here. If this
    file is regenerated in an environment where `moveit_configs_utils` is
    confirmed importable, switching to `MoveItConfigsBuilder` is the
    better long-term approach and should replace this hand-rolled version.

    Simplified/omitted relative to a real moveit_setup_assistant package
    (documented explicitly, not silently dropped):
      * No OMPL/planning-pipeline-specific parameters (planner configs,
        default planning time, etc.) -- `move_group` is started with just
        the robot/semantic/kinematics/joint-limits/controller parameters
        needed to plan and execute, not a full tuned pipeline.
      * No generated MoveIt RViz config (`moveit.rviz` with the
        MotionPlanning display pre-added) -- `rviz2` is launched plain;
        a user adds the MotionPlanning display by hand, or a future
        generator can emit one the way `package.py` emits a plain
        display `.rviz` file for URDF-only viewing.
      * No `warehouse`/`move_group_capabilities`/sensor-plugin wiring.
      * Assumes a conventional `<robot>_moveit_config` package share
        directory layout of `config/<robot>.srdf`, `config/kinematics.yaml`,
        `config/joint_limits.yaml`, `config/moveit_controllers.yaml` --
        actually writing those files into that layout is
        fusion_addin/generators/package.py's job (not this module's);
        this launch file only encodes the assumption of where they'll be.
        Pass `moveit_config_package` to override this default (e.g.
        Fusion2ROS's own integration in fusion_addin/app.py writes every
        generator's output into ONE combined `<robot>` package rather than
        splitting out a separate `<robot>_moveit_config` package, so it
        passes `moveit_config_package=robot.name` here to match).
    """
    robot.validate()

    description_package = robot.name
    moveit_config_package = moveit_config_package or f"{robot.name}_moveit_config"
    srdf_file = f"{robot.name}.srdf"

    content = f'''"""MoveIt 2 demo launch file for "{robot.name}" (planning group "{group_name}").

Auto-generated by fusion_addin/generators/moveit.py's generate_moveit_demo_launch.
Regenerate rather than editing by hand.

Hand-rolled rather than using moveit_configs_utils.MoveItConfigsBuilder -- MoveIt 2
was not installed in the environment this was generated against, so that helper's
API/availability could not be verified. See generate_moveit_demo_launch's docstring
in fusion_addin/generators/moveit.py for the full list of simplifications relative
to a real moveit_setup_assistant-generated package (no tuned OMPL pipeline params,
no pre-built MotionPlanning rviz display, etc).

Assumes the standard package layout this generator was built against:
  "{description_package}"        share dir: urdf/{robot.name}.urdf.xacro
  "{moveit_config_package}" share dir: config/{srdf_file}, config/kinematics.yaml,
                                          config/joint_limits.yaml,
                                          config/moveit_controllers.yaml
Assembling that package tree from these generated file contents is
fusion_addin/generators/package.py's job, not this launch file's.
"""

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DESCRIPTION_PACKAGE = "{description_package}"
MOVEIT_CONFIG_PACKAGE = "{moveit_config_package}"
URDF_XACRO_FILE = "{robot.name}.urdf.xacro"
SRDF_FILE = "{srdf_file}"
PLANNING_GROUP = "{group_name}"


def _load_file(package_name, relative_path):
    path = PathJoinSubstitution([FindPackageShare(package_name), relative_path])
    # Resolved eagerly (not as a launch Substitution) because move_group's
    # robot_description_semantic parameter must be a plain string, not a
    # Substitution object -- this mirrors the common MoveIt tutorial pattern.
    absolute_path = f"{{get_package_share_directory(package_name)}}/{{relative_path}}"
    with open(absolute_path, "r") as f:
        return f.read()


def _load_yaml(package_name, relative_path):
    absolute_path = f"{{get_package_share_directory(package_name)}}/{{relative_path}}"
    with open(absolute_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    urdf_xacro_path = PathJoinSubstitution(
        [FindPackageShare(DESCRIPTION_PACKAGE), "urdf", URDF_XACRO_FILE]
    )
    # Processed via the `xacro` command-line filter at launch time, same
    # convention fusion_addin/generators/package.py's display.launch.py uses --
    # no import-time dependency on the `xacro` Python module.
    robot_description = {{"robot_description": Command(["xacro ", urdf_xacro_path])}}

    robot_description_semantic = {{
        "robot_description_semantic": _load_file(MOVEIT_CONFIG_PACKAGE, f"config/{{SRDF_FILE}}")
    }}

    kinematics_yaml = _load_yaml(MOVEIT_CONFIG_PACKAGE, "config/kinematics.yaml")
    robot_description_kinematics = {{"robot_description_kinematics": kinematics_yaml}}

    joint_limits_yaml = _load_yaml(MOVEIT_CONFIG_PACKAGE, "config/joint_limits.yaml")
    robot_description_planning = {{"robot_description_planning": joint_limits_yaml}}

    controllers_yaml = _load_yaml(MOVEIT_CONFIG_PACKAGE, "config/moveit_controllers.yaml")
    moveit_controllers = {{
        "moveit_simple_controller_manager": controllers_yaml["moveit_simple_controller_manager"],
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            moveit_controllers,
            {{"allow_trajectory_execution": True}},
            {{"publish_monitored_planning_scene": True}},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_group_node, rviz_node])
'''
    return content
