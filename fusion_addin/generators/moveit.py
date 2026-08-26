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
to guess which branch is "the arm" -- `detect_moveit_suitability` itself
is UNCHANGED and still behaves exactly this way for every robot, branchy
or not (every existing caller/test of it keeps working identically). A
caller with a legitimate branchy robot (e.g. arm + gripper) can still use
these generators today by calling `generate_srdf` (etc.) once per branch
with explicit `base_link`/`tip_link` -- only the automatic base_link/
tip_link derivation (and `detect_moveit_suitability`'s suitability
verdict) assumes a single chain.

Two-group heuristic (implemented): `detect_moveit_groups`
------------------------------------------------------------------------
The "longest-chain-plus-secondary-group" heuristic the project brief
calls out as a legitimate stretch goal (e.g. an arm with a gripper
hanging off the last link) is implemented as a SEPARATE function,
`detect_moveit_groups`, rather than folded into `detect_moveit_suitability`
-- this keeps the original single-chain suitability verdict's meaning
completely unchanged while adding real support for the common
"arm with a gripper" shape as new, additive capability.

Scope of what IS handled: a robot whose kinematic tree has EXACTLY ONE
branch point (one link with exactly two child joints -- e.g. a wrist link
that leads to both the arm's own nominal tool frame and a separate
gripper linkage). `detect_moveit_groups` picks the longer of the two
root-reachable leaf chains as the primary "arm" group (root_link -> its
deepest leaf) and the shorter branch as a secondary "gripper" group
(branch_link -> its own leaf), returning both as `(name, base_link,
tip_link)` triples alongside an empty problems list.

Scope of what is NOT handled (still refused, on purpose): a robot with
ZERO branch points behaves exactly like the existing single-chain
detection (one group, name "arm") -- not new behavior, just
`detect_moveit_groups`'s equivalent of `detect_moveit_suitability` for
that shape. A robot with MORE THAN ONE branch point anywhere in the tree,
or a single branch point with more than two children (a 3+-way fork), is
still refused with a clear message: deciding which of several branches is
"the arm" vs. "a gripper" vs. "a second gripper" vs. "an attached sensor
mount" is a materially harder, genuinely ambiguous problem (there's no
longer one obvious "longer chain" answer, nor a principled way to guess
which forks should even become their own MoveIt groups) that stays out of
scope here, same spirit as the original single-branch-point refusal.

`generate_srdf_multi_group` / `generate_joint_limits_yaml_multi_group` /
`generate_kinematics_yaml_multi_group` /
`generate_moveit_controllers_yaml_multi_group` /
`generate_moveit_demo_launch_multi_group` accept the `groups` list
`detect_moveit_groups` returns (or any hand-built list of `(name,
base_link, tip_link)` triples) and emit config covering all of them in
one call, alongside (not replacing) the original single-group functions.
`generate_moveit_controllers_yaml_multi_group` gives any group literally
named "gripper" MoveIt's real `GripperCommand` controller shape (the
single-DOF `control_msgs/GripperCommand` action, `action_ns: gripper_cmd`)
instead of `FollowJointTrajectory` -- confirmed against this environment's
actually-installed `moveit_simple_controller_manager` package, which ships
a real `gripper_command_controller_handle.hpp` implementing exactly that
action interface (`GripperCommandControllerHandle` wraps
`ActionBasedControllerHandle<control_msgs::action::GripperCommand>`); see
that function's docstring for the full verification trail, including the
real `action_ns: gripper_cmd` / `type: GripperCommand` / `joints` /
`command_joint` / `parallel` YAML field names confirmed against MoveIt's
own controller-configuration tutorial.

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


def _base_suitability_problems(robot: Robot) -> Tuple[List[str], bool]:
    """Shared by `detect_moveit_suitability` and `detect_moveit_groups`:
    the validation/no-moving-joints/drivetrain checks that have nothing to
    do with chain shape.

    Returns `(problems, stop)`. `stop=True` means `robot.validate()` itself
    failed -- `problems` is then exactly the one validation-failure message
    and the caller must return it immediately without running any further
    (chain-shape) checks, matching `detect_moveit_suitability`'s original,
    unchanged early-return behavior. `stop=False` means `problems` (possibly
    empty) holds the no-moving-joints/drivetrain verdicts and the caller
    should go on to its own chain-shape checks.
    """
    validation_problems = robot.validate(raise_on_error=False)
    if validation_problems:
        return (
            [
                "Robot fails basic structural validation, which must be fixed before "
                "MoveIt suitability can be assessed: " + "; ".join(validation_problems)
            ],
            True,
        )

    problems: List[str] = []

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

    return problems, False


def detect_moveit_suitability(robot: Robot) -> List[str]:
    """Return a list of problems that make `robot` unsuitable for the
    single-group MoveIt scaffolding this module generates. Empty list ==
    suitable.

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
        branch is "the arm" (documented MVP scope, not an oversight) --
        `detect_moveit_groups` is the separate function that implements
        the longest-chain-plus-secondary-group heuristic for the common
        "one branch point" case; it does not change this function's
        verdict for a branchy robot.
    """
    problems, stop = _base_suitability_problems(robot)
    if stop:
        return problems

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
                "group heuristic for cases like an arm with a gripper is implemented "
                "separately -- see detect_moveit_groups in this module). "
                f"Leaf links found: {leaves}. Pass explicit base_link/tip_link to "
                "generate_srdf (etc.) for each branch you want as its own group instead."
            )
        elif len(leaves) != 1:
            problems.append(f"Robot does not have exactly one leaf link (found {leaves}).")

    return problems


def detect_moveit_groups(robot: Robot) -> Tuple[List[str], List[Tuple[str, str, str]]]:
    """Return `(problems, groups)` -- the longest-chain-plus-secondary-group
    heuristic documented in this module's docstring ("Two-group heuristic").

    `problems` follows the same convention as `detect_moveit_suitability`:
    empty means `groups` is usable. `groups` is a list of `(group_name,
    base_link, tip_link)` triples suitable for passing straight to
    `generate_srdf_multi_group` (etc.):

      * Zero branch points: same shape `detect_moveit_suitability` already
        accepts -- one group, `[("arm", root_link, the_one_leaf)]`.
      * Exactly one branch point with exactly two child joints (the "arm
        with a gripper" case): two groups --
        `[("arm", root_link, longer_branch_leaf), ("gripper", branch_link,
        shorter_branch_leaf)]`. Ties (equal-length branches) resolve to the
        first-encountered leaf (sorted order) as "arm" -- an arbitrary but
        deterministic choice, since nothing about a tie says which side is
        "the arm".
      * Anything else (more than one branch point anywhere, or a single
        branch point with more than two children) is refused with a clear
        message -- see the module docstring's "Two-group heuristic"
        section for why this stays out of scope rather than guessing.

    Reuses `_joints_by_parent`/`_leaf_link_names`/`_chain_joints` (the same
    tree-walking helpers `detect_moveit_suitability` and the single-group
    generators use) rather than reimplementing chain-walking logic.
    """
    problems, _stop = _base_suitability_problems(robot)
    if problems:
        # Either robot.validate() failed, or a base problem (no non-fixed
        # joints / flagged as a drivetrain) already rules the robot out --
        # chain shape is irrelevant at that point, so don't bother computing
        # it (matches the "empty problems == groups usable" contract).
        return problems, []

    root = robot.root_link()
    if root is None:
        problems.append("Robot has no unique root link; cannot determine a kinematic chain.")
        return problems, []

    joints_by_parent = _joints_by_parent(robot)
    branch_points = sorted(name for name, js in joints_by_parent.items() if len(js) > 1)
    leaves = sorted(_leaf_link_names(robot))

    if len(branch_points) > 1:
        problems.append(
            f"Robot's kinematic tree branches at multiple links {branch_points}. "
            "detect_moveit_groups only handles a robot with EXACTLY ONE branch "
            "point (the common 'arm with a gripper' shape) -- with more than one "
            "fork, there is no longer one obvious 'longer chain is the arm' answer, "
            "and no principled way to guess which branches deserve their own MoveIt "
            f"group. Leaf links found: {leaves}. Pass explicit base_link/tip_link to "
            "generate_srdf (etc.) for each branch you want as its own group instead."
        )
        return problems, []

    if not branch_points:
        if len(leaves) != 1:
            problems.append(f"Robot does not have exactly one leaf link (found {leaves}).")
            return problems, []
        return problems, [("arm", root.name, leaves[0])]

    branch_link = branch_points[0]
    children = joints_by_parent[branch_link]
    if len(children) != 2:
        problems.append(
            f"Link {branch_link!r} is the single branch point in this robot's "
            f"kinematic tree, but has {len(children)} child joints, not 2. "
            "detect_moveit_groups only handles a simple 2-way fork (e.g. an arm "
            "ending in a gripper) -- a 3+-way fork at one link is refused for the "
            "same reason multiple branch points are: no principled way to decide "
            f"which children deserve their own MoveIt group. Leaf links found: {leaves}."
        )
        return problems, []

    if len(leaves) != 2:
        # Defensive: with exactly one 2-way branch point and no other forks,
        # the tree should have exactly 2 leaves. Refuse cleanly rather than
        # guess if that invariant somehow doesn't hold.
        problems.append(
            f"Robot has one 2-way branch point at {branch_link!r} but "
            f"{len(leaves)} leaf links (found {leaves}), not the 2 expected for a "
            "simple arm-plus-gripper shape -- refusing rather than guessing which "
            "leaf belongs to which branch."
        )
        return problems, []

    leaf_a, leaf_b = leaves
    chain_a = _chain_joints(robot, root.name, leaf_a)
    chain_b = _chain_joints(robot, root.name, leaf_b)
    if len(chain_a) >= len(chain_b):
        primary_leaf, secondary_leaf = leaf_a, leaf_b
    else:
        primary_leaf, secondary_leaf = leaf_b, leaf_a

    groups = [
        ("arm", root.name, primary_leaf),
        ("gripper", branch_link, secondary_leaf),
    ]
    return problems, groups


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


def _emit_group_chain(root_elem: ET.Element, group_name: str, base_link: str, tip_link: str) -> ET.Element:
    group = ET.SubElement(root_elem, "group", {"name": group_name})
    ET.SubElement(group, "chain", {"base_link": base_link, "tip_link": tip_link})
    return group


def _emit_home_group_state(root_elem: ET.Element, group_name: str, chain_joints: List[Joint]) -> ET.Element:
    state = ET.SubElement(root_elem, "group_state", {"name": "home", "group": group_name})
    for joint in _non_fixed(chain_joints):
        ET.SubElement(state, "joint", {"name": joint.name, "value": _fmt_float(_home_value(joint))})
    return state


def _emit_disable_collisions(root_elem: ET.Element, robot: Robot) -> None:
    for joint in robot.joints:
        ET.SubElement(root_elem, "disable_collisions", {"link1": joint.parent, "link2": joint.child, "reason": "Adjacent"})


def _srdf_document(root_elem: ET.Element) -> str:
    ET.indent(root_elem, space="  ")
    body = ET.tostring(root_elem, encoding="unicode")
    return f'<?xml version="1.0"?>\n{body}\n'


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
    note. Pass them explicitly to target one branch of a branchy robot, or
    use `generate_srdf_multi_group` to emit several groups (e.g. from
    `detect_moveit_groups`) in one SRDF document.

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

    root = ET.Element("robot", {"name": robot.name})
    _emit_group_chain(root, group_name, base_link, tip_link)
    _emit_home_group_state(root, group_name, chain_joints)
    _emit_disable_collisions(root, robot)
    return _srdf_document(root)


def generate_srdf_multi_group(
    robot: Robot,
    groups: List[Tuple[str, str, str]],
    combined_group_name: Optional[str] = None,
) -> str:
    """Render `robot` to SRDF XML text covering MULTIPLE planning groups in
    one document -- e.g. the `("arm", ...)`/`("gripper", ...)` pair
    `detect_moveit_groups` returns for a branchy-but-supported robot.

    `groups` is a list of `(group_name, base_link, tip_link)` triples, each
    rendered as its own `<group><chain .../></group>` exactly like
    `generate_srdf` would for that one group.

    Called with a single group, this produces SRDF equivalent to what
    `generate_srdf` produces for that same group (same `<group>`/`<chain>`,
    same `<group_state name="home">` joints/values, same
    `<disable_collisions>` set) -- this module does not duplicate
    `generate_srdf`'s logic, it shares the same `_emit_*`/`_group_chain`
    helpers.

    Called with more than one group, an additional COMPOSITE `<group>`
    (named `combined_group_name`, default: the groups' names joined with
    "_", e.g. "arm_gripper") is emitted containing a `<group name=".."/>`
    subgroup reference to each of the individual groups -- SRDF's
    documented, real mechanism for a group made of other groups (confirmed
    against this environment's installed `srdfdom` Python bindings:
    `srdfdom.srdf.Group` reflects a nested `AggregateElement("group",
    Group)`, i.e. `<group>` elements may contain child `<group name="x"/>`
    references to other top-level groups -- this is exactly how a real
    combined "arm_with_gripper"-style planning group is built by
    `moveit_setup_assistant`). Exactly one `<group_state name="home">` is
    then emitted for that composite group, covering every non-fixed joint
    across ALL groups (srdfdom's `GroupState` does not restrict which
    joints may appear under a given `group` attribute at the DOM level, so
    this is well-formed, real SRDF).

    `<disable_collisions>` is still emitted for every directly-connected
    link pair across the WHOLE robot, not just the given groups' chains --
    same rule `generate_srdf` follows.

    Raises:
        ValueError: if `groups` is empty, or (propagated from
            `_chain_joints`) if any `(base_link, tip_link)` pair doesn't
            describe a real, unambiguous chain in `robot`.
    """
    robot.validate()
    if not groups:
        raise ValueError("generate_srdf_multi_group requires at least one (group_name, base_link, tip_link) triple.")

    root = ET.Element("robot", {"name": robot.name})

    per_group_chain_joints: List[Tuple[str, List[Joint]]] = []
    for name, base_link, tip_link in groups:
        chain_joints = _chain_joints(robot, base_link, tip_link)
        _emit_group_chain(root, name, base_link, tip_link)
        per_group_chain_joints.append((name, chain_joints))

    if len(per_group_chain_joints) == 1:
        name, chain_joints = per_group_chain_joints[0]
        _emit_home_group_state(root, name, chain_joints)
    else:
        combined_name = combined_group_name or "_".join(name for name, _ in per_group_chain_joints)
        combined_group = ET.SubElement(root, "group", {"name": combined_name})
        for name, _ in per_group_chain_joints:
            ET.SubElement(combined_group, "group", {"name": name})

        state = ET.SubElement(root, "group_state", {"name": "home", "group": combined_name})
        for _, chain_joints in per_group_chain_joints:
            for joint in _non_fixed(chain_joints):
                ET.SubElement(state, "joint", {"name": joint.name, "value": _fmt_float(_home_value(joint))})

    _emit_disable_collisions(root, robot)
    return _srdf_document(root)


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


def generate_joint_limits_yaml_multi_group(robot: Robot, groups: List[Tuple[str, str, str]]) -> str:
    """Like `generate_joint_limits_yaml`, but covers every non-fixed joint
    across ALL of `groups` (a list of `(group_name, base_link, tip_link)`
    triples, e.g. from `detect_moveit_groups`) in one `joint_limits.yaml`.
    `joint_limits.yaml` has no per-group namespacing at all -- it's just a
    flat `joint_limits: {<joint_name>: {...}}` map covering every joint
    MoveIt might plan for, regardless of which group(s) it belongs to -- so
    a multi-group robot needs exactly one such combined file, not one per
    group. A joint appearing in more than one group's chain (not possible
    for the two disjoint chains `detect_moveit_groups` returns, but
    guarded here defensively) is only emitted once.

    Raises the same ValueError as `generate_joint_limits_yaml` for any
    non-fixed joint missing `velocity_limit`.
    """
    robot.validate()
    if not groups:
        raise ValueError("generate_joint_limits_yaml_multi_group requires at least one group.")

    lines = [
        "# Generated by fusion_addin.generators.moveit.generate_joint_limits_yaml_multi_group",
        f'# MoveIt joint_limits.yaml for planning groups {[name for name, _, _ in groups]!r} '
        f'of robot "{robot.name}".',
        "joint_limits:",
    ]
    seen_joint_names = set()
    for _, base_link, tip_link in groups:
        for joint in _non_fixed(_chain_joints(robot, base_link, tip_link)):
            if joint.name in seen_joint_names:
                continue
            seen_joint_names.add(joint.name)
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


def generate_kinematics_yaml_multi_group(robot: Robot, groups: List[Tuple[str, str, str]]) -> str:
    """Like `generate_kinematics_yaml`, but covers every group in `groups`
    (a list of `(group_name, base_link, tip_link)` triples, e.g. from
    `detect_moveit_groups`) in one `kinematics.yaml`.

    MoveIt's `kinematics.yaml` has no wrapping structure at all -- it is
    simply a flat mapping of `<group_name>: {kinematics_solver: ...}`
    entries, one per planning group that needs IK, with no nesting under a
    single top-level key. A multi-group robot's `kinematics.yaml` is
    therefore just this same per-group block repeated once per group, all
    at the top level of the same file -- there is no dedicated MoveIt 2
    multi-group example config installed in this environment to compare
    against byte-for-byte (`moveit_configs_utils/default_configs/` ships
    only planner configs, not a kinematics.yaml, and no `moveit_resources`/
    example robot package with a multi-group kinematics.yaml is installed
    here either), but this flat-top-level-keys structure is the documented,
    unambiguous shape of the format itself, not a per-robot convention.
    """
    robot.validate()
    if not groups:
        raise ValueError("generate_kinematics_yaml_multi_group requires at least one group.")

    lines = [
        "# Generated by fusion_addin.generators.moveit.generate_kinematics_yaml_multi_group",
        f'# MoveIt kinematics.yaml for planning groups {[name for name, _, _ in groups]!r} '
        f'of robot "{robot.name}".',
        "# KDL: MoveIt's standard, always-available, no-extra-dependency default IK",
        "# plugin. A specialized solver (TRAC-IK/IKFast) is not assumed to be",
        "# available and is not used here.",
    ]
    for group_name, _, _ in groups:
        lines.append(f"{group_name}:")
        lines.append("  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin")
        lines.append("  kinematics_solver_search_resolution: 0.005")
        lines.append("  kinematics_solver_timeout: 0.005")
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


def generate_moveit_controllers_yaml_multi_group(
    robot: Robot,
    groups: List[Tuple[str, str, str]],
    gripper_group_names: Optional[List[str]] = None,
) -> str:
    """Like `generate_moveit_controllers_yaml`, but emits one
    `moveit_simple_controller_manager` controller entry per group in
    `groups` (a list of `(group_name, base_link, tip_link)` triples, e.g.
    from `detect_moveit_groups`), each named `"<group_name>_controller"`
    -- same naming convention as the single-group function, see the module
    docstring's "Shared ros2_control naming convention" note.

    Every group gets the existing `FollowJointTrajectory` shape EXCEPT a
    group whose name is in `gripper_group_names` (default: any group
    literally named `"gripper"`, matching `detect_moveit_groups`'
    secondary-group naming) -- that group instead gets MoveIt's real
    `GripperCommand` controller shape:

        <group>_controller:
          action_ns: gripper_cmd
          type: GripperCommand
          default: true
          joints:
            - <the group's non-fixed joint(s)>

    Verification trail for this shape (MoveIt 2 IS installed in this
    environment, unlike when the rest of this module was first written --
    see docs/ARCHITECTURE.md):
      * `type: GripperCommand` is a REAL, registered
        `moveit_simple_controller_manager` controller-handle type, not
        invented here: this environment's installed
        `moveit_simple_controller_manager` package ships
        `include/moveit_simple_controller_manager/
        gripper_command_controller_handle.hpp`, defining
        `GripperCommandControllerHandle`, which wraps
        `ActionBasedControllerHandle<control_msgs::action::GripperCommand>`
        -- a single-DOF `control_msgs/GripperCommand` action client, exactly
        matching a single prismatic/revolute gripper joint the way
        `detect_moveit_groups`' "gripper" group is shaped.
      * `action_ns: gripper_cmd`, plus the `name`/`action_ns`/`type`/
        `default`/`joints`/`command_joint`/`parallel` field names, are
        confirmed against MoveIt's own controller-configuration tutorial
        (moveit2_tutorials' `controller_configuration_tutorial.rst`), which
        documents exactly this YAML shape for a `GripperCommand` entry
        alongside a `FollowJointTrajectory` one. `gripper_cmd` is also the
        long-standing conventional action namespace `ros2_controllers`'
        gripper action controllers (`GripperActionController` and its
        `parallel_gripper_action_controller` successor) publish under.
      * NOT verified against a live `move_group` in THIS session (see the
        module docstring / this repository's git history for the
        single-group `FollowJointTrajectory` path's live verification) --
        the corresponding ros2_control-side controller (a
        `parallel_gripper_action_controller/GripperActionController` or
        similar, actually registered in `controllers.yaml` and spawned) is
        `fusion_addin/generators/ros2_control.py`'s responsibility, which
        this module does not touch or assume exists yet for a gripper
        group -- same "assumed shared convention, not independently
        verifiable from this file" caveat the single-group function
        already documents for its own `FollowJointTrajectory` controller.

    Raises the same ValueError as `_chain_joints` for any group whose
    `(base_link, tip_link)` doesn't describe a real, unambiguous chain.
    """
    robot.validate()
    if not groups:
        raise ValueError("generate_moveit_controllers_yaml_multi_group requires at least one group.")

    if gripper_group_names is None:
        gripper_names = {name for name, _, _ in groups if name == "gripper"}
    else:
        gripper_names = set(gripper_group_names)

    controller_names = [f"{name}_controller" for name, _, _ in groups]

    lines = [
        "# Generated by fusion_addin.generators.moveit.generate_moveit_controllers_yaml_multi_group",
        f'# MoveIt moveit_controllers.yaml for planning groups {[name for name, _, _ in groups]!r} '
        f'of robot "{robot.name}".',
        "#",
        "# Any group named in gripper_group_names (default: a group literally named",
        '# "gripper") gets a GripperCommand controller instead of FollowJointTrajectory',
        "# -- see generate_moveit_controllers_yaml_multi_group's docstring for the",
        "# verification trail (confirmed against this environment's installed",
        "# moveit_simple_controller_manager package and MoveIt's own controller",
        "# configuration tutorial).",
        "#",
        "# ASSUMED SHARED CONVENTION (same as the single-group function): each",
        '# "<group_name>_controller" name below is expected to be defined by',
        "# fusion_addin/generators/ros2_control.py as the matching ros2_control",
        "# controller for that group -- not importable/verifiable from this file.",
        "moveit_simple_controller_manager:",
        "  controller_names:",
    ]
    for controller_name in controller_names:
        lines.append(f"    - {controller_name}")

    for group_name, base_link, tip_link in groups:
        non_fixed_chain_joints = _non_fixed(_chain_joints(robot, base_link, tip_link))
        controller_name = f"{group_name}_controller"
        lines.append(f"  {controller_name}:")
        if group_name in gripper_names:
            lines.append("    action_ns: gripper_cmd")
            lines.append("    type: GripperCommand")
        else:
            lines.append("    action_ns: follow_joint_trajectory")
            lines.append("    type: FollowJointTrajectory")
        lines.append("    default: true")
        lines.append("    joints:")
        for joint in non_fixed_chain_joints:
            lines.append(f"      - {joint.name}")

    return "\n".join(lines) + "\n"


def generate_ompl_planning_yaml() -> str:
    """Return the OMPL planning-pipeline registration `move_group` requires
    to start at all -- confirmed for real (not merely "simplified", the
    original documented scope for this generator) that `move_group` throws
    `std::runtime_error("Planning plugin name is empty or not defined in
    namespace 'move_group'. Please choose one of the available plugins:
    chomp_interface/CHOMPPlanner, ompl_interface/OMPLPlanner,
    pilz_industrial_motion_planner/CommandPlanner, stomp_moveit/StompPlanner")`
    and terminates immediately if NO planning pipeline is registered at all --
    this isn't tunable-but-optional the way OMPL's per-algorithm
    `planner_configs` presets are, it's a hard startup requirement.

    Content matches this environment's actually-installed
    `moveit_configs_utils/default_configs/ompl_planning.yaml` verbatim
    (`planning_plugins`/`request_adapters`/`response_adapters` -- the
    `ompl_defaults.yaml` `planner_configs` block, ~130 lines of generic
    per-algorithm OMPL tuning parameters unrelated to any specific robot, is
    deliberately not duplicated here: it's not required for `move_group` to
    start or plan with the default planner, only to select a *specific*
    named algorithm/tuning by `planner_id` -- a real future enhancement,
    not something to invent values for now).

    No robot-specific data, so this takes no `robot` parameter and returns
    identical content for every robot -- unlike every other generator in
    this module.
    """
    return (
        "planning_plugins:\n"
        "  - ompl_interface/OMPLPlanner\n"
        "request_adapters:\n"
        "  - default_planning_request_adapters/ResolveConstraintFrames\n"
        "  - default_planning_request_adapters/ValidateWorkspaceBounds\n"
        "  - default_planning_request_adapters/CheckStartStateBounds\n"
        "  - default_planning_request_adapters/CheckStartStateCollision\n"
        "response_adapters:\n"
        "  - default_planning_response_adapters/AddTimeOptimalParameterization\n"
        "  - default_planning_response_adapters/ValidateSolution\n"
        "  - default_planning_response_adapters/DisplayMotionPath\n"
    )


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

    # move_group throws std::runtime_error and terminates immediately with
    # no planning pipeline registered at all -- confirmed for real, not a
    # tunable simplification. See generate_ompl_planning_yaml's docstring in
    # fusion_addin/generators/moveit.py.
    ompl_yaml = _load_yaml(MOVEIT_CONFIG_PACKAGE, "config/ompl_planning.yaml")
    planning_pipeline_config = {{
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_yaml,
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
            planning_pipeline_config,
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


def generate_moveit_demo_launch_multi_group(
    robot: Robot,
    groups: List[Tuple[str, str, str]],
    moveit_config_package: Optional[str] = None,
) -> str:
    """Like `generate_moveit_demo_launch`, but for a robot with multiple
    MoveIt groups (e.g. the `("arm", ...)`/`("gripper", ...)` pair
    `detect_moveit_groups` returns).

    Loading-wise this is almost identical to the single-group launch file:
    `robot_description_semantic` still comes from ONE `<robot>.srdf` file
    (now containing multiple `<group>` elements -- see
    `generate_srdf_multi_group`), `robot_description_kinematics` still
    comes from ONE `kinematics.yaml` (now containing one top-level entry
    per group -- see `generate_kinematics_yaml_multi_group`), and
    `moveit_simple_controller_manager` still comes from ONE
    `moveit_controllers.yaml` (now containing one controller entry per
    group, gripper groups getting `GripperCommand` -- see
    `generate_moveit_controllers_yaml_multi_group`). `move_group` reads all
    of these as plain parameter dicts regardless of how many groups/
    controllers they describe internally, so passing config for both
    groups to `move_group` requires no structural change to how this
    launch file loads/wires those parameters -- only the documentation/
    constants below are group-aware (`PLANNING_GROUPS` lists every group
    name instead of one), which is why this is a separate function
    (matching `generate_srdf_multi_group`'s sibling-function shape) rather
    than a branch inside `generate_moveit_demo_launch` -- the single-group
    function's LOGIC didn't need to change to support multiple groups, but
    duplicating this hand-rolled template avoids adding multi-group
    conditionals to the well-tested single-group generator. See
    `generate_moveit_demo_launch`'s own docstring for the full list of
    simplifications this hand-rolled launch file makes relative to a real
    `moveit_setup_assistant` package (no tuned OMPL pipeline params, no
    pre-built MotionPlanning rviz display, etc) -- all of that applies here
    unchanged.
    """
    robot.validate()
    if not groups:
        raise ValueError("generate_moveit_demo_launch_multi_group requires at least one group.")

    description_package = robot.name
    moveit_config_package = moveit_config_package or f"{robot.name}_moveit_config"
    srdf_file = f"{robot.name}.srdf"
    group_names = [name for name, _, _ in groups]

    content = f'''"""MoveIt 2 demo launch file for "{robot.name}" (planning groups {group_names!r}).

Auto-generated by fusion_addin/generators/moveit.py's
generate_moveit_demo_launch_multi_group. Regenerate rather than editing by hand.

Hand-rolled rather than using moveit_configs_utils.MoveItConfigsBuilder -- see
generate_moveit_demo_launch's docstring in fusion_addin/generators/moveit.py for the
full list of simplifications relative to a real moveit_setup_assistant-generated
package (no tuned OMPL pipeline params, no pre-built MotionPlanning rviz display,
etc), all of which apply here unchanged.

Assumes the standard package layout this generator was built against:
  "{description_package}"        share dir: urdf/{robot.name}.urdf.xacro
  "{moveit_config_package}" share dir: config/{srdf_file}, config/kinematics.yaml,
                                          config/joint_limits.yaml,
                                          config/moveit_controllers.yaml
Assembling that package tree from these generated file contents is
fusion_addin/generators/package.py's job, not this launch file's. The SRDF/
kinematics/joint_limits/controllers files above each cover ALL of
{group_names!r} in one file (see generate_srdf_multi_group /
generate_kinematics_yaml_multi_group / generate_joint_limits_yaml_multi_group /
generate_moveit_controllers_yaml_multi_group) -- move_group is handed each whole
file's contents as one parameter dict, same as the single-group launch file.
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
PLANNING_GROUPS = {group_names!r}


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

    # Covers every group in PLANNING_GROUPS -- kinematics.yaml has one
    # top-level entry per group, no per-group loading needed.
    kinematics_yaml = _load_yaml(MOVEIT_CONFIG_PACKAGE, "config/kinematics.yaml")
    robot_description_kinematics = {{"robot_description_kinematics": kinematics_yaml}}

    joint_limits_yaml = _load_yaml(MOVEIT_CONFIG_PACKAGE, "config/joint_limits.yaml")
    robot_description_planning = {{"robot_description_planning": joint_limits_yaml}}

    # Covers every group in PLANNING_GROUPS -- moveit_controllers.yaml has
    # one controller entry per group (gripper groups get GripperCommand,
    # see generate_moveit_controllers_yaml_multi_group).
    controllers_yaml = _load_yaml(MOVEIT_CONFIG_PACKAGE, "config/moveit_controllers.yaml")
    moveit_controllers = {{
        "moveit_simple_controller_manager": controllers_yaml["moveit_simple_controller_manager"],
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }}

    # move_group throws std::runtime_error and terminates immediately with
    # no planning pipeline registered at all -- confirmed for real, not a
    # tunable simplification. See generate_ompl_planning_yaml's docstring in
    # fusion_addin/generators/moveit.py.
    ompl_yaml = _load_yaml(MOVEIT_CONFIG_PACKAGE, "config/ompl_planning.yaml")
    planning_pipeline_config = {{
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_yaml,
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
            planning_pipeline_config,
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
