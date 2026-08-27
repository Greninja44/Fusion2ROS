"""Best-effort auto-detection of a differential-drive `robot.metadata["drivetrain"]`
dict (the shape `fusion_addin/generators/ros2_control.py`/`nav2.py`/`gazebo.py`
all key off -- see `ros2_control.py`'s module docstring) from a `Robot`'s
joint names and geometry alone.

Motivation: real live testing this session showed a user has to look up and
type exact Fusion joint names (e.g. "Revolute_WheelMidA") plus wheel
separation/radius by hand into the "Generate ROS 2 Package" dialog -- tedious
and error-prone, and the numbers (separation, radius) require either a CAD
measurement or a guess. This module pre-fills that guess; the Fusion UI
(`fusion_addin/ui/command.py`) always shows it as an editable, overridable
suggestion, never as a silent/forced choice.

Pure function of `robot_model.Robot` plus stdlib math -- no Fusion API, no
filesystem, no network -- same testability constraint as every other module
under `fusion_addin/extraction/` and `fusion_addin/generators/`.

Deliberately conservative: returns `None` (no guess at all) rather than a
low-confidence one whenever the heuristic can't reach a clear answer (fewer
than two wheel-like joints, no geometry to size a wheel from, etc.) -- an
empty UI field the user fills in by hand is a much smaller cost than a wrong
number silently shipped into a generated package.

Only "differential_drive" is attempted here, not "mecanum_drive" -- a
mecanum robot's four wheels are geometrically symmetric to each other in the
same way a 2-or-6-wheel differential-drive rover's are, so there is no
reliable geometric signal here to tell "this is mecanum" apart from "this is
a 6-wheel differential-drive rover with unusually-placed wheels" without
also inspecting wheel roller geometry (mecanum wheels have angled rollers
this project's `Geometry`/mesh-only extraction has no way to see) --
guessing wrong there would silently mis-configure a real robot's motion
model, worse than leaving the field blank.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from robot_model import Joint, JointType, Robot

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]

_IDENTITY_ROTATION: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_WHEEL_NAME_HINT = "wheel"
_DRIVEN_JOINT_TYPES = (JointType.CONTINUOUS, JointType.REVOLUTE)


def _rotation_matrix(rpy: Vec3) -> Mat3:
    """Rotation matrix for URDF's `<origin rpy="roll pitch yaw">` convention:
    extrinsic X-Y-Z rotation, i.e. R = Rz(yaw) @ Ry(pitch) @ Rx(roll) applied
    to a column vector -- the same convention `fusion_addin/generators/urdf.py`
    writes and every URDF-consuming tool (robot_state_publisher, RViz,
    Gazebo) parses. Confirmed against URDF's own `<origin>` specification
    (ros.org/wiki/urdf/XML/joint: "rpy... representing the rotation around
    the fixed axes in the order roll, pitch, yaw" -- i.e. Rz*Ry*Rx, not the
    reverse)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat_mul(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _rotate(m: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _compose(parent_rot: Mat3, parent_pos: Vec3, local_xyz: Vec3, local_rpy: Vec3) -> Tuple[Mat3, Vec3]:
    local_rot = _rotation_matrix(local_rpy)
    world_rot = _mat_mul(parent_rot, local_rot)
    offset = _rotate(parent_rot, local_xyz)
    world_pos = tuple(parent_pos[i] + offset[i] for i in range(3))  # type: ignore[assignment]
    return world_rot, world_pos


def _world_positions(robot: Robot) -> Dict[str, Vec3]:
    """Link name -> world-frame position, via forward kinematics composed
    through each joint's `origin` (parent-frame-to-child-frame transform,
    per URDF convention -- see `_rotation_matrix`'s docstring). Robust to
    any input joint ordering (a worklist that only resolves a joint once its
    parent's position is already known, looping until nothing more can be
    resolved) -- real Fusion-extracted robots have no guaranteed joint
    order. Links unreachable from the root (should not happen for a
    `Robot` that has passed `validate()`, but this module never calls that
    itself -- see module docstring) are simply absent from the result."""
    root = robot.root_link()
    if root is None:
        return {}

    positions: Dict[str, Vec3] = {root.name: (0.0, 0.0, 0.0)}
    rotations: Dict[str, Mat3] = {root.name: _IDENTITY_ROTATION}

    remaining: List[Joint] = list(robot.joints)
    progressed = True
    while remaining and progressed:
        progressed = False
        still_remaining = []
        for j in remaining:
            if j.parent in positions:
                rot, pos = _compose(rotations[j.parent], positions[j.parent], j.origin.xyz, j.origin.rpy)
                positions[j.child] = pos
                rotations[j.child] = rot
                progressed = True
            else:
                still_remaining.append(j)
        remaining = still_remaining

    return positions


def _estimate_wheel_radius(robot: Robot, link_name: str) -> Optional[float]:
    """A wheel's link geometry, in this project, is either a real `Geometry`
    primitive (kind="cylinder", `.radius` known exactly) or -- the common
    case for a raw Fusion-extracted link, before `attach_collision_proxies`
    ever runs -- mesh-only, with only `link.metadata["bounding_box_size"]`
    (an axis-aligned bounding box in meters, populated unconditionally by
    `extraction/converter.py` regardless of the `use_bounding_box_collision`
    flag) available. Prefer the exact primitive; fall back to the bounding
    box's MEDIAN dimension / 2 -- a wheel (a disc, not a sphere) has two
    dimensions close to its diameter and one (the tread width) close to
    zero by comparison, so the median of the three is a robust diameter
    estimate even when a hub/axle detail makes one of the two "diameter"
    dimensions slightly larger than the other."""
    link = robot.link(link_name)
    if link is None:
        return None

    for geometry in (link.collision_geometry, link.visual_geometry):
        if geometry is not None and geometry.kind == "cylinder" and geometry.radius:
            return float(geometry.radius)

    bbox = (link.metadata or {}).get("bounding_box_size")
    if not bbox:
        return None
    dims = sorted(float(d) for d in bbox)
    return dims[1] / 2.0


def detect_drivetrain(robot: Robot) -> Optional[Dict[str, object]]:
    """Best-effort `robot.metadata["drivetrain"]` guess for a
    "differential_drive" robot (see module docstring for why
    "mecanum_drive" is never attempted).

    Heuristic, in order:

    1. Candidate wheel joints: non-fixed joints (continuous or revolute --
       URDF's usual choice for a driven wheel) whose name contains "wheel"
       (case-insensitive) -- every real robot examined during this
       project's live Fusion testing named its wheel joints this way
       (e.g. "Revolute_WheelMidA"), and joint names are the one piece of
       CAD author intent extraction has no other way to recover.
    2. Fewer than two candidates, or fewer than two with a resolvable
       forward-kinematics position (see `_world_positions`) -> give up
       (return None).
    3. The left/right separation axis is whichever of X/Y/Z splits all
       candidate wheel positions into the most EVEN two-way count around
       their mean (e.g. 3-vs-3 for a 6-wheel rover) -- NOT simply whichever
       axis has the largest spread: a rover that's longer front-to-back
       than it is wide (the common case) has a larger spread along its
       front-back axis than its left-right one, so spread alone picks the
       wrong axis. An even count split is the actual defining geometric
       property of "left vs right" (every side has the same wheel count),
       whereas a front-back axis's split is uneven whenever wheels aren't
       symmetrically doubled front and rear (ties broken by larger spread).
       Candidates are split into two groups by which side of the mean they
       fall on along the chosen axis.
    4. For more than one wheel per side (e.g. a 6-wheel rover with
       front/mid/rear wheels per side), the wheel closest to that SIDE's
       own midpoint is picked as its single "driven" joint -- mirroring the
       real, manual choice made during this project's own live rover
       testing (the middle wheel of each side), and the only sensible
       single-joint-per-side reduction `ros2_control.py`/`nav2.py`/
       `gazebo.py`'s `differential_drive` schema supports today. The
       front-back axis and its mean are computed SEPARATELY for each side
       (picking, among the two non-side-axis coordinates, whichever one
       has the largest spread WITHIN THAT SIDE'S OWN wheels) rather than
       shared between both sides -- a real rover examined during this
       project's live testing has a front wheel mounted on a separate
       fixed leg from its mid/rear wheels (which share a pivoting bogie
       arm instead), so the two sides' front-to-back axis do not always
       agree after forward kinematics composes through each side's own
       chain of rotations, even for an overall bilaterally-symmetric
       design.
    5. `wheel_separation` is the straight-line distance between the two
       chosen wheels' world positions; `wheel_radius` comes from
       `_estimate_wheel_radius` on either side's wheel link (whichever
       resolves first). Either failing to produce a usable value ->
       give up.

    Which geometric group ends up labeled "left" vs "right" is arbitrary
    (this module has no notion of the robot's forward-facing convention) --
    swapping them only flips the sign of which way a positive angular
    velocity turns the robot, not whether the numbers themselves are
    correct, and the Fusion UI always shows this as an editable suggestion
    to confirm or swap, never a silent final answer.
    """
    candidates = [
        j for j in robot.joints if j.type in _DRIVEN_JOINT_TYPES and _WHEEL_NAME_HINT in j.name.lower()
    ]
    if len(candidates) < 2:
        return None

    positions = _world_positions(robot)
    candidates = [j for j in candidates if j.child in positions]
    if len(candidates) < 2:
        return None

    coords = [positions[j.child] for j in candidates]
    spreads = [max(c[axis] for c in coords) - min(c[axis] for c in coords) for axis in range(3)]
    means = [sum(c[axis] for c in coords) / len(coords) for axis in range(3)]

    def _count_balance(axis: int) -> int:
        positive = sum(1 for c in coords if c[axis] > means[axis])
        return abs(positive - (len(coords) - positive))

    side_axis = min(range(3), key=lambda a: (_count_balance(a), -spreads[a]))
    side_mean = means[side_axis]

    left = [(j, c) for j, c in zip(candidates, coords) if c[side_axis] > side_mean]
    right = [(j, c) for j, c in zip(candidates, coords) if c[side_axis] <= side_mean]
    if not left or not right:
        return None
    # If even the BEST of the three axes can't split the candidates into a
    # roughly even two-way count (off by more than one), none of X/Y/Z
    # actually represents "left vs right" for this robot's geometry -- e.g.
    # a real rover hit during this project's own live testing mounts its
    # front wheel on a separate fixed leg rather than the same pivoting
    # bogie arm as its mid/rear wheels, landing it geometrically closer to
    # the OPPOSITE side's wheels than its own side's. Guessing a left/right
    # split here would silently pair wheels that don't belong together;
    # returning None (no guess) is the conservative choice this module's
    # docstring commits to.
    if abs(len(left) - len(right)) > 1:
        return None

    remaining_axes = [a for a in range(3) if a != side_axis]

    def _closest_to_own_side_midline(group):
        if len(group) == 1:
            return group[0]
        axis = max(remaining_axes, key=lambda a: max(c[a] for _, c in group) - min(c[a] for _, c in group))
        mean = sum(c[axis] for _, c in group) / len(group)
        return min(group, key=lambda pair: abs(pair[1][axis] - mean))

    left_joint, left_pos = _closest_to_own_side_midline(left)
    right_joint, right_pos = _closest_to_own_side_midline(right)

    wheel_separation = math.dist(left_pos, right_pos)
    if wheel_separation <= 0:
        return None

    wheel_radius = _estimate_wheel_radius(robot, left_joint.child) or _estimate_wheel_radius(
        robot, right_joint.child
    )
    if not wheel_radius or wheel_radius <= 0:
        return None

    return {
        "type": "differential_drive",
        "left_wheel_joint": left_joint.name,
        "right_wheel_joint": right_joint.name,
        "wheel_separation": round(wheel_separation, 6),
        "wheel_radius": round(wheel_radius, 6),
    }
