"""Pure conversion logic: FusionDesignReader -> robot_model.Robot.

No `adsk` import anywhere in this file, and none is allowed -- it depends
only on `interface.py` (plain-Python data) and `robot_model` (plain-Python
schema). That is what makes it fully unit-testable without a running Fusion
360 process: hand a `build_robot_model` a hand-built fake reader and it runs
identically to how it would run inside the real add-in.

This is also where all unit conversion happens (Fusion's native cm/kg/rad ->
RobotModel's SI meters/kg/radians), and where Fusion's joint-type names are
mapped onto `robot_model.JointType`. See interface.py's module docstring for
the confirmed unit facts this relies on.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from robot_model import Inertial, Joint, JointType, Link, Pose, Robot

from .interface import FusionDesignReader, FusionInertia, FusionJointInfo, FusionOccurrence, FusionPose, Vec3

CM_TO_M = 0.01
CM2_TO_M2 = CM_TO_M * CM_TO_M  # kg*cm^2 -> kg*m^2


class ExtractionError(Exception):
    """Raised when a Fusion design cannot be converted into a valid Robot --
    e.g. a broken/disconnected joint graph, or a reference to an occurrence
    that doesn't exist."""


class UnsupportedJointTypeError(ExtractionError):
    """Raised when a Fusion joint's type has no representation in
    robot_model.JointType (fixed/revolute/prismatic/continuous)."""


# ---------------------------------------------------------------------------
# Fusion joint type -> robot_model.JointType
# ---------------------------------------------------------------------------

# Exact spellings of the adsk.fusion.JointTypes enum members that map cleanly
# onto a single-DOF (or zero-DOF) URDF joint.
_FIXED_TYPES = {"RigidJointType"}
_REVOLUTE_TYPES = {"RevoluteJointType"}
_PRISMATIC_TYPES = {"SliderJointType"}

# Fusion joint types that exist in the CAD assembly model but have no
# single-axis URDF equivalent (multi-DOF joints). Explicitly named (rather
# than "anything else") so the error message is precise and so a genuinely
# unrecognized string (typo, future Fusion joint type, etc.) is reported
# differently from a *known-but-unsupported* one.
_KNOWN_UNSUPPORTED_TYPES = {
    "CylindricalJointType",
    "PinSlotJointType",
    "PlanarJointType",
    "BallJointType",
}


def _map_joint_kinematics(
    j: FusionJointInfo,
) -> Tuple[JointType, Optional[Vec3], Optional[float], Optional[float], Optional[float], Optional[float]]:
    """Returns (joint_type, axis, lower_limit, upper_limit, velocity_limit,
    effort_limit) all in SI units, given a FusionJointInfo in Fusion's native
    units. Raises UnsupportedJointTypeError for a joint type with no URDF
    equivalent, or ExtractionError if required data is missing."""

    if j.joint_type in _KNOWN_UNSUPPORTED_TYPES:
        raise UnsupportedJointTypeError(
            f"Joint {j.name!r} has Fusion joint type {j.joint_type!r}, which has multiple "
            "degrees of freedom and cannot be represented as a single URDF joint "
            f"(fixed/revolute/prismatic/continuous). Known unsupported types: "
            f"{sorted(_KNOWN_UNSUPPORTED_TYPES)}."
        )

    if j.joint_type in _FIXED_TYPES:
        return JointType.FIXED, None, None, None, None, None

    if j.joint_type in _REVOLUTE_TYPES:
        if j.axis is None:
            raise ExtractionError(f"Revolute joint {j.name!r} has no rotation axis")
        if j.lower_limit is None or j.upper_limit is None:
            # Fusion allows a revolute joint with no rotation limit enabled
            # in one or both directions -- that is an unlimited (continuous)
            # rotation, which is exactly robot_model.JointType.CONTINUOUS.
            if j.lower_limit is not None or j.upper_limit is not None:
                raise ExtractionError(
                    f"Revolute joint {j.name!r} has only one of lower/upper rotation limit "
                    "enabled; robot_model requires both or neither (continuous)."
                )
            velocity = j.velocity_limit  # rad/s, no conversion
            effort = j.effort_limit
            return JointType.CONTINUOUS, j.axis, None, None, velocity, effort
        return (
            JointType.REVOLUTE,
            j.axis,
            j.lower_limit,  # radians already
            j.upper_limit,
            j.velocity_limit,  # rad/s already
            j.effort_limit,
        )

    if j.joint_type in _PRISMATIC_TYPES:
        if j.axis is None:
            raise ExtractionError(f"Prismatic joint {j.name!r} has no slide axis")
        if j.lower_limit is None or j.upper_limit is None:
            raise ExtractionError(
                f"Prismatic joint {j.name!r} must have both a lower and upper slide limit "
                "enabled in Fusion -- robot_model has no representation of an unbounded "
                "prismatic joint."
            )
        velocity = j.velocity_limit * CM_TO_M if j.velocity_limit is not None else None
        return (
            JointType.PRISMATIC,
            j.axis,
            j.lower_limit * CM_TO_M,
            j.upper_limit * CM_TO_M,
            velocity,
            j.effort_limit,
        )

    raise UnsupportedJointTypeError(
        f"Joint {j.name!r} has unrecognized Fusion joint type {j.joint_type!r}. "
        f"Recognized types: {sorted(_FIXED_TYPES | _REVOLUTE_TYPES | _PRISMATIC_TYPES | _KNOWN_UNSUPPORTED_TYPES)}."
    )


def _convert_pose(pose: FusionPose) -> Pose:
    xyz = tuple(v * CM_TO_M for v in pose.xyz)
    return Pose(xyz=xyz, rpy=pose.rpy)


# ---------------------------------------------------------------------------
# 3x3 matrix / vector helpers (pure Python, no numpy dependency -- matches
# robot_model's own stdlib-only ethos). A Mat3 is a tuple of three row
# 3-tuples. rpy uses the same fixed-axis convention as robot_model.Pose.rpy:
# R = Rz(yaw) * Ry(pitch) * Rx(roll).
# ---------------------------------------------------------------------------

Mat3 = Tuple[Vec3, Vec3, Vec3]
_IDENTITY3: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def rpy_to_matrix(rpy: Vec3) -> Mat3:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def matrix_to_rpy(m: Mat3) -> Vec3:
    r20 = m[2][0]
    r20 = max(-1.0, min(1.0, r20))
    pitch = math.asin(-r20)
    cp = math.cos(pitch)
    if abs(cp) > 1e-9:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:
        # Gimbal lock (pitch == +-pi/2): roll and yaw are not independently
        # observable from the matrix alone. Convention: fold everything into
        # roll, yaw = 0.
        roll = math.atan2(-m[0][1], m[1][1])
        yaw = 0.0
    return (roll, pitch, yaw)


def matrix_from_basis_vectors(x_axis: Vec3, y_axis: Vec3, z_axis: Vec3) -> Mat3:
    """Build a rotation matrix whose columns are the given (already
    orthonormal) basis vectors, e.g. from Matrix3D.getAsCoordinateSystem's
    xAxis/yAxis/zAxis."""
    return (
        (x_axis[0], y_axis[0], z_axis[0]),
        (x_axis[1], y_axis[1], z_axis[1]),
        (x_axis[2], y_axis[2], z_axis[2]),
    )


def _mat_transpose(m: Mat3) -> Mat3:
    return tuple(tuple(m[r][c] for r in range(3)) for c in range(3))  # type: ignore[return-value]


def _mat_mult(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)) for r in range(3)
    )


def _mat_vec(m: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(m[r][c] * v[c] for c in range(3)) for r in range(3))  # type: ignore[return-value]


def _vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _outer(a: Vec3, b: Vec3) -> Mat3:
    return tuple(tuple(a[i] * b[j] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _mat_sub(a: Mat3, b: Mat3) -> Mat3:
    return tuple(tuple(a[r][c] - b[r][c] for c in range(3)) for r in range(3))  # type: ignore[return-value]


def _mat_scale(a: Mat3, s: float) -> Mat3:
    return tuple(tuple(a[r][c] * s for c in range(3)) for r in range(3))  # type: ignore[return-value]


def _symmetric_from_components(ixx, iyy, izz, ixy, ixz, iyz) -> Mat3:
    return ((ixx, ixy, ixz), (ixy, iyy, iyz), (ixz, iyz, izz))


def _components_from_symmetric(m: Mat3):
    # Off-diagonals are theoretically symmetric already; average the two
    # copies to absorb floating-point noise from the matrix multiplications.
    ixx, iyy, izz = m[0][0], m[1][1], m[2][2]
    ixy = 0.5 * (m[0][1] + m[1][0])
    ixz = 0.5 * (m[0][2] + m[2][0])
    iyz = 0.5 * (m[1][2] + m[2][1])
    return ixx, iyy, izz, ixy, ixz, iyz


def _convert_inertial(occ: FusionOccurrence) -> Inertial:
    """Turns Fusion's raw, world-frame/world-origin inertia data into a
    robot_model.Inertial expressed about the occurrence's own center of
    mass, in the occurrence's own local (rotated) frame, in SI units.

    Two physical corrections are applied, both required per the confirmed
    Fusion API behavior documented in interface.py:
    1. Parallel axis theorem: shift the tensor from "about world origin" to
       "about the occurrence's center of mass" (still world-aligned axes).
    2. Rotate the (now COM-centered) tensor from world-aligned axes into the
       occurrence's own local axes, using its own orientation.
    """
    inertia = occ.inertia
    mass = inertia.mass
    com_world = inertia.center_of_mass
    origin_world = occ.pose.xyz

    m_origin = _symmetric_from_components(
        inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz
    )

    # Step 1: parallel axis theorem, shifting from the world origin to the
    # center of mass: I_about_point = I_com + m*(|d|^2 * Id - d(x)d), where
    # d is the vector from the center of mass to that point. Here the known
    # tensor is about the world origin (d = -com_world, but |d|^2 and d(x)d
    # are unaffected by the sign flip), so: I_com = I_origin - m*(|c|^2*Id - c(x)c).
    shift = _mat_scale(
        _mat_sub(_mat_scale(_IDENTITY3, _vec_dot(com_world, com_world)), _outer(com_world, com_world)),
        mass,
    )
    m_com_world_aligned = _mat_sub(m_origin, shift)

    # Step 2: rotate into the occurrence's own local frame: I_local = R^T * I * R.
    rot = rpy_to_matrix(occ.pose.rpy)
    rot_t = _mat_transpose(rot)
    m_local = _mat_mult(_mat_mult(rot_t, m_com_world_aligned), rot)
    ixx, iyy, izz, ixy, ixz, iyz = _components_from_symmetric(m_local)

    # Center of mass expressed in the occurrence's own local frame.
    local_com = _mat_vec(rot_t, _vec_sub(com_world, origin_world))
    local_com_m = tuple(v * CM_TO_M for v in local_com)

    return Inertial(
        mass=mass,
        center_of_mass=local_com_m,
        ixx=ixx * CM2_TO_M2,
        iyy=iyy * CM2_TO_M2,
        izz=izz * CM2_TO_M2,
        ixy=ixy * CM2_TO_M2,
        ixz=ixz * CM2_TO_M2,
        iyz=iyz * CM2_TO_M2,
    )


def _bounding_box_size(occ: FusionOccurrence) -> Optional[Vec3]:
    """Converts a FusionOccurrence's optional world-aligned bounding box into
    a full-extent (dx, dy, dz) size in meters, for use as a simplified box
    collision PROXY (see fusion_addin/app.py's attach_collision_proxies).

    Deliberate simplification, in the same spirit as _convert_inertial's own
    documented approximations elsewhere in this file: this is the occurrence's
    bounding box in Fusion's flattened WORLD/assembly frame, axis-aligned to
    world axes -- it is NOT rotated into the occurrence's own local frame the
    way _convert_inertial does for the inertia tensor. For a link whose local
    frame is rotated relative to the world, a world-aligned box can be larger
    than a tightly-oriented box would be. That's an accepted, honest trade-off
    for a cheap collision proxy, not a bug: a slightly oversized world-aligned
    box is still far cheaper for a physics engine than the full visual mesh,
    and correcting for orientation would require the occurrence's local body
    geometry, not just its bounding box -- out of scope here.

    Returns None if the occurrence has no bounding_box data (e.g. a fake/test
    reader that didn't set one, or a real occurrence with no visible geometry).
    """
    if occ.bounding_box is None:
        return None
    min_corner, max_corner = occ.bounding_box
    return tuple((max_corner[i] - min_corner[i]) * CM_TO_M for i in range(3))  # type: ignore[return-value]


_INVALID_NAME_CHARS = str.maketrans({c: "_" for c in " :/\\.()[]{}<>"})


def sanitize_link_name(name: str) -> str:
    """Fusion occurrence names commonly look like "base_link:1" or contain
    spaces; URDF link/joint names are conventionally simple identifiers.
    This is a pure naming convenience -- Fusion itself places no such
    restriction, so this is a converter.py design decision, not a confirmed
    API fact."""
    cleaned = name.translate(_INVALID_NAME_CHARS)
    while "__" in cleaned:
        cleaned = cleaned.replace("__", "_")
    return cleaned.strip("_") or "link"


def _merge_fusion_inertia(items: List[FusionInertia]) -> FusionInertia:
    """Combines several occurrences' raw inertia into one, as if they were a
    single rigid body. Valid because FusionInertia's ixx/iyy/izz/ixy/ixz/iyz
    are, per interface.py's documented convention, all about the SAME
    reference point (the world origin) and the SAME axes (world-aligned) for
    every occurrence -- inertia tensors about a common point are directly
    additive for a system of rigid bodies, so no shifting/rotating is needed
    before summing; _convert_inertial does that step once, afterward, using
    the host occurrence's own pose."""
    total_mass = sum(i.mass for i in items)
    if total_mass > 0:
        com = tuple(sum(i.mass * i.center_of_mass[k] for i in items) / total_mass for k in range(3))
    else:
        com = items[0].center_of_mass
    return FusionInertia(
        mass=total_mass,
        center_of_mass=com,  # type: ignore[arg-type]
        ixx=sum(i.ixx for i in items),
        iyy=sum(i.iyy for i in items),
        izz=sum(i.izz for i in items),
        ixy=sum(i.ixy for i in items),
        ixz=sum(i.ixz for i in items),
        iyz=sum(i.iyz for i in items),
    )


def _merge_bounding_boxes(boxes: List[Optional[Tuple[Vec3, Vec3]]]) -> Optional[Tuple[Vec3, Vec3]]:
    present = [b for b in boxes if b is not None]
    if not present:
        return None
    mins = tuple(min(b[0][k] for b in present) for k in range(3))
    maxs = tuple(max(b[1][k] for b in present) for k in range(3))
    return (mins, maxs)  # type: ignore[return-value]


def _fuse_occurrences(host: FusionOccurrence, orphans: List[FusionOccurrence]) -> FusionOccurrence:
    """Builds a synthetic FusionOccurrence standing in for `host` plus every
    `orphans` occurrence fused onto it -- same name/pose as host (so it still
    becomes exactly one robot_model.Link, in host's own local frame), but
    with orphans' mass/inertia, body names, and bounding box folded in. See
    build_robot_model's docstring for why orphans exist at all.

    NOTE: only mass/inertia/collision-bbox are fused this way -- mesh export
    (fusion_addin/generators/mesh.py) still exports only the host occurrence's
    own Fusion bodies, so fused hardware (screws, resistors, etc.) affects
    physics correctly but does not yet appear in the generated visual mesh.
    """
    if not orphans:
        return host
    ordered_orphans = sorted(orphans, key=lambda o: o.name)
    merged_body_names = list(host.body_names) + [n for o in ordered_orphans for n in o.body_names]
    merged_bbox = _merge_bounding_boxes([host.bounding_box, *(o.bounding_box for o in ordered_orphans)])
    merged_inertia = _merge_fusion_inertia([host.inertia, *(o.inertia for o in ordered_orphans)])
    return FusionOccurrence(
        name=host.name,
        pose=host.pose,
        inertia=merged_inertia,
        body_names=merged_body_names,
        bounding_box=merged_bbox,
    )


def _nearest_by_name(orphan: FusionOccurrence, candidates: Dict[str, FusionOccurrence]) -> str:
    """Nearest candidate occurrence to `orphan` by straight-line distance
    between their (world-frame) pose.xyz -- the only spatial hint available
    (see interface.py: occurrence nesting/parenting in Fusion's own component
    tree is deliberately not exposed here, only the flattened joint graph).
    Ties broken by sorted name order for determinism."""

    def dist2(name: str) -> float:
        c, o = candidates[name].pose.xyz, orphan.pose.xyz
        return sum((c[k] - o[k]) ** 2 for k in range(3))

    return min(sorted(candidates), key=dist2)


def build_robot_model(reader: FusionDesignReader, robot_name: str) -> Robot:
    """Walks a FusionDesignReader's occurrences and joints and produces a
    fully-populated, validated robot_model.Robot.

    Topology: one robot_model.Link per JOINTED FusionOccurrence (i.e. one
    that appears as either side of at least one FusionJointInfo). Parent/
    child structure comes entirely from FusionJointInfo.occurrence_one
    (parent) / occurrence_two (child); the single jointed occurrence never
    named as a child is the tree's root link.

    Occurrences that appear in NO joint at all ("orphans" -- real CAD
    assemblies routinely have dozens of these: screws, resistors, wires,
    other fasteners positioned in place but never given an explicit Fusion
    Joint feature) are not turned into their own links -- Fusion's assembly
    nesting is deliberately not exposed here (see FusionDesignReader's
    docstring), so there is no reliable way to know which link they should
    parent under. Instead, each orphan is fused onto whichever JOINTED
    occurrence is spatially closest to it (see _nearest_by_name): its mass
    and inertia contribute to that link's physics, and its bounding box
    contributes to that link's collision proxy. This is a deliberate
    approximation, not a bug -- see _fuse_occurrences' docstring for the one
    thing it does NOT do (fuse into the exported visual mesh).

    This fusing only kicks in once at least one real joint exists somewhere
    in the design. A design with NO joints at all keeps the original,
    stricter behavior (a single occurrence is trivially the whole robot;
    multiple occurrences with zero joints between them is still a hard
    error) -- with no joint anywhere, there is no kinematic information to
    even choose a sensible "nearest jointed occurrence" host from.
    """
    occurrences: Dict[str, FusionOccurrence] = {}
    for occ in reader.list_occurrences():
        if occ.name in occurrences:
            raise ExtractionError(f"Duplicate occurrence name {occ.name!r}")
        occurrences[occ.name] = occ

    if not occurrences:
        raise ExtractionError("Fusion design has no occurrences to convert")

    joints_info = reader.list_joints()

    joint_by_child: Dict[str, FusionJointInfo] = {}
    for j in joints_info:
        if j.occurrence_one not in occurrences:
            raise ExtractionError(f"Joint {j.name!r} references unknown occurrence {j.occurrence_one!r}")
        if j.occurrence_two not in occurrences:
            raise ExtractionError(f"Joint {j.name!r} references unknown occurrence {j.occurrence_two!r}")
        if j.occurrence_two in joint_by_child:
            raise ExtractionError(
                f"Occurrence {j.occurrence_two!r} is the child of multiple joints "
                f"({joint_by_child[j.occurrence_two].name!r} and {j.name!r}); "
                "a link may have only one parent joint."
            )
        joint_by_child[j.occurrence_two] = j

    jointed_names = {j.occurrence_one for j in joints_info} | {j.occurrence_two for j in joints_info}

    host_of_orphan: Dict[str, str] = {}
    if jointed_names:
        jointed_roots = [name for name in jointed_names if name not in joint_by_child]
        if len(jointed_roots) == 0:
            raise ExtractionError(
                "No root occurrence found among jointed occurrences -- every jointed "
                "occurrence is the child of some joint, which means the joint graph has a cycle."
            )
        if len(jointed_roots) > 1:
            raise ExtractionError(
                f"Design has {len(jointed_roots)} separate joint-connected mechanisms with no "
                f"joint tying them together {sorted(jointed_roots)}; a robot must be a single "
                "connected tree with exactly one root. (Occurrences with no joint at all are "
                "fused onto their nearest jointed neighbor instead of counting here -- see "
                "build_robot_model's docstring.)"
            )
        kept_names = jointed_names
        orphan_names = [name for name in occurrences if name not in jointed_names]
        jointed_occurrences = {name: occurrences[name] for name in jointed_names}
        for orphan_name in orphan_names:
            host_of_orphan[orphan_name] = _nearest_by_name(occurrences[orphan_name], jointed_occurrences)
    else:
        roots = list(occurrences)  # joint_by_child is empty -> trivially everyone
        if len(roots) > 1:
            raise ExtractionError(
                f"Design has {len(roots)} unconnected root occurrences {sorted(roots)}; "
                "a robot must be a single connected tree with exactly one root."
            )
        kept_names = set(occurrences)

    orphans_of: Dict[str, List[str]] = {name: [] for name in kept_names}
    for orphan_name, host_name in host_of_orphan.items():
        orphans_of[host_name].append(orphan_name)

    sanitized: Dict[str, str] = {}
    for name in occurrences:
        if name not in kept_names:
            continue
        clean = sanitize_link_name(name)
        if clean in sanitized.values():
            raise ExtractionError(
                f"Occurrence names {name!r} and an earlier occurrence both sanitize to link "
                f"name {clean!r}; rename one of them in Fusion to disambiguate."
            )
        sanitized[name] = clean

    links: List[Link] = []
    for name in occurrences:
        if name not in kept_names:
            continue
        fused_names = orphans_of[name]
        occ = _fuse_occurrences(occurrences[name], [occurrences[n] for n in fused_names])
        parent_joint = joint_by_child.get(name)
        parent_link = sanitized[parent_joint.occurrence_one] if parent_joint else None
        metadata: Dict[str, object] = {
            "fusion_occurrence": name,
            "fusion_body_names": list(occ.body_names),
        }
        if fused_names:
            # Present only when at least one unjointed occurrence was fused
            # onto this link -- see build_robot_model's docstring.
            metadata["fused_occurrences"] = sorted(fused_names)
        bbox_size = _bounding_box_size(occ)
        if bbox_size is not None:
            # Present only when Fusion supplied a bounding box (see
            # FusionOccurrence.bounding_box) -- absent, not None, when it
            # didn't, so downstream code (attach_collision_proxies) can use
            # plain `.get("bounding_box_size")` / `in` checks.
            metadata["bounding_box_size"] = bbox_size
        links.append(
            Link(
                name=sanitized[name],
                parent=parent_link,
                origin=Pose.IDENTITY,
                inertial=_convert_inertial(occ),
                metadata=metadata,
            )
        )

    joints: List[Joint] = []
    for j in joints_info:
        joint_type, axis, lower, upper, velocity, effort = _map_joint_kinematics(j)
        joints.append(
            Joint(
                name=j.name,
                type=joint_type,
                parent=sanitized[j.occurrence_one],
                child=sanitized[j.occurrence_two],
                origin=_convert_pose(j.origin),
                axis=axis,
                lower_limit=lower,
                upper_limit=upper,
                velocity_limit=velocity,
                effort_limit=effort,
            )
        )

    robot = Robot(
        name=robot_name,
        links=links,
        joints=joints,
        metadata={
            "source": "fusion_addin.extraction",
            "unit_conversion": "Fusion native (cm, kg, rad) -> SI (m, kg, rad)",
        },
    )
    robot.validate()
    return robot
