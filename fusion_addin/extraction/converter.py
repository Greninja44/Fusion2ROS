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

from .interface import FusionDesignReader, FusionJointInfo, FusionOccurrence, FusionPose, Vec3

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


def build_robot_model(reader: FusionDesignReader, robot_name: str) -> Robot:
    """Walks a FusionDesignReader's occurrences and joints and produces a
    fully-populated, validated robot_model.Robot.

    Topology: exactly one robot_model.Link per FusionOccurrence. Parent/child
    structure comes entirely from FusionJointInfo.occurrence_one (parent) /
    occurrence_two (child); the single occurrence never named as a child is
    the tree's root link.
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

    roots = [name for name in occurrences if name not in joint_by_child]
    if len(roots) == 0:
        raise ExtractionError(
            "No root occurrence found -- every occurrence is the child of some joint, "
            "which means the joint graph has a cycle."
        )
    if len(roots) > 1:
        raise ExtractionError(
            f"Design has {len(roots)} unconnected root occurrences {sorted(roots)}; "
            "a robot must be a single connected tree with exactly one root."
        )

    sanitized: Dict[str, str] = {}
    for name in occurrences:
        clean = sanitize_link_name(name)
        if clean in sanitized.values():
            raise ExtractionError(
                f"Occurrence names {name!r} and an earlier occurrence both sanitize to link "
                f"name {clean!r}; rename one of them in Fusion to disambiguate."
            )
        sanitized[name] = clean

    links: List[Link] = []
    for name, occ in occurrences.items():
        parent_joint = joint_by_child.get(name)
        parent_link = sanitized[parent_joint.occurrence_one] if parent_joint else None
        links.append(
            Link(
                name=sanitized[name],
                parent=parent_link,
                origin=Pose.IDENTITY,
                inertial=_convert_inertial(occ),
                metadata={
                    "fusion_occurrence": name,
                    "fusion_body_names": list(occ.body_names),
                },
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
