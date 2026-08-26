"""Read-only abstraction over a Fusion 360 design.

This module is deliberately Fusion-symbol-free: nothing here imports `adsk`
anything, so both `converter.py` (which consumes this interface) and this
module itself can be fully unit-tested with a hand-written fake — no running
Fusion 360 process required. `fusion_adapter.py` is the only file in this
package that is allowed to know about `adsk.core`/`adsk.fusion`; it produces
these same value objects by reading a live Fusion design.

Units — deliberately NOT SI. Everything here is in Fusion's *native* units,
confirmed from Autodesk's Fusion 360 API docs:

- length            : centimeters (cm)   -- Fusion's internal/database unit,
                       regardless of the document's *display* unit setting.
                       (help.autodesk.com/.../Units_UM.htm)
- mass               : kilograms (kg)    -- PhysicalProperties.mass docs say
                       "Gets the mass in kilograms" directly; no conversion
                       needed for mass.
- angle              : radians            -- e.g. RevoluteJointMotion.rotationValue,
                       JointLimits values for angular joints.
- moment of inertia  : kg*cm^2            -- PhysicalProperties.getXYZMomentsOfInertia
                       docs: "Unit for returned values is kg*cm^2." Also
                       confirmed (Autodesk Community, "API: Access Moment of
                       Inertia Properties About the Centre of Mass...") that
                       this is the tensor about the WORLD coordinate system's
                       *origin*, axis-aligned to world axes — NOT about the
                       occurrence's own center of mass. converter.py is
                       responsible for shifting it to the center of mass
                       (parallel axis theorem) and rotating it into the
                       occurrence's own local frame before it can be treated
                       as a robot_model.Inertial (which wants the tensor
                       about the link's own center of mass, in the link's own
                       frame).

Converting all of the above into RobotModel's SI units (meters, kg, radians)
happens exclusively in converter.py, never here and never in fusion_adapter.py.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class FusionPose:
    """A rigid transform in Fusion's native units.

    xyz: position, centimeters.
    rpy: orientation as roll-pitch-yaw about *fixed* (extrinsic) axes, applied
         roll about X, then pitch about Y, then yaw about Z -- i.e. the same
         convention robot_model.Pose.rpy uses, so converter.py only needs to
         convert xyz's units, not re-derive the rotation convention. Radians.
    """

    xyz: Vec3 = (0.0, 0.0, 0.0)
    rpy: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class FusionInertia:
    """Raw mass properties for one occurrence, in Fusion's native units and
    native reference frame -- i.e. exactly what
    `Occurrence.getPhysicalProperties()` hands back, with no massaging.

    mass            : kg
    center_of_mass  : cm, expressed in the same world/assembly-context frame
                      as the owning FusionOccurrence.pose.
    ixx/iyy/izz/
    ixy/ixz/iyz     : kg*cm^2, the inertia tensor about the WORLD coordinate
                      system's origin, axis-aligned to world axes (see module
                      docstring). NOT yet about the occurrence's center of
                      mass, and NOT yet expressed in the occurrence's own
                      (possibly rotated) local frame.
    """

    mass: float
    center_of_mass: Vec3
    ixx: float
    iyy: float
    izz: float
    ixy: float
    ixz: float
    iyz: float


@dataclass(frozen=True)
class FusionOccurrence:
    """One node of the assembly tree -- becomes exactly one robot_model.Link.

    name        : the occurrence's name, unique within the design. Maps to
                  `Occurrence.name` (e.g. Fusion's default "PartName:1" style
                  naming). converter.py sanitizes this into a URDF-safe link
                  name; this interface hands back Fusion's raw name.
    pose        : the occurrence's position/orientation, in the design's
                  flattened assembly (world) context -- i.e. what
                  `Occurrence.transform2` gives for an occurrence obtained via
                  `rootComponent.allOccurrences` (docs describe these as
                  proxies with their transform already composed relative to
                  the root). Used only to convert `inertia`'s world-frame
                  tensor into the occurrence's own local frame -- it is
                  deliberately NOT used to derive joint placement (see
                  FusionJointInfo.origin).
    inertia     : raw mass properties, see FusionInertia.
    body_names  : names of the BRep/mesh bodies belonging to this occurrence's
                  component. Not consumed by converter.py directly today --
                  carried through into Link.metadata so a later mesh-export
                  step (fusion_addin/generators/mesh.py, out of scope here)
                  knows which Fusion bodies correspond to which link.
    bounding_box : optional (min_corner, max_corner) pair, each an (x, y, z)
                  Vec3 in centimeters, expressed in the same flattened
                  assembly (world) context as `pose` -- i.e. what
                  `Occurrence.boundingBox` (an `adsk.core.BoundingBox3D`)
                  gives via its `.minPoint`/`.maxPoint` (each an
                  `adsk.core.Point3D` with `.x`/`.y`/`.z`), all in Fusion's
                  native centimeters per the module docstring's length-unit
                  note. World-AXIS-ALIGNED, not oriented to the occurrence's
                  own local frame -- converter.py uses this only to derive a
                  simplified world-aligned box size for a collision proxy,
                  never to place or orient anything. Defaults to None so
                  existing fakes/tests that don't set it keep working
                  unmodified; None also covers occurrences with no visible/
                  tessellatable geometry, where Fusion may have nothing
                  meaningful to report.
    """

    name: str
    pose: FusionPose
    inertia: FusionInertia
    body_names: List[str] = field(default_factory=list)
    bounding_box: Optional[Tuple[Vec3, Vec3]] = None


@dataclass(frozen=True)
class FusionJointInfo:
    """One Fusion joint -- becomes exactly one robot_model.Joint.

    name             : the joint's name. Maps to `Joint.name`.
    joint_type       : Fusion's joint type name as a string, exactly as
                       spelled by the `adsk.fusion.JointTypes` enum member,
                       e.g. "RigidJointType", "RevoluteJointType",
                       "SliderJointType", "CylindricalJointType",
                       "PinSlotJointType", "PlanarJointType", "BallJointType".
                       converter.py maps the first three to
                       robot_model.JointType.{FIXED,REVOLUTE,PRISMATIC} (or
                       CONTINUOUS for an unlimited revolute) and raises a
                       clear error for the other four, which have no
                       single-axis URDF equivalent.
    occurrence_one   : name of the FusionOccurrence on the "parent" side of
                       this joint. NOTE: Fusion's Joint API itself does not
                       distinguish a "parent" and "child" occurrence -- it
                       just has occurrenceOne/occurrenceTwo. This interface
                       imposes the convention occurrence_one == parent,
                       occurrence_two == child; fusion_adapter.py is
                       responsible for that assignment and it is the single
                       biggest point of interpretation in this package (see
                       its module docstring).
    occurrence_two   : name of the FusionOccurrence on the "child" side.
    origin           : the joint frame's pose, expressed relative to the
                       PARENT occurrence's (occurrence_one's) local frame.
                       xyz in cm, rpy in radians. Passed through to
                       robot_model.Joint.origin after only a unit conversion
                       -- converter.py does not attempt to re-derive this
                       from occurrence poses.
    axis             : unit vector (dimensionless -- direction only, no unit
                       conversion needed), expressed in the same frame as
                       `origin`. None for fixed joints.
    lower_limit/
    upper_limit      : joint position limits in the joint's native unit --
                       radians for a revolute joint, centimeters for a
                       slider/prismatic joint. None if Fusion has no limit
                       enabled in that direction (e.g. an unlimited revolute
                       -> converter.py maps it to JointType.CONTINUOUS).
    velocity_limit/
    effort_limit     : optional actuation limits, same length convention as
                       lower/upper_limit for velocity (rad/s or cm/s) and
                       otherwise consumer-defined for effort. Fusion's CAD
                       joint model has no native concept of these (there is
                       no motor/actuator on a plain Joint object) -- the real
                       adapter always reports None here; this interface
                       exposes the fields so a fake (or a future data source,
                       e.g. a motion study or user-entered metadata) can
                       supply them, and so converter.py's limit-handling code
                       is exercised by tests even though fusion_adapter.py
                       itself never populates them today.
    """

    name: str
    joint_type: str
    occurrence_one: str
    occurrence_two: str
    origin: FusionPose
    axis: Optional[Vec3] = None
    lower_limit: Optional[float] = None
    upper_limit: Optional[float] = None
    velocity_limit: Optional[float] = None
    effort_limit: Optional[float] = None


class FusionDesignReader(abc.ABC):
    """Top-level, read-only view of a Fusion design that converter.py needs.

    Implemented by:
    - a hand-written `FakeFusionDesignReader` in tests, with no Fusion
      dependency at all;
    - `fusion_adapter.FusionDesignReaderAdapter`, backed by a real, active
      Fusion `Design` (adsk.fusion.Design).

    `list_occurrences()` intentionally returns the FULL, flattened set of
    occurrences in the design (i.e. equivalent to Fusion's
    `rootComponent.allOccurrences`), not just the immediate top-level
    children of the root component -- joints in a real assembly routinely
    connect occurrences nested several components deep, and Fusion's own
    `allJoints`/`allOccurrences` collections flatten the hierarchy for
    exactly this reason (their transforms/contexts are already composed
    relative to the root). The robot's tree structure is derived purely from
    `list_joints()`, not from Fusion's component nesting.
    """

    @abc.abstractmethod
    def list_occurrences(self) -> List[FusionOccurrence]:
        """Every occurrence in the design, flattened, each appearing once."""
        raise NotImplementedError

    @abc.abstractmethod
    def list_joints(self) -> List[FusionJointInfo]:
        """Every joint in the design, flattened (equivalent to Fusion's
        `rootComponent.allJoints`)."""
        raise NotImplementedError
