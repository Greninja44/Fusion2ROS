"""Real FusionDesignReader, backed by a live Fusion 360 Design.

*** THIS FILE IS UNVERIFIED. ***

Nothing in this sandbox can run a real Fusion 360 process or import
`adsk.core`/`adsk.fusion`, so nothing in this file has ever actually been
executed. Every symbol used below was checked against Autodesk's official
Fusion 360 API documentation (help.autodesk.com/cloudhelp/ENU/Fusion-360-API/
and the fusion360 help.autodesk.com/view/fusion360/ENU tree) or, where noted,
against Autodesk Community forum threads describing documented API
behavior. Where the docs were incomplete or ambiguous, that is called out
explicitly in a comment at the point of use -- do not trust those spots
without validating against a live Fusion session first. See the top-level
task report for the full list of citations and open uncertainties.

Everything in this file is a thin, mechanical translation from `adsk`
objects to the plain-Python value objects in `interface.py`. All actual
logic (unit conversion, joint-type mapping, tree-building, inertia-frame
math) lives in `converter.py` and is fully unit-tested there -- this file
should stay as dumb as possible on purpose, to minimize the amount of
genuinely untestable code.
"""

from __future__ import annotations

from typing import List, Optional

from .converter import Vec3, matrix_from_basis_vectors, matrix_to_rpy
from .interface import FusionDesignReader, FusionInertia, FusionJointInfo, FusionOccurrence, FusionPose

try:
    import adsk.core
    import adsk.fusion

    _ADSK_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised only outside Fusion
    adsk = None  # type: ignore[assignment]
    _ADSK_AVAILABLE = False


# Fusion joint types with a single-DOF (or zero-DOF) URDF equivalent, plus
# the ones that don't -- confirmed via help.autodesk.com/.../JointMotion_jointType.htm
# and the "Joint types" concept page (GUID-8818AE31-958A-4A59-989B-9875A174C67A):
# Fusion has exactly seven joint types -- Rigid, Revolute, Slider, Cylindrical,
# Pin-Slot, Planar, Ball. The Python enum member spellings themselves
# (`adsk.fusion.JointTypes.RigidJointType` etc.) were NOT found spelled out
# verbatim on any single fetched doc page (the JointTypes enum reference page
# render did not include the member list in what we could fetch); the names
# used here follow the consistent pattern seen across every other joint-type
# doc page (RevoluteJointMotion, SliderJointMotion, RigidJointMotion, ...)
# where the *Motion class name matches "<Name>JointMotion" and the concept
# docs list the same seven names with the same capitalization. This mapping
# should be double-checked against a live `dir(adsk.fusion.JointTypes)` in a
# real Fusion session before shipping.
_JOINT_TYPE_ENUM_TO_STR = {
    "RigidJointType": "RigidJointType",
    "RevoluteJointType": "RevoluteJointType",
    "SliderJointType": "SliderJointType",
    "CylindricalJointType": "CylindricalJointType",
    "PinSlotJointType": "PinSlotJointType",
    "PlanarJointType": "PlanarJointType",
    "BallJointType": "BallJointType",
}


def _require_adsk() -> None:
    if not _ADSK_AVAILABLE:
        raise RuntimeError(
            "fusion_adapter requires Fusion 360's adsk.core/adsk.fusion modules, which are only "
            "available inside a running Fusion 360 Python process. Use converter.build_robot_model() "
            "with a fake/test FusionDesignReader outside Fusion."
        )


def _matrix3d_to_pose(matrix) -> FusionPose:  # matrix: adsk.core.Matrix3D
    """Confirmed: Matrix3D.getAsCoordinateSystem exists and, per its doc page
    (Matrix3D_getAsCoordinateSystem.htm), "gets the matrix data as the
    components that define a coordinate system" via origin/xAxis/yAxis/zAxis.

    UNCERTAIN: that doc page's prose describes origin/xAxis/yAxis/zAxis as
    pass-by-reference "output arguments" (C++-style), but Fusion's Python
    bindings are inconsistent about this -- PhysicalProperties.getXYZMomentsOfInertia
    is documented the same way yet its *confirmed* Python signature returns a
    tuple `(returnValue, xx, yy, zz, xy, yz, xz)` instead of mutating passed-in
    doubles. We follow that same tuple-return pattern here as the best guess;
    if wrong, this needs to change to pre-creating Point3D/Vector3D objects
    and passing them in by reference instead.
    """
    origin, x_axis, y_axis, z_axis = matrix.getAsCoordinateSystem()
    xyz: Vec3 = (origin.x, origin.y, origin.z)
    rot = matrix_from_basis_vectors(
        (x_axis.x, x_axis.y, x_axis.z),
        (y_axis.x, y_axis.y, y_axis.z),
        (z_axis.x, z_axis.y, z_axis.z),
    )
    rpy = matrix_to_rpy(rot)
    return FusionPose(xyz=xyz, rpy=rpy)


def _physical_properties_to_inertia(props) -> FusionInertia:  # props: adsk.fusion.PhysicalProperties
    """Confirmed via PhysicalProperties.htm / PhysicalProperties_mass.htm /
    PhysicalProperties_getXYZMomentsOfInertia.htm:
    - `mass` is in kilograms.
    - `centerOfMass` is a Point3D (length unit -- cm, per Fusion's
      "internal units are always centimeters" convention confirmed in
      Units_UM.htm; the doc page for centerOfMass itself did not restate
      the unit, so this is inferred from the general internal-units rule,
      not a direct quote).
    - `getXYZMomentsOfInertia()` returns, per its doc page, the Python tuple
      `(returnValue, xx, yy, zz, xy, yz, xz)` in kg*cm^2, about the WORLD
      coordinate system's origin (confirmed via Autodesk Community thread
      "API: Access Moment of Inertia Properties About the Centre of Mass...").
      NOTE the argument order is xx, yy, zz, xy, **yz**, **xz** -- yz before
      xz -- easy to transpose by mistake.
    """
    com = props.centerOfMass
    ok, ixx, iyy, izz, ixy, iyz, ixz = props.getXYZMomentsOfInertia()
    if not ok:
        raise RuntimeError("Occurrence.getPhysicalProperties().getXYZMomentsOfInertia() reported failure")
    return FusionInertia(
        mass=props.mass,
        center_of_mass=(com.x, com.y, com.z),
        ixx=ixx,
        iyy=iyy,
        izz=izz,
        ixy=ixy,
        ixz=ixz,
        iyz=iyz,
    )


def _body_names(occurrence) -> List[str]:  # occurrence: adsk.fusion.Occurrence
    """Confirmed via Occurrence.htm: `bRepBodies` returns body proxies for
    the B-Rep bodies referenced by this occurrence."""
    names: List[str] = []
    for body in occurrence.bRepBodies:
        names.append(body.name)
    return names


def _occurrence_bounding_box(occurrence):  # occurrence: adsk.fusion.Occurrence
    """Confirmed via Occurrence.htm / BoundingBox3D.htm: `Occurrence.boundingBox`
    returns an `adsk.core.BoundingBox3D` with `.minPoint`/`.maxPoint`, each an
    `adsk.core.Point3D` exposing `.x`/`.y`/`.z` in Fusion's native centimeters
    (same internal-units rule as everywhere else in this file), expressed in
    the same flattened assembly/world context as `occurrence.transform2`. This
    is a long-standing, stable, widely-used part of the API (every third-party
    bounding-box sample found during research uses exactly this shape) -- one
    of the more confidently-verifiable calls in this file, even though it has
    not been exercised against a live Fusion process from this sandbox.

    Returns None if Fusion reports no bounding box for this occurrence (e.g.
    an empty/reference-only occurrence with no visible geometry) -- some
    third-party reports describe `boundingBox` as potentially None in that
    case, so we defend against it rather than assume it is always populated.
    """
    bbox = occurrence.boundingBox
    if bbox is None:
        return None
    min_pt, max_pt = bbox.minPoint, bbox.maxPoint
    return (
        (min_pt.x, min_pt.y, min_pt.z),
        (max_pt.x, max_pt.y, max_pt.z),
    )


def _occurrence_to_fusion_occurrence(occurrence) -> FusionOccurrence:  # occurrence: adsk.fusion.Occurrence
    pose = _matrix3d_to_pose(occurrence.transform2)
    props = occurrence.getPhysicalProperties()
    inertia = _physical_properties_to_inertia(props)
    return FusionOccurrence(
        name=occurrence.name,
        pose=pose,
        inertia=inertia,
        body_names=_body_names(occurrence),
        bounding_box=_occurrence_bounding_box(occurrence),
    )


def _joint_geometry_pose(geom_or_origin) -> FusionPose:
    """*** Highest-uncertainty function in this file. ***

    `Joint.geometryOrOriginOne`/`Two` return either a JointGeometry or a
    JointOrigin object (Joint.htm). We need a full pose (origin + rotation)
    for the joint frame, expressed relative to the owning occurrence's local
    frame, but the docs we could fetch for these two object types did not
    agree on what's available, and neither page stated the coordinate frame
    the values are expressed in (world? owning-component-local?):

    - JointOrigin exposes a `transform` property directly ("Returns the
      position and orientation of the joint geometry associated with this
      joint origin") -- used here when present, via the same
      getAsCoordinateSystem path as occurrence poses.
    - JointGeometry has no such `transform` in what we could fetch; instead
      it exposes `origin` (Point3D) plus `primaryAxisVector` /
      `secondaryAxisVector` / `thirdAxisVector`, described as "conceptually"
      the Z, X, and Y axes respectively of a computed coordinate system
      (note that non-alphabetical Z/X/Y ordering -- easy to get backwards).
      We build a rotation matrix from those three vectors as a fallback.

    This function has NOT been validated against a live Fusion session.
    Before relying on generated URDF geometry, sanity-check a simple
    known assembly (e.g. two boxes joined by a revolute joint at a known
    point) end-to-end in real Fusion.
    """
    transform = getattr(geom_or_origin, "transform", None)
    if transform is not None:
        return _matrix3d_to_pose(transform)

    origin = geom_or_origin.origin
    primary = geom_or_origin.primaryAxisVector  # conceptually Z
    secondary = geom_or_origin.secondaryAxisVector  # conceptually X
    third = geom_or_origin.thirdAxisVector  # conceptually Y
    xyz: Vec3 = (origin.x, origin.y, origin.z)
    rot = matrix_from_basis_vectors(
        (secondary.x, secondary.y, secondary.z),
        (third.x, third.y, third.z),
        (primary.x, primary.y, primary.z),
    )
    rpy = matrix_to_rpy(rot)
    return FusionPose(xyz=xyz, rpy=rpy)


def _revolute_info(motion) -> tuple:  # motion: adsk.fusion.RevoluteJointMotion
    """Confirmed via RevoluteJointMotion.htm: `rotationAxisVector` (may be
    None per the doc's own caveat "may be null from JointInput" -- for a
    resolved Joint (not a JointInput) it should be populated, but that
    distinction is inferred, not directly confirmed) and `rotationLimits`
    (a JointLimits object). JointLimits.htm confirms `minimumValue`/
    `maximumValue`/`isMinimumValueEnabled`/`isMaximumValueEnabled` in
    radians for an angular limit."""
    axis_vec = motion.rotationAxisVector
    axis: Optional[Vec3] = (axis_vec.x, axis_vec.y, axis_vec.z) if axis_vec is not None else None
    limits = motion.rotationLimits
    lower = limits.minimumValue if limits.isMinimumValueEnabled else None
    upper = limits.maximumValue if limits.isMaximumValueEnabled else None
    return axis, lower, upper


def _slider_info(motion) -> tuple:  # motion: adsk.fusion.SliderJointMotion
    """Confirmed via SliderJointMotion.htm: `slideDirectionVector` and
    `slideLimits` (JointLimits, values in centimeters per JointLimits.htm)."""
    axis_vec = motion.slideDirectionVector
    axis: Optional[Vec3] = (axis_vec.x, axis_vec.y, axis_vec.z) if axis_vec is not None else None
    limits = motion.slideLimits
    lower = limits.minimumValue if limits.isMinimumValueEnabled else None
    upper = limits.maximumValue if limits.isMaximumValueEnabled else None
    return axis, lower, upper


def _joint_to_fusion_joint_info(joint) -> FusionJointInfo:  # joint: adsk.fusion.Joint
    """Confirmed via Joint.htm: `name`, `occurrenceOne`, `occurrenceTwo`,
    `jointMotion`, `geometryOrOriginOne`. `jointMotion.jointType` is
    confirmed via JointMotion_jointType.htm to return a `JointTypes` enum
    value; we convert it to our string form via its `.name` attribute
    (standard for Fusion API enums exposed to Python) -- NOT independently
    confirmed for JointTypes specifically.

    UNCERTAIN (design decision, not an API fact): the Fusion Joint API does
    not itself label occurrenceOne/occurrenceTwo as "parent"/"child" -- see
    FusionJointInfo.occurrence_one's docstring in interface.py. We take
    occurrenceOne as the parent side. If a real assembly's joints turn out
    to be authored the other way around in practice, swap this mapping.
    """
    motion = joint.jointMotion
    joint_type_str = motion.jointType.name  # UNCONFIRMED: JointTypes enum member -> str via .name

    axis: Optional[Vec3] = None
    lower: Optional[float] = None
    upper: Optional[float] = None
    if joint_type_str in ("RevoluteJointType",):
        axis, lower, upper = _revolute_info(motion)
    elif joint_type_str in ("SliderJointType",):
        axis, lower, upper = _slider_info(motion)
    # Rigid and the unsupported multi-DOF types carry no axis/limits here;
    # converter.py raises a clear error for the unsupported ones.

    origin = _joint_geometry_pose(joint.geometryOrOriginOne)

    return FusionJointInfo(
        name=joint.name,
        joint_type=joint_type_str,
        occurrence_one=joint.occurrenceOne.name,
        occurrence_two=joint.occurrenceTwo.name,
        origin=origin,
        axis=axis,
        lower_limit=lower,
        upper_limit=upper,
        velocity_limit=None,  # Fusion's CAD Joint object has no motor/velocity concept.
        effort_limit=None,  # Fusion's CAD Joint object has no motor/effort concept.
    )


class FusionDesignReaderAdapter(FusionDesignReader):
    """Real FusionDesignReader backed by a live `adsk.fusion.Design`.

    Confirmed via Design_rootComponent.htm: `Design.rootComponent` returns
    the root Component. Confirmed via Component.htm: `Component.allJoints`
    ("Returns all joints in this component and any sub components") and
    `Component.allOccurrences` ("Returns all of the occurrences in the
    assembly regardless of their level within the assembly structure") are
    both real, documented properties of `Component` -- not just of the
    design's root component specifically, which is what makes the optional
    `root_component` scoping parameter below valid: any Component (e.g. one
    reached via a selected Occurrence's `.component`) supports the same two
    properties, not only `design.rootComponent`.
    """

    def __init__(self, design=None, root_component=None):
        # design: Optional[adsk.fusion.Design]
        # root_component: Optional[adsk.fusion.Component] -- ADDITIVE, optional
        # scoping parameter (added for the Fusion UI's root/occurrence
        # selection input, ui/command.py). When given, extraction is scoped to
        # this component's own occurrences/joints (and its subcomponents) via
        # `Component.allOccurrences`/`Component.allJoints`, instead of the
        # whole design's `rootComponent`. Confirmed real, documented
        # properties (Component.htm, fetched from help.autodesk.com during
        # this change): allOccurrences -- "Returns all of the occurrences in
        # the assembly regardless of their level within the assembly
        # structure"; allJoints -- "Returns all joints in this component and
        # any sub components." The intended caller passes an
        # `Occurrence.component` (Occurrence_component.htm: "The component
        # this occurrence references") obtained from a
        # SelectionCommandInput restricted to the "Occurrences" selection
        # filter. When None (default), behavior is unchanged from before this
        # parameter existed: the whole design's root component is used.
        _require_adsk()
        if design is None:
            app = adsk.core.Application.get()
            design = adsk.fusion.Design.cast(app.activeProduct)
            if design is None:
                raise RuntimeError("No active Fusion Design (activeProduct is not a Design)")
        self._design = design
        self._root = root_component if root_component is not None else design.rootComponent

    def list_occurrences(self) -> List[FusionOccurrence]:
        return [_occurrence_to_fusion_occurrence(occ) for occ in self._root.allOccurrences]

    def list_joints(self) -> List[FusionJointInfo]:
        return [_joint_to_fusion_joint_info(j) for j in self._root.allJoints]
