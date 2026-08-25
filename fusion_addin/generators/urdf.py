"""RobotModel -> URDF/Xacro text generator.

Pure function of a `robot_model.Robot`: no Fusion API, no ROS/rclpy, no
filesystem access, no network. Safe to import and call from Fusion's
embedded interpreter, from plain WSL `python3`, or from CI with neither
installed — same constraint the schema itself is held to (see
robot_model/schema.py and docs/ARCHITECTURE.md).

Design notes
------------
* `robot.validate()` is called first and its `ValidationError` is allowed to
  propagate unmodified — this module never tries to render a broken graph.
* Output is deterministic: link/joint ordering follows `Robot.links` /
  `Robot.joints` list order (never a dict/set iteration), floats are
  formatted with a fixed, non-scientific-notation scheme, and XML attribute
  order is the literal order each element is built in (Python dicts, and
  therefore `xml.etree.ElementTree` attribute serialization, preserve
  insertion order). Calling `generate_urdf_xacro` twice on the same `Robot`
  yields byte-identical strings.
* URDF's <limit> tag is mandatory for revolute/prismatic joints and, per the
  URDF spec, MUST carry `effort` and `velocity` attributes for those joint
  types. RobotModel's schema deliberately allows `velocity_limit` /
  `effort_limit` to be `None` (e.g. an in-progress extraction that hasn't
  captured motor specs yet) — this generator refuses to invent a physical
  value for either. If a revolute/prismatic joint reaches here without both
  set, `generate_urdf_xacro` raises `ValueError` naming the joint and the
  missing attribute(s) rather than emitting a `<limit>` with a guessed
  number. A `continuous` joint's `<limit>` is optional per spec (no position
  limits apply); if exactly one of velocity_limit/effort_limit is set (but
  not both), that is treated as an inconsistent partial specification and
  also raises `ValueError` rather than silently dropping data or guessing.
"""

import xml.etree.ElementTree as ET

from robot_model import Geometry, Inertial, Joint, JointType, Link, Material, Pose, Robot

XACRO_XMLNS = "http://www.ros.org/wiki/xacro"

_LIMIT_REQUIRED_TYPES = (JointType.REVOLUTE, JointType.PRISMATIC)
_AXIS_TYPES = (JointType.REVOLUTE, JointType.PRISMATIC, JointType.CONTINUOUS)


def generate_urdf_xacro(robot: Robot) -> str:
    """Render `robot` (a validated `Robot`) to URDF/Xacro XML text.

    Raises:
        robot_model.ValidationError: if `robot.validate()` finds problems.
            Propagated unmodified — this function never renders invalid
            input.
        ValueError: if a revolute/prismatic joint is missing
            `velocity_limit` and/or `effort_limit` (both are mandatory
            attributes of URDF's <limit> element for those joint types), or
            if a continuous joint specifies only one of the two.
    """
    robot.validate()  # raises ValidationError on problems; let it propagate

    root = ET.Element("robot", {"name": robot.name, "xmlns:xacro": XACRO_XMLNS})

    for link in robot.links:
        root.append(_build_link_element(link))

    for joint in robot.joints:
        root.append(_build_joint_element(joint))

    ET.indent(root, space="  ")
    body = ET.tostring(root, encoding="unicode")
    return f'<?xml version="1.0"?>\n{body}\n'


# --- links -----------------------------------------------------------------


def _build_link_element(link: Link) -> ET.Element:
    elem = ET.Element("link", {"name": link.name})
    if link.visual_geometry is not None:
        elem.append(_build_visual_element(link))
    if link.collision_geometry is not None:
        elem.append(_build_collision_element(link))
    if link.inertial is not None:
        elem.append(_build_inertial_element(link.inertial))
    return elem


def _build_visual_element(link: Link) -> ET.Element:
    visual = ET.Element("visual")
    _add_origin(visual, link.origin)
    visual.append(_build_geometry_element(link.visual_geometry))
    if link.material is not None:
        visual.append(_build_material_element(link.material))
    return visual


def _build_collision_element(link: Link) -> ET.Element:
    collision = ET.Element("collision")
    _add_origin(collision, link.origin)
    collision.append(_build_geometry_element(link.collision_geometry))
    return collision


def _build_material_element(material: Material) -> ET.Element:
    elem = ET.Element("material", {"name": material.name})
    if material.rgba is not None:
        ET.SubElement(elem, "color", {"rgba": _fmt_vec(material.rgba)})
    return elem


def _build_inertial_element(inertial: Inertial) -> ET.Element:
    elem = ET.Element("inertial")
    ET.SubElement(elem, "origin", {"xyz": _fmt_vec(inertial.center_of_mass), "rpy": "0 0 0"})
    ET.SubElement(elem, "mass", {"value": _fmt_float(inertial.mass)})
    ET.SubElement(
        elem,
        "inertia",
        {
            "ixx": _fmt_float(inertial.ixx),
            "iyy": _fmt_float(inertial.iyy),
            "izz": _fmt_float(inertial.izz),
            "ixy": _fmt_float(inertial.ixy),
            "ixz": _fmt_float(inertial.ixz),
            "iyz": _fmt_float(inertial.iyz),
        },
    )
    return elem


def _build_geometry_element(geometry: Geometry) -> ET.Element:
    geom = ET.Element("geometry")
    if geometry.kind == "mesh":
        ET.SubElement(geom, "mesh", {"filename": geometry.mesh_path, "scale": _fmt_vec(geometry.scale)})
    elif geometry.kind == "box":
        ET.SubElement(geom, "box", {"size": _fmt_vec(geometry.size)})
    elif geometry.kind == "cylinder":
        ET.SubElement(
            geom, "cylinder", {"radius": _fmt_float(geometry.radius), "length": _fmt_float(geometry.length)}
        )
    elif geometry.kind == "sphere":
        ET.SubElement(geom, "sphere", {"radius": _fmt_float(geometry.radius)})
    else:  # pragma: no cover - Geometry.__post_init__ already rejects this
        raise ValueError(f"Unsupported geometry kind {geometry.kind!r}")
    return geom


# --- joints ------------------------------------------------------------


def _build_joint_element(joint: Joint) -> ET.Element:
    elem = ET.Element("joint", {"name": joint.name, "type": joint.type.value})
    ET.SubElement(elem, "parent", {"link": joint.parent})
    ET.SubElement(elem, "child", {"link": joint.child})
    _add_origin(elem, joint.origin)

    if joint.type in _AXIS_TYPES:
        # Schema guarantees axis is set (non-zero) for these joint types.
        ET.SubElement(elem, "axis", {"xyz": _fmt_vec(joint.axis)})

    if joint.type in _LIMIT_REQUIRED_TYPES:
        missing = [
            name
            for name, value in (("velocity_limit", joint.velocity_limit), ("effort_limit", joint.effort_limit))
            if value is None
        ]
        if missing:
            raise ValueError(
                f"Joint {joint.name!r} (type={joint.type.value}) is missing "
                f"{' and '.join(missing)}: URDF's <limit> element requires both "
                "'effort' and 'velocity' attributes for revolute/prismatic joints "
                "per the URDF spec, and this generator will not invent a physical "
                "value. Set Joint.velocity_limit and Joint.effort_limit before "
                "generating URDF."
            )
        # lower_limit/upper_limit are guaranteed non-None by the schema for
        # these joint types.
        ET.SubElement(
            elem,
            "limit",
            {
                "lower": _fmt_float(joint.lower_limit),
                "upper": _fmt_float(joint.upper_limit),
                "effort": _fmt_float(joint.effort_limit),
                "velocity": _fmt_float(joint.velocity_limit),
            },
        )
    elif joint.type == JointType.CONTINUOUS:
        has_velocity = joint.velocity_limit is not None
        has_effort = joint.effort_limit is not None
        if has_velocity != has_effort:
            raise ValueError(
                f"Joint {joint.name!r} (type=continuous) has only one of "
                "velocity_limit/effort_limit set. URDF's <limit> element requires "
                "'effort' and 'velocity' together, or neither — set both, or "
                "leave both unset to omit <limit> for this continuous joint."
            )
        if has_velocity and has_effort:
            ET.SubElement(
                elem,
                "limit",
                {"effort": _fmt_float(joint.effort_limit), "velocity": _fmt_float(joint.velocity_limit)},
            )
    # JointType.FIXED: no axis, no limit.

    return elem


# --- shared helpers ----------------------------------------------------


def _add_origin(parent_elem: ET.Element, pose: Pose) -> None:
    ET.SubElement(parent_elem, "origin", {"xyz": _fmt_vec(pose.xyz), "rpy": _fmt_vec(pose.rpy)})


def _fmt_float(value: float) -> str:
    """Deterministic, non-scientific-notation float formatting.

    Fixed-point with 8 decimal digits, then trailing zeros trimmed (keeping
    at least one digit after the decimal point). Avoids `repr()`/`str()`
    switching to scientific notation for very small/large magnitudes, which
    some URDF/xacro consumers don't accept.
    """
    s = f"{float(value):.8f}"
    if "." in s:
        s = s.rstrip("0")
        if s.endswith("."):
            s += "0"
    return s


def _fmt_vec(values) -> str:
    return " ".join(_fmt_float(v) for v in values)
