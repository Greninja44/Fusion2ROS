"""Canonical RobotModel schema.

This module is the one piece of code shared, unmodified, between the Windows
Fusion 360 add-in process and the WSL/ROS 2 tooling. It MUST stay pure
Python 3 standard library — no Fusion API imports, no ROS/rclpy imports, no
third-party packages — so it can be imported as-is from Fusion's embedded
interpreter, from plain WSL `python3`, and from CI with neither installed.

All quantities are SI: meters, kilograms, radians, seconds. Fusion's native
units (typically centimeters) must be converted to SI at the extraction
boundary in fusion_addin/extraction/ — never here, and never downstream.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import ClassVar, Dict, List, Optional, Tuple

from .errors import ValidationError

Vec3 = Tuple[float, float, float]


class JointType(str, Enum):
    FIXED = "fixed"
    REVOLUTE = "revolute"
    PRISMATIC = "prismatic"
    CONTINUOUS = "continuous"


def _check_vec3(value, field_name: str) -> Vec3:
    if not (isinstance(value, (tuple, list)) and len(value) == 3):
        raise ValueError(f"{field_name} must be a 3-tuple of floats, got {value!r}")
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class Pose:
    """A rigid transform, expressed the way URDF's <origin> is: xyz in
    meters plus roll-pitch-yaw in radians (applied about fixed axes)."""

    xyz: Vec3 = (0.0, 0.0, 0.0)
    rpy: Vec3 = (0.0, 0.0, 0.0)

    def __post_init__(self):
        object.__setattr__(self, "xyz", _check_vec3(self.xyz, "Pose.xyz"))
        object.__setattr__(self, "rpy", _check_vec3(self.rpy, "Pose.rpy"))

    IDENTITY: ClassVar["Pose"]


Pose.IDENTITY = Pose()


@dataclass(frozen=True)
class Geometry:
    """Visual/collision shape. Either a mesh reference or a primitive.

    Not every Fusion body converts cleanly to a mesh (e.g. failed tessellation,
    or a deliberately simplified collision proxy) — primitives are a first-class
    fallback, not an afterthought.
    """

    kind: str  # "mesh" | "box" | "cylinder" | "sphere"
    mesh_path: Optional[str] = None  # package://<robot>/meshes/... for kind == "mesh"
    scale: Vec3 = (1.0, 1.0, 1.0)
    size: Optional[Vec3] = None  # box: full extents (m)
    radius: Optional[float] = None  # cylinder/sphere (m)
    length: Optional[float] = None  # cylinder (m)

    _VALID_KINDS = {"mesh", "box", "cylinder", "sphere"}

    def __post_init__(self):
        if self.kind not in self._VALID_KINDS:
            raise ValueError(f"Geometry.kind must be one of {self._VALID_KINDS}, got {self.kind!r}")
        object.__setattr__(self, "scale", _check_vec3(self.scale, "Geometry.scale"))
        if self.kind == "mesh" and not self.mesh_path:
            raise ValueError("Geometry(kind='mesh') requires mesh_path")
        if self.kind == "box" and self.size is None:
            raise ValueError("Geometry(kind='box') requires size")
        if self.kind in ("cylinder", "sphere") and self.radius is None:
            raise ValueError(f"Geometry(kind={self.kind!r}) requires radius")
        if self.kind == "cylinder" and self.length is None:
            raise ValueError("Geometry(kind='cylinder') requires length")
        if self.size is not None:
            object.__setattr__(self, "size", _check_vec3(self.size, "Geometry.size"))


@dataclass(frozen=True)
class Material:
    name: str
    rgba: Optional[Tuple[float, float, float, float]] = None

    def __post_init__(self):
        if not self.name:
            raise ValueError("Material.name must be non-empty")
        if self.rgba is not None:
            if len(self.rgba) != 4:
                raise ValueError("Material.rgba must be an (r, g, b, a) 4-tuple")
            for c in self.rgba:
                if not (0.0 <= float(c) <= 1.0):
                    raise ValueError("Material.rgba components must be in [0, 1]")


@dataclass(frozen=True)
class Inertial:
    """Mass properties. Inertia tensor components are about the link's
    center of mass, in kg*m^2, matching URDF's <inertia> convention."""

    mass: float
    center_of_mass: Vec3 = (0.0, 0.0, 0.0)
    ixx: float = 0.0
    iyy: float = 0.0
    izz: float = 0.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    def __post_init__(self):
        if self.mass <= 0.0:
            raise ValueError(f"Inertial.mass must be > 0 kg, got {self.mass}")
        object.__setattr__(self, "center_of_mass", _check_vec3(self.center_of_mass, "Inertial.center_of_mass"))
        for diag_name in ("ixx", "iyy", "izz"):
            if getattr(self, diag_name) < 0.0:
                raise ValueError(f"Inertial.{diag_name} must be >= 0")


@dataclass
class Link:
    name: str
    parent: Optional[str] = None  # parent LINK name; None only for the root link
    visual_geometry: Optional[Geometry] = None
    collision_geometry: Optional[Geometry] = None
    origin: Pose = field(default_factory=lambda: Pose.IDENTITY)
    material: Optional[Material] = None
    inertial: Optional[Inertial] = None
    metadata: Dict[str, object] = field(default_factory=dict)

    def __post_init__(self):
        if not self.name:
            raise ValueError("Link.name must be non-empty")


@dataclass
class Joint:
    name: str
    type: JointType
    parent: str  # parent link name
    child: str  # child link name
    origin: Pose = field(default_factory=lambda: Pose.IDENTITY)
    axis: Optional[Vec3] = None
    lower_limit: Optional[float] = None
    upper_limit: Optional[float] = None
    velocity_limit: Optional[float] = None
    effort_limit: Optional[float] = None

    def __post_init__(self):
        if not self.name:
            raise ValueError("Joint.name must be non-empty")
        if not isinstance(self.type, JointType):
            self.type = JointType(self.type)
        if not self.parent or not self.child:
            raise ValueError(f"Joint {self.name!r} must have both parent and child link names")
        if self.parent == self.child:
            raise ValueError(f"Joint {self.name!r} has identical parent and child link {self.parent!r}")

        if self.type in (JointType.REVOLUTE, JointType.PRISMATIC, JointType.CONTINUOUS):
            if self.axis is None:
                raise ValueError(f"Joint {self.name!r} of type {self.type.value} requires an axis")
            self.axis = _check_vec3(self.axis, f"Joint {self.name!r}.axis")
            if all(abs(c) < 1e-12 for c in self.axis):
                raise ValueError(f"Joint {self.name!r}.axis must not be the zero vector")
        elif self.axis is not None:
            self.axis = _check_vec3(self.axis, f"Joint {self.name!r}.axis")

        if self.type in (JointType.REVOLUTE, JointType.PRISMATIC):
            if self.lower_limit is None or self.upper_limit is None:
                raise ValueError(f"Joint {self.name!r} of type {self.type.value} requires lower_limit and upper_limit")
        if self.type == JointType.CONTINUOUS:
            if self.lower_limit is not None or self.upper_limit is not None:
                raise ValueError(f"Joint {self.name!r} of type continuous must not have position limits")

        if self.lower_limit is not None and self.upper_limit is not None:
            if self.lower_limit > self.upper_limit:
                raise ValueError(
                    f"Joint {self.name!r} lower_limit ({self.lower_limit}) exceeds upper_limit ({self.upper_limit})"
                )
        for limit_name in ("velocity_limit", "effort_limit"):
            v = getattr(self, limit_name)
            if v is not None and v < 0.0:
                raise ValueError(f"Joint {self.name!r}.{limit_name} must be >= 0")


@dataclass
class Sensor:
    name: str
    type: str  # "camera" | "lidar" | "imu" | ... (open set, downstream generators decide support)
    parent_link: str
    origin: Pose = field(default_factory=lambda: Pose.IDENTITY)
    parameters: Dict[str, object] = field(default_factory=dict)

    def __post_init__(self):
        if not self.name:
            raise ValueError("Sensor.name must be non-empty")
        if not self.type:
            raise ValueError(f"Sensor {self.name!r} must have a type")
        if not self.parent_link:
            raise ValueError(f"Sensor {self.name!r} must have a parent_link")


@dataclass
class Actuator:
    name: str
    type: str  # e.g. "electric_motor"
    joint: str
    interface: str = "position"  # ros2_control hardware interface: position|velocity|effort
    limits: Dict[str, float] = field(default_factory=dict)
    metadata: Dict[str, object] = field(default_factory=dict)

    def __post_init__(self):
        if not self.name:
            raise ValueError("Actuator.name must be non-empty")
        if not self.joint:
            raise ValueError(f"Actuator {self.name!r} must reference a joint")


@dataclass
class Robot:
    name: str
    links: List[Link] = field(default_factory=list)
    joints: List[Joint] = field(default_factory=list)
    sensors: List[Sensor] = field(default_factory=list)
    actuators: List[Actuator] = field(default_factory=list)
    metadata: Dict[str, object] = field(default_factory=dict)

    def link(self, name: str) -> Optional[Link]:
        return next((l for l in self.links if l.name == name), None)

    def joint(self, name: str) -> Optional[Joint]:
        return next((j for j in self.joints if j.name == name), None)

    def root_link(self) -> Optional[Link]:
        roots = [l for l in self.links if l.parent is None]
        return roots[0] if len(roots) == 1 else None

    def validate(self, raise_on_error: bool = True) -> List[str]:
        """Check graph-level invariants that individual dataclasses can't
        check on their own (uniqueness, references, tree connectivity).

        Returns the list of problems found (empty if valid). Raises
        ValidationError instead when raise_on_error is True (the default).
        """
        problems: List[str] = []

        if not self.name:
            problems.append("Robot.name must be non-empty")

        link_names = [l.name for l in self.links]
        _report_duplicates(link_names, "link", problems)
        link_set = set(link_names)

        joint_names = [j.name for j in self.joints]
        _report_duplicates(joint_names, "joint", problems)

        sensor_names = [s.name for s in self.sensors]
        _report_duplicates(sensor_names, "sensor", problems)

        actuator_names = [a.name for a in self.actuators]
        _report_duplicates(actuator_names, "actuator", problems)

        if not self.links:
            problems.append("Robot has no links")

        roots = [l for l in self.links if l.parent is None]
        if len(roots) == 0 and self.links:
            problems.append("Robot has no root link (every link declares a parent)")
        elif len(roots) > 1:
            problems.append(f"Robot has multiple root links: {[l.name for l in roots]} (must have exactly one)")

        # Joint parent/child must reference real links.
        for j in self.joints:
            if j.parent not in link_set:
                problems.append(f"Joint {j.name!r} references unknown parent link {j.parent!r}")
            if j.child not in link_set:
                problems.append(f"Joint {j.name!r} references unknown child link {j.child!r}")

        # Each non-root link must be the child of exactly one joint, and that
        # joint's parent must agree with Link.parent (the two are redundant
        # by design for quick lookups; they must never disagree).
        joints_by_child: Dict[str, List[Joint]] = {}
        for j in self.joints:
            joints_by_child.setdefault(j.child, []).append(j)

        for l in self.links:
            if l.parent is None:
                continue
            child_joints = joints_by_child.get(l.name, [])
            if len(child_joints) == 0:
                problems.append(f"Link {l.name!r} declares parent {l.parent!r} but no joint has it as child")
            elif len(child_joints) > 1:
                problems.append(f"Link {l.name!r} is the child of multiple joints: {[j.name for j in child_joints]}")
            else:
                jparent = child_joints[0].parent
                if jparent != l.parent:
                    problems.append(
                        f"Link {l.name!r}.parent ({l.parent!r}) disagrees with its joint "
                        f"{child_joints[0].name!r}.parent ({jparent!r})"
                    )

        # Connectivity + cycle check via BFS from the root.
        if len(roots) == 1:
            root = roots[0]
            children: Dict[str, List[str]] = {}
            for j in self.joints:
                children.setdefault(j.parent, []).append(j.child)

            visited = set()
            frontier = [root.name]
            while frontier:
                name = frontier.pop()
                if name in visited:
                    problems.append(f"Cycle detected in robot tree at link {name!r}")
                    continue
                visited.add(name)
                frontier.extend(children.get(name, []))

            unreached = link_set - visited
            if unreached:
                problems.append(f"Links disconnected from root {root.name!r}: {sorted(unreached)}")

        # Sensor/actuator references.
        for s in self.sensors:
            if s.parent_link not in link_set:
                problems.append(f"Sensor {s.name!r} references unknown parent_link {s.parent_link!r}")

        joint_set = set(joint_names)
        for a in self.actuators:
            if a.joint not in joint_set:
                problems.append(f"Actuator {a.name!r} references unknown joint {a.joint!r}")

        if raise_on_error and problems:
            raise ValidationError(problems)
        return problems


def _report_duplicates(names: List[str], kind: str, problems: List[str]) -> None:
    seen = set()
    dupes = set()
    for n in names:
        if n in seen:
            dupes.add(n)
        seen.add(n)
    for d in sorted(dupes):
        problems.append(f"Duplicate {kind} name: {d!r}")
