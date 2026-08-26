"""JSON export/import for Robot -- lets a RobotModel be saved, shared, or fed
into fusion_addin/app.py's pipeline without a live Fusion session at all
(see scripts/generate_from_json.py for exactly that standalone CLI use).

Pure stdlib, like the rest of this package: `json` + `dataclasses.asdict`.
`JointType` is a `str` subclass (see schema.py), so `json.dumps` already
serializes it as a plain string with no custom encoder -- the only real work
here is reconstructing nested dataclass instances on the way back in, since
`json.load` only ever hands back plain dicts/lists.
"""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path
from typing import Any, Dict, Optional, Type, TypeVar

from .schema import Actuator, Geometry, Inertial, Joint, JointType, Link, Material, Pose, Robot

T = TypeVar("T")


def robot_to_dict(robot: Robot) -> Dict[str, Any]:
    """Robot -> a plain, JSON-safe dict (tuples become lists; JointType,
    being a str subclass, becomes a plain string automatically)."""
    return dataclasses.asdict(robot)


def robot_to_json(robot: Robot, indent: int = 2) -> str:
    return json.dumps(robot_to_dict(robot), indent=indent, sort_keys=False) + "\n"


def save_robot_json(robot: Robot, path) -> None:
    Path(path).write_text(robot_to_json(robot), encoding="utf-8")


def _construct(cls: Type[T], data: Optional[Dict[str, Any]]) -> Optional[T]:
    """`cls(**data)`, or None if `data` is None -- for Optional[<dataclass>]
    fields (Link.material, Link.inertial, Geometry doesn't need this since
    it's never Optional-nested, etc). Field-level validation (e.g. Pose's
    xyz/rpy length checks, Inertial's mass > 0) happens exactly the same
    way as constructing these directly in code -- a malformed JSON file
    fails with the same clear ValueError a malformed direct construction
    would, not a separate, worse error path.
    """
    if data is None:
        return None
    return cls(**data)


def _link_from_dict(data: Dict[str, Any]) -> Link:
    data = dict(data)  # don't mutate the caller's dict
    data["origin"] = _construct(Pose, data.get("origin")) or Pose.IDENTITY
    data["visual_geometry"] = _construct(Geometry, data.get("visual_geometry"))
    data["collision_geometry"] = _construct(Geometry, data.get("collision_geometry"))
    data["material"] = _construct(Material, data.get("material"))
    data["inertial"] = _construct(Inertial, data.get("inertial"))
    return Link(**data)


def _joint_from_dict(data: Dict[str, Any]) -> Joint:
    data = dict(data)
    data["type"] = JointType(data["type"])
    data["origin"] = _construct(Pose, data.get("origin")) or Pose.IDENTITY
    if data.get("axis") is not None:
        data["axis"] = tuple(data["axis"])
    return Joint(**data)


def _sensor_from_dict(data: Dict[str, Any]) -> "Sensor":
    from .schema import Sensor

    data = dict(data)
    data["origin"] = _construct(Pose, data.get("origin")) or Pose.IDENTITY
    return Sensor(**data)


def robot_from_dict(data: Dict[str, Any]) -> Robot:
    """The inverse of `robot_to_dict`. Raises the same ValueError/TypeError
    a malformed direct `Robot(...)`/`Link(...)`/etc. construction would --
    no separate, softer validation path for JSON input. Does NOT call
    `robot.validate()` itself (matching every other Robot-constructing
    function in this codebase, e.g. fusion_addin/extraction/converter.py's
    build_robot_model calls it explicitly at the end) -- callers that need
    a guaranteed-valid Robot should call `.validate()` themselves.
    """
    data = dict(data)
    data["links"] = [_link_from_dict(d) for d in data.get("links", [])]
    data["joints"] = [_joint_from_dict(d) for d in data.get("joints", [])]
    data["sensors"] = [_sensor_from_dict(d) for d in data.get("sensors", [])]
    data["actuators"] = [Actuator(**d) for d in data.get("actuators", [])]
    return Robot(**data)


def robot_from_json(text: str) -> Robot:
    return robot_from_dict(json.loads(text))


def load_robot_json(path) -> Robot:
    return robot_from_json(Path(path).read_text(encoding="utf-8"))
