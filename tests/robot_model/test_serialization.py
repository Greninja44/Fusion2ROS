"""Tests for robot_model.serialization. Must run with plain `python3 -m
pytest` -- no Fusion, no ROS, pure stdlib json."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from robot_model import (
    Actuator,
    Geometry,
    Inertial,
    Joint,
    JointType,
    Link,
    Material,
    Pose,
    Robot,
    Sensor,
    ValidationError,
    load_robot_json,
    robot_from_dict,
    robot_from_json,
    robot_to_dict,
    robot_to_json,
    save_robot_json,
)


def make_full_robot() -> Robot:
    base = Link(
        name="base_link",
        origin=Pose(xyz=(0.0, 0.0, 0.0)),
        visual_geometry=Geometry(kind="cylinder", radius=0.08, length=0.05),
        collision_geometry=Geometry(kind="box", size=(0.1, 0.1, 0.1)),
        material=Material(name="grey", rgba=(0.5, 0.5, 0.5, 1.0)),
        inertial=Inertial(mass=1.0, center_of_mass=(0.01, 0.0, 0.0), ixx=0.01, iyy=0.01, izz=0.01, ixy=0.001),
        metadata={"note": "root link"},
    )
    arm = Link(
        name="arm",
        parent="base_link",
        visual_geometry=Geometry(kind="mesh", mesh_path="package://demo/meshes/arm.stl"),
        inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001),
    )
    joint = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="arm",
        origin=Pose(xyz=(0.0, 0.0, 0.1), rpy=(0.0, 0.0, 1.5707963)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        velocity_limit=2.0,
        effort_limit=10.0,
    )
    sensor = Sensor(
        name="cam1",
        type="camera",
        parent_link="arm",
        origin=Pose(xyz=(0.0, 0.0, 0.05)),
        parameters={"width": 640, "height": 480},
    )
    actuator = Actuator(
        name="joint1_motor",
        type="electric_motor",
        joint="joint1",
        interface="position",
        limits={"max_current": 5.0},
        metadata={"vendor": "acme"},
    )
    return Robot(
        name="full_demo",
        links=[base, arm],
        joints=[joint],
        sensors=[sensor],
        actuators=[actuator],
        metadata={"drivetrain": {"type": "differential_drive", "left_wheel_joint": "joint1"}},
    )


def test_round_trip_preserves_everything():
    robot = make_full_robot()
    robot.validate()

    restored = robot_from_dict(robot_to_dict(robot))
    restored.validate()

    assert restored.name == robot.name
    assert len(restored.links) == 2
    assert restored.link("base_link").visual_geometry.kind == "cylinder"
    assert restored.link("base_link").visual_geometry.radius == 0.08
    assert restored.link("base_link").collision_geometry.size == (0.1, 0.1, 0.1)
    assert restored.link("base_link").material.rgba == (0.5, 0.5, 0.5, 1.0)
    assert restored.link("base_link").inertial.ixy == 0.001
    assert restored.link("base_link").metadata == {"note": "root link"}
    assert restored.link("arm").visual_geometry.mesh_path == "package://demo/meshes/arm.stl"

    j = restored.joint("joint1")
    assert j.type == JointType.REVOLUTE
    assert isinstance(j.type, JointType)
    assert j.axis == (0.0, 0.0, 1.0)
    assert j.origin.rpy == (0.0, 0.0, 1.5707963)
    assert j.velocity_limit == 2.0

    s = restored.sensors[0]
    assert s.type == "camera"
    assert s.parameters == {"width": 640, "height": 480}

    a = restored.actuators[0]
    assert a.interface == "position"
    assert a.limits == {"max_current": 5.0}
    assert a.metadata == {"vendor": "acme"}

    assert restored.metadata["drivetrain"]["type"] == "differential_drive"


def test_json_round_trip_via_string():
    robot = make_full_robot()
    text = robot_to_json(robot)
    assert isinstance(text, str)
    restored = robot_from_json(text)
    restored.validate()
    assert restored.name == robot.name


def test_json_round_trip_via_file(tmp_path):
    robot = make_full_robot()
    path = tmp_path / "robot.json"
    save_robot_json(robot, path)
    assert path.exists()

    restored = load_robot_json(path)
    restored.validate()
    assert restored.name == robot.name
    assert len(restored.joints) == 1


def test_to_dict_uses_plain_json_safe_types():
    import json

    robot = make_full_robot()
    data = robot_to_dict(robot)
    # JointType (a str subclass) must serialize as a plain string -- what
    # makes json.dumps work without a custom encoder. asdict() leaves
    # tuples as tuples (not list), but json.dumps still accepts them fine
    # (they serialize as JSON arrays) -- confirm that end-to-end instead of
    # asserting an internal representation detail.
    assert data["joints"][0]["type"] == "revolute"
    assert isinstance(data["joints"][0]["type"], str)
    text = json.dumps(data)
    assert json.loads(text)["links"][0]["origin"]["xyz"] == [0.0, 0.0, 0.0]


def test_malformed_json_raises_same_errors_as_direct_construction():
    # A negative mass should fail exactly the way constructing
    # Inertial(mass=-1.0) directly would -- no separate, softer path.
    robot = make_full_robot()
    data = robot_to_dict(robot)
    data["links"][0]["inertial"]["mass"] = -1.0
    with pytest.raises(ValueError):
        robot_from_dict(data)


def test_link_without_optional_fields_round_trips():
    minimal = Robot(name="minimal", links=[Link(name="only_link")], joints=[])
    restored = robot_from_dict(robot_to_dict(minimal))
    restored.validate()
    link = restored.link("only_link")
    assert link.visual_geometry is None
    assert link.collision_geometry is None
    assert link.material is None
    assert link.inertial is None


def test_empty_robot_round_trips():
    robot = Robot(name="empty")
    restored = robot_from_dict(robot_to_dict(robot))
    assert restored.name == "empty"
    assert restored.links == []
    assert restored.joints == []
    assert restored.sensors == []
    assert restored.actuators == []
