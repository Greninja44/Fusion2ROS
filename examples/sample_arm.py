"""A small, hand-authored 3-link arm RobotModel -- no Fusion needed.

This is exactly ARCHITECTURE.md's "First vertical-slice milestone" step 8:
a hand-built Robot fixture flowing through URDF generation, package
generation, and a colcon build/RViz launch, proving the non-Fusion half of
the pipeline end to end before a real Fusion extraction is available to
test against.

Uses primitive (box/cylinder) visual geometry rather than mesh files, since
there's no Fusion session here to export real STLs from -- primitives are a
first-class Geometry kind for exactly this reason (see robot_model/schema.py).
"""

import math

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
)


def build_sample_arm() -> Robot:
    base = Link(
        name="base_link",
        origin=Pose.IDENTITY,
        visual_geometry=Geometry(kind="cylinder", radius=0.08, length=0.05),
        collision_geometry=Geometry(kind="cylinder", radius=0.08, length=0.05),
        material=Material(name="base_grey", rgba=(0.4, 0.4, 0.4, 1.0)),
        inertial=Inertial(mass=1.5, center_of_mass=(0.0, 0.0, 0.0), ixx=0.004, iyy=0.004, izz=0.006),
    )
    upper_arm = Link(
        name="upper_arm",
        parent="base_link",
        origin=Pose.IDENTITY,
        visual_geometry=Geometry(kind="box", size=(0.05, 0.05, 0.3)),
        collision_geometry=Geometry(kind="box", size=(0.05, 0.05, 0.3)),
        material=Material(name="arm_blue", rgba=(0.2, 0.4, 0.9, 1.0)),
        inertial=Inertial(mass=0.8, center_of_mass=(0.0, 0.0, 0.15), ixx=0.006, iyy=0.006, izz=0.0007),
    )
    forearm = Link(
        name="forearm",
        parent="upper_arm",
        origin=Pose.IDENTITY,
        visual_geometry=Geometry(kind="box", size=(0.04, 0.04, 0.25)),
        collision_geometry=Geometry(kind="box", size=(0.04, 0.04, 0.25)),
        material=Material(name="arm_orange", rgba=(0.9, 0.5, 0.1, 1.0)),
        inertial=Inertial(mass=0.4, center_of_mass=(0.0, 0.0, 0.125), ixx=0.002, iyy=0.002, izz=0.0003),
    )

    shoulder = Joint(
        name="shoulder_joint",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="upper_arm",
        origin=Pose(xyz=(0.0, 0.0, 0.025)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-math.pi,
        upper_limit=math.pi,
        velocity_limit=2.0,
        effort_limit=20.0,
    )
    elbow = Joint(
        name="elbow_joint",
        type=JointType.REVOLUTE,
        parent="upper_arm",
        child="forearm",
        origin=Pose(xyz=(0.0, 0.0, 0.3)),
        axis=(0.0, 1.0, 0.0),
        lower_limit=-2.0,
        upper_limit=2.0,
        velocity_limit=2.0,
        effort_limit=10.0,
    )

    shoulder_motor = Actuator(name="shoulder_motor", type="electric_motor", joint="shoulder_joint", interface="position")
    elbow_motor = Actuator(name="elbow_motor", type="electric_motor", joint="elbow_joint", interface="position")

    robot = Robot(
        name="sample_arm",
        links=[base, upper_arm, forearm],
        joints=[shoulder, elbow],
        actuators=[shoulder_motor, elbow_motor],
        metadata={"source": "examples.sample_arm (hand-authored, no Fusion)"},
    )
    robot.validate()
    return robot


if __name__ == "__main__":
    r = build_sample_arm()
    print(f"Built {r.name}: {len(r.links)} links, {len(r.joints)} joints, validate() OK")
