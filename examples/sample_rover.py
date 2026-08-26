"""A small, hand-authored differential-drive rover RobotModel -- no Fusion
needed. Exercises the mobile-base path (ros2_control's diff_drive_controller,
Gazebo simulation, Nav2) the way examples/sample_arm.py exercises the
arm/manipulator path (ros2_control's joint_trajectory_controller, MoveIt 2).

Uses the documented `robot.metadata["drivetrain"]` convention shared by
fusion_addin/generators/ros2_control.py and fusion_addin/generators/nav2.py.
"""

from robot_model import Actuator, Geometry, Inertial, Joint, JointType, Link, Material, Pose, Robot


def build_sample_rover() -> Robot:
    base = Link(
        name="base_link",
        origin=Pose.IDENTITY,
        visual_geometry=Geometry(kind="box", size=(0.4, 0.3, 0.15)),
        collision_geometry=Geometry(kind="box", size=(0.4, 0.3, 0.15)),
        material=Material(name="chassis_green", rgba=(0.2, 0.6, 0.2, 1.0)),
        inertial=Inertial(mass=5.0, center_of_mass=(0.0, 0.0, 0.0), ixx=0.05, iyy=0.08, izz=0.09),
    )
    left_wheel = Link(
        name="left_wheel",
        parent="base_link",
        origin=Pose(rpy=(1.5707963, 0.0, 0.0)),
        visual_geometry=Geometry(kind="cylinder", radius=0.1, length=0.04),
        collision_geometry=Geometry(kind="cylinder", radius=0.1, length=0.04),
        material=Material(name="wheel_black", rgba=(0.1, 0.1, 0.1, 1.0)),
        inertial=Inertial(mass=0.3, center_of_mass=(0.0, 0.0, 0.0), ixx=0.0008, iyy=0.0008, izz=0.0015),
    )
    right_wheel = Link(
        name="right_wheel",
        parent="base_link",
        origin=Pose(rpy=(1.5707963, 0.0, 0.0)),
        visual_geometry=Geometry(kind="cylinder", radius=0.1, length=0.04),
        collision_geometry=Geometry(kind="cylinder", radius=0.1, length=0.04),
        material=Material(name="wheel_black", rgba=(0.1, 0.1, 0.1, 1.0)),
        inertial=Inertial(mass=0.3, center_of_mass=(0.0, 0.0, 0.0), ixx=0.0008, iyy=0.0008, izz=0.0015),
    )
    caster = Link(
        name="caster_wheel",
        parent="base_link",
        origin=Pose.IDENTITY,
        visual_geometry=Geometry(kind="sphere", radius=0.05),
        collision_geometry=Geometry(kind="sphere", radius=0.05),
        material=Material(name="wheel_black", rgba=(0.1, 0.1, 0.1, 1.0)),
        inertial=Inertial(mass=0.1, center_of_mass=(0.0, 0.0, 0.0), ixx=0.0001, iyy=0.0001, izz=0.0001),
    )

    left_joint = Joint(
        name="left_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="left_wheel",
        origin=Pose(xyz=(0.0, 0.175, -0.05)),
        axis=(0.0, 1.0, 0.0),
    )
    right_joint = Joint(
        name="right_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="right_wheel",
        origin=Pose(xyz=(0.0, -0.175, -0.05)),
        axis=(0.0, 1.0, 0.0),
    )
    caster_joint = Joint(
        name="caster_wheel_joint",
        type=JointType.FIXED,
        parent="base_link",
        child="caster_wheel",
        origin=Pose(xyz=(0.15, 0.0, -0.1)),
    )

    left_motor = Actuator(name="left_wheel_motor", type="electric_motor", joint="left_wheel_joint", interface="velocity")
    right_motor = Actuator(
        name="right_wheel_motor", type="electric_motor", joint="right_wheel_joint", interface="velocity"
    )

    robot = Robot(
        name="sample_rover",
        links=[base, left_wheel, right_wheel, caster],
        joints=[left_joint, right_joint, caster_joint],
        actuators=[left_motor, right_motor],
        metadata={
            "source": "examples.sample_rover (hand-authored, no Fusion)",
            "drivetrain": {
                "type": "differential_drive",
                "left_wheel_joint": "left_wheel_joint",
                "right_wheel_joint": "right_wheel_joint",
                "wheel_separation": 0.35,
                "wheel_radius": 0.1,
            },
        },
    )
    robot.validate()
    return robot


if __name__ == "__main__":
    r = build_sample_rover()
    print(f"Built {r.name}: {len(r.links)} links, {len(r.joints)} joints, validate() OK")
