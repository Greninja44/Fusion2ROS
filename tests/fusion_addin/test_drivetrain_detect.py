"""Tests for fusion_addin.extraction.drivetrain_detect.detect_drivetrain.

Pure-Python, no Fusion/adsk mocking needed -- this module only touches
robot_model.Robot and stdlib math.
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.extraction.drivetrain_detect import _world_positions, detect_drivetrain
from robot_model import Geometry, Inertial, Joint, JointType, Link, Pose, Robot


def _link(name, parent=None, bbox=None, geometry=None):
    metadata = {"bounding_box_size": bbox} if bbox else {}
    return Link(
        name=name,
        parent=parent,
        inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001),
        collision_geometry=geometry,
        metadata=metadata,
    )


def _continuous_joint(name, parent, child, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    return Joint(
        name=name,
        type=JointType.CONTINUOUS,
        parent=parent,
        child=child,
        origin=Pose(xyz=xyz, rpy=rpy),
        axis=(0.0, 1.0, 0.0),
    )


# ---------------------------------------------------------------------------
# _world_positions (forward kinematics)
# ---------------------------------------------------------------------------


def test_world_positions_simple_translation_chain():
    base = _link("base_link")
    arm = _link("arm", parent="base_link")
    robot = Robot(name="r", links=[base, arm], joints=[_continuous_joint("j1", "base_link", "arm", xyz=(1.0, 2.0, 3.0))])

    positions = _world_positions(robot)

    assert positions["base_link"] == pytest.approx((0.0, 0.0, 0.0))
    assert positions["arm"] == pytest.approx((1.0, 2.0, 3.0))


def test_world_positions_composes_through_a_yaw_rotation():
    # base_link -> rotated_link (90 deg yaw, no translation) -> tip (1m along
    # the rotated link's local +X, which after a 90 deg yaw points along
    # world +Y).
    base = _link("base_link")
    rotated = _link("rotated_link", parent="base_link")
    tip = _link("tip", parent="rotated_link")
    robot = Robot(
        name="r",
        links=[base, rotated, tip],
        joints=[
            _continuous_joint("j1", "base_link", "rotated_link", xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 2)),
            _continuous_joint("j2", "rotated_link", "tip", xyz=(1.0, 0.0, 0.0)),
        ],
    )

    positions = _world_positions(robot)

    assert positions["tip"] == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)


def test_world_positions_resolves_regardless_of_input_joint_order():
    base = _link("base_link")
    mid = _link("mid", parent="base_link")
    tip = _link("tip", parent="mid")
    # child-before-parent order in the input list -- the worklist must not
    # depend on joints being given in root-to-leaf order.
    robot = Robot(
        name="r",
        links=[base, mid, tip],
        joints=[
            _continuous_joint("j2", "mid", "tip", xyz=(0.0, 0.0, 1.0)),
            _continuous_joint("j1", "base_link", "mid", xyz=(1.0, 0.0, 0.0)),
        ],
    )

    positions = _world_positions(robot)

    assert positions["tip"] == pytest.approx((1.0, 0.0, 1.0))


# ---------------------------------------------------------------------------
# detect_drivetrain
# ---------------------------------------------------------------------------


def _simple_two_wheel_robot(wheel_radius_bbox=(0.2, 0.04, 0.2)):
    base = _link("base_link")
    left_wheel = _link("left_wheel_link", parent="base_link", bbox=wheel_radius_bbox)
    right_wheel = _link("right_wheel_link", parent="base_link", bbox=wheel_radius_bbox)
    return Robot(
        name="rover",
        links=[base, left_wheel, right_wheel],
        joints=[
            _continuous_joint("left_wheel_joint", "base_link", "left_wheel_link", xyz=(0.0, 0.2, 0.0)),
            _continuous_joint("right_wheel_joint", "base_link", "right_wheel_link", xyz=(0.0, -0.2, 0.0)),
        ],
    )


def test_detects_simple_two_wheel_differential_drive():
    robot = _simple_two_wheel_robot()

    drivetrain = detect_drivetrain(robot)

    assert drivetrain is not None
    assert drivetrain["type"] == "differential_drive"
    assert {drivetrain["left_wheel_joint"], drivetrain["right_wheel_joint"]} == {
        "left_wheel_joint",
        "right_wheel_joint",
    }
    assert drivetrain["wheel_separation"] == pytest.approx(0.4)
    assert drivetrain["wheel_radius"] == pytest.approx(0.1)  # median(0.2, 0.04, 0.2) / 2


def test_prefers_exact_cylinder_geometry_over_bounding_box_estimate():
    base = _link("base_link")
    left_wheel = _link(
        "left_wheel_link",
        parent="base_link",
        bbox=(0.2, 0.04, 0.2),  # would estimate 0.1m if used -- must NOT be used
        geometry=Geometry(kind="cylinder", radius=0.075, length=0.04),
    )
    right_wheel = _link("right_wheel_link", parent="base_link", bbox=(0.2, 0.04, 0.2))
    robot = Robot(
        name="rover",
        links=[base, left_wheel, right_wheel],
        joints=[
            _continuous_joint("left_wheel_joint", "base_link", "left_wheel_link", xyz=(0.0, 0.2, 0.0)),
            _continuous_joint("right_wheel_joint", "base_link", "right_wheel_link", xyz=(0.0, -0.2, 0.0)),
        ],
    )

    drivetrain = detect_drivetrain(robot)

    assert drivetrain["wheel_radius"] == pytest.approx(0.075)


def test_six_wheel_rover_picks_the_middle_wheel_per_side():
    # Mirrors the real rover shape hit during live testing: three wheels per
    # side (front/mid/rear), each hanging off its OWN separate parent link
    # (no shared parent between same-side wheels) -- detect_drivetrain must
    # still pick the middle one per side via forward kinematics, not naive
    # same-parent grouping.
    base = _link("base_link")
    links = [base]
    joints = []
    bbox = (0.1, 0.03, 0.1)
    layout = {
        "front_left": (0.3, 0.2, 0.0),
        "mid_left": (0.0, 0.2, 0.0),
        "rear_left": (-0.3, 0.2, 0.0),
        "front_right": (0.3, -0.2, 0.0),
        "mid_right": (0.0, -0.2, 0.0),
        "rear_right": (-0.3, -0.2, 0.0),
    }
    for name, xyz in layout.items():
        hub = f"hub_{name}"
        wheel = f"wheel_link_{name}"
        links.append(_link(hub, parent="base_link"))
        links.append(_link(wheel, parent=hub, bbox=bbox))
        joints.append(_continuous_joint(f"rigid_{hub}", "base_link", hub, xyz=xyz))
        joints.append(_continuous_joint(f"wheel_joint_{name}", hub, wheel, xyz=(0.0, 0.0, 0.0)))

    robot = Robot(name="six_wheel_rover", links=links, joints=joints)

    drivetrain = detect_drivetrain(robot)

    assert drivetrain is not None
    assert {drivetrain["left_wheel_joint"], drivetrain["right_wheel_joint"]} == {
        "wheel_joint_mid_left",
        "wheel_joint_mid_right",
    }
    assert drivetrain["wheel_separation"] == pytest.approx(0.4)


def test_no_wheel_named_joints_returns_none():
    base = _link("base_link")
    arm = _link("arm", parent="base_link")
    robot = Robot(name="arm_robot", links=[base, arm], joints=[_continuous_joint("joint1", "base_link", "arm")])

    assert detect_drivetrain(robot) is None


def test_single_wheel_joint_returns_none():
    base = _link("base_link")
    wheel = _link("wheel_link", parent="base_link", bbox=(0.2, 0.04, 0.2))
    robot = Robot(
        name="unicycle",
        links=[base, wheel],
        joints=[_continuous_joint("left_wheel_joint", "base_link", "wheel_link")],
    )

    assert detect_drivetrain(robot) is None


def test_no_geometry_at_all_returns_none_rather_than_guess():
    base = _link("base_link")
    left_wheel = _link("left_wheel_link", parent="base_link")  # no bbox, no cylinder
    right_wheel = _link("right_wheel_link", parent="base_link")
    robot = Robot(
        name="rover",
        links=[base, left_wheel, right_wheel],
        joints=[
            _continuous_joint("left_wheel_joint", "base_link", "left_wheel_link", xyz=(0.0, 0.2, 0.0)),
            _continuous_joint("right_wheel_joint", "base_link", "right_wheel_link", xyz=(0.0, -0.2, 0.0)),
        ],
    )

    assert detect_drivetrain(robot) is None


def test_real_six_wheel_rover_geometry_from_live_fusion_testing_declines_to_guess():
    # Exact joint names/origins/rpy copied from a real Fusion-generated
    # ~/ros2_ws/src/main_assembly/urdf/main_assembly.urdf.xacro produced
    # during this project's live testing. The user manually chose
    # Revolute_WheelMidA/Revolute_WheelMidB as the differential_drive pair
    # by hand; detect_drivetrain, run against the same real geometry,
    # correctly DECLINES to guess (returns None) rather than picking wrong
    # -- this rover mounts its front wheel on a separate fixed leg (not the
    # same pivoting bogie arm its mid/rear wheels share), which lands it
    # geometrically closer to the OPPOSITE side's wheels than its own
    # side's along every one of X/Y/Z (confirmed by hand and independently
    # cross-checked against PyKDL's FK during this test's development), so
    # no axis cleanly splits all six wheels into two even groups. This is a
    # real, known limitation (see detect_drivetrain's docstring / the
    # `abs(len(left) - len(right)) > 1` guard) for an asymmetric
    # front-leg/rear-bogie design, not a bug -- returning no guess here is
    # the conservative, documented behavior, not a false one.
    root = _link("Part_1_1")
    links = [
        root,
        _link("Part_2_1_1", parent="Part_1_1"),
        _link("Part_1_4_1", parent="Part_1_1"),
        _link("Part_2_1", parent="Part_1_1"),
        _link("Part_1_5_1", parent="Part_1_1"),
        _link("Part_1_3_5", parent="Part_2_1_1"),
        _link("Part_1_3_2", parent="Part_2_1"),
        _link("Part_1_3_6", parent="Part_2_1"),
        _link("Part_1_3_4", parent="Part_1_4_1"),
        _link("Part_1_3_1", parent="Part_1_5_1"),
        _link("Part_1_3_3", parent="Part_1_5_1"),
        _link("Wheels_v8_2", parent="Part_1_3_5", bbox=(0.1, 0.04, 0.1)),
        _link("Wheels_v8_1", parent="Part_1_3_2", bbox=(0.1, 0.04, 0.1)),
        _link("Wheels_v8_1_3", parent="Part_1_3_6", bbox=(0.1, 0.04, 0.1)),
        _link("Wheels_v8_1_2", parent="Part_1_3_4", bbox=(0.1, 0.04, 0.1)),
        _link("Wheels_v8_3", parent="Part_1_3_1", bbox=(0.1, 0.04, 0.1)),
        _link("Wheels_v8_1_1", parent="Part_1_3_3", bbox=(0.1, 0.04, 0.1)),
    ]
    joints = [
        Joint(name="Rigid_SingleLegA_to_Plate", type=JointType.FIXED, parent="Part_1_1", child="Part_2_1_1",
              origin=Pose(xyz=(0.06275095, -0.17333635, 0.02826985), rpy=(1.57079633, 0.0, -0.0))),
        Joint(name="Rigid_SingleLegB_to_Plate", type=JointType.FIXED, parent="Part_1_1", child="Part_1_4_1",
              origin=Pose(xyz=(0.06275095, -0.37333635, 0.02826985), rpy=(1.57079633, 0.0, 0.0))),
        _continuous_joint("Revolute_BogieA_Bearing", "Part_1_1", "Part_2_1",
                           xyz=(-0.24824905, -0.07323635, 0.11553085), rpy=(-1.57079633, -0.0, -1.57079633)),
        _continuous_joint("Revolute_BogieB_Bearing", "Part_1_1", "Part_1_5_1",
                           xyz=(-0.24824905, -0.37373635, 0.11553085), rpy=(1.57079632, -0.0, 1.57079633)),
        Joint(name="Rigid_HubFrontA_to_LegA", type=JointType.FIXED, parent="Part_2_1_1", child="Part_1_3_5",
              origin=Pose(xyz=(0.06275095, -0.07323635, 0.02826985), rpy=(1.57079633, 0.0, 0.0))),
        Joint(name="Rigid_HubMidA_to_BogieA", type=JointType.FIXED, parent="Part_2_1", child="Part_1_3_2",
              origin=Pose(xyz=(-0.13724905, -0.07313635, 0.02826945), rpy=(1.57079633, -0.0, 0.0))),
        Joint(name="Rigid_HubRearA_to_BogieA", type=JointType.FIXED, parent="Part_2_1", child="Part_1_3_6",
              origin=Pose(xyz=(-0.33724905, -0.07313635, 0.02826945), rpy=(1.57079633, -0.0, 0.0))),
        Joint(name="Rigid_HubFrontB_to_LegB", type=JointType.FIXED, parent="Part_1_4_1", child="Part_1_3_4",
              origin=Pose(xyz=(0.06275095, -0.37343635, 0.02826985), rpy=(1.57079633, -0.0, -3.14159265))),
        Joint(name="Rigid_HubMidB_to_BogieB", type=JointType.FIXED, parent="Part_1_5_1", child="Part_1_3_1",
              origin=Pose(xyz=(-0.13724905, -0.37383635, 0.02826945), rpy=(1.57079633, 0.0, -3.14159265))),
        Joint(name="Rigid_HubRearB_to_BogieB", type=JointType.FIXED, parent="Part_1_5_1", child="Part_1_3_3",
              origin=Pose(xyz=(-0.33724905, -0.37383635, 0.02826945), rpy=(1.57079633, 0.0, -3.14159265))),
        _continuous_joint("Revolute_WheelFrontA", "Part_1_3_5", "Wheels_v8_2",
                           xyz=(0.06275095, -0.11563635, 0.04126985), rpy=(0.0, 0.0, 1.57079633)),
        _continuous_joint("Revolute_WheelMidA", "Part_1_3_2", "Wheels_v8_1",
                           xyz=(-0.13724905, -0.11553635, 0.04126945), rpy=(-0.0, 0.0, 1.57079633)),
        _continuous_joint("Revolute_WheelRearA", "Part_1_3_6", "Wheels_v8_1_3",
                           xyz=(-0.33724905, -0.11553635, 0.04126945), rpy=(-0.0, 0.0, 1.57079633)),
        _continuous_joint("Revolute_WheelFrontB", "Part_1_3_4", "Wheels_v8_1_2",
                           xyz=(0.06275095, -0.33103635, 0.04126985), rpy=(-0.0, 0.0, -1.57079633)),
        _continuous_joint("Revolute_WheelMidB", "Part_1_3_1", "Wheels_v8_3",
                           xyz=(-0.13724905, -0.33143635, 0.04126945), rpy=(0.0, 0.0, -1.57079633)),
        _continuous_joint("Revolute_WheelRearB", "Part_1_3_3", "Wheels_v8_1_1",
                           xyz=(-0.33724905, -0.33143635, 0.04126945), rpy=(0.0, 0.0, -1.57079633)),
    ]
    robot = Robot(name="main_assembly", links=links, joints=joints)

    assert detect_drivetrain(robot) is None


def test_uneven_side_split_returns_none_rather_than_a_wrong_guess():
    # 3 wheels geometrically clustered on one side, 1 on the other -- no
    # axis gives a clean two-way split, so this must decline rather than
    # pair up wheels that don't belong together.
    base = _link("base_link")
    bbox = (0.1, 0.04, 0.1)
    links = [base]
    joints = []
    for i, y in enumerate((0.2, 0.25, 0.3)):
        name = f"clustered_{i}"
        links.append(_link(f"wheel_link_{name}", parent="base_link", bbox=bbox))
        joints.append(_continuous_joint(f"wheel_joint_{name}", "base_link", f"wheel_link_{name}", xyz=(0.0, y, 0.0)))
    links.append(_link("wheel_link_lone", parent="base_link", bbox=bbox))
    joints.append(_continuous_joint("wheel_joint_lone", "base_link", "wheel_link_lone", xyz=(0.0, -0.2, 0.0)))
    robot = Robot(name="lopsided", links=links, joints=joints)

    assert detect_drivetrain(robot) is None


def test_fixed_wheel_named_joints_are_not_candidates():
    # A "wheel" in the name alone isn't enough -- must be a driven
    # (continuous/revolute) joint type, matching how a real wheel is
    # actually actuated.
    base = _link("base_link")
    left_wheel = _link("left_wheel_link", parent="base_link", bbox=(0.2, 0.04, 0.2))
    right_wheel = _link("right_wheel_link", parent="base_link", bbox=(0.2, 0.04, 0.2))
    fixed_joint = Joint(
        name="left_wheel_joint", type=JointType.FIXED, parent="base_link", child="left_wheel_link"
    )
    other_fixed = Joint(
        name="right_wheel_joint", type=JointType.FIXED, parent="base_link", child="right_wheel_link"
    )
    robot = Robot(name="rover", links=[base, left_wheel, right_wheel], joints=[fixed_joint, other_fixed])

    assert detect_drivetrain(robot) is None
