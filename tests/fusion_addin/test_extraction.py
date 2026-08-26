"""Tests for fusion_addin.extraction. Must run with plain `python3 -m pytest`
-- no Fusion, no adsk, no ROS, no network. That's the whole point: interface.py
and converter.py are Fusion-symbol-free, so they're testable with a
hand-written fake FusionDesignReader instead of a live Fusion 360 process.
"""

import math
import sys
from pathlib import Path
from typing import List

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.extraction.converter import (
    ExtractionError,
    UnsupportedJointTypeError,
    build_robot_model,
    matrix_from_basis_vectors,
    matrix_to_rpy,
    rpy_to_matrix,
    sanitize_link_name,
)
from fusion_addin.extraction.interface import (
    FusionDesignReader,
    FusionInertia,
    FusionJointInfo,
    FusionOccurrence,
    FusionPose,
)
from robot_model import JointType


# ---------------------------------------------------------------------------
# Fake FusionDesignReader
# ---------------------------------------------------------------------------


class FakeFusionDesignReader(FusionDesignReader):
    """Hand-built fake -- just stores and returns whatever occurrences/joints
    it's given. No Fusion dependency at all."""

    def __init__(self, occurrences: List[FusionOccurrence], joints: List[FusionJointInfo]):
        self._occurrences = list(occurrences)
        self._joints = list(joints)

    def list_occurrences(self) -> List[FusionOccurrence]:
        return list(self._occurrences)

    def list_joints(self) -> List[FusionJointInfo]:
        return list(self._joints)


# ---------------------------------------------------------------------------
# Main fixture: a 3-link arm, base_link -[revolute]-> link1 -[fixed]-> link2.
# ---------------------------------------------------------------------------


def make_three_link_arm_reader() -> FakeFusionDesignReader:
    base = FusionOccurrence(
        name="base_link:1",
        pose=FusionPose(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        inertia=FusionInertia(
            mass=2.0,
            center_of_mass=(0.0, 0.0, 0.0),
            ixx=100.0, iyy=200.0, izz=300.0, ixy=10.0, ixz=20.0, iyz=30.0,
        ),
        body_names=["base_body"],
    )
    link1 = FusionOccurrence(
        name="link1:1",
        pose=FusionPose(xyz=(10.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        inertia=FusionInertia(
            # About WORLD ORIGIN, per FusionInertia's documented convention.
            # COM is offset (10, 0, 0) cm from the origin, so the parallel
            # axis theorem requires iyy/izz here to exceed mass*dx^2 = 100
            # for the about-COM tensor to come out non-negative: 160-100=60,
            # 170-100=70 (matches test_unit_conversion_link_inertial's
            # expected 0.006/0.007 m^2). ixx is unaffected since the offset
            # is purely along x.
            mass=1.0, center_of_mass=(10.0, 0.0, 0.0),
            ixx=50.0, iyy=160.0, izz=170.0, ixy=0.0, ixz=0.0, iyz=0.0,
        ),
        body_names=["link1_body"],
    )
    link2 = FusionOccurrence(
        name="link2:1",
        pose=FusionPose(xyz=(20.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        inertia=FusionInertia(
            # Same reasoning: COM offset (20, 0, 0) cm, mass 0.5 kg ->
            # mass*dx^2 = 200, so iyy/izz about-origin must exceed 200 for a
            # physically valid (non-negative) about-COM result (210-200=10).
            mass=0.5, center_of_mass=(20.0, 0.0, 0.0),
            ixx=10.0, iyy=210.0, izz=210.0, ixy=0.0, ixz=0.0, iyz=0.0,
        ),
        body_names=["link2_body"],
    )

    joint1 = FusionJointInfo(
        name="joint1",
        joint_type="RevoluteJointType",
        occurrence_one="base_link:1",
        occurrence_two="link1:1",
        origin=FusionPose(xyz=(10.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        velocity_limit=2.0,
        effort_limit=5.0,
    )
    joint2 = FusionJointInfo(
        name="joint2",
        joint_type="RigidJointType",
        occurrence_one="link1:1",
        occurrence_two="link2:1",
        origin=FusionPose(xyz=(10.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
    )

    return FakeFusionDesignReader([base, link1, link2], [joint1, joint2])


def test_link_count_and_names():
    robot = build_robot_model(make_three_link_arm_reader(), "test_arm")
    assert len(robot.links) == 3
    names = {l.name for l in robot.links}
    assert names == {"base_link_1", "link1_1", "link2_1"}


def test_parent_child_structure():
    robot = build_robot_model(make_three_link_arm_reader(), "test_arm")
    base = robot.link("base_link_1")
    link1 = robot.link("link1_1")
    link2 = robot.link("link2_1")
    assert base.parent is None
    assert link1.parent == "base_link_1"
    assert link2.parent == "link1_1"
    assert robot.root_link().name == "base_link_1"


def test_joint_names_and_endpoints():
    robot = build_robot_model(make_three_link_arm_reader(), "test_arm")
    joint1 = robot.joint("joint1")
    joint2 = robot.joint("joint2")
    assert joint1.parent == "base_link_1" and joint1.child == "link1_1"
    assert joint2.parent == "link1_1" and joint2.child == "link2_1"


def test_joint_type_mapping_revolute_and_fixed():
    robot = build_robot_model(make_three_link_arm_reader(), "test_arm")
    assert robot.joint("joint1").type == JointType.REVOLUTE
    assert robot.joint("joint2").type == JointType.FIXED
    assert robot.joint("joint2").axis is None
    assert robot.joint("joint2").lower_limit is None


def test_unit_conversion_link_inertial():
    robot = build_robot_model(make_three_link_arm_reader(), "test_arm")
    base = robot.link("base_link_1")
    assert base.inertial.mass == pytest.approx(2.0)
    assert base.inertial.center_of_mass == pytest.approx((0.0, 0.0, 0.0))
    # kg*cm^2 -> kg*m^2 is a /10000 factor: 100 cm^2 -> 0.01 m^2, etc.
    assert base.inertial.ixx == pytest.approx(0.01)
    assert base.inertial.iyy == pytest.approx(0.02)
    assert base.inertial.izz == pytest.approx(0.03)
    assert base.inertial.ixy == pytest.approx(0.001)
    assert base.inertial.ixz == pytest.approx(0.002)
    assert base.inertial.iyz == pytest.approx(0.003)

    link1 = robot.link("link1_1")
    assert link1.inertial.mass == pytest.approx(1.0)
    assert link1.inertial.center_of_mass == pytest.approx((0.0, 0.0, 0.0))
    assert link1.inertial.ixx == pytest.approx(0.005)
    assert link1.inertial.iyy == pytest.approx(0.006)
    assert link1.inertial.izz == pytest.approx(0.007)


def test_unit_conversion_joint_origin_and_limits():
    robot = build_robot_model(make_three_link_arm_reader(), "test_arm")
    joint1 = robot.joint("joint1")
    # 10 cm -> 0.1 m
    assert joint1.origin.xyz == pytest.approx((0.1, 0.0, 0.0))
    assert joint1.origin.rpy == pytest.approx((0.0, 0.0, 0.0))
    assert joint1.axis == pytest.approx((0.0, 0.0, 1.0))
    # revolute limits are already radians -- no conversion
    assert joint1.lower_limit == pytest.approx(-1.0)
    assert joint1.upper_limit == pytest.approx(1.0)
    assert joint1.velocity_limit == pytest.approx(2.0)
    assert joint1.effort_limit == pytest.approx(5.0)

    joint2 = robot.joint("joint2")
    assert joint2.origin.xyz == pytest.approx((0.1, 0.0, 0.0))


def test_robot_validates_cleanly():
    robot = build_robot_model(make_three_link_arm_reader(), "test_arm")
    problems = robot.validate(raise_on_error=False)
    assert problems == []


# ---------------------------------------------------------------------------
# Continuous (unlimited revolute) and prismatic joint mapping
# ---------------------------------------------------------------------------


def make_two_link_reader(joint_info: FusionJointInfo) -> FakeFusionDesignReader:
    base = FusionOccurrence(
        name="base:1",
        pose=FusionPose(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0.0, 0.0, 0.0),
                               ixx=10.0, iyy=10.0, izz=10.0, ixy=0.0, ixz=0.0, iyz=0.0),
    )
    top = FusionOccurrence(
        name="top:1",
        pose=FusionPose(xyz=(5.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
        # About WORLD ORIGIN (see FusionInertia docs). COM offset (5,0,0) cm,
        # mass 1 kg -> mass*dx^2 = 25, so iyy/izz about-origin must exceed 25
        # for a physically valid about-COM tensor (35-25=10, matching ixx=10
        # so the about-COM tensor stays isotropic, same as before this fix).
        inertia=FusionInertia(mass=1.0, center_of_mass=(5.0, 0.0, 0.0),
                               ixx=10.0, iyy=35.0, izz=35.0, ixy=0.0, ixz=0.0, iyz=0.0),
    )
    return FakeFusionDesignReader([base, top], [joint_info])


def test_unlimited_revolute_maps_to_continuous():
    joint_info = FusionJointInfo(
        name="cont_joint",
        joint_type="RevoluteJointType",
        occurrence_one="base:1",
        occurrence_two="top:1",
        origin=FusionPose(xyz=(5.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=None,
        upper_limit=None,
        velocity_limit=3.0,
    )
    robot = build_robot_model(make_two_link_reader(joint_info), "cont_robot")
    joint = robot.joint("cont_joint")
    assert joint.type == JointType.CONTINUOUS
    assert joint.lower_limit is None
    assert joint.upper_limit is None
    assert joint.axis == pytest.approx((0.0, 0.0, 1.0))
    assert joint.velocity_limit == pytest.approx(3.0)
    robot.validate()  # must not raise


def test_slider_maps_to_prismatic_with_unit_conversion():
    joint_info = FusionJointInfo(
        name="slide_joint",
        joint_type="SliderJointType",
        occurrence_one="base:1",
        occurrence_two="top:1",
        origin=FusionPose(xyz=(5.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        lower_limit=-3.0,
        upper_limit=8.0,
        velocity_limit=15.0,
        effort_limit=100.0,
    )
    robot = build_robot_model(make_two_link_reader(joint_info), "slider_robot")
    joint = robot.joint("slide_joint")
    assert joint.type == JointType.PRISMATIC
    assert joint.lower_limit == pytest.approx(-0.03)
    assert joint.upper_limit == pytest.approx(0.08)
    assert joint.velocity_limit == pytest.approx(0.15)
    assert joint.effort_limit == pytest.approx(100.0)  # not a length quantity -> unconverted
    robot.validate()


def test_revolute_with_only_one_limit_enabled_raises():
    joint_info = FusionJointInfo(
        name="bad_joint",
        joint_type="RevoluteJointType",
        occurrence_one="base:1",
        occurrence_two="top:1",
        origin=FusionPose(),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=None,
    )
    with pytest.raises(ExtractionError):
        build_robot_model(make_two_link_reader(joint_info), "bad_robot")


def test_prismatic_without_limits_raises():
    joint_info = FusionJointInfo(
        name="unbounded_slide",
        joint_type="SliderJointType",
        occurrence_one="base:1",
        occurrence_two="top:1",
        origin=FusionPose(),
        axis=(1.0, 0.0, 0.0),
    )
    with pytest.raises(ExtractionError):
        build_robot_model(make_two_link_reader(joint_info), "bad_robot")


# ---------------------------------------------------------------------------
# Error paths: unsupported / unrecognized joint types
# ---------------------------------------------------------------------------


def test_known_unsupported_joint_type_raises_specific_error():
    joint_info = FusionJointInfo(
        name="ball_joint",
        joint_type="BallJointType",
        occurrence_one="base:1",
        occurrence_two="top:1",
        origin=FusionPose(),
    )
    with pytest.raises(UnsupportedJointTypeError, match="BallJointType"):
        build_robot_model(make_two_link_reader(joint_info), "bad_robot")


def test_unrecognized_joint_type_string_raises_not_silently_dropped():
    joint_info = FusionJointInfo(
        name="mystery_joint",
        joint_type="SomeFutureJointTypeNobodyHasSeenYet",
        occurrence_one="base:1",
        occurrence_two="top:1",
        origin=FusionPose(),
    )
    with pytest.raises(UnsupportedJointTypeError):
        build_robot_model(make_two_link_reader(joint_info), "bad_robot")


# ---------------------------------------------------------------------------
# Structural error paths
# ---------------------------------------------------------------------------


def test_duplicate_occurrence_name_raises():
    occ = FusionOccurrence(
        name="dup:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=1, iyy=1, izz=1, ixy=0, ixz=0, iyz=0),
    )
    reader = FakeFusionDesignReader([occ, occ], [])
    with pytest.raises(ExtractionError):
        build_robot_model(reader, "bad_robot")


def test_multiple_disconnected_roots_raises():
    a = FusionOccurrence(
        name="a:1", pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=1, iyy=1, izz=1, ixy=0, ixz=0, iyz=0),
    )
    b = FusionOccurrence(
        name="b:1", pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=1, iyy=1, izz=1, ixy=0, ixz=0, iyz=0),
    )
    reader = FakeFusionDesignReader([a, b], [])  # no joint connects them -> 2 roots
    with pytest.raises(ExtractionError, match="unconnected root"):
        build_robot_model(reader, "bad_robot")


def test_child_of_two_joints_raises():
    reader = make_three_link_arm_reader()
    # Add a second joint that also claims link2:1 as a child.
    extra = FusionJointInfo(
        name="joint3",
        joint_type="RigidJointType",
        occurrence_one="base_link:1",
        occurrence_two="link2:1",
        origin=FusionPose(),
    )
    reader._joints.append(extra)
    with pytest.raises(ExtractionError, match="multiple joints"):
        build_robot_model(reader, "bad_robot")


def test_joint_referencing_unknown_occurrence_raises():
    reader = make_three_link_arm_reader()
    bogus = FusionJointInfo(
        name="joint4",
        joint_type="RigidJointType",
        occurrence_one="link2:1",
        occurrence_two="does_not_exist:1",
        origin=FusionPose(),
    )
    reader._joints.append(bogus)
    with pytest.raises(ExtractionError):
        build_robot_model(reader, "bad_robot")


# ---------------------------------------------------------------------------
# sanitize_link_name
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("base_link:1", "base_link_1"),
        ("Base Link:1 (v2)", "Base_Link_1_v2"),
        ("simple_name", "simple_name"),
        ("a::b", "a_b"),
    ],
)
def test_sanitize_link_name(raw, expected):
    assert sanitize_link_name(raw) == expected


# ---------------------------------------------------------------------------
# Rotation + parallel-axis-theorem inertia math (hand-verified case).
#
# Occurrence at world origin, rotated 90 deg (yaw) about Z. Local X axis
# then points along world +Y. mass=2 kg, center of mass at world (0, 2, 0) cm
# (i.e. 2 cm along the occurrence's local +X, since world +Y == local +X
# here). The "raw" about-world-origin tensor below was constructed by
# applying the parallel axis theorem forward (by hand) to a chosen
# about-COM, world-aligned tensor diag(40, 50, 60) kg*cm^2, so converter.py
# must recover that exact tensor via the inverse operation, then rotate it
# into the local frame, where the 90 deg yaw swaps the x/y diagonal entries:
# local ixx becomes world iyy (50), local iyy becomes world ixx (40).
# ---------------------------------------------------------------------------


def test_inertia_rotation_and_parallel_axis_theorem():
    occ = FusionOccurrence(
        name="rot_link:1",
        pose=FusionPose(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, math.pi / 2)),
        inertia=FusionInertia(
            mass=2.0,
            center_of_mass=(0.0, 2.0, 0.0),
            ixx=48.0, iyy=50.0, izz=68.0, ixy=0.0, ixz=0.0, iyz=0.0,
        ),
    )
    reader = FakeFusionDesignReader([occ], [])
    robot = build_robot_model(reader, "rot_robot")
    link = robot.link("rot_link_1")

    assert link.inertial.center_of_mass == pytest.approx((0.02, 0.0, 0.0), abs=1e-9)
    assert link.inertial.ixx == pytest.approx(50.0 / 10000.0, abs=1e-9)
    assert link.inertial.iyy == pytest.approx(40.0 / 10000.0, abs=1e-9)
    assert link.inertial.izz == pytest.approx(60.0 / 10000.0, abs=1e-9)
    assert link.inertial.ixy == pytest.approx(0.0, abs=1e-9)
    assert link.inertial.ixz == pytest.approx(0.0, abs=1e-9)
    assert link.inertial.iyz == pytest.approx(0.0, abs=1e-9)
    robot.validate()


# ---------------------------------------------------------------------------
# Pure rotation-matrix helpers
# ---------------------------------------------------------------------------


def test_rpy_matrix_round_trip():
    for rpy in [(0.0, 0.0, 0.0), (0.3, -0.2, 1.1), (1.5, 0.4, -2.3), (0.0, 0.0, math.pi / 2)]:
        m = rpy_to_matrix(rpy)
        back = matrix_to_rpy(m)
        m2 = rpy_to_matrix(back)
        for r in range(3):
            for c in range(3):
                assert m[r][c] == pytest.approx(m2[r][c], abs=1e-9)


def test_matrix_from_basis_vectors_matches_rpy_to_matrix_for_identity():
    identity = matrix_from_basis_vectors((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    assert identity == rpy_to_matrix((0.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# bounding_box -> Link.metadata["bounding_box_size"]
# ---------------------------------------------------------------------------


def test_bounding_box_converted_to_bounding_box_size_in_meters():
    # 10cm x 20cm x 5cm box (min at origin) -> 0.1 x 0.2 x 0.05 m, easy to
    # hand-verify: CM_TO_M is a straight /100 per axis on (max - min).
    occ = FusionOccurrence(
        name="boxy:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=1, iyy=1, izz=1, ixy=0, ixz=0, iyz=0),
        bounding_box=((0.0, 0.0, 0.0), (10.0, 20.0, 5.0)),
    )
    reader = FakeFusionDesignReader([occ], [])
    robot = build_robot_model(reader, "bbox_robot")
    link = robot.link("boxy_1")
    assert link.metadata["bounding_box_size"] == pytest.approx((0.1, 0.2, 0.05))


def test_bounding_box_offset_min_corner_still_gives_correct_extents():
    # min/max not at the origin -- only the DIFFERENCE (extents) matters.
    occ = FusionOccurrence(
        name="boxy2:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=1, iyy=1, izz=1, ixy=0, ixz=0, iyz=0),
        bounding_box=((3.0, -2.0, 1.0), (13.0, 18.0, 6.0)),
    )
    reader = FakeFusionDesignReader([occ], [])
    robot = build_robot_model(reader, "bbox_robot2")
    link = robot.link("boxy2_1")
    assert link.metadata["bounding_box_size"] == pytest.approx((0.1, 0.2, 0.05))


def test_missing_bounding_box_leaves_metadata_key_absent():
    # Default bounding_box=None (the fixture reader never sets it) must NOT
    # produce a "bounding_box_size" key at all -- absent, not None, so
    # downstream code can use plain .get()/"in" checks.
    robot = build_robot_model(make_three_link_arm_reader(), "test_arm")
    base = robot.link("base_link_1")
    assert "bounding_box_size" not in base.metadata
