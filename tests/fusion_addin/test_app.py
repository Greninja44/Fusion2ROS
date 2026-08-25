import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.app import (
    PipelineError,
    attach_mesh_references,
    check_missing_actuator_limits,
    generate_ros_package,
    run_pipeline,
)
from robot_model import Inertial, Joint, JointType, Link, Robot

from tests.fusion_addin.test_extraction import FakeFusionDesignReader
from fusion_addin.extraction.interface import FusionInertia, FusionJointInfo, FusionOccurrence, FusionPose


def make_simple_robot(with_limits=True):
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    arm = Link(name="arm", parent="base_link", inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001))
    joint = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="arm",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        velocity_limit=2.0 if with_limits else None,
        effort_limit=5.0 if with_limits else None,
    )
    return Robot(name="app_test_robot", links=[base, arm], joints=[joint])


def test_check_missing_actuator_limits_clean_robot():
    robot = make_simple_robot(with_limits=True)
    assert check_missing_actuator_limits(robot) == []


def test_check_missing_actuator_limits_reports_joint():
    robot = make_simple_robot(with_limits=False)
    problems = check_missing_actuator_limits(robot)
    assert len(problems) == 1
    assert "joint1" in problems[0]
    assert "velocity_limit" in problems[0]
    assert "effort_limit" in problems[0]


def test_attach_mesh_references(tmp_path):
    robot = make_simple_robot()
    fake_mesh = tmp_path / "base_link.stl"
    fake_mesh.write_bytes(b"fake stl")
    attach_mesh_references(robot, {"base_link": fake_mesh})

    base = robot.link("base_link")
    assert base.visual_geometry.kind == "mesh"
    assert base.visual_geometry.mesh_path == "package://app_test_robot/meshes/base_link.stl"
    assert base.collision_geometry.mesh_path == base.visual_geometry.mesh_path

    arm = robot.link("arm")
    assert arm.visual_geometry is None  # not in mesh_files -> left alone


def test_generate_ros_package_raises_pipeline_error_for_missing_limits(tmp_path):
    robot = make_simple_robot(with_limits=False)
    with pytest.raises(PipelineError, match="joint1"):
        generate_ros_package(robot, {}, tmp_path)


def test_generate_ros_package_succeeds(tmp_path):
    robot = make_simple_robot(with_limits=True)
    package_dir = generate_ros_package(robot, {}, tmp_path)
    assert package_dir == tmp_path / "app_test_robot"
    assert (package_dir / "package.xml").exists()
    assert (package_dir / "urdf" / "app_test_robot.urdf.xacro").exists()


def test_generate_ros_package_with_meshes_lands_under_meshes_dir(tmp_path):
    # Exercises the mesh_files re-keying boundary in generate_ros_package:
    # export_link_meshes-shaped input (keyed by LINK NAME) must end up
    # correctly placed under the package's meshes/ dir (keyed by FILENAME),
    # and the URDF's mesh reference must point at that same filename.
    robot = make_simple_robot(with_limits=True)
    fake_mesh = tmp_path / "exported" / "base_link.stl"
    fake_mesh.parent.mkdir()
    fake_mesh.write_bytes(b"fake stl content")

    package_dir = generate_ros_package(robot, {"base_link": fake_mesh}, tmp_path)

    copied = package_dir / "meshes" / "base_link.stl"
    assert copied.exists()
    assert copied.read_bytes() == b"fake stl content"

    urdf_text = (package_dir / "urdf" / "app_test_robot.urdf.xacro").read_text()
    assert "package://app_test_robot/meshes/base_link.stl" in urdf_text


def test_run_pipeline_end_to_end(tmp_path):
    base = FusionOccurrence(
        name="base_link:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=100, iyy=100, izz=100, ixy=0, ixz=0, iyz=0),
    )
    arm = FusionOccurrence(
        name="arm:1",
        pose=FusionPose(xyz=(5.0, 0.0, 0.0)),
        inertia=FusionInertia(mass=1.0, center_of_mass=(5, 0, 0), ixx=100, iyy=135, izz=135, ixy=0, ixz=0, iyz=0),
    )
    joint = FusionJointInfo(
        name="joint1",
        joint_type="RevoluteJointType",
        occurrence_one="base_link:1",
        occurrence_two="arm:1",
        origin=FusionPose(xyz=(5.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
    )
    reader = FakeFusionDesignReader([base, arm], [joint])

    # Extraction alone (no motor limits from Fusion) should raise a clear
    # PipelineError, not a bare ValueError -- this is the real-world shape
    # every Fusion-sourced robot hits until actuators are assigned.
    with pytest.raises(PipelineError):
        run_pipeline(reader, "e2e_robot", tmp_path)

    # Once a caller sets the limits an extractor can't provide, the full
    # pipeline runs through to a real package on disk.
    robot, package_dir = None, None
    from fusion_addin.extraction.converter import build_robot_model

    robot = build_robot_model(reader, "e2e_robot")
    robot.joint("joint1").velocity_limit = 2.0
    robot.joint("joint1").effort_limit = 10.0
    package_dir = generate_ros_package(robot, {}, tmp_path)
    assert (package_dir / "urdf" / "e2e_robot.urdf.xacro").exists()
