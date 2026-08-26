import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.app import (
    PipelineError,
    attach_collision_proxies,
    attach_mesh_references,
    check_missing_actuator_limits,
    generate_ros_package,
    run_pipeline,
)
from robot_model import Actuator, Geometry, Inertial, Joint, JointType, Link, Robot

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


# ---------------------------------------------------------------------------
# Integration of the ros2_control / Gazebo / MoveIt 2 / Nav2 generators into
# generate_ros_package's include_* flags.
# ---------------------------------------------------------------------------


def make_diff_drive_robot():
    from robot_model import Geometry

    base = Link(
        name="base_link",
        collision_geometry=Geometry(kind="box", size=(0.4, 0.3, 0.15)),
        inertial=Inertial(mass=5.0, ixx=0.1, iyy=0.1, izz=0.1),
    )
    left_wheel = Link(
        name="left_wheel",
        parent="base_link",
        collision_geometry=Geometry(kind="cylinder", radius=0.1, length=0.04),
        inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001),
    )
    right_wheel = Link(
        name="right_wheel",
        parent="base_link",
        collision_geometry=Geometry(kind="cylinder", radius=0.1, length=0.04),
        inertial=Inertial(mass=0.2, ixx=0.001, iyy=0.001, izz=0.001),
    )
    left_joint = Joint(
        name="left_wheel_joint", type=JointType.CONTINUOUS, parent="base_link", child="left_wheel", axis=(0, 1, 0)
    )
    right_joint = Joint(
        name="right_wheel_joint", type=JointType.CONTINUOUS, parent="base_link", child="right_wheel", axis=(0, 1, 0)
    )
    return Robot(
        name="app_test_rover",
        links=[base, left_wheel, right_wheel],
        joints=[left_joint, right_joint],
        metadata={
            "drivetrain": {
                "type": "differential_drive",
                "left_wheel_joint": "left_wheel_joint",
                "right_wheel_joint": "right_wheel_joint",
                "wheel_separation": 0.4,
                "wheel_radius": 0.1,
            }
        },
    )


def test_generate_ros_package_with_ros2_control_arm(tmp_path):
    robot = make_simple_robot(with_limits=True)
    robot.actuators.append(Actuator(name="joint1_motor", type="electric_motor", joint="joint1", interface="position"))

    package_dir = generate_ros_package(robot, {}, tmp_path, include_ros2_control=True)

    assert (package_dir / "config" / "controllers.yaml").exists()
    assert (package_dir / "launch" / "control.launch.py").exists()
    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    assert "<ros2_control" in urdf_text
    assert "</robot>" in urdf_text.strip().splitlines()[-1] or urdf_text.rstrip().endswith("</robot>")


def test_generate_ros_package_with_gazebo(tmp_path):
    robot = make_simple_robot(with_limits=True)

    package_dir = generate_ros_package(robot, {}, tmp_path, include_gazebo=True)

    assert (package_dir / "worlds" / "empty.sdf").exists()
    assert (package_dir / "launch" / "gazebo.launch.py").exists()
    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    assert "<gazebo" in urdf_text
    assert "gazebo_fragment" not in urdf_text  # wrapper must be unwrapped, not leaked into the URDF


def test_generate_ros_package_with_ros2_control_and_gazebo_both_splice(tmp_path):
    robot = make_simple_robot(with_limits=True)
    robot.actuators.append(Actuator(name="joint1_motor", type="electric_motor", joint="joint1", interface="position"))

    package_dir = generate_ros_package(robot, {}, tmp_path, include_ros2_control=True, include_gazebo=True)

    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    assert "<ros2_control" in urdf_text
    assert "<gazebo" in urdf_text
    import xml.etree.ElementTree as ET

    ET.fromstring(urdf_text)  # must still be well-formed XML with both fragments spliced in


def test_generate_ros_package_with_moveit_suitable(tmp_path):
    robot = make_simple_robot(with_limits=True)

    package_dir = generate_ros_package(robot, {}, tmp_path, include_moveit=True, moveit_group_name="arm")

    assert (package_dir / "config" / f"{robot.name}.srdf").exists()
    assert (package_dir / "config" / "joint_limits.yaml").exists()
    assert (package_dir / "config" / "kinematics.yaml").exists()
    assert (package_dir / "config" / "moveit_controllers.yaml").exists()
    # Regression: move_group won't start without this (see
    # generate_ompl_planning_yaml's docstring) -- confirmed for real.
    assert (package_dir / "config" / "ompl_planning.yaml").exists()
    launch_text = (package_dir / "launch" / "moveit_demo.launch.py").read_text()
    assert f'MOVEIT_CONFIG_PACKAGE = "{robot.name}"' in launch_text
    assert (package_dir / "launch" / "moveit_demo.launch.py").exists()


def test_generate_ros_package_with_moveit_unsuitable_raises(tmp_path):
    robot = make_diff_drive_robot()  # a drivetrain robot is not MoveIt-suitable
    with pytest.raises(PipelineError):
        generate_ros_package(robot, {}, tmp_path, include_moveit=True)


def test_generate_ros_package_with_nav2_suitable(tmp_path):
    robot = make_diff_drive_robot()

    package_dir = generate_ros_package(robot, {}, tmp_path, include_nav2=True)

    assert (package_dir / "config" / "nav2_params.yaml").exists()
    assert (package_dir / "launch" / "nav2_bringup.launch.py").exists()
    assert (package_dir / "config" / "map.yaml").exists()


def test_generate_ros_package_with_nav2_unsuitable_raises(tmp_path):
    robot = make_simple_robot(with_limits=True)  # an arm has no drivetrain metadata
    with pytest.raises(PipelineError):
        generate_ros_package(robot, {}, tmp_path, include_nav2=True)


def test_generate_ros_package_all_four_together_on_diff_drive(tmp_path):
    # A mobile base with wheel actuators requesting every optional output at
    # once (ros2_control + gazebo + nav2; NOT moveit, since a drivetrain
    # robot is correctly refused by detect_moveit_suitability).
    robot = make_diff_drive_robot()
    robot.actuators.append(
        Actuator(name="left_wheel_motor", type="electric_motor", joint="left_wheel_joint", interface="velocity")
    )
    robot.actuators.append(
        Actuator(name="right_wheel_motor", type="electric_motor", joint="right_wheel_joint", interface="velocity")
    )

    package_dir = generate_ros_package(
        robot, {}, tmp_path, include_ros2_control=True, include_gazebo=True, include_nav2=True
    )

    urdf_text = (package_dir / "urdf" / f"{robot.name}.urdf.xacro").read_text()
    import xml.etree.ElementTree as ET

    ET.fromstring(urdf_text)
    assert (package_dir / "config" / "controllers.yaml").exists()
    assert (package_dir / "worlds" / "empty.sdf").exists()
    assert (package_dir / "config" / "nav2_params.yaml").exists()
# attach_collision_proxies
# ---------------------------------------------------------------------------


def make_robot_with_bounding_box():
    base = Link(
        name="base_link",
        inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01),
        metadata={"bounding_box_size": (0.2, 0.3, 0.1)},
    )
    # "arm" deliberately has NO bounding_box_size -- e.g. a hand-authored
    # link, or a Fusion occurrence Fusion reported no bounding box for.
    arm = Link(name="arm", parent="base_link", inertial=Inertial(mass=0.5, ixx=0.001, iyy=0.001, izz=0.001))
    joint = Joint(
        name="joint1",
        type=JointType.REVOLUTE,
        parent="base_link",
        child="arm",
        axis=(0.0, 0.0, 1.0),
        lower_limit=-1.0,
        upper_limit=1.0,
        velocity_limit=2.0,
        effort_limit=5.0,
    )
    return Robot(name="bbox_test_robot", links=[base, arm], joints=[joint])


def test_attach_collision_proxies_disabled_by_default_leaves_geometry_untouched():
    robot = make_robot_with_bounding_box()
    mesh = Geometry(kind="mesh", mesh_path="package://bbox_test_robot/meshes/base_link.stl")
    robot.link("base_link").visual_geometry = mesh
    robot.link("base_link").collision_geometry = mesh

    result = attach_collision_proxies(robot)  # use_bounding_box_collision defaults to False
    assert result is robot
    base = robot.link("base_link")
    assert base.collision_geometry is mesh
    assert base.visual_geometry is mesh


def test_attach_collision_proxies_replaces_collision_only_for_links_with_bbox():
    robot = make_robot_with_bounding_box()
    mesh = Geometry(kind="mesh", mesh_path="package://bbox_test_robot/meshes/base_link.stl")
    robot.link("base_link").visual_geometry = mesh
    robot.link("base_link").collision_geometry = mesh
    arm_mesh = Geometry(kind="mesh", mesh_path="package://bbox_test_robot/meshes/arm.stl")
    robot.link("arm").visual_geometry = arm_mesh
    robot.link("arm").collision_geometry = arm_mesh

    attach_collision_proxies(robot, use_bounding_box_collision=True)

    base = robot.link("base_link")
    assert base.collision_geometry.kind == "box"
    assert base.collision_geometry.size == pytest.approx((0.2, 0.3, 0.1))
    assert base.visual_geometry is mesh  # visual untouched

    # "arm" has no bounding_box_size metadata -> left completely alone.
    arm = robot.link("arm")
    assert arm.collision_geometry is arm_mesh
    assert arm.visual_geometry is arm_mesh


def test_attach_collision_proxies_no_metadata_at_all_is_a_noop():
    # A hand-authored Robot (e.g. examples/sample_arm.py) has no
    # bounding_box_size metadata on any link -- must be left untouched even
    # with use_bounding_box_collision=True.
    robot = make_simple_robot()
    mesh = Geometry(kind="mesh", mesh_path="package://app_test_robot/meshes/base_link.stl")
    robot.link("base_link").collision_geometry = mesh

    attach_collision_proxies(robot, use_bounding_box_collision=True)
    assert robot.link("base_link").collision_geometry is mesh


def test_generate_ros_package_with_bounding_box_collision_opt_in(tmp_path):
    robot = make_robot_with_bounding_box()
    package_dir = generate_ros_package(robot, {}, tmp_path, use_bounding_box_collision=True)
    assert (package_dir / "urdf" / "bbox_test_robot.urdf.xacro").exists()
    base = robot.link("base_link")
    assert base.collision_geometry.kind == "box"
    assert base.collision_geometry.size == pytest.approx((0.2, 0.3, 0.1))
    # "arm" had no bounding box metadata -> left with no collision geometry
    # at all (attach_mesh_references never set one either, since no mesh
    # was supplied for it in this test).
    assert robot.link("arm").collision_geometry is None


# ---------------------------------------------------------------------------
# Backward compatibility: leaving the new parameter unset must be byte-for-
# byte identical to this change never having happened.
# ---------------------------------------------------------------------------


def _package_file_bytes(package_dir: Path) -> dict:
    return {
        str(p.relative_to(package_dir)): p.read_bytes()
        for p in sorted(package_dir.rglob("*"))
        if p.is_file()
    }


def test_generate_ros_package_default_output_unchanged_with_bbox_metadata(tmp_path):
    # Same robot, same mesh_files, one run with the new parameter explicitly
    # False/unset and one with it explicitly True -- the False/unset run must
    # be identical to pre-change behavior (full mesh collision), proving the
    # opt-in default doesn't alter existing output even when bounding-box
    # metadata IS present and available to use.
    robot_default = make_robot_with_bounding_box()
    fake_mesh = tmp_path / "base_link.stl"
    fake_mesh.write_bytes(b"fake stl content")

    out_default = tmp_path / "default"
    out_default.mkdir()
    package_default = generate_ros_package(robot_default, {"base_link": fake_mesh}, out_default)
    assert robot_default.link("base_link").collision_geometry.kind == "mesh"

    robot_explicit_false = make_robot_with_bounding_box()
    out_explicit = tmp_path / "explicit_false"
    out_explicit.mkdir()
    package_explicit = generate_ros_package(
        robot_explicit_false, {"base_link": fake_mesh}, out_explicit, use_bounding_box_collision=False
    )

    assert _package_file_bytes(package_default) == _package_file_bytes(package_explicit)


def test_run_pipeline_default_matches_pre_change_behavior(tmp_path):
    base = FusionOccurrence(
        name="base_link:1",
        pose=FusionPose(),
        inertia=FusionInertia(mass=1.0, center_of_mass=(0, 0, 0), ixx=100, iyy=100, izz=100, ixy=0, ixz=0, iyz=0),
        bounding_box=((0.0, 0.0, 0.0), (10.0, 10.0, 10.0)),
    )
    arm = FusionOccurrence(
        name="arm:1",
        pose=FusionPose(xyz=(5.0, 0.0, 0.0)),
        inertia=FusionInertia(mass=1.0, center_of_mass=(5, 0, 0), ixx=100, iyy=135, izz=135, ixy=0, ixz=0, iyz=0),
        bounding_box=((0.0, 0.0, 0.0), (4.0, 4.0, 4.0)),
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
        velocity_limit=2.0,
        effort_limit=10.0,
    )

    from fusion_addin.extraction.converter import build_robot_model

    reader = FakeFusionDesignReader([base, arm], [joint])
    robot_default = build_robot_model(reader, "pipeline_robot")
    out_default = tmp_path / "default"
    out_default.mkdir()
    _, package_default = run_pipeline(reader, "pipeline_robot", out_default)

    reader2 = FakeFusionDesignReader([base, arm], [joint])
    out_explicit = tmp_path / "explicit"
    out_explicit.mkdir()
    _, package_explicit = run_pipeline(
        reader2, "pipeline_robot", out_explicit, use_bounding_box_collision=False
    )

    assert _package_file_bytes(package_default) == _package_file_bytes(package_explicit)


