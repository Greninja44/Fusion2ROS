"""Tests for fusion_addin.generators.nav2.

Must run with plain `python3 -m pytest` -- no Fusion, no Nav2 install, no
live ROS needed. `import yaml` (PyYAML) is used only to *verify* the
generated text parses as real YAML; the generator module itself never
imports it (see fusion_addin/generators/nav2.py's module docstring).
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from robot_model import Geometry, Inertial, Joint, JointType, Link, Pose, Robot, ValidationError

from fusion_addin.generators.nav2 import (
    CONTROLLER_PLUGIN,
    MECANUM_CONTROLLER_PLUGIN,
    PLANNER_PLUGIN,
    compute_footprint_radius,
    detect_nav2_suitability,
    generate_map_yaml_stub,
    generate_nav2_bringup_launch,
    generate_nav2_params_yaml,
)

try:
    import yaml

    HAVE_YAML = True
except ImportError:  # pragma: no cover - environment dependent
    HAVE_YAML = False

requires_yaml = pytest.mark.skipif(not HAVE_YAML, reason="PyYAML not installed in this environment")


# --- fixture: a simple differential-drive rover -----------------------------

WHEEL_SEPARATION = 0.35
WHEEL_RADIUS = 0.05


def build_sample_rover(drivetrain_overrides: dict = None) -> Robot:
    """A small hand-authored differential-drive rover, similar in spirit to
    examples/sample_arm.py: a boxy base_link, two continuous-jointed drive
    wheels (cylinders), and a fixed caster (sphere) -- all box/cylinder/
    sphere primitives so footprint-radius computation is exercised without
    any mesh geometry."""
    base_link = Link(
        name="base_link",
        visual_geometry=Geometry(kind="box", size=(0.4, 0.3, 0.15)),
        collision_geometry=Geometry(kind="box", size=(0.4, 0.3, 0.15)),
        inertial=Inertial(mass=5.0, ixx=0.05, iyy=0.08, izz=0.08),
    )
    left_wheel = Link(
        name="left_wheel",
        parent="base_link",
        visual_geometry=Geometry(kind="cylinder", radius=WHEEL_RADIUS, length=0.03),
        collision_geometry=Geometry(kind="cylinder", radius=WHEEL_RADIUS, length=0.03),
        inertial=Inertial(mass=0.3, ixx=0.0003, iyy=0.0003, izz=0.0005),
    )
    right_wheel = Link(
        name="right_wheel",
        parent="base_link",
        visual_geometry=Geometry(kind="cylinder", radius=WHEEL_RADIUS, length=0.03),
        collision_geometry=Geometry(kind="cylinder", radius=WHEEL_RADIUS, length=0.03),
        inertial=Inertial(mass=0.3, ixx=0.0003, iyy=0.0003, izz=0.0005),
    )
    caster_wheel = Link(
        name="caster_wheel",
        parent="base_link",
        visual_geometry=Geometry(kind="sphere", radius=0.03),
        collision_geometry=Geometry(kind="sphere", radius=0.03),
        inertial=Inertial(mass=0.1, ixx=0.00001, iyy=0.00001, izz=0.00001),
    )

    left_wheel_joint = Joint(
        name="left_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="left_wheel",
        origin=Pose(xyz=(0.0, WHEEL_SEPARATION / 2.0, -0.05)),
        axis=(0.0, 1.0, 0.0),
    )
    right_wheel_joint = Joint(
        name="right_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="right_wheel",
        origin=Pose(xyz=(0.0, -WHEEL_SEPARATION / 2.0, -0.05)),
        axis=(0.0, 1.0, 0.0),
    )
    caster_joint = Joint(
        name="caster_joint",
        type=JointType.FIXED,
        parent="base_link",
        child="caster_wheel",
        origin=Pose(xyz=(0.15, 0.0, -0.08)),
    )

    drivetrain = {
        "type": "differential_drive",
        "left_wheel_joint": "left_wheel_joint",
        "right_wheel_joint": "right_wheel_joint",
        "wheel_separation": WHEEL_SEPARATION,
        "wheel_radius": WHEEL_RADIUS,
    }
    if drivetrain_overrides is not None:
        drivetrain.update(drivetrain_overrides)

    robot = Robot(
        name="sample_rover",
        links=[base_link, left_wheel, right_wheel, caster_wheel],
        joints=[left_wheel_joint, right_wheel_joint, caster_joint],
        metadata={"drivetrain": drivetrain},
    )
    robot.validate()
    return robot


# --- fixture: a small mecanum-drive rover -----------------------------------
#
# Hand-built for this file (not imported from test_ros2_control.py's
# make_mecanum_robot, per this module's instructions to touch only nav2.py /
# test_nav2.py) but mirrors that fixture's geometry pattern: a boxy base_link
# and four CONTINUOUS-jointed cylinder wheels, all box/cylinder primitives so
# footprint-radius computation is exercised without mesh geometry.

MECANUM_WHEEL_RADIUS = 0.08
_HALF_WHEELBASE = 0.2
_HALF_TRACK_WIDTH = 0.15


def build_mecanum_rover(drivetrain_overrides: dict = None) -> Robot:
    base_link = Link(
        name="base_link",
        visual_geometry=Geometry(kind="box", size=(0.5, 0.4, 0.15)),
        collision_geometry=Geometry(kind="box", size=(0.5, 0.4, 0.15)),
        inertial=Inertial(mass=8.0, ixx=0.1, iyy=0.12, izz=0.15),
    )

    def _wheel_link(name: str) -> Link:
        return Link(
            name=name,
            parent="base_link",
            visual_geometry=Geometry(kind="cylinder", radius=MECANUM_WHEEL_RADIUS, length=0.04),
            collision_geometry=Geometry(kind="cylinder", radius=MECANUM_WHEEL_RADIUS, length=0.04),
            inertial=Inertial(mass=0.4, ixx=0.0006, iyy=0.0006, izz=0.001),
        )

    def _wheel_joint(name: str, child: str, x: float, y: float) -> Joint:
        return Joint(
            name=name,
            type=JointType.CONTINUOUS,
            parent="base_link",
            child=child,
            origin=Pose(xyz=(x, y, -0.05)),
            axis=(0.0, 1.0, 0.0),
        )

    front_left = _wheel_link("front_left_wheel")
    front_right = _wheel_link("front_right_wheel")
    back_left = _wheel_link("back_left_wheel")
    back_right = _wheel_link("back_right_wheel")

    fl_joint = _wheel_joint("front_left_wheel_joint", "front_left_wheel", _HALF_WHEELBASE, _HALF_TRACK_WIDTH)
    fr_joint = _wheel_joint("front_right_wheel_joint", "front_right_wheel", _HALF_WHEELBASE, -_HALF_TRACK_WIDTH)
    bl_joint = _wheel_joint("back_left_wheel_joint", "back_left_wheel", -_HALF_WHEELBASE, _HALF_TRACK_WIDTH)
    br_joint = _wheel_joint("back_right_wheel_joint", "back_right_wheel", -_HALF_WHEELBASE, -_HALF_TRACK_WIDTH)

    drivetrain = {
        "type": "mecanum_drive",
        "front_left_wheel_joint": "front_left_wheel_joint",
        "front_right_wheel_joint": "front_right_wheel_joint",
        "back_left_wheel_joint": "back_left_wheel_joint",
        "back_right_wheel_joint": "back_right_wheel_joint",
        "wheel_radius": MECANUM_WHEEL_RADIUS,
        # lx + ly: half wheelbase + half track width.
        "sum_of_robot_center_projection_on_x_y_axis": _HALF_WHEELBASE + _HALF_TRACK_WIDTH,
    }
    if drivetrain_overrides is not None:
        drivetrain.update(drivetrain_overrides)

    robot = Robot(
        name="mecanum_rover",
        links=[base_link, front_left, front_right, back_left, back_right],
        joints=[fl_joint, fr_joint, bl_joint, br_joint],
        metadata={"drivetrain": drivetrain},
    )
    robot.validate()
    return robot


# --- detect_nav2_suitability -------------------------------------------------


def test_suitable_rover_has_no_problems():
    robot = build_sample_rover()
    assert detect_nav2_suitability(robot) == []


def test_no_drivetrain_metadata_is_unsuitable():
    robot = Robot(
        name="armless",
        links=[Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))],
    )
    robot.validate()
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("drivetrain" in p for p in problems)


def test_wrong_drivetrain_type_is_unsuitable():
    robot = build_sample_rover()
    robot.metadata["drivetrain"]["type"] = "ackermann"
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("ackermann" in p for p in problems)


def test_missing_wheel_radius_is_reported_specifically():
    drivetrain = {
        "type": "differential_drive",
        "left_wheel_joint": "left_wheel_joint",
        "right_wheel_joint": "right_wheel_joint",
        "wheel_separation": WHEEL_SEPARATION,
        # wheel_radius intentionally omitted
    }
    robot = build_sample_rover()
    robot.metadata["drivetrain"] = drivetrain
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("wheel_radius" in p for p in problems)
    # Must not also complain about the fields that *are* present.
    assert not any("wheel_separation" in p for p in problems)
    assert not any("left_wheel_joint" in p for p in problems)


def test_wheel_joint_not_in_robot_joints_is_reported():
    robot = build_sample_rover()
    robot.metadata["drivetrain"]["left_wheel_joint"] = "does_not_exist"
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("does_not_exist" in p for p in problems)


def test_negative_wheel_separation_is_reported():
    robot = build_sample_rover()
    robot.metadata["drivetrain"]["wheel_separation"] = -0.1
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("wheel_separation" in p for p in problems)


def test_wheel_joint_of_wrong_type_is_reported():
    robot = build_sample_rover()
    # Repoint the drivetrain at the fixed caster joint instead of a driveable one.
    robot.metadata["drivetrain"]["left_wheel_joint"] = "caster_joint"
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("caster_joint" in p for p in problems)


# --- detect_nav2_suitability: mecanum drive ---------------------------------


def test_suitable_mecanum_rover_has_no_problems():
    robot = build_mecanum_rover()
    assert detect_nav2_suitability(robot) == []


def test_mecanum_missing_required_field_is_reported_specifically():
    robot = build_mecanum_rover()
    del robot.metadata["drivetrain"]["sum_of_robot_center_projection_on_x_y_axis"]
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("sum_of_robot_center_projection_on_x_y_axis" in p for p in problems)
    # Must not also complain about fields that *are* present.
    assert not any("wheel_radius" in p for p in problems)
    assert not any("front_left_wheel_joint" in p for p in problems)


def test_mecanum_missing_wheel_radius_is_reported():
    robot = build_mecanum_rover()
    del robot.metadata["drivetrain"]["wheel_radius"]
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("wheel_radius" in p for p in problems)


def test_mecanum_wheel_joint_not_in_robot_joints_is_reported():
    robot = build_mecanum_rover()
    robot.metadata["drivetrain"]["back_right_wheel_joint"] = "does_not_exist"
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("does_not_exist" in p for p in problems)


def test_unrecognized_drivetrain_type_is_unsuitable_not_silently_ignored():
    # Regression guard: an unrecognized "type" must be reported as a
    # suitability problem, never silently treated as "no drivetrain" or as
    # one of the two known shapes (mirrors ros2_control.py's own regression
    # test for the same historical bug).
    robot = build_mecanum_rover()
    robot.metadata["drivetrain"]["type"] = "omni_drive"
    problems = detect_nav2_suitability(robot)
    assert problems
    assert any("omni_drive" in p for p in problems)


# --- generators refuse unsuitable robots -------------------------------------


def test_generators_raise_for_unsuitable_robot():
    robot = Robot(
        name="armless",
        links=[Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))],
    )
    robot.validate()
    with pytest.raises(ValueError):
        generate_nav2_params_yaml(robot)
    with pytest.raises(ValueError):
        generate_nav2_bringup_launch(robot)
    with pytest.raises(ValueError):
        generate_map_yaml_stub(robot)


# --- compute_footprint_radius -----------------------------------------------


def test_footprint_radius_matches_known_fixture_extents():
    robot = build_sample_rover()
    radius, skipped = compute_footprint_radius(robot)
    assert skipped == []
    # base_link is a (0.4, 0.3, 0.15) box centered on the root -> half-diagonal
    # sqrt(0.2^2 + 0.15^2 + 0.075^2) ~= 0.260 m. Wheels/caster sit closer to
    # the root than that once their own small radius is added, so base_link's
    # half-diagonal should dominate.
    import math

    expected = math.sqrt(0.2 ** 2 + 0.15 ** 2 + 0.075 ** 2)
    assert 0.2 < radius < 0.35
    assert radius == pytest.approx(expected, rel=1e-6)


def test_footprint_radius_skips_mesh_links_and_still_succeeds():
    robot = build_sample_rover()
    # Add a mesh-only decorative link that should be skipped, not measured.
    mesh_link = Link(
        name="decorative_shell",
        parent="base_link",
        visual_geometry=Geometry(kind="mesh", mesh_path="package://sample_rover/meshes/shell.stl"),
    )
    fixed_joint = Joint(
        name="decorative_shell_joint",
        type=JointType.FIXED,
        parent="base_link",
        child="decorative_shell",
        origin=Pose(xyz=(0.0, 0.0, 0.1)),
    )
    robot.links.append(mesh_link)
    robot.joints.append(fixed_joint)
    robot.validate()

    radius, skipped = compute_footprint_radius(robot)
    assert skipped == ["decorative_shell"]
    assert radius > 0.0


def test_footprint_radius_raises_when_only_mesh_geometry_present():
    base_link = Link(
        name="base_link",
        visual_geometry=Geometry(kind="mesh", mesh_path="package://x/meshes/base.stl"),
    )
    left_wheel = Link(name="left_wheel", parent="base_link")
    right_wheel = Link(name="right_wheel", parent="base_link")
    left_joint = Joint(
        name="left_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="left_wheel",
        axis=(0.0, 1.0, 0.0),
    )
    right_joint = Joint(
        name="right_wheel_joint",
        type=JointType.CONTINUOUS,
        parent="base_link",
        child="right_wheel",
        axis=(0.0, 1.0, 0.0),
    )
    robot = Robot(
        name="mesh_only_rover",
        links=[base_link, left_wheel, right_wheel],
        joints=[left_joint, right_joint],
        metadata={
            "drivetrain": {
                "type": "differential_drive",
                "left_wheel_joint": "left_wheel_joint",
                "right_wheel_joint": "right_wheel_joint",
                "wheel_separation": WHEEL_SEPARATION,
                "wheel_radius": WHEEL_RADIUS,
            }
        },
    )
    robot.validate()
    assert detect_nav2_suitability(robot) == []
    with pytest.raises(ValueError):
        compute_footprint_radius(robot)
    with pytest.raises(ValueError):
        generate_nav2_params_yaml(robot)


# --- compute_footprint_radius: mecanum drive (drivetrain-agnostic check) ----


def test_footprint_radius_works_for_four_wheel_mecanum_fixture():
    # compute_footprint_radius operates purely on link geometry/kinematic
    # tree, with no drivetrain-type awareness at all -- confirm it produces
    # the same kind of sane result for a 4-mecanum-wheel robot as it does
    # for the differential-drive fixture above, with zero changes needed to
    # the function itself.
    robot = build_mecanum_rover()
    radius, skipped = compute_footprint_radius(robot)
    assert skipped == []
    # base_link is a (0.5, 0.4, 0.15) box centered on the root -> half-diagonal
    # sqrt(0.25^2 + 0.2^2 + 0.075^2) ~= 0.329 m. Each wheel sits at
    # (+-0.2, +-0.15, -0.05) -- distance sqrt(0.2^2 + 0.15^2 + 0.05^2) ~=
    # 0.255 m from the root -- plus its own cylinder bounding radius
    # sqrt(0.08^2 + 0.02^2) ~= 0.0825 m, for ~0.337 m: slightly *more* than
    # base_link's half-diagonal, so (unlike the differential-drive fixture
    # above, where the base dominates) a wheel sets the radius here. Both
    # are exercised by this suite, and either way the function needs zero
    # drivetrain-specific logic to get it right.
    import math

    expected = math.sqrt(0.2 ** 2 + 0.15 ** 2 + 0.05 ** 2) + math.sqrt(0.08 ** 2 + 0.02 ** 2)
    assert radius == pytest.approx(expected, rel=1e-6)


# --- generate_nav2_params_yaml: mecanum drive --------------------------------


def test_mecanum_params_yaml_uses_mppi_controller_with_omni_motion_model():
    robot = build_mecanum_rover()
    text = generate_nav2_params_yaml(robot)
    assert MECANUM_CONTROLLER_PLUGIN in text
    assert CONTROLLER_PLUGIN not in text  # RPP must not appear for a mecanum robot
    assert 'motion_model: "Omni"' in text
    assert PLANNER_PLUGIN in text  # planner choice is drivetrain-agnostic


@requires_yaml
def test_mecanum_params_yaml_is_parseable_and_structured_correctly():
    robot = build_mecanum_rover()
    text = generate_nav2_params_yaml(robot)
    doc = yaml.safe_load(text)

    assert set(
        [
            "amcl",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "behavior_server",
            "waypoint_follower",
            "map_server",
            "local_costmap",
            "global_costmap",
        ]
    ) <= set(doc.keys())

    follow_path = doc["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow_path["plugin"] == MECANUM_CONTROLLER_PLUGIN
    assert follow_path["motion_model"] == "Omni"
    assert "vy_max" in follow_path  # holonomic-only parameter, confirmed real (see module docstring)
    assert "TwirlingCritic" in follow_path["critics"]

    assert doc["planner_server"]["ros__parameters"]["GridBased"]["plugin"] == PLANNER_PLUGIN
    assert doc["amcl"]["ros__parameters"]["robot_model_type"] == "nav2_amcl::OmniMotionModel"

    local_radius = doc["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"]
    global_radius = doc["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"]
    expected_radius, _ = compute_footprint_radius(robot)
    assert local_radius == pytest.approx(expected_radius, abs=1e-4)
    assert global_radius == pytest.approx(expected_radius, abs=1e-4)


def test_mecanum_bringup_launch_and_map_stub_generate_cleanly():
    # generate_nav2_bringup_launch / generate_map_yaml_stub are
    # drivetrain-agnostic -- confirm they simply work for a mecanum robot
    # too, with no drivetrain-specific content expected in either.
    robot = build_mecanum_rover()
    launch_text = generate_nav2_bringup_launch(robot)
    compile(launch_text, "<string>", "exec")
    assert "bringup_launch.py" in launch_text

    map_text = generate_map_yaml_stub(robot)
    assert "STUB" in map_text or "PLACEHOLDER" in map_text.upper()


# --- generate_nav2_params_yaml ----------------------------------------------


def test_params_yaml_contains_required_sections():
    robot = build_sample_rover()
    text = generate_nav2_params_yaml(robot)
    for key in (
        "amcl:",
        "controller_server:",
        "planner_server:",
        "bt_navigator:",
        "behavior_server:",
        "waypoint_follower:",
        "local_costmap:",
        "global_costmap:",
    ):
        assert key in text, f"missing section {key!r}"
    assert CONTROLLER_PLUGIN in text
    assert PLANNER_PLUGIN in text


@requires_yaml
def test_params_yaml_is_parseable_and_structured_correctly():
    robot = build_sample_rover()
    text = generate_nav2_params_yaml(robot)
    doc = yaml.safe_load(text)

    assert set(
        [
            "amcl",
            "controller_server",
            "planner_server",
            "bt_navigator",
            "behavior_server",
            "waypoint_follower",
            "map_server",
            "local_costmap",
            "global_costmap",
        ]
    ) <= set(doc.keys())

    assert doc["controller_server"]["ros__parameters"]["FollowPath"]["plugin"] == CONTROLLER_PLUGIN
    assert doc["planner_server"]["ros__parameters"]["GridBased"]["plugin"] == PLANNER_PLUGIN

    local_radius = doc["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"]
    global_radius = doc["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"]
    expected_radius, _ = compute_footprint_radius(robot)
    assert local_radius == pytest.approx(expected_radius, abs=1e-4)
    assert global_radius == pytest.approx(expected_radius, abs=1e-4)

    assert doc["amcl"]["ros__parameters"]["base_frame_id"] == "base_link"
    assert doc["bt_navigator"]["ros__parameters"]["robot_base_frame"] == "base_link"


# --- generate_nav2_bringup_launch -------------------------------------------


def test_bringup_launch_is_valid_python():
    robot = build_sample_rover()
    text = generate_nav2_bringup_launch(robot)
    compile(text, "<string>", "exec")  # raises SyntaxError if malformed
    assert "bringup_launch.py" in text
    assert "nav2_bringup" in text
    assert "params_file" in text
    assert "use_sim_time" in text
    assert "generate_launch_description" in text


# --- generate_map_yaml_stub --------------------------------------------------


def test_map_yaml_stub_has_expected_fields():
    robot = build_sample_rover()
    text = generate_map_yaml_stub(robot)
    for key in ("image:", "resolution:", "origin:", "occupied_thresh:", "free_thresh:", "negate:"):
        assert key in text
    # Must clearly flag itself as a placeholder, not a real map.
    assert "STUB" in text or "PLACEHOLDER" in text.upper()


@requires_yaml
def test_map_yaml_stub_is_parseable():
    robot = build_sample_rover()
    text = generate_map_yaml_stub(robot)
    doc = yaml.safe_load(text)
    assert doc["image"] == "map.pgm"
    assert doc["resolution"] == pytest.approx(0.05)
    assert doc["origin"] == [0.0, 0.0, 0.0]
    assert 0.0 <= doc["free_thresh"] <= 1.0
    assert 0.0 <= doc["occupied_thresh"] <= 1.0
