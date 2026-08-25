"""RobotModel -> Nav2 navigation stack config generator.

Nav2 was not part of the original Fusion2ROS brief (URDF / ROS 2 package /
ros2_control / Gazebo / MoveIt2) -- it only makes sense for a mobile/wheeled
robot, and `robot_model.Robot` is a general kinematic tree with no built-in
"this is a mobile base" concept. This module therefore:

1. Detects whether a `Robot` is even suitable for Nav2 (`detect_nav2_suitability`)
   using a documented convention on `Robot.metadata["drivetrain"]`:

       robot.metadata["drivetrain"] = {
           "type": "differential_drive",
           "left_wheel_joint": "<joint name, JointType.CONTINUOUS or REVOLUTE>",
           "right_wheel_joint": "<joint name>",
           "wheel_separation": <float, meters>,
           "wheel_radius": <float, meters>,
       }

   This is the SAME convention `fusion_addin/generators/ros2_control.py` uses
   to detect a `diff_drive_controller` -- both generators key off the exact
   same `Robot.metadata["drivetrain"]` shape so a single extraction step can
   feed both.

2. Only if suitable, generates:
   - `generate_nav2_params_yaml`: a Nav2 stack parameter file.
   - `generate_nav2_bringup_launch`: a launch file that includes
     `nav2_bringup`'s own `bringup_launch.py`.
   - `generate_map_yaml_stub`: a placeholder `nav2_map_server` map YAML (this
     project has no SLAM/mapping step -- see that function's docstring).

Pure stdlib + `robot_model`. No Fusion API, no ROS/rclpy imports, no
third-party packages (including no PyYAML) -- testable with plain
`python3 -m pytest`, no Nav2 installation required. Nav2's parameter/launch
schemas are hand-authored as text (mirroring the style `fusion_addin/generators/urdf.py`
uses for XML) rather than emitted via a YAML library, to keep this module
dependency-free.

Plugin choices (see module-level constants below for the exact strings used):

- Controller (`FollowPath`): `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`,
  not `dwb_core::DWBLocalPlanner`. Nav2's own docs recommend Regulated Pure
  Pursuit specifically for differential-drive and Ackermann bases (exactly
  what `detect_nav2_suitability` requires here), and it needs far fewer
  hand-tuned parameters than DWB's critic stack. DWB remains nav2_bringup's
  own shipped default as of the Humble/Iron/Jazzy params.yaml (verified
  against https://github.com/ros-navigation/navigation2 at the time this was
  written) and is a perfectly reasonable alternative -- it just isn't the
  choice made here.
- Planner (`GridBased`): `nav2_navfn_planner::NavfnPlanner` -- the
  long-standing default grid planner, unchanged across recent Nav2 releases
  (still the shipped default in nav2_bringup's params.yaml as of this
  writing), so there is nothing to second-guess here.

Footprint: modeled as a single circle (`robot_radius`), sized to the largest
bounding sphere among the robot's link geometries. Only `box`/`cylinder`/
`sphere` primitives can be measured this way -- a `mesh` link's true extent
isn't available from `RobotModel` alone (that would require loading and
measuring the actual mesh file, which this module deliberately does not do),
so mesh-geometry links are skipped and reported back to the caller/reader via
a comment in the generated YAML. If a robot's only meaningful bulk is in mesh
links, the computed radius will be an underestimate -- callers should treat
this as a documented limitation, not a bug.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from robot_model import Geometry, Joint, JointType, Link, Pose, Robot

Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]

_IDENTITY_ROTATION: Mat3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

# --- plugin choices (see module docstring for the "why") --------------------

CONTROLLER_PLUGIN = "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
PLANNER_PLUGIN = "nav2_navfn_planner::NavfnPlanner"

_REQUIRED_DRIVETRAIN_TYPE = "differential_drive"


# --- suitability detection ---------------------------------------------------


def detect_nav2_suitability(robot: Robot) -> List[str]:
    """Return a list of problems that make `robot` unsuitable for Nav2
    generation. An empty list means `robot` is suitable.

    A robot is suitable only if `robot.metadata["drivetrain"]` declares a
    `"differential_drive"` base (see module docstring for the exact shape),
    with both wheel joints actually present in `robot.joints` (and of a
    driveable type), and both `wheel_separation`/`wheel_radius` present as
    positive numbers. This function never guesses or defaults a missing
    field -- every problem it reports names the specific thing that is
    missing or invalid.
    """
    problems: List[str] = []

    metadata = robot.metadata or {}
    drivetrain = metadata.get("drivetrain")

    if drivetrain is None:
        problems.append(
            "robot.metadata['drivetrain'] is not set. Nav2 generation only "
            "supports a differential-drive mobile base declared via that key "
            "(see fusion_addin/generators/nav2.py module docstring for the "
            "required shape); this Robot has no drivetrain metadata at all."
        )
        return problems

    if not isinstance(drivetrain, dict):
        problems.append(
            f"robot.metadata['drivetrain'] must be a dict, got {type(drivetrain).__name__}."
        )
        return problems

    drive_type = drivetrain.get("type")
    if drive_type != _REQUIRED_DRIVETRAIN_TYPE:
        problems.append(
            f"robot.metadata['drivetrain']['type'] is {drive_type!r}; Nav2 "
            f"generation currently only supports {_REQUIRED_DRIVETRAIN_TYPE!r}."
        )
        return problems

    joints_by_name: Dict[str, Joint] = {j.name: j for j in robot.joints}

    for field_name in ("left_wheel_joint", "right_wheel_joint"):
        joint_name = drivetrain.get(field_name)
        if not joint_name:
            problems.append(f"drivetrain[{field_name!r}] is missing or empty.")
            continue
        joint = joints_by_name.get(joint_name)
        if joint is None:
            problems.append(
                f"drivetrain[{field_name!r}] = {joint_name!r} does not match any "
                "joint name in robot.joints."
            )
        elif joint.type not in (JointType.CONTINUOUS, JointType.REVOLUTE):
            problems.append(
                f"drivetrain[{field_name!r}] joint {joint_name!r} has type "
                f"{joint.type.value!r}; a driven wheel joint must be "
                "CONTINUOUS or REVOLUTE."
            )

    for field_name in ("wheel_separation", "wheel_radius"):
        value = drivetrain.get(field_name)
        if value is None:
            problems.append(f"drivetrain[{field_name!r}] is missing.")
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            problems.append(f"drivetrain[{field_name!r}] = {value!r} is not a number.")
            continue
        if not (float(value) > 0.0):
            problems.append(f"drivetrain[{field_name!r}] = {value!r} must be a positive number.")

    return problems


def _require_suitable(robot: Robot) -> None:
    """Shared guard for all three generators below: re-validate the graph
    itself, then re-run `detect_nav2_suitability` and raise a single clear
    `ValueError` naming every problem found, rather than generating output
    for a robot Nav2 cannot actually run on."""
    robot.validate()  # raises robot_model.ValidationError on graph problems
    problems = detect_nav2_suitability(robot)
    if problems:
        raise ValueError(
            f"Robot {robot.name!r} is not suitable for Nav2 generation:\n- "
            + "\n- ".join(problems)
        )


# --- footprint radius (forward kinematics over box/cylinder/sphere) --------


def _rotation_matrix(rpy: Vec3) -> Mat3:
    """URDF's fixed-axis roll-pitch-yaw convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll),
    matching `robot_model.schema.Pose`'s documented convention."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mat_vec(m: Mat3, v: Vec3) -> Vec3:
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))  # type: ignore[return-value]


def _mat_mat(a: Mat3, b: Mat3) -> Mat3:
    return tuple(  # type: ignore[return-value]
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _vec_add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vec_norm(v: Vec3) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def _link_world_transforms(robot: Robot) -> Dict[str, Tuple[Vec3, Mat3]]:
    """Position + rotation of every link's frame relative to the root link,
    computed by chaining each joint's static `origin` (position + rpy) from
    root outward. Joint *motion* (the variable position/angle a real joint
    would take) is ignored -- this is a nominal, zero-configuration pose,
    which is the only pose `RobotModel` (a static description) can express.
    """
    transforms: Dict[str, Tuple[Vec3, Mat3]] = {}
    root = robot.root_link()
    if root is None:
        return transforms
    transforms[root.name] = ((0.0, 0.0, 0.0), _IDENTITY_ROTATION)

    children_joints: Dict[str, List[Joint]] = {}
    for j in robot.joints:
        children_joints.setdefault(j.parent, []).append(j)

    frontier = [root.name]
    while frontier:
        parent_name = frontier.pop()
        parent_pos, parent_rot = transforms[parent_name]
        for j in children_joints.get(parent_name, []):
            joint_rot = _rotation_matrix(j.origin.rpy)
            child_rot = _mat_mat(parent_rot, joint_rot)
            child_pos = _vec_add(parent_pos, _mat_vec(parent_rot, j.origin.xyz))
            transforms[j.child] = (child_pos, child_rot)
            frontier.append(j.child)
    return transforms


def _geometry_bounding_radius(geometry: Geometry) -> Optional[float]:
    """Radius of the smallest sphere, centered on the geometry's own origin,
    that contains it -- orientation-independent, so the geometry's own
    rotation never needs to be considered, only its origin's position.
    Returns None for mesh geometry (unmeasurable from RobotModel alone)."""
    if geometry.kind == "box":
        hx, hy, hz = (s / 2.0 for s in geometry.size)  # type: ignore[union-attr]
        return math.sqrt(hx * hx + hy * hy + hz * hz)
    if geometry.kind == "cylinder":
        return math.sqrt(geometry.radius ** 2 + (geometry.length / 2.0) ** 2)  # type: ignore[operator]
    if geometry.kind == "sphere":
        return geometry.radius
    return None  # mesh


def compute_footprint_radius(robot: Robot) -> Tuple[float, List[str]]:
    """Conservative circular footprint radius for `robot`: the largest
    distance, from the root link's origin, to the far edge of any link's
    box/cylinder/sphere collision (falling back to visual) geometry, found by
    walking the kinematic tree with each joint's static origin.

    Returns `(radius_m, skipped_mesh_link_names)`. Raises `ValueError` if no
    link contributes a measurable primitive geometry at all (e.g. every link
    is mesh-only) -- there is nothing to conservatively measure in that case,
    and inventing a number would be worse than refusing.
    """
    transforms = _link_world_transforms(robot)
    max_radius = 0.0
    skipped_mesh_links: List[str] = []
    found_any = False

    for link in robot.links:
        geometry = link.collision_geometry or link.visual_geometry
        if geometry is None:
            continue
        if geometry.kind == "mesh":
            skipped_mesh_links.append(link.name)
            continue

        bounding = _geometry_bounding_radius(geometry)
        if bounding is None:
            continue  # pragma: no cover - unreachable given Geometry's valid kinds

        link_pos, link_rot = transforms.get(link.name, ((0.0, 0.0, 0.0), _IDENTITY_ROTATION))
        geom_world_pos = _vec_add(link_pos, _mat_vec(link_rot, link.origin.xyz))
        radius = _vec_norm(geom_world_pos) + bounding

        found_any = True
        max_radius = max(max_radius, radius)

    if not found_any:
        raise ValueError(
            f"Robot {robot.name!r} has no link with box/cylinder/sphere collision "
            "or visual geometry -- cannot compute a Nav2 footprint radius. "
            "(Mesh-only links are skipped because RobotModel cannot measure a "
            "mesh's true extent; add primitive collision geometry to at least "
            "the links that define the robot's outer bulk.)"
        )

    return max_radius, sorted(skipped_mesh_links)


def _find_scan_topic(robot: Robot) -> str:
    """Best-effort scan topic name: use a declared lidar sensor's name (or
    its `parameters['topic']` override) if one exists on the robot, else fall
    back to Nav2's conventional default topic name "scan". Never invents a
    topic name beyond what the robot or Nav2 convention already supplies."""
    for sensor in robot.sensors:
        if sensor.type.lower() == "lidar":
            topic = sensor.parameters.get("topic") if sensor.parameters else None
            return str(topic) if topic else sensor.name
    return "scan"


def _fmt_num(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    s = f"{float(value):.6f}".rstrip("0")
    if s.endswith("."):
        s += "0"
    return s


# --- generate_nav2_params_yaml ----------------------------------------------


def generate_nav2_params_yaml(robot: Robot) -> str:
    """Render a Nav2 stack parameter file for `robot`.

    Covers `amcl`, `controller_server` (FollowPath = Regulated Pure Pursuit),
    `planner_server` (GridBased = NavfnPlanner), `bt_navigator`,
    `behavior_server`, `waypoint_follower`, `map_server`, and both
    `local_costmap`/`global_costmap` (circular footprint via `robot_radius`,
    see `compute_footprint_radius`). This is a starter config covering the
    core stack, not a reproduction of every optional Nav2 feature (keepout/
    speed filters, docking_server, route_server, collision_monitor,
    smoother_server, velocity_smoother, ... are all left out -- add them by
    hand if this robot needs them).

    Raises `ValueError` (via `detect_nav2_suitability`) if `robot` isn't a
    differential-drive base per `robot.metadata["drivetrain"]`, and
    `robot_model.ValidationError` if the graph itself is invalid.
    """
    _require_suitable(robot)

    base_frame = robot.root_link().name  # type: ignore[union-attr]  -- guaranteed by validate()
    footprint_radius, skipped_mesh_links = compute_footprint_radius(robot)
    radius_str = _fmt_num(footprint_radius)
    scan_topic = _find_scan_topic(robot)

    mesh_note = ""
    if skipped_mesh_links:
        mesh_note = (
            "#\n"
            "# NOTE: the following link(s) use mesh geometry and were SKIPPED\n"
            "# when computing the footprint radius below -- RobotModel has no\n"
            "# way to measure a mesh's true extent, only box/cylinder/sphere\n"
            "# primitives were counted. If any of these links extend further\n"
            "# than the radius below, widen it by hand:\n"
            + "\n".join(f"#   - {name}" for name in skipped_mesh_links)
            + "\n"
        )

    has_lidar_sensor = any(s.type.lower() == "lidar" for s in robot.sensors)
    if has_lidar_sensor:
        scan_topic_note = "from a declared lidar Sensor on this robot"
    else:
        scan_topic_note = (
            "no lidar Sensor found on this robot; using Nav2's conventional "
            "default -- update if yours differs"
        )

    header = f"""# Nav2 parameter file for robot "{robot.name}", generated by Fusion2ROS
# (fusion_addin/generators/nav2.py). Structure follows nav2_bringup's
# standard nav2_params.yaml (see https://github.com/ros-navigation/navigation2
# nav2_bringup/params/nav2_params.yaml).
#
# Controller plugin: {CONTROLLER_PLUGIN}
#   Chosen over dwb_core::DWBLocalPlanner (nav2_bringup's own shipped default)
#   because Nav2's docs recommend Regulated Pure Pursuit specifically for
#   differential-drive bases like this one, with far fewer parameters to tune.
# Planner plugin: {PLANNER_PLUGIN}
#   The long-standing default grid planner; unchanged across recent Nav2
#   releases, so there was no real alternative to weigh here.
#
# Footprint: a single circle of radius {radius_str} m (`robot_radius` below),
# the largest bounding sphere among this robot's link geometries relative to
# base frame "{base_frame}". Conservative, not a tight fit.
{mesh_note}#
# Frame assumptions: "map" / "odom" / "{base_frame}" (this robot's root link)
# are used as the map/odom/base frames throughout, following standard ROS 2
# navigation convention. Scan topic assumed: "{scan_topic}" ({scan_topic_note}).
"""

    amcl = f"""
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "{base_frame}"
    global_frame_id: "map"
    odom_frame_id: "odom"
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: {scan_topic}
"""

    controller_server = f"""
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    general_goal_checker:
      stateful: true
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
    FollowPath:
      plugin: "{CONTROLLER_PLUGIN}"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_approach_linear_velocity: 0.05
      approach_velocity_scaling_dist: 1.0
      use_collision_detection: true
      max_allowed_time_to_collision_up_to_carrot: 1.0
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_fixed_curvature_lookahead: false
      curvature_lookahead_dist: 1.0
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_angular_accel: 3.2
      max_robot_pose_search_dist: 10.0
"""

    planner_server = f"""
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "{PLANNER_PLUGIN}"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
"""

    bt_navigator = f"""
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: {base_frame}
    odom_topic: odom
    bt_loop_duration: 10
    default_server_timeout: 20
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
"""

    behavior_server = f"""
behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw
    global_costmap_topic: global_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors::DriveOnHeading"
    wait:
      plugin: "nav2_behaviors::Wait"
    assisted_teleop:
      plugin: "nav2_behaviors::AssistedTeleop"
    local_frame: odom
    global_frame: map
    robot_base_frame: {base_frame}
    transform_tolerance: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
"""

    waypoint_follower = """
waypoint_follower:
  ros__parameters:
    loop_rate: 20
    stop_on_failure: false
    waypoint_task_executor_plugin: "wait_at_waypoint"
    wait_at_waypoint:
      plugin: "nav2_waypoint_follower::WaitAtWaypoint"
      enabled: true
      waypoint_pause_duration: 200
"""

    map_server = """
map_server:
  ros__parameters:
    yaml_filename: ""
"""

    local_costmap = f"""
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: {base_frame}
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: {radius_str}
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: {scan_topic}
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      always_send_full_costmap: true
"""

    global_costmap = f"""
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: {base_frame}
      robot_radius: {radius_str}
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: {scan_topic}
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      always_send_full_costmap: true
"""

    return "".join(
        [
            header,
            amcl,
            controller_server,
            planner_server,
            bt_navigator,
            behavior_server,
            waypoint_follower,
            map_server,
            local_costmap,
            global_costmap,
        ]
    )


# --- generate_nav2_bringup_launch -------------------------------------------


def generate_nav2_bringup_launch(robot: Robot) -> str:
    """Render a `launch`/`launch_ros` Python launch file that brings up the
    full Nav2 stack for `robot` by including `nav2_bringup`'s own
    `bringup_launch.py` (the standard, documented way to start Nav2's
    lifecycle-managed nodes -- see
    https://github.com/ros-navigation/navigation2 nav2_bringup/launch/bringup_launch.py),
    rather than hand-rolling individual lifecycle nodes here.

    Assumes the generated package (see `fusion_addin/generators/package.py`)
    ships this module's `generate_nav2_params_yaml` output at
    `<pkg_share>/params/nav2_params.yaml` and `generate_map_yaml_stub`'s
    output at `<pkg_share>/maps/map.yaml` -- both overridable at launch time
    via the `params_file`/`map` launch arguments.

    Raises the same errors as `generate_nav2_params_yaml` for an unsuitable
    or invalid `robot`.
    """
    _require_suitable(robot)

    package_name = robot.name

    return f'''"""Nav2 bringup launch file for the "{package_name}" robot.

Auto-generated by Fusion2ROS (fusion_addin/generators/nav2.py). Includes
nav2_bringup's own bringup_launch.py -- the standard, documented way to start
Nav2's full lifecycle-managed stack -- rather than hand-rolling individual
lifecycle nodes here.

Regenerate from Fusion rather than editing by hand.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "{package_name}"


def generate_launch_description():
    pkg_share = FindPackageShare(PACKAGE_NAME)
    nav2_bringup_share = FindPackageShare("nav2_bringup")

    default_params_file = PathJoinSubstitution([pkg_share, "params", "nav2_params.yaml"])
    default_map_file = PathJoinSubstitution([pkg_share, "maps", "map.yaml"])
    bringup_launch_file = PathJoinSubstitution([nav2_bringup_share, "launch", "bringup_launch.py"])

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )
    map_arg = DeclareLaunchArgument(
        "map", default_value=default_map_file, description="Full path to the map yaml file to load"
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file", default_value=default_params_file, description="Full path to the Nav2 parameters file"
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically start the Nav2 lifecycle nodes"
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_file),
        launch_arguments={{
            "namespace": LaunchConfiguration("namespace"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "autostart": LaunchConfiguration("autostart"),
        }}.items(),
    )

    return LaunchDescription(
        [
            namespace_arg,
            use_sim_time_arg,
            map_arg,
            params_file_arg,
            autostart_arg,
            nav2_bringup,
        ]
    )
'''


# --- generate_map_yaml_stub --------------------------------------------------


def generate_map_yaml_stub(robot: Robot) -> str:
    """Render a placeholder `nav2_map_server` map YAML for `robot`.

    Fusion2ROS has no mapping/SLAM step, so there is no real occupancy grid
    to ship. This is a STUB: it points at a placeholder image filename that
    does not exist and is not created by this function. It exists only so
    Nav2's launch/config wiring is complete and so the user has an obvious,
    documented placeholder to replace -- it must NOT be mistaken for a real
    map. See the comment embedded in the returned text.
    """
    _require_suitable(robot)

    return f"""# STUB map for robot "{robot.name}" -- generated by Fusion2ROS
# (fusion_addin/generators/nav2.py).
#
# THIS IS A PLACEHOLDER, NOT A REAL MAP. Fusion2ROS has no mapping/SLAM step
# -- "map.pgm" referenced below does not exist and this file describes no
# real environment. amcl/map_server will load it without erroring, but Nav2
# WILL NOT NAVIGATE CORRECTLY against it.
#
# Before using Nav2 for real:
#   1. Drive this robot around its real (or simulated) environment with a
#      SLAM tool, e.g. `slam_toolbox`.
#   2. Save the resulting map, e.g. via
#      `ros2 run nav2_map_server map_saver_cli -f map`, and use the files it
#      produces (map.yaml + map.pgm) in place of this stub.
#
# Standard nav2_map_server YAML fields:
image: map.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
