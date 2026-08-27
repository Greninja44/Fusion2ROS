"""RobotModel -> Gazebo Sim (gz-sim) simulation artifacts.

Produces three SEPARATE, ADDITIONAL pieces of text that a later integration
step splices together with the existing generators
(``fusion_addin/generators/urdf.py`` for the URDF/xacro body, and the
parallel ``fusion_addin/generators/ros2_control.py`` for the
``<ros2_control>`` XML + ``controllers.yaml``):

1. ``generate_gazebo_xml`` -- a ``<gazebo>`` XML fragment carrying gz-sim
   material colors and the ``gz_ros2_control`` plugin block.
2. ``generate_world_sdf`` -- a minimal, valid gz-sim SDF world file.
3. ``generate_spawn_launch`` -- a ``launch``/``launch_ros`` Python launch
   file that starts gz-sim, ``robot_state_publisher``, and spawns the robot.

Pure function of a ``robot_model.Robot`` plus stdlib string templating: no
Fusion API, no ROS/rclpy imports, no filesystem access, no network. Safe to
import and call from Fusion's embedded interpreter, from plain WSL
``python3``, or from CI with neither installed -- same constraint
``robot_model`` and ``fusion_addin/generators/urdf.py`` are held to (see
robot_model/schema.py and docs/ARCHITECTURE.md).

This project's ROS 2 install ("lyrical") ships **Gazebo Sim (gz-sim,
formerly "Ignition")**, confirmed via ``ros-lyrical-ros-gz`` /
``ros-lyrical-gz-sim-vendor`` and a live ``gz sim --version`` -> ``10.4.0``
-- NOT classic Gazebo. Every plugin filename, tag shape, and executable name
below was checked against that fact rather than assumed from classic-Gazebo
memory; see each function's docstring for exactly how each claim was
verified in this session (a locally installed, gz-sim-native reference file
read directly off disk, an installed launch file's source, a live
``--help``/binary run, or a fetched upstream doc when nothing local carried
the answer).

Design notes
------------
* ``generate_gazebo_xml`` returns one wrapper root element,
  ``<gazebo_fragment>``, so the function's own output is independently
  parseable/testable XML. It is NOT meant to be spliced in as-is: the real
  integration step (not implemented here, per the task boundary) is expected
  to take the wrapper's ``<gazebo>`` children and append each one as a
  sibling of ``<link>``/``<joint>`` inside the ``<robot>`` root that
  ``generate_urdf_xacro`` produces -- exactly where classic and gz-sim URDF
  both expect ``<gazebo>`` extension tags to live. One ``<gazebo
  reference="LINK">`` element is emitted per link that has a material with
  an ``rgba`` set (links with no material, or a material with no color, are
  skipped -- there is nothing gz-sim-specific to say about them). Exactly
  one un-referenced, robot-level ``<gazebo>`` carries the ``gz_ros2_control``
  plugin block.
* Floats are formatted with the same fixed, non-scientific-notation scheme
  as ``urdf.py`` (reimplemented locally rather than importing urdf.py's
  private helpers, to keep this module decoupled per the task brief).
* This module never calls ``robot.validate()`` itself -- ``generate_gazebo_xml``
  only reads ``robot.links`` (name + material), so a robot that's invalid for
  other reasons (e.g. a dangling joint reference) doesn't stop it from
  rendering material blocks. Callers that need the full-pipeline validation
  guarantee already get it from ``generate_urdf_xacro``, which this module
  does not duplicate.
"""

import xml.etree.ElementTree as ET
from typing import Dict, List

from robot_model import JointType, Robot

__all__ = [
    "generate_gazebo_xml",
    "generate_world_sdf",
    "generate_spawn_launch",
    "generate_gazebo_ros_bridge_yaml",
]


# --- gz_ros2_control plugin surface ----------------------------------------
#
# Verified against control.ros.org's rendered gz_ros2_control documentation
# (https://control.ros.org/rolling/doc/gz_ros2_control/doc/index.html, itself
# built from https://github.com/ros-controls/gz_ros2_control's doc/index.rst),
# fetched live in this session. Its own worked example is:
#
#   <gazebo>
#     <plugin filename="libgz_ros2_control-system.so"
#             name="gz_ros2_control::GazeboSimROS2ControlPlugin">
#       <parameters>$(find gz_ros2_control_demos)/config/cart_controller.yaml
#       </parameters>
#     </plugin>
#   </gazebo>
#
# `gz_ros2_control` was NOT installed in this sandbox (`ros2 pkg list | grep
# -i gz_ros2_control` and `dpkg -l | grep gz-ros2-control` both came back
# empty), so this plugin filename/name pair could not additionally be
# cross-checked against a live `.so` on disk -- the doc fetch above is the
# only verification for this specific pair. The `$(find <pkg>)/...` form is
# reproduced literally (rather than substituted for xacro's ROS 2
# `$(find-pkg-share ...)`) because that is the exact, only substitution style
# gz_ros2_control's own example uses in its `<parameters>` tag -- it is
# resolved by the plugin itself at load time, not by xacro, so inventing a
# different substitution syntax here would silently break it.
GZ_ROS2_CONTROL_PLUGIN_FILENAME = "libgz_ros2_control-system.so"
GZ_ROS2_CONTROL_PLUGIN_NAME = "gz_ros2_control::GazeboSimROS2ControlPlugin"


# --- native gz-sim DiffDrive / JointStatePublisher plugins ------------------
#
# `gz_ros2_control` (above) is real-verified to SIGSEGV on load against this
# machine's installed gz-sim 10.4.0 -- a confirmed, filed upstream bug
# (ros-controls/gz_ros2_control#848, "Segfault in
# GazeboSimROS2ControlPlugin::Configure()"), not something fixable from this
# project's generator code (see docs/ARCHITECTURE.md's "Gazebo" verification
# section for the full root-cause trace). For a `Robot` with a
# `metadata["drivetrain"]` of type "differential_drive" (the same convention
# `ros2_control.py`/`nav2.py` already key off), gz-sim ships a NATIVE
# alternative that sidesteps gz_ros2_control entirely: `DiffDrive`, a
# built-in gz-sim system plugin with no ros2_control dependency at all.
#
# Every tag name/plugin filename/message-type pairing below is copied from
# two real files read directly off this machine's disk, not invented:
#   - the plugin tag shape (filename, name, left_joint/right_joint,
#     wheel_separation/wheel_radius, topic/odom_topic/frame_id/
#     child_frame_id/odom_publish_frequency/tf_topic) from
#     `/opt/ros/lyrical/share/turtlebot3_gazebo/models/turtlebot3_waffle/
#     model.sdf`'s own real, shipped `DiffDrive` + `JointStatePublisher`
#     plugin blocks -- a genuine ROS-shipped differential-drive robot using
#     this exact gz-sim version;
#   - the paired bridge message types (`geometry_msgs/msg/TwistStamped` <->
#     `gz.msgs.Twist` for cmd_vel, `nav_msgs/msg/Odometry` <->
#     `gz.msgs.Odometry` for odom, `tf2_msgs/msg/TFMessage` <->
#     `gz.msgs.Pose_V` for tf, `sensor_msgs/msg/JointState` <->
#     `gz.msgs.Model` for joint_states) from that same package's installed
#     `turtlebot3_waffle_bridge.yaml`, which bridges exactly this plugin
#     pair's topics for real.
# `<odom_publish_frequency>` (not turtlebot3's own file's
# "odom_publisher_frequency", almost certainly a harmless upstream typo
# silently ignored by the plugin) is spelled per the plugin's own API docs
# (gazebosim.org/api/sim/10/classgz_1_1sim_1_1systems_1_1DiffDrive.html).
GZ_DIFF_DRIVE_PLUGIN_FILENAME = "gz-sim-diff-drive-system"
GZ_DIFF_DRIVE_PLUGIN_NAME = "gz::sim::systems::DiffDrive"
GZ_JOINT_STATE_PUBLISHER_PLUGIN_FILENAME = "gz-sim-joint-state-publisher-system"
GZ_JOINT_STATE_PUBLISHER_PLUGIN_NAME = "gz::sim::systems::JointStatePublisher"


def _controllers_yaml_param(robot_name: str) -> str:
    """Standard ROS 2 package-share convention: `generate_controllers_yaml`
    (the parallel ros2_control.py generator) is assumed to land its output
    at `share/<robot_name>/config/controllers.yaml` -- a plain, ordinary
    ament package layout, not anything gz-sim-specific. Expressed with the
    `$(find <pkg>)` form gz_ros2_control's own docs use (see module-level
    comment above)."""
    return f"$(find {robot_name})/config/controllers.yaml"


def _is_differential_drive(robot: Robot) -> bool:
    drivetrain = robot.metadata.get("drivetrain") if robot.metadata else None
    return bool(drivetrain) and drivetrain.get("type") == "differential_drive"


def _moving_joint_names(robot: Robot) -> List[str]:
    return [j.name for j in robot.joints if j.type != JointType.FIXED]


def _base_frame_name(robot: Robot) -> str:
    root = robot.root_link()
    return root.name if root is not None else "base_link"


def generate_gazebo_xml(robot: Robot) -> str:
    """Render a ``<gazebo_fragment>`` XML string carrying, per `robot`:

    - one ``<gazebo reference="LINK">`` block per link whose
      ``Link.material.rgba`` is set, giving it a gz-sim-style
      ``<visual><material><ambient>/<diffuse>/<specular></material></visual>``
      color (gz-sim does NOT understand classic Gazebo's
      ``<material>Gazebo/Blue</material>`` string-name convention -- see the
      tag-shape citation below).
    - if `robot` has any non-fixed joint, one un-referenced ``<gazebo>``
      block loading the native ``JointStatePublisher`` plugin (see the
      module-level comment above it) so every joint's real simulated motion
      reaches ROS as ``/joint_states`` -- without this, `robot_state_publisher`
      has nothing to drive non-fixed joints' TF with in Gazebo, regardless
      of drivetrain type.
    - exactly one further un-referenced, robot-level ``<gazebo>`` block for
      actuation: the native ``DiffDrive`` plugin (see the module-level
      comment above it) when ``robot.metadata["drivetrain"]["type"] ==
      "differential_drive"`` -- this fully replaces the need for
      ``gz_ros2_control`` for that case, and avoids its confirmed SIGSEGV
      entirely. Any other robot (no drivetrain, or an unsupported drivetrain
      type like "mecanum_drive", which has no native gz-sim equivalent
      confirmed here) falls back to the pre-existing ``gz_ros2_control``
      plugin, unchanged -- still subject to the documented upstream crash,
      but no regression for shapes this project can't yet route natively.

    Ambient/diffuse/specular are all set to the same RGBA -- there is no
    separate "shininess" concept in RobotModel's single-color `Material`, so
    rather than invent an arbitrary ambient/specular attenuation this uses
    the one color RobotModel actually carries for all three, matching the
    convention several real xacro ``gazebo.xacro`` color macros use (e.g.
    ROS-Industrial's `ur_description`).

    Does not call `robot.validate()` -- see module docstring.
    """
    fragment = ET.Element("gazebo_fragment")

    for link in robot.links:
        if link.material is None or link.material.rgba is None:
            continue
        fragment.append(_build_material_gazebo_element(link.name, link.material.rgba))

    moving_joints = _moving_joint_names(robot)
    if moving_joints:
        fragment.append(_build_joint_state_publisher_element(moving_joints))

    if _is_differential_drive(robot):
        fragment.append(_build_diff_drive_plugin_element(robot.metadata["drivetrain"], _base_frame_name(robot)))
    else:
        fragment.append(_build_ros2_control_plugin_element(robot.name))

    ET.indent(fragment, space="  ")
    body = ET.tostring(fragment, encoding="unicode")
    return f"{body}\n"


def _build_material_gazebo_element(link_name: str, rgba) -> ET.Element:
    """<gazebo reference="LINK"><visual><material><ambient>/<diffuse>/
    <specular></material></visual></gazebo>.

    Tag nesting confirmed via sdformat.org's own "SDFormat extensions to
    URDF (the 'gazebo' tag)" tutorial, v1.6
    (https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/1.6/),
    fetched live in this session. That page explicitly contrasts the two
    conventions:

        Gazebo-classic (does NOT apply here):
            <gazebo reference='base_link'>
              <material>Gazebo/Orange</material>
            </gazebo>

        New Gazebo / gz-sim (what this function emits):
            <gazebo reference='base_link'>
              <visual>
                <material>
                  <diffuse>0 0 1 1</diffuse>
                </material>
              </visual>
            </gazebo>

    ...and states plainly: "This tag is only relevant when using
    Gazebo-classic as the new version of Gazebo does not use material
    scripts." <ambient>/<specular> are added alongside <diffuse> per the SDF
    <material> element's own spec (linked from the same page), which defines
    all three as independent, optional color components.
    """
    elem = ET.Element("gazebo", {"reference": link_name})
    visual = ET.SubElement(elem, "visual")
    material = ET.SubElement(visual, "material")
    rgba_str = _fmt_vec(rgba)
    ET.SubElement(material, "ambient").text = rgba_str
    ET.SubElement(material, "diffuse").text = rgba_str
    ET.SubElement(material, "specular").text = rgba_str
    return elem


def _build_ros2_control_plugin_element(robot_name: str) -> ET.Element:
    elem = ET.Element("gazebo")
    plugin = ET.SubElement(
        elem,
        "plugin",
        {"filename": GZ_ROS2_CONTROL_PLUGIN_FILENAME, "name": GZ_ROS2_CONTROL_PLUGIN_NAME},
    )
    ET.SubElement(plugin, "parameters").text = _controllers_yaml_param(robot_name)
    return elem


def _build_diff_drive_plugin_element(drivetrain: Dict[str, object], base_frame: str) -> ET.Element:
    """<gazebo><plugin filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">...</plugin></gazebo> -- tag shape and
    values per the module-level GZ_DIFF_DRIVE_PLUGIN_* comment's citation.
    `topic`/`odom_topic`/`frame_id`/`tf_topic` are fixed, plain names (not
    gz-sim's own `/model/<name>/...` defaults) so they match the plain
    `cmd_vel`/`odom`/`tf` topic names `generate_gazebo_ros_bridge_yaml`
    bridges into ROS -- and `odom_frame_id`/`base_frame_id` conventions
    `nav2.py` already assumes (`odom`, `robot.root_link().name`)."""
    elem = ET.Element("gazebo")
    plugin = ET.SubElement(
        elem, "plugin", {"filename": GZ_DIFF_DRIVE_PLUGIN_FILENAME, "name": GZ_DIFF_DRIVE_PLUGIN_NAME}
    )
    ET.SubElement(plugin, "left_joint").text = str(drivetrain["left_wheel_joint"])
    ET.SubElement(plugin, "right_joint").text = str(drivetrain["right_wheel_joint"])
    ET.SubElement(plugin, "wheel_separation").text = _fmt_float(drivetrain["wheel_separation"])
    ET.SubElement(plugin, "wheel_radius").text = _fmt_float(drivetrain["wheel_radius"])
    ET.SubElement(plugin, "topic").text = "cmd_vel"
    ET.SubElement(plugin, "odom_topic").text = "odom"
    ET.SubElement(plugin, "frame_id").text = "odom"
    ET.SubElement(plugin, "child_frame_id").text = base_frame
    ET.SubElement(plugin, "odom_publish_frequency").text = "50"
    ET.SubElement(plugin, "tf_topic").text = "/tf"
    return elem


def _build_joint_state_publisher_element(joint_names: List[str]) -> ET.Element:
    """<gazebo><plugin filename="gz-sim-joint-state-publisher-system"
    name="gz::sim::systems::JointStatePublisher"><topic>joint_states</topic>
    one <joint_name> per non-fixed joint</plugin></gazebo> -- tag shape per
    the module-level GZ_JOINT_STATE_PUBLISHER_PLUGIN_* comment's citation."""
    elem = ET.Element("gazebo")
    plugin = ET.SubElement(
        elem,
        "plugin",
        {"filename": GZ_JOINT_STATE_PUBLISHER_PLUGIN_FILENAME, "name": GZ_JOINT_STATE_PUBLISHER_PLUGIN_NAME},
    )
    ET.SubElement(plugin, "topic").text = "joint_states"
    for name in joint_names:
        ET.SubElement(plugin, "joint_name").text = name
    return elem


# --- world SDF --------------------------------------------------------------


def generate_world_sdf(world_name: str = "empty") -> str:
    """Render a minimal, valid gz-sim SDF world: the three system plugins
    every gz-sim world needs to actually simulate and be interactively
    usable (unlike classic Gazebo, gz-sim does not auto-load these -- a
    world with no ``<plugin>`` tags will not step physics, and critically
    will not expose the ``/world/<world>/create`` service that
    ``ros_gz_sim``'s spawner (see `generate_spawn_launch`) depends on to add
    the robot at runtime), plus a ground plane and a sun light -- the
    standard minimal starting point shown in effectively every gz-sim
    tutorial and every bundled gz-sim example world.

    Verification: rather than author this from memory, its shape (system
    plugin filenames/names, ground-plane model structure, light element
    fields) is a trimmed-down copy of
    ``/opt/ros/lyrical/share/ros_gz_sim_demos/worlds/default.sdf`` --  a
    real, gz-sim-native (SDF 1.8) world file that ships with this machine's
    actual installed ``ros_gz_sim_demos`` package, read directly off disk in
    this session. This function drops that file's GUI-plugin block and
    atmosphere/scene tuning; Physics, UserCommands, and SceneBroadcaster are
    kept because they're load-bearing (stepping physics, spawning, and
    state broadcast/GUI sync respectively).

    The Sensors and Imu system plugins ARE included (unlike an earlier
    version of this function, which dropped them as "not needed for the
    minimal does-a-simple-robot-launch proof") -- confirmed load-bearing for
    real: fusion_addin/generators/sensors.py's generated <sensor> elements
    (camera/lidar/imu) do not actually produce any live Gazebo Transport
    topics without gz-sim-sensors-system loaded in the world (confirmed via
    a live gz-sim headless run: before/after topic list comparison), and an
    IMU sensor specifically additionally needs gz-sim-imu-system (confirmed
    the same way). Both plugins' exact filename/name and the Sensors
    plugin's <render_engine>ogre2</render_engine> child are copied verbatim
    from the same real installed default.sdf referenced above. Contact and
    AirPressure are still dropped -- nothing this project generates uses
    either.

    Real-run confirmation: this exact template was written to a temp file
    and run for real in this session as
    ``gz sim -s -r --iterations 5 <file>.sdf`` (headless, `gz` binary at
    ``/opt/ros/lyrical/opt/gz_tools_vendor/bin/gz``, gz-sim 10.4.0) and
    exited 0 with no errors.
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <world name="{world_name}">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <gravity>0 0 -9.8</gravity>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""


# --- spawn launch file -------------------------------------------------


def generate_spawn_launch(robot: Robot, include_bridge: bool = False) -> str:
    """Render a ``launch``/``launch_ros`` Python launch file that:

    1. Includes ``ros_gz_sim``'s own ``gz_sim.launch.py`` to start the gz-sim
       server+GUI against this package's generated world file (see
       `generate_world_sdf`), via that launch file's documented ``gz_args``
       launch argument.
    2. Starts ``robot_state_publisher`` against the robot's xacro-processed
       URDF (same ``Command(["xacro ", ...])`` pattern already used by
       ``fusion_addin/generators/package.py``'s ``display.launch.py``, for
       consistency within the generated package).
    3. Spawns the robot into the running simulation using ``ros_gz_sim``'s
       ``create`` node, reading the URDF from the live ``/robot_description``
       topic ``robot_state_publisher`` just published -- the documented,
       supported way to spawn from a running RSP rather than a second file
       read (``-topic`` in ``create``'s own ``--help`` output: "Load XML from
       a ROS string publisher").
    4. If ``include_bridge`` (opt-in; the caller, ``app.py``, passes True
       exactly when it actually wrote a non-empty
       ``config/ros_gz_bridge.yaml``), also starts ``ros_gz_bridge``'s
       ``parameter_bridge`` node against that file -- confirmed the real,
       documented parameter name for a YAML bridge list via that
       executable's own ``--help`` (see
       ``fusion_addin/generators/sensors.py``'s
       ``generate_ros_gz_bridge_yaml`` docstring). Before this, the bridge
       config was written to disk but nothing ever started the bridge
       process against it -- a real, previously-undiscovered gap: even the
       pre-existing sensor bridging silently required a manual
       ``ros2 run ros_gz_bridge parameter_bridge`` invocation the generated
       package never mentioned or automated.

    Verification, all against this machine's actually-installed `ros_gz_sim`
    package (``/opt/ros/lyrical/share/ros_gz_sim``, confirmed present via
    ``ros2 pkg list``) rather than assumed from classic-Gazebo
    ``gazebo_ros``/``spawn_entity.py`` memory:

    - ``ros_gz_sim/launch/gz_sim.launch.py`` (read directly off disk in this
      session) is the real, shipped launch file for starting gz-sim with a
      ``gz_args`` string (e.g. ``"-r <world>.sdf"``) -- there is no separate
      "gzserver"/"gzclient" pair the way classic Gazebo had.
    - The executable is genuinely named ``create`` in package ``ros_gz_sim``
      (``/opt/ros/lyrical/lib/ros_gz_sim/create`` exists on disk; its
      ``--help`` output, captured live in this session, lists ``-topic``,
      ``-name``, ``-world``, ``-x/-y/-z/-R/-P/-Y`` as real flags -- there is
      no ``spawn_entity.py``/``gazebo_ros`` equivalent in this stack).
    - The exact ``Node(package="ros_gz_sim", executable="create",
      parameters=[{{"name": ..., "topic": "/robot_description"}}])`` shape
      (parameters, not CLI arguments) is copied from
      ``ros_gz_sim_demos``'s own installed
      ``launch/robot_description_publisher.launch.py`` example, read
      directly off disk in this session -- the one ``ros_gz_sim_demos``
      example that spawns specifically from a live ``robot_state_publisher``
      topic rather than a static file, matching this use case exactly.
    """
    package_name = robot.name
    xacro_filename = f"{robot.name}.urdf.xacro"
    world_filename = "empty.sdf"
    entity_name = robot.name

    bridge_node_def = (
        '''
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": PathJoinSubstitution([pkg_share, "config", "ros_gz_bridge.yaml"])}],
    )
'''
        if include_bridge
        else ""
    )
    bridge_launch_entry = "\n        bridge_node," if include_bridge else ""

    return f'''"""Gazebo Sim (gz-sim) spawn launch file for the "{package_name}" robot.

Auto-generated by Fusion2ROS (fusion_addin/generators/gazebo.py).
Regenerate rather than editing by hand.

Starts gz-sim (via ros_gz_sim's gz_sim.launch.py) against this package's
worlds/{world_filename}, publishes /robot_description via
robot_state_publisher (xacro-processed at launch time), and spawns the robot
into the running simulation from that live topic using ros_gz_sim's `create`
node -- see generate_spawn_launch's docstring in
fusion_addin/generators/gazebo.py for exactly how each of those names was
confirmed real on this machine's ROS 2 "lyrical" / gz-sim 10.4.0 install.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "{package_name}"
XACRO_FILE = "{xacro_filename}"
WORLD_FILE = "{world_filename}"
ENTITY_NAME = "{entity_name}"


def generate_launch_description():
    pkg_share = FindPackageShare(PACKAGE_NAME)
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    xacro_path = PathJoinSubstitution([pkg_share, "urdf", XACRO_FILE])
    world_path = PathJoinSubstitution([pkg_share, "worlds", WORLD_FILE])
    gz_sim_launch_path = PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_sim.launch.py"])

    # Processed via the `xacro` command-line filter at launch time rather
    # than the `xacro` Python module, so this launch file has no import-time
    # dependency on xacro being importable in the launching interpreter --
    # same convention as this package's display.launch.py.
    robot_description = {{
        "robot_description": Command(["xacro ", xacro_path])
    }}

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch_path),
        launch_arguments={{"gz_args": [TextSubstitution(text="-r "), world_path]}}.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_{package_name}",
        output="screen",
        parameters=[{{
            "name": ENTITY_NAME,
            "topic": "/robot_description",
            "allow_renaming": True,
        }}],
    )
{bridge_node_def}
    return LaunchDescription([
        gz_sim,
        robot_state_publisher_node,
        spawn_node,{bridge_launch_entry}
    ])
'''


# --- generate_gazebo_ros_bridge_yaml ----------------------------------------


def generate_gazebo_ros_bridge_yaml(robot: Robot) -> str:
    """Render the `ros_gz_bridge` `parameter_bridge` YAML entries needed for
    the plugins `generate_gazebo_xml` emits -- joint_states (whenever `robot`
    has any non-fixed joint) and, for a "differential_drive" `robot`,
    cmd_vel/odom/tf for the native `DiffDrive` plugin. Same flat
    `- ros_topic_name: ...` list style (no `[...]` wrapper) as
    `fusion_addin/generators/sensors.py`'s `generate_ros_gz_bridge_yaml`, so
    `app.py` can string-concatenate this with that module's sensor entries
    into one combined bridge config file.

    Returns `""` (not `sensors.py`'s empty-list `"[]\\n"`) when there is
    nothing to bridge -- concatenating `""` with another module's entries
    (or with nothing at all) stays valid YAML either way, whereas
    concatenating `"[]\\n"` with real entries would not.

    Message-type pairings are the same real, installed-file-sourced ones
    cited in the module-level `GZ_DIFF_DRIVE_PLUGIN_*`/
    `GZ_JOINT_STATE_PUBLISHER_PLUGIN_*` comment above.
    """
    entries: List[Dict[str, str]] = []

    if _moving_joint_names(robot):
        entries.append(
            {
                "ros_topic_name": "joint_states",
                "gz_topic_name": "joint_states",
                "ros_type_name": "sensor_msgs/msg/JointState",
                "gz_type_name": "gz.msgs.Model",
                "direction": "GZ_TO_ROS",
            }
        )

    if _is_differential_drive(robot):
        entries.append(
            {
                "ros_topic_name": "cmd_vel",
                "gz_topic_name": "cmd_vel",
                "ros_type_name": "geometry_msgs/msg/TwistStamped",
                "gz_type_name": "gz.msgs.Twist",
                "direction": "ROS_TO_GZ",
            }
        )
        entries.append(
            {
                "ros_topic_name": "odom",
                "gz_topic_name": "odom",
                "ros_type_name": "nav_msgs/msg/Odometry",
                "gz_type_name": "gz.msgs.Odometry",
                "direction": "GZ_TO_ROS",
            }
        )
        entries.append(
            {
                "ros_topic_name": "tf",
                "gz_topic_name": "tf",
                "ros_type_name": "tf2_msgs/msg/TFMessage",
                "gz_type_name": "gz.msgs.Pose_V",
                "direction": "GZ_TO_ROS",
            }
        )

    if not entries:
        return ""

    lines = []
    for entry in entries:
        lines.append(f"- ros_topic_name: \"{entry['ros_topic_name']}\"")
        lines.append(f"  gz_topic_name: \"{entry['gz_topic_name']}\"")
        lines.append(f"  ros_type_name: \"{entry['ros_type_name']}\"")
        lines.append(f"  gz_type_name: \"{entry['gz_type_name']}\"")
        lines.append(f"  direction: {entry['direction']}")
    return "\n".join(lines) + "\n"


# --- shared helpers ----------------------------------------------------


def _fmt_float(value: float) -> str:
    """Deterministic, non-scientific-notation float formatting -- same
    fixed-point-then-trim scheme as urdf.py's `_fmt_float`, reimplemented
    locally rather than imported to keep this module decoupled (see module
    docstring)."""
    s = f"{float(value):.8f}"
    if "." in s:
        s = s.rstrip("0")
        if s.endswith("."):
            s += "0"
    return s


def _fmt_vec(values) -> str:
    return " ".join(_fmt_float(v) for v in values)
