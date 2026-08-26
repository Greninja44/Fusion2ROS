"""RobotModel `Sensor` objects -> real Gazebo Sim (gz-sim) sensor XML plus a
`ros_gz_bridge` parameter-bridge YAML config.

Produces two SEPARATE pieces of text, mirroring the split already used by
`fusion_addin/generators/gazebo.py` (the parallel, robot-level Gazebo
generator this module intentionally does not duplicate or import from, to
stay decoupled -- same convention gazebo.py itself documents against
urdf.py/ros2_control.py):

1. `generate_sensor_gazebo_xml` -- a `<gazebo_fragment>`-wrapped XML string
   carrying one `<gazebo reference="{sensor.parent_link}">` block per
   `Sensor` in `robot.sensors`, each containing a real gz-sim `<sensor>`
   element. Uses the SAME `<gazebo_fragment>` wrapper tag gazebo.py uses, so
   `fusion_addin/app.py`'s existing `splice_xml_fragments` unwraps it (and
   only it -- unrecognized tags are inserted as-is) without any change to
   that function.
2. `generate_ros_gz_bridge_yaml` -- a `ros_gz_bridge` `parameter_bridge`
   config (a YAML list of `{ros_topic_name, gz_topic_name, ros_type_name,
   gz_type_name, direction}` bridge entries) mapping each sensor's gz-sim
   transport topic(s) to a ROS 2 topic of the matching message type.

Pure function of a `robot_model.Robot` plus stdlib string templating: no
Fusion API, no ROS/rclpy imports, no third-party packages (no `yaml`
import -- the YAML text is hand-built, same convention
`fusion_addin/generators/ros2_control.py`'s `generate_controllers_yaml`
uses), no filesystem access, no network. Safe to import and call from
Fusion's embedded interpreter, from plain WSL `python3`, or from CI with
neither Fusion nor ROS installed.

This project's ROS 2 install ("lyrical") ships Gazebo Sim (gz-sim 10.4.0,
confirmed via `gz sim --version` and `ros2 pkg list | grep -i ros_gz` in
this session) plus `ros_gz_bridge`/`ros_gz_image` -- NOT classic Gazebo.
Every SDF sensor tag, gz message type string, and the `ros_gz_bridge` YAML
schema itself were confirmed against real, installed, on-disk artifacts on
this machine rather than assumed from memory or classic-Gazebo convention;
see the per-section comments below for exactly which file was read for each
claim.

Sensor `parameters` dict shape (documented here since `robot_model.Sensor`
deliberately leaves it an open `Dict[str, object]` -- see
`robot_model/schema.py`)
----------------------------------------------------------------------------
All keys optional; every default below is itself lifted from a real,
installed gz-sim example world file (cited inline) rather than invented.

`type == "camera"`:
    horizontal_fov  float, radians   default 1.047   (~60 deg)
    width           int, pixels      default 640
    height          int, pixels      default 480
    format          str              default "R8G8B8"
    near            float, meters    default 0.1
    far             float, meters    default 100.0
    update_rate     float, Hz        default 30.0

`type in ("lidar", "gpu_lidar")` (both map to gz-sim's real `gpu_lidar` SDF
sensor type -- see the LIDAR section below for why there is no separate
plain-"lidar" gz-sim sensor type worth emitting):
    horizontal_samples     int            default 640
    horizontal_min_angle   float, rad     default -3.141592653589793 (-pi)
    horizontal_max_angle   float, rad     default 3.141592653589793 (pi)
    vertical_samples       int            default 1   (1 => 2D scanning
                                           lidar: the <vertical> block is
                                           omitted entirely, matching real
                                           gz-sim 2D lidar SDF -- see below)
    vertical_min_angle     float, rad     default 0.0
    vertical_max_angle     float, rad     default 0.0
    range_min              float, meters  default 0.08
    range_max              float, meters  default 10.0
    range_resolution       float, meters  default 0.01
    update_rate            float, Hz      default 10.0

`type == "imu"`:
    update_rate     float, Hz   default 100.0

Any other `Sensor.type` raises `ValueError` naming the sensor and its
unrecognized type -- this module does not invent XML for a sensor kind it
hasn't verified against a real gz-sim tag shape, same "don't invent, fail
clearly" principle `fusion_addin/generators/urdf.py` uses for missing joint
limits.

Known limitation, found via real end-to-end verification (see below): a
world SDF needs the `gz-sim-sensors-system` (`gz::sim::systems::Sensors`)
world plugin loaded for ANY sensor here to publish at all, and the "imu"
type specifically ALSO needs a separate `gz-sim-imu-system`
(`gz::sim::systems::Imu`) world plugin -- without it the camera/lidar
topics come up fine but the IMU sensor never advertises its topic at all
(confirmed by reproducing exactly that split live: `gz topic -l` showed
`/front_camera/image` and `/main_lidar` but no `/body_imu` until the Imu
system plugin was added to the test world, after which all three came up).
`fusion_addin/generators/gazebo.py`'s `generate_world_sdf` -- out of this
module's scope to touch or duplicate -- does not currently load either
plugin (by its own docstring, it deliberately trimmed "extra sensor-support
plugins" for its own minimal "does a robot come up" proof); a caller who
wants `robot.sensors` to actually produce live gz topics needs to add both
plugins to whatever world SDF the robot is spawned into.

Real, live, end-to-end verification performed in this session (not just
unit tests): a hand-built two-link, three-sensor `Robot` was run through
`generate_urdf_xacro` + this module's `generate_sensor_gazebo_xml`,
spliced via `fusion_addin/app.py`'s `splice_xml_fragments`, and the result
both passed `check_urdf` and was converted to real SDF via `gz sdf
--print` (sdformat's own URDF importer) with zero warnings -- producing a
`<sensor>` block per type that visually matches every citation above. That
converted model was embedded in a hand-written world SDF carrying the two
plugins named above, loaded for real via `gz sim -s -r` (gz-sim 10.4.0,
headless), and `gz topic -l` showed all five expected gz topics
(`/front_camera/image`, `/front_camera/camera_info`, `/main_lidar`,
`/main_lidar/points`, `/body_imu`) publishing real data (`gz topic -e`
showed live IMU orientation/lidar range readings). `ros_gz_bridge`'s
`parameter_bridge` was then started for real against this module's own
`generate_ros_gz_bridge_yaml` output, and `ros2 topic list` /
`ros2 topic info` / `ros2 topic echo` confirmed all five ROS 2 topics came
up with exactly the documented message types and live data flowing through
end to end.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

from robot_model import Robot, Sensor

__all__ = ["generate_sensor_gazebo_xml", "generate_ros_gz_bridge_yaml"]


# --- topic convention ---------------------------------------------------
#
# Every sensor's own <topic> is "/{sensor.name}" -- EXCEPT camera, which uses
# "/{sensor.name}/image" specifically because of a real gz-sensors quirk
# (see _camera_info_topic's docstring below): a bare "/{name}" main topic
# would make gz-sensors' CameraSensor publish camera_info at the *global*
# "/camera_info" gz topic (colliding across every camera on the robot),
# whereas a "/{name}/image" main topic makes it publish camera_info at
# "/{name}/camera_info" (unique per sensor). This is the one place this
# module's topic choice is NOT simply "/{sensor.name}" -- documented here
# and re-stated at each call site below.


def _default_topic(sensor: Sensor) -> str:
    return f"/{sensor.name}"


def _camera_image_topic(sensor: Sensor) -> str:
    return f"/{sensor.name}/image"


def _camera_info_topic(sensor: Sensor) -> str:
    """gz-sensors' CameraSensor derives its camera_info topic from the main
    image topic by splitting the topic on '/', dropping the LAST path
    component, and appending "/camera_info" to what's left (verified by
    reading gz-sensors' CameraSensor.cc, `gz-sensors9` branch, live in this
    session -- the exact split/pop_back/rebuild logic is reproduced in this
    function's return value). For a main topic of "/{name}" (a single path
    component) that rule collapses to the *global* "/camera_info" -- which
    is why `_camera_image_topic` uses "/{name}/image" instead: dropping the
    last component ("image") leaves "/{name}", giving a per-sensor
    "/{name}/camera_info" that can't collide with a second camera on the
    same robot.
    """
    return f"/{sensor.name}/camera_info"


def _lidar_points_topic(sensor: Sensor) -> str:
    """gz-sensors' GpuLidarSensor always additionally publishes a
    PointCloudPacked cloud at `<main topic> + "/points"` (verified by
    reading gz-sensors' GpuLidarSensor.cc, `gz-sensors9` branch, live in
    this session: `this->SetTopic(this->Topic() + "/points")` right before
    advertising the `gz::msgs::PointCloudPacked` publisher) -- in addition
    to, not instead of, the base LaserScan-shaped topic gz-sensors' parent
    `Lidar` class advertises at the sensor's own `<topic>` (also verified
    live, in `Lidar.cc`: `Advertise<gz::msgs::LaserScan>(this->Topic())`).
    So every lidar/gpu_lidar sensor here gets bridged twice: once for the
    LaserScan on `/{name}`, once for the PointCloud2 on `/{name}/points`.
    """
    return f"{_default_topic(sensor)}/points"


# --- generate_sensor_gazebo_xml ------------------------------------------


def generate_sensor_gazebo_xml(robot: Robot) -> str:
    """Render a `<gazebo_fragment>` XML string carrying one
    `<gazebo reference="{sensor.parent_link}">` block per `Sensor` in
    `robot.sensors`, each wrapping a real gz-sim `<sensor>` element.

    Returns an empty-but-valid `<gazebo_fragment />` (no children) when
    `robot.sensors` is empty -- not an error, since "no sensors" is a
    perfectly ordinary robot.

    Raises `ValueError` (naming the offending sensor and its `type`) for any
    `Sensor.type` other than "camera", "lidar", "gpu_lidar", or "imu" --
    this module does not guess at gz-sim tag shapes it hasn't verified.

    Does not call `robot.validate()` -- consistent with `gazebo.py`'s
    `generate_gazebo_xml`, this only reads `robot.sensors`, so a robot
    that's invalid for unrelated reasons doesn't block sensor rendering.
    Callers that need full-pipeline validation already get it from
    `generate_urdf_xacro`.
    """
    fragment = ET.Element("gazebo_fragment")

    for sensor in robot.sensors:
        gazebo_elem = ET.SubElement(fragment, "gazebo", {"reference": sensor.parent_link})
        gazebo_elem.append(_build_sensor_element(sensor))

    ET.indent(fragment, space="  ")
    body = ET.tostring(fragment, encoding="unicode")
    return f"{body}\n"


def _build_sensor_element(sensor: Sensor) -> ET.Element:
    if sensor.type == "camera":
        return _build_camera_sensor(sensor)
    if sensor.type in ("lidar", "gpu_lidar"):
        return _build_lidar_sensor(sensor)
    if sensor.type == "imu":
        return _build_imu_sensor(sensor)
    raise ValueError(
        f"Sensor {sensor.name!r} has unrecognized type {sensor.type!r} -- "
        "generate_sensor_gazebo_xml only knows how to emit gz-sim XML for "
        "'camera', 'lidar'/'gpu_lidar', and 'imu' (see sensors.py's module "
        "docstring for the documented parameters dict shape each expects)."
    )


def _common_sensor_attrs_and_children(sensor: Sensor, sdf_type: str, topic: str, default_update_rate: float):
    """Shared `<sensor name=... type=...>` element plus its `<pose>`,
    `<always_on>`, `<update_rate>`, `<topic>` children -- every gz-sim
    `<sensor>` example read in this session (e.g.
    `/opt/ros/lyrical/opt/gz_sim_vendor/share/gz/gz-sim/worlds/sensors_demo.sdf`,
    `.../gpu_lidar_sensor.sdf`, `.../sensors.sdf`'s "imu" block) carries
    exactly this shape before its type-specific body."""
    elem = ET.Element("sensor", {"name": sensor.name, "type": sdf_type})
    ET.SubElement(elem, "pose").text = _fmt_pose(sensor.origin)
    ET.SubElement(elem, "always_on").text = "1"
    ET.SubElement(elem, "update_rate").text = _fmt_float(sensor.parameters.get("update_rate", default_update_rate))
    ET.SubElement(elem, "topic").text = topic
    return elem


def _build_camera_sensor(sensor: Sensor) -> ET.Element:
    """<sensor type="camera"> shape confirmed against
    `.../gz-sim/worlds/sensors_demo.sdf`'s `cameras_alone`/`camera` sensors
    (read live in this session): `<camera><horizontal_fov>`,
    `<image><width>/<height>`, `<clip><near>/<far>`, plus `<always_on>`,
    `<update_rate>`, `<topic>` at the `<sensor>` level. `<format>` under
    `<image>` confirmed via the same directory's
    `visualize_frustum_rgb_camera.sdf` ("R8G8B8") and `depth_camera_sensor.sdf`
    ("R_FLOAT32")."""
    p = sensor.parameters
    elem = _common_sensor_attrs_and_children(
        sensor, "camera", _camera_image_topic(sensor), default_update_rate=30.0
    )

    camera = ET.SubElement(elem, "camera")
    ET.SubElement(camera, "horizontal_fov").text = _fmt_float(p.get("horizontal_fov", 1.047))
    image = ET.SubElement(camera, "image")
    ET.SubElement(image, "width").text = str(int(p.get("width", 640)))
    ET.SubElement(image, "height").text = str(int(p.get("height", 480)))
    ET.SubElement(image, "format").text = str(p.get("format", "R8G8B8"))
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = _fmt_float(p.get("near", 0.1))
    ET.SubElement(clip, "far").text = _fmt_float(p.get("far", 100.0))

    return elem


def _build_lidar_sensor(sensor: Sensor) -> ET.Element:
    """<sensor type="gpu_lidar"> shape (both robot_model "lidar" and
    "gpu_lidar" sensor types map to this -- gz-sim's real, GPU-accelerated
    lidar sensor; there is no distinct plain-"lidar" SDF sensor type used in
    any installed gz-sim example world found in this session, so treating
    the two robot_model spellings as synonyms for the one real tag is a
    documented simplification, not a guess at an unverified second tag)
    confirmed against `.../gz-sim/worlds/gpu_lidar_sensor.sdf` (3D: has a
    `<vertical>` scan block) and `.../export_occupancy_grid.sdf` (2D: the
    `<vertical>` block is entirely absent) -- both read live in this
    session. `<horizontal>`/`<vertical>` each carry `<samples>/<resolution>/
    <min_angle>/<max_angle>`; `<range>` carries `<min>/<max>/<resolution>`.
    """
    p = sensor.parameters
    elem = _common_sensor_attrs_and_children(sensor, "gpu_lidar", _default_topic(sensor), default_update_rate=10.0)

    lidar = ET.SubElement(elem, "lidar")
    scan = ET.SubElement(lidar, "scan")
    horizontal = ET.SubElement(scan, "horizontal")
    ET.SubElement(horizontal, "samples").text = str(int(p.get("horizontal_samples", 640)))
    ET.SubElement(horizontal, "resolution").text = _fmt_float(1.0)
    ET.SubElement(horizontal, "min_angle").text = _fmt_float(p.get("horizontal_min_angle", -3.141592653589793))
    ET.SubElement(horizontal, "max_angle").text = _fmt_float(p.get("horizontal_max_angle", 3.141592653589793))

    vertical_samples = int(p.get("vertical_samples", 1))
    if vertical_samples > 1:
        vertical = ET.SubElement(scan, "vertical")
        ET.SubElement(vertical, "samples").text = str(vertical_samples)
        ET.SubElement(vertical, "resolution").text = _fmt_float(1.0)
        ET.SubElement(vertical, "min_angle").text = _fmt_float(p.get("vertical_min_angle", 0.0))
        ET.SubElement(vertical, "max_angle").text = _fmt_float(p.get("vertical_max_angle", 0.0))

    range_elem = ET.SubElement(lidar, "range")
    ET.SubElement(range_elem, "min").text = _fmt_float(p.get("range_min", 0.08))
    ET.SubElement(range_elem, "max").text = _fmt_float(p.get("range_max", 10.0))
    ET.SubElement(range_elem, "resolution").text = _fmt_float(p.get("range_resolution", 0.01))

    return elem


def _build_imu_sensor(sensor: Sensor) -> ET.Element:
    """<sensor type="imu"> shape confirmed against
    `.../gz-sim/worlds/sensors.sdf`'s "imu" sensor and `track_drive.sdf`'s
    "imu_sensor" (both read live in this session): just `<always_on>`,
    `<update_rate>`, `<topic>` at the `<sensor>` level -- gz-sim's IMU
    sensor needs no required type-specific child element (the optional
    `<imu>` element exists only for noise-model tuning, which RobotModel's
    `Sensor.parameters` doesn't carry a documented shape for here, so it is
    correctly omitted rather than guessed at)."""
    return _common_sensor_attrs_and_children(sensor, "imu", _default_topic(sensor), default_update_rate=100.0)


# --- generate_ros_gz_bridge_yaml ------------------------------------------


def generate_ros_gz_bridge_yaml(robot: Robot) -> str:
    """Render a `ros_gz_bridge` `parameter_bridge` YAML config: a top-level
    YAML list, one entry per `{ros_topic_name, gz_topic_name, ros_type_name,
    gz_type_name, direction}` bridge, for every `Sensor` in `robot.sensors`.

    Schema confirmed against this machine's actually-installed
    `ros_gz_bridge` C++ header,
    `/opt/ros/lyrical/include/ros_gz_bridge/ros_gz_bridge/bridge_config.hpp`
    (its `BridgeConfig` struct's field names are exactly
    `ros_type_name`/`ros_topic_name`/`gz_type_name`/`gz_topic_name`/
    `direction`, and `BridgeDirection` includes `GZ_TO_ROS`), read live in
    this session -- and cross-checked against two real, installed example
    bridge YAML files using that exact key set:
    `/opt/ros/lyrical/share/ros_gz_sim_demos/config/rgbd_camera_bridge.yaml`
    and
    `/opt/ros/lyrical/share/turtlebot3_gazebo/params/turtlebot3_waffle_bridge.yaml`.
    Direction is `GZ_TO_ROS` for every entry here since all of these are
    sensor readings flowing out of simulation into ROS (never the reverse).

    Per-sensor-type topic/message-type mapping (gz message type strings
    themselves copied verbatim from the two files above, e.g. `gz.msgs.IMU`
    -- note the all-caps "IMU", confirmed from
    `turtlebot3_waffle_bridge.yaml` line 40, NOT "gz.msgs.Imu"):

    - "camera": TWO entries -- `sensor_msgs/msg/Image` <-> `gz.msgs.Image`
      on `/{name}/image`, and `sensor_msgs/msg/CameraInfo` <->
      `gz.msgs.CameraInfo` on `/{name}/camera_info` (the camera_info topic
      gz-sensors itself derives from the image topic -- see
      `_camera_info_topic`'s docstring in this module for the exact rule
      and its citation).
    - "lidar"/"gpu_lidar": TWO entries -- `sensor_msgs/msg/LaserScan` <->
      `gz.msgs.LaserScan` on `/{name}` (gz-sensors' `Lidar` base class's own
      main topic), and `sensor_msgs/msg/PointCloud2` <->
      `gz.msgs.PointCloudPacked` on `/{name}/points` (gz-sensors'
      `GpuLidarSensor` ALWAYS additionally publishes this -- see
      `_lidar_points_topic`'s docstring for the citation; this is not a
      2D-vs-3D choice, both are bridged unconditionally).
    - "imu": ONE entry -- `sensor_msgs/msg/Imu` <-> `gz.msgs.IMU` on
      `/{name}`.

    Raises the same `ValueError` as `generate_sensor_gazebo_xml` for any
    unrecognized `Sensor.type`.

    Returns an empty-but-valid `[]\\n` (a valid, empty YAML list) when
    `robot.sensors` is empty.

    To actually run the bridge against this config once it's written to
    disk (e.g. at a generated package's `config/ros_gz_bridge.yaml`):

        ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=<path>

    (`parameter_bridge`'s own `--help`, and `ros_gz_bridge`'s installed
    `launch/ros_gz_bridge.launch.py`, both confirm `config_file` is the real,
    documented parameter name for a YAML bridge list -- as opposed to that
    same executable's older positional `<gz_topic>@<ros_type>@<gz_type>` CLI
    form, which this module deliberately does not use since it can't express
    `GZ_TO_ROS`-only direction or an explicit `ros_topic_name` distinct from
    the gz topic name the way the YAML form can).
    """
    entries: List[Dict[str, str]] = []
    for sensor in robot.sensors:
        entries.extend(_bridge_entries_for_sensor(sensor))

    if not entries:
        return "[]\n"

    lines = []
    for entry in entries:
        lines.append(f"- ros_topic_name: \"{entry['ros_topic_name']}\"")
        lines.append(f"  gz_topic_name: \"{entry['gz_topic_name']}\"")
        lines.append(f"  ros_type_name: \"{entry['ros_type_name']}\"")
        lines.append(f"  gz_type_name: \"{entry['gz_type_name']}\"")
        lines.append(f"  direction: {entry['direction']}")
    return "\n".join(lines) + "\n"


def _bridge_entries_for_sensor(sensor: Sensor) -> List[Tuple[str, str, str, str, str]]:
    if sensor.type == "camera":
        image_topic = _camera_image_topic(sensor)
        info_topic = _camera_info_topic(sensor)
        return [
            _bridge_entry(image_topic, "sensor_msgs/msg/Image", "gz.msgs.Image"),
            _bridge_entry(info_topic, "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        ]
    if sensor.type in ("lidar", "gpu_lidar"):
        scan_topic = _default_topic(sensor)
        points_topic = _lidar_points_topic(sensor)
        return [
            _bridge_entry(scan_topic, "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
            _bridge_entry(points_topic, "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"),
        ]
    if sensor.type == "imu":
        topic = _default_topic(sensor)
        return [_bridge_entry(topic, "sensor_msgs/msg/Imu", "gz.msgs.IMU")]
    raise ValueError(
        f"Sensor {sensor.name!r} has unrecognized type {sensor.type!r} -- "
        "generate_ros_gz_bridge_yaml only knows how to bridge 'camera', "
        "'lidar'/'gpu_lidar', and 'imu'."
    )


def _bridge_entry(topic: str, ros_type_name: str, gz_type_name: str) -> Dict[str, str]:
    # ROS and gz topic names are the same string here -- there's no reason
    # for them to differ, and keeping them identical is what makes this
    # config's topic names match generate_sensor_gazebo_xml's <topic>
    # values (and each other's _camera_image_topic/_lidar_points_topic/
    # _default_topic helper) exactly, by construction.
    return {
        "ros_topic_name": topic,
        "gz_topic_name": topic,
        "ros_type_name": ros_type_name,
        "gz_type_name": gz_type_name,
        "direction": "GZ_TO_ROS",
    }


# --- shared helpers -------------------------------------------------------


def _fmt_float(value) -> str:
    """Deterministic, non-scientific-notation float formatting -- same
    fixed-point-then-trim scheme as urdf.py's/gazebo.py's `_fmt_float`,
    reimplemented locally rather than imported to keep this module
    decoupled (see module docstring)."""
    s = f"{float(value):.8f}"
    if "." in s:
        s = s.rstrip("0")
        if s.endswith("."):
            s += "0"
    return s


def _fmt_pose(pose) -> str:
    """SDF's <pose> element is a single space-separated "x y z roll pitch
    yaw" string (roll/pitch/yaw in radians) -- confirmed against every
    <pose> in every gz-sim example world read in this session, e.g.
    `track_drive.sdf`'s `<pose frame=''>0 0.1985 0 0 -0 0</pose>`."""
    values = list(pose.xyz) + list(pose.rpy)
    return " ".join(_fmt_float(v) for v in values)
