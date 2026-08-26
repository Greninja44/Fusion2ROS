"""Tests for fusion_addin.generators.sensors.

Must run with plain `python3 -m pytest` -- no Fusion, no live ROS/Gazebo
needed. Follows the fixture/assertion style of tests/generators/test_gazebo.py.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.generators.sensors import generate_ros_gz_bridge_yaml, generate_sensor_gazebo_xml
from robot_model import Inertial, Link, Pose, Robot, Sensor


# --- fixtures ---------------------------------------------------------------


def make_robot_with_sensors() -> Robot:
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    camera_mount = Link(
        name="camera_mount", parent="base_link", inertial=Inertial(mass=0.1, ixx=0.001, iyy=0.001, izz=0.001)
    )
    lidar_mount = Link(
        name="lidar_mount", parent="base_link", inertial=Inertial(mass=0.1, ixx=0.001, iyy=0.001, izz=0.001)
    )

    from robot_model import Joint, JointType

    j1 = Joint(name="camera_mount_joint", type=JointType.FIXED, parent="base_link", child="camera_mount")
    j2 = Joint(name="lidar_mount_joint", type=JointType.FIXED, parent="base_link", child="lidar_mount")

    camera = Sensor(
        name="front_camera",
        type="camera",
        parent_link="camera_mount",
        origin=Pose(xyz=(0.1, 0.0, 0.05), rpy=(0.0, 0.0, 0.0)),
        parameters={"horizontal_fov": 1.5, "width": 800, "height": 600},
    )
    lidar = Sensor(
        name="main_lidar",
        type="lidar",
        parent_link="lidar_mount",
        origin=Pose(xyz=(0.0, 0.0, 0.1), rpy=(0.0, 0.0, 0.0)),
        parameters={"horizontal_samples": 360},
    )
    imu = Sensor(
        name="body_imu",
        type="imu",
        parent_link="base_link",
        origin=Pose.IDENTITY,
    )

    return Robot(
        name="sensor_bot",
        links=[base, camera_mount, lidar_mount],
        joints=[j1, j2],
        sensors=[camera, lidar, imu],
    )


def make_robot_with_gpu_lidar_3d() -> Robot:
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    lidar = Sensor(
        name="depth_lidar",
        type="gpu_lidar",
        parent_link="base_link",
        parameters={"vertical_samples": 16, "vertical_min_angle": -0.2, "vertical_max_angle": 0.2},
    )
    return Robot(name="lidar3d_bot", links=[base], sensors=[lidar])


def make_robot_with_unrecognized_sensor() -> Robot:
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    weird = Sensor(name="mystery", type="tricorder", parent_link="base_link")
    return Robot(name="weird_bot", links=[base], sensors=[weird])


def make_robot_with_no_sensors() -> Robot:
    base = Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))
    return Robot(name="no_sensor_bot", links=[base])


# --- generate_sensor_gazebo_xml -------------------------------------------


def test_sensor_xml_parses_and_has_expected_root():
    robot = make_robot_with_sensors()
    root = ET.fromstring(generate_sensor_gazebo_xml(robot))
    assert root.tag == "gazebo_fragment"


def test_sensor_xml_one_gazebo_block_per_sensor_with_correct_reference():
    robot = make_robot_with_sensors()
    root = ET.fromstring(generate_sensor_gazebo_xml(robot))
    gazebo_blocks = root.findall("gazebo")
    assert len(gazebo_blocks) == 3
    refs = {g.attrib["reference"] for g in gazebo_blocks}
    assert refs == {"camera_mount", "lidar_mount", "base_link"}


def _find_sensor(root, name):
    for gazebo_block in root.findall("gazebo"):
        sensor = gazebo_block.find("sensor")
        if sensor is not None and sensor.attrib.get("name") == name:
            return sensor
    return None


def test_camera_sensor_shape():
    robot = make_robot_with_sensors()
    root = ET.fromstring(generate_sensor_gazebo_xml(robot))
    sensor = _find_sensor(root, "front_camera")
    assert sensor is not None
    assert sensor.attrib["type"] == "camera"

    pose = sensor.find("pose")
    assert pose is not None
    assert pose.text == "0.1 0.0 0.05 0.0 0.0 0.0"

    assert sensor.find("always_on").text == "1"
    assert sensor.find("update_rate") is not None
    topic = sensor.find("topic")
    assert topic.text == "/front_camera/image"

    camera = sensor.find("camera")
    assert camera is not None
    assert camera.find("horizontal_fov").text == "1.5"
    image = camera.find("image")
    assert image.find("width").text == "800"
    assert image.find("height").text == "600"
    assert image.find("format").text == "R8G8B8"
    clip = camera.find("clip")
    assert clip.find("near") is not None
    assert clip.find("far") is not None


def test_lidar_sensor_shape_2d_omits_vertical_block():
    robot = make_robot_with_sensors()
    root = ET.fromstring(generate_sensor_gazebo_xml(robot))
    sensor = _find_sensor(root, "main_lidar")
    assert sensor is not None
    assert sensor.attrib["type"] == "gpu_lidar"
    assert sensor.find("topic").text == "/main_lidar"

    lidar = sensor.find("lidar")
    assert lidar is not None
    scan = lidar.find("scan")
    horizontal = scan.find("horizontal")
    assert horizontal.find("samples").text == "360"
    assert horizontal.find("min_angle") is not None
    assert horizontal.find("max_angle") is not None
    # 2D lidar (default vertical_samples=1) -> no <vertical> block at all.
    assert scan.find("vertical") is None

    range_elem = lidar.find("range")
    assert range_elem.find("min") is not None
    assert range_elem.find("max") is not None
    assert range_elem.find("resolution") is not None


def test_lidar_sensor_3d_includes_vertical_block():
    robot = make_robot_with_gpu_lidar_3d()
    root = ET.fromstring(generate_sensor_gazebo_xml(robot))
    sensor = _find_sensor(root, "depth_lidar")
    scan = sensor.find("lidar").find("scan")
    vertical = scan.find("vertical")
    assert vertical is not None
    assert vertical.find("samples").text == "16"
    assert vertical.find("min_angle").text == "-0.2"
    assert vertical.find("max_angle").text == "0.2"


def test_imu_sensor_shape():
    robot = make_robot_with_sensors()
    root = ET.fromstring(generate_sensor_gazebo_xml(robot))
    sensor = _find_sensor(root, "body_imu")
    assert sensor is not None
    assert sensor.attrib["type"] == "imu"
    assert sensor.find("topic").text == "/body_imu"
    assert sensor.find("always_on").text == "1"
    assert sensor.find("update_rate").text == "100.0"


def test_unrecognized_sensor_type_raises_value_error_naming_sensor_and_type():
    robot = make_robot_with_unrecognized_sensor()
    with pytest.raises(ValueError, match="mystery"):
        generate_sensor_gazebo_xml(robot)
    with pytest.raises(ValueError, match="tricorder"):
        generate_sensor_gazebo_xml(robot)


def test_no_sensors_produces_empty_but_valid_fragment():
    robot = make_robot_with_no_sensors()
    text = generate_sensor_gazebo_xml(robot)
    root = ET.fromstring(text)
    assert root.tag == "gazebo_fragment"
    assert list(root) == []


def test_sensor_xml_is_deterministic():
    robot = make_robot_with_sensors()
    assert generate_sensor_gazebo_xml(robot) == generate_sensor_gazebo_xml(robot)


# --- generate_ros_gz_bridge_yaml -------------------------------------------


def test_bridge_yaml_parses_and_has_expected_entries():
    robot = make_robot_with_sensors()
    text = generate_ros_gz_bridge_yaml(robot)
    entries = yaml.safe_load(text)
    assert isinstance(entries, list)
    # camera -> 2 entries (image + camera_info), lidar -> 2 (scan + points),
    # imu -> 1.
    assert len(entries) == 5

    by_ros_topic = {e["ros_topic_name"]: e for e in entries}

    image_entry = by_ros_topic["/front_camera/image"]
    assert image_entry["gz_topic_name"] == "/front_camera/image"
    assert image_entry["ros_type_name"] == "sensor_msgs/msg/Image"
    assert image_entry["gz_type_name"] == "gz.msgs.Image"
    assert image_entry["direction"] == "GZ_TO_ROS"

    info_entry = by_ros_topic["/front_camera/camera_info"]
    assert info_entry["ros_type_name"] == "sensor_msgs/msg/CameraInfo"
    assert info_entry["gz_type_name"] == "gz.msgs.CameraInfo"

    scan_entry = by_ros_topic["/main_lidar"]
    assert scan_entry["ros_type_name"] == "sensor_msgs/msg/LaserScan"
    assert scan_entry["gz_type_name"] == "gz.msgs.LaserScan"

    points_entry = by_ros_topic["/main_lidar/points"]
    assert points_entry["ros_type_name"] == "sensor_msgs/msg/PointCloud2"
    assert points_entry["gz_type_name"] == "gz.msgs.PointCloudPacked"

    imu_entry = by_ros_topic["/body_imu"]
    assert imu_entry["ros_type_name"] == "sensor_msgs/msg/Imu"
    assert imu_entry["gz_type_name"] == "gz.msgs.IMU"

    # every entry direction is GZ_TO_ROS
    assert all(e["direction"] == "GZ_TO_ROS" for e in entries)


def test_bridge_yaml_topics_match_sensor_xml_topics():
    robot = make_robot_with_sensors()
    xml_root = ET.fromstring(generate_sensor_gazebo_xml(robot))
    yaml_entries = yaml.safe_load(generate_ros_gz_bridge_yaml(robot))

    camera_sensor = _find_sensor(xml_root, "front_camera")
    assert camera_sensor.find("topic").text == "/front_camera/image"
    assert any(e["ros_topic_name"] == "/front_camera/image" for e in yaml_entries)

    lidar_sensor = _find_sensor(xml_root, "main_lidar")
    assert lidar_sensor.find("topic").text == "/main_lidar"
    assert any(e["ros_topic_name"] == "/main_lidar" for e in yaml_entries)


def test_unrecognized_sensor_type_raises_in_bridge_yaml_too():
    robot = make_robot_with_unrecognized_sensor()
    with pytest.raises(ValueError, match="tricorder"):
        generate_ros_gz_bridge_yaml(robot)


def test_no_sensors_produces_empty_but_valid_yaml_list():
    robot = make_robot_with_no_sensors()
    text = generate_ros_gz_bridge_yaml(robot)
    entries = yaml.safe_load(text)
    assert entries == []


def test_bridge_yaml_is_deterministic():
    robot = make_robot_with_sensors()
    assert generate_ros_gz_bridge_yaml(robot) == generate_ros_gz_bridge_yaml(robot)
