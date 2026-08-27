"""Tests for fusion_addin.generators.bringup.generate_bringup_launch."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.generators.bringup import generate_bringup_launch
from robot_model import Inertial, Link, Robot


def make_robot(name="demo_bot"):
    return Robot(name=name, links=[Link(name="base_link", inertial=Inertial(mass=1.0, ixx=0.01, iyy=0.01, izz=0.01))])


def test_returns_none_when_neither_nav2_nor_moveit():
    robot = make_robot()
    assert generate_bringup_launch(robot, True, True, False, False) is None
    assert generate_bringup_launch(robot, False, False, False, False) is None


def test_gazebo_is_the_base_when_both_gazebo_and_ros2_control_set():
    robot = make_robot()
    result = generate_bringup_launch(robot, True, True, False, True)
    assert '"gazebo.launch.py"' in result
    assert '"control.launch.py"' not in result


def test_ros2_control_is_the_base_without_gazebo():
    robot = make_robot()
    result = generate_bringup_launch(robot, True, False, False, True)
    assert '"control.launch.py"' in result
    assert '"gazebo.launch.py"' not in result


def test_display_is_the_fallback_base():
    robot = make_robot()
    result = generate_bringup_launch(robot, False, False, True, False)
    assert '"display.launch.py"' in result
    assert '"gazebo.launch.py"' not in result
    assert '"control.launch.py"' not in result


def test_nav2_and_moveit_both_included_when_both_requested():
    robot = make_robot()
    result = generate_bringup_launch(robot, False, True, True, True)
    assert '"gazebo.launch.py"' in result
    assert '"nav2_bringup.launch.py"' in result
    assert '"moveit_demo.launch.py"' in result


def test_only_requested_stack_is_included():
    robot = make_robot()
    nav2_only = generate_bringup_launch(robot, False, True, False, True)
    assert '"nav2_bringup.launch.py"' in nav2_only
    assert '"moveit_demo.launch.py"' not in nav2_only

    moveit_only = generate_bringup_launch(robot, False, True, True, False)
    assert '"moveit_demo.launch.py"' in moveit_only
    assert '"nav2_bringup.launch.py"' not in moveit_only


def test_package_name_matches_robot_name():
    robot = make_robot(name="my_cool_robot")
    result = generate_bringup_launch(robot, False, False, True, False)
    assert 'PACKAGE_NAME = "my_cool_robot"' in result


def test_generated_launch_compiles_for_every_relevant_flag_combination():
    robot = make_robot()
    for ros2_control in (False, True):
        for gazebo in (False, True):
            for moveit in (False, True):
                for nav2 in (False, True):
                    if not (moveit or nav2):
                        continue
                    result = generate_bringup_launch(robot, ros2_control, gazebo, moveit, nav2)
                    compile(result, "bringup.launch.py", "exec")
