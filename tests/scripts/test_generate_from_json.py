"""Tests for scripts/generate_from_json.py, the standalone (no-Fusion) CLI."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from robot_model import save_robot_json
from examples.sample_arm import build_sample_arm

import generate_from_json


def test_generates_package_from_json(tmp_path):
    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)

    exit_code = generate_from_json.main(
        [str(robot_json), "--output-dir", str(tmp_path / "out"), "--ros2-control", "--moveit"]
    )

    assert exit_code == 0
    package_dir = tmp_path / "out" / "sample_arm"
    assert (package_dir / "package.xml").exists()
    assert (package_dir / "config" / "controllers.yaml").exists()
    assert (package_dir / "config" / "ompl_planning.yaml").exists()


def test_missing_file_reports_error(tmp_path, capsys):
    exit_code = generate_from_json.main([str(tmp_path / "does_not_exist.json")])
    assert exit_code == 1
    assert "does not exist" in capsys.readouterr().err


def test_invalid_robot_reports_error(tmp_path, capsys):
    bad_json = tmp_path / "bad.json"
    bad_json.write_text('{"name": "bad", "links": [{"name": "a", "inertial": {"mass": -1}}], "joints": []}')

    exit_code = generate_from_json.main([str(bad_json)])

    assert exit_code == 1
    assert "not a valid RobotModel JSON" in capsys.readouterr().err


def test_validation_failure_reports_error(tmp_path, capsys):
    # Structurally malformed (disconnected link) rather than a bad field
    # value -- exercises robot.validate()'s error path, not JSON parsing's.
    bad_json = tmp_path / "disconnected.json"
    bad_json.write_text(
        '{"name": "broken", "links": [{"name": "a"}, {"name": "b", "parent": "ghost"}], "joints": []}'
    )

    exit_code = generate_from_json.main([str(bad_json), "--output-dir", str(tmp_path / "out")])

    assert exit_code == 1
    assert "failed validation" in capsys.readouterr().err


def test_pipeline_error_reports_error(tmp_path, capsys):
    # A revolute joint with no velocity/effort limit -> PipelineError from
    # generate_ros_package, not a validation or JSON error.
    robot = build_sample_arm()
    robot.joint("shoulder_joint").velocity_limit = None
    robot_json = tmp_path / "robot.json"
    save_robot_json(robot, robot_json)

    exit_code = generate_from_json.main([str(robot_json), "--output-dir", str(tmp_path / "out")])

    assert exit_code == 1
    assert "velocity_limit" in capsys.readouterr().err
