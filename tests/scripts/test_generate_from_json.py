"""Tests for scripts/generate_from_json.py, the standalone (no-Fusion) CLI."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from robot_model import save_robot_json
from examples.sample_arm import build_sample_arm
from examples.sample_rover import build_sample_rover

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


def test_batch_mode_generates_all_robots(tmp_path, capsys):
    arm_json = tmp_path / "arm.json"
    rover_json = tmp_path / "rover.json"
    save_robot_json(build_sample_arm(), arm_json)
    save_robot_json(build_sample_rover(), rover_json)

    exit_code = generate_from_json.main(
        [str(arm_json), str(rover_json), "--output-dir", str(tmp_path / "out")]
    )

    assert exit_code == 0
    assert (tmp_path / "out" / "sample_arm" / "package.xml").exists()
    assert (tmp_path / "out" / "sample_rover" / "package.xml").exists()
    assert "2/2 robots generated successfully" in capsys.readouterr().out


def test_batch_mode_continues_past_one_failure(tmp_path, capsys):
    # One good robot, one bad (invalid JSON) -- the good one must still be
    # generated, and the overall exit code must reflect the failure.
    good_json = tmp_path / "good.json"
    save_robot_json(build_sample_arm(), good_json)
    bad_json = tmp_path / "bad.json"
    bad_json.write_text("not json at all")

    exit_code = generate_from_json.main(
        [str(good_json), str(bad_json), "--output-dir", str(tmp_path / "out")]
    )

    assert exit_code == 1
    assert (tmp_path / "out" / "sample_arm" / "package.xml").exists()
    assert "1/2 robots generated successfully" in capsys.readouterr().out


def test_single_robot_batch_output_unchanged(tmp_path, capsys):
    # A single robot_json arg must NOT print batch-mode summary lines --
    # keeps single-robot usage's output exactly as it was before batch
    # mode existed.
    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)

    generate_from_json.main([str(robot_json), "--output-dir", str(tmp_path / "out")])

    out = capsys.readouterr().out
    assert "===" not in out
    assert "robots generated successfully" not in out
