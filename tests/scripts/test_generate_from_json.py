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


def test_missing_joint_type_field_reports_clear_error_not_traceback(tmp_path, capsys):
    # robot_model.serialization._joint_from_dict reads data["type"] via
    # plain indexing (not .get), so a joint dict missing "type" used to
    # raise a raw, uncaught KeyError all the way out of main() -- a real
    # traceback, not the same one-line message every other malformed-JSON
    # case gets.
    bad_json = tmp_path / "no_type.json"
    bad_json.write_text(
        '{"name": "x", "links": [{"name": "a"}, {"name": "b", "parent": "a"}], '
        '"joints": [{"name": "j1", "parent": "a", "child": "b"}]}'
    )

    exit_code = generate_from_json.main([str(bad_json)])

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "not a valid RobotModel JSON" in err
    assert "'type'" in err


# --- --dry-run -------------------------------------------------------------


def test_dry_run_writes_no_files_and_prints_manifest(tmp_path, capsys):
    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)
    out_dir = tmp_path / "out"

    exit_code = generate_from_json.main(
        [str(robot_json), "--output-dir", str(out_dir), "--dry-run", "--ros2-control"]
    )

    assert exit_code == 0
    assert not out_dir.exists()
    out = capsys.readouterr().out
    assert "Dry run" in out
    assert "No files were written" in out
    assert "package.xml" in out
    assert "config/controllers.yaml" in out


def test_dry_run_still_reports_pipeline_errors(tmp_path, capsys):
    robot = build_sample_arm()
    robot.joint("shoulder_joint").velocity_limit = None
    robot_json = tmp_path / "robot.json"
    save_robot_json(robot, robot_json)

    exit_code = generate_from_json.main([str(robot_json), "--output-dir", str(tmp_path / "out"), "--dry-run"])

    assert exit_code == 1
    assert "velocity_limit" in capsys.readouterr().err


def test_dry_run_batch_summary_says_validated_not_generated(tmp_path, capsys):
    arm_json = tmp_path / "arm.json"
    save_robot_json(build_sample_arm(), arm_json)
    rover_json = tmp_path / "rover.json"
    save_robot_json(build_sample_rover(), rover_json)

    exit_code = generate_from_json.main(
        [str(arm_json), str(rover_json), "--output-dir", str(tmp_path / "out"), "--dry-run"]
    )

    assert exit_code == 0
    out = capsys.readouterr().out
    assert "2/2 robots validated (dry run) successfully" in out
    assert not (tmp_path / "out").exists()


# --- config file integration ------------------------------------------------


def test_config_file_supplies_output_dir_and_include_flags(tmp_path, capsys):
    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)
    out_dir = tmp_path / "configured_out"
    config_path = tmp_path / "my_config.yaml"
    config_path.write_text(f"output_dir: {out_dir}\ninclude:\n  ros2_control: true\n")

    exit_code = generate_from_json.main([str(robot_json), "--config", str(config_path)])

    assert exit_code == 0
    assert (out_dir / "sample_arm" / "config" / "controllers.yaml").exists()
    assert f"Using config file: {config_path}" in capsys.readouterr().out


def test_cli_flag_overrides_config_file_value(tmp_path):
    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)
    config_out_dir = tmp_path / "from_config"
    cli_out_dir = tmp_path / "from_cli"
    config_path = tmp_path / "my_config.yaml"
    config_path.write_text(f"output_dir: {config_out_dir}\ninclude:\n  moveit: true\n")

    # --no-moveit explicitly overrides the config file's `moveit: true`;
    # --output-dir explicitly overrides the config file's output_dir.
    exit_code = generate_from_json.main(
        [str(robot_json), "--config", str(config_path), "--output-dir", str(cli_out_dir), "--no-moveit"]
    )

    assert exit_code == 0
    assert not config_out_dir.exists()
    assert (cli_out_dir / "sample_arm" / "package.xml").exists()
    assert not (cli_out_dir / "sample_arm" / "config" / "ompl_planning.yaml").exists()


def test_no_config_flag_ignores_an_applicable_config_file(tmp_path, monkeypatch):
    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)
    cwd = tmp_path / "project"
    cwd.mkdir()
    (cwd / ".fusion2ros.yaml").write_text("include:\n  moveit: true\n")
    monkeypatch.chdir(cwd)

    exit_code = generate_from_json.main(
        [str(robot_json), "--output-dir", str(tmp_path / "out"), "--no-config"]
    )

    assert exit_code == 0
    assert not (tmp_path / "out" / "sample_arm" / "config" / "ompl_planning.yaml").exists()


def test_missing_explicit_config_file_reports_clear_error(tmp_path, capsys):
    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)

    exit_code = generate_from_json.main([str(robot_json), "--config", str(tmp_path / "nope.yaml")])

    assert exit_code == 1
    assert "does not exist" in capsys.readouterr().err


def test_malformed_config_file_reports_clear_error(tmp_path, capsys):
    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)
    config_path = tmp_path / "bad_config.yaml"
    config_path.write_text("include:\n  frobnicate: true\n")

    exit_code = generate_from_json.main([str(robot_json), "--config", str(config_path)])

    assert exit_code == 1
    assert "unknown include key" in capsys.readouterr().err


def test_config_file_maintainer_and_license_fill_package_xml(tmp_path):
    import xml.etree.ElementTree as ET

    robot_json = tmp_path / "robot.json"
    save_robot_json(build_sample_arm(), robot_json)
    config_path = tmp_path / "my_config.yaml"
    config_path.write_text(
        "maintainer:\n  name: Config Person\n  email: config@example.com\nlicense: MIT\n"
    )
    out_dir = tmp_path / "out"

    exit_code = generate_from_json.main(
        [str(robot_json), "--config", str(config_path), "--output-dir", str(out_dir)]
    )

    assert exit_code == 0
    root = ET.parse(out_dir / "sample_arm" / "package.xml").getroot()
    assert root.findtext("maintainer") == "Config Person"
    assert root.find("maintainer").get("email") == "config@example.com"
    assert root.findtext("license") == "MIT"


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
