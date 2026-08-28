"""Tests for fusion_addin.config -- the optional `.fusion2ros.yaml` config
file scripts/generate_from_json.py reads for repeated CLI flag defaults.

Plain `python3 -m pytest`, no Fusion, no ROS, no network -- pure stdlib +
PyYAML, same constraints as every other fusion_addin module tested here.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fusion_addin.config import (
    ConfigError,
    Fusion2ROSConfig,
    default_config_search_paths,
    find_config_file,
    load_config_file,
    resolve_config,
    resolve_value,
)


# --- resolve_value: the core precedence rule -----------------------------


def test_resolve_value_cli_wins_when_given():
    assert resolve_value(True, False, False) is True
    assert resolve_value("cli", "config", "default") == "cli"


def test_resolve_value_falls_back_to_config_when_cli_is_none():
    assert resolve_value(None, "config", "default") == "config"


def test_resolve_value_falls_back_to_builtin_default_when_both_are_none():
    assert resolve_value(None, None, "default") == "default"


def test_resolve_value_cli_false_is_not_treated_as_unset():
    # A CLI flag explicitly set to False (e.g. --no-gazebo) must win over a
    # config file's True -- only None means "the CLI didn't say".
    assert resolve_value(False, True, True) is False


# --- default_config_search_paths / find_config_file -----------------------


def test_default_config_search_paths_orders_cwd_before_home(tmp_path):
    cwd = tmp_path / "project"
    home = tmp_path / "home"
    cwd.mkdir()
    home.mkdir()
    paths = default_config_search_paths(cwd=cwd, home=home)
    assert paths == [cwd / ".fusion2ros.yaml", home / ".fusion2ros.yaml"]


def test_find_config_file_prefers_cwd_over_home(tmp_path):
    cwd = tmp_path / "project"
    home = tmp_path / "home"
    cwd.mkdir()
    home.mkdir()
    (cwd / ".fusion2ros.yaml").write_text("output_dir: from_cwd\n")
    (home / ".fusion2ros.yaml").write_text("output_dir: from_home\n")

    found = find_config_file(cwd=cwd, home=home)
    assert found == cwd / ".fusion2ros.yaml"


def test_find_config_file_falls_back_to_home(tmp_path):
    cwd = tmp_path / "project"
    home = tmp_path / "home"
    cwd.mkdir()
    home.mkdir()
    (home / ".fusion2ros.yaml").write_text("output_dir: from_home\n")

    found = find_config_file(cwd=cwd, home=home)
    assert found == home / ".fusion2ros.yaml"


def test_find_config_file_returns_none_when_neither_exists(tmp_path):
    cwd = tmp_path / "project"
    home = tmp_path / "home"
    cwd.mkdir()
    home.mkdir()

    assert find_config_file(cwd=cwd, home=home) is None


# --- load_config_file: parsing + validation -------------------------------


def test_load_config_file_parses_every_known_field(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text(
        "output_dir: output/\n"
        "include:\n"
        "  ros2_control: true\n"
        "  gazebo: false\n"
        "  moveit: true\n"
        "  nav2: false\n"
        "moveit_group: arm\n"
        "maintainer:\n"
        "  name: Jane Doe\n"
        "  email: jane@example.com\n"
        "license: Apache-2.0\n"
    )

    cfg = load_config_file(config_path)

    assert cfg.output_dir == "output/"
    assert cfg.include_ros2_control is True
    assert cfg.include_gazebo is False
    assert cfg.include_moveit is True
    assert cfg.include_nav2 is False
    assert cfg.moveit_group_name == "arm"
    assert cfg.maintainer_name == "Jane Doe"
    assert cfg.maintainer_email == "jane@example.com"
    assert cfg.license == "Apache-2.0"
    assert cfg.source_path == config_path


def test_load_config_file_empty_file_is_all_none(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("")

    cfg = load_config_file(config_path)

    assert cfg.output_dir is None
    assert cfg.include_ros2_control is None
    assert cfg.source_path == config_path


def test_load_config_file_partial_file_leaves_rest_none(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("include:\n  gazebo: true\n")

    cfg = load_config_file(config_path)

    assert cfg.include_gazebo is True
    assert cfg.include_ros2_control is None
    assert cfg.output_dir is None
    assert cfg.maintainer_name is None


def test_load_config_file_rejects_invalid_yaml(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("include: [this is not: a mapping\n")

    with pytest.raises(ConfigError, match="not valid YAML"):
        load_config_file(config_path)


def test_load_config_file_rejects_non_mapping_top_level(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("- just\n- a\n- list\n")

    with pytest.raises(ConfigError, match="mapping"):
        load_config_file(config_path)


def test_load_config_file_rejects_unknown_top_level_key(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("outptu_dir: typo/\n")

    with pytest.raises(ConfigError, match="unknown key"):
        load_config_file(config_path)


def test_load_config_file_rejects_unknown_include_key(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("include:\n  sensors: true\n")

    with pytest.raises(ConfigError, match="unknown include key"):
        load_config_file(config_path)


def test_load_config_file_rejects_non_bool_include_value(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("include:\n  gazebo: yes\n")

    # PyYAML parses bare `yes` as the real bool True in YAML 1.1, so use a
    # value that survives as a non-bool to actually exercise the type check.
    config_path.write_text("include:\n  gazebo: 'yes'\n")

    with pytest.raises(ConfigError, match="must be true/false"):
        load_config_file(config_path)


def test_load_config_file_rejects_non_string_output_dir(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("output_dir: 123\n")

    with pytest.raises(ConfigError, match="must be a string"):
        load_config_file(config_path)


def test_load_config_file_rejects_unknown_maintainer_key(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("maintainer:\n  full_name: Jane\n")

    with pytest.raises(ConfigError, match="unknown maintainer key"):
        load_config_file(config_path)


def test_load_config_file_rejects_non_mapping_include(tmp_path):
    config_path = tmp_path / ".fusion2ros.yaml"
    config_path.write_text("include: true\n")

    with pytest.raises(ConfigError, match="'include' must be a mapping"):
        load_config_file(config_path)


def test_load_config_file_missing_file_raises_config_error(tmp_path):
    with pytest.raises(ConfigError, match="could not read"):
        load_config_file(tmp_path / "does_not_exist.yaml")


# --- resolve_config: end-to-end lookup ------------------------------------


def test_resolve_config_explicit_path_missing_raises(tmp_path):
    with pytest.raises(ConfigError, match="does not exist"):
        resolve_config(explicit_path=tmp_path / "nope.yaml")


def test_resolve_config_explicit_path_used_verbatim(tmp_path):
    config_path = tmp_path / "custom.yaml"
    config_path.write_text("output_dir: custom/\n")

    cfg = resolve_config(explicit_path=config_path)

    assert cfg.output_dir == "custom/"
    assert cfg.source_path == config_path


def test_resolve_config_no_file_found_returns_all_none_config(tmp_path):
    cwd = tmp_path / "project"
    home = tmp_path / "home"
    cwd.mkdir()
    home.mkdir()

    cfg = resolve_config(cwd=cwd, home=home)

    assert cfg == Fusion2ROSConfig()
    assert cfg.source_path is None


def test_resolve_config_finds_cwd_file_when_no_explicit_path(tmp_path):
    cwd = tmp_path / "project"
    home = tmp_path / "home"
    cwd.mkdir()
    home.mkdir()
    (cwd / ".fusion2ros.yaml").write_text("license: MIT\n")

    cfg = resolve_config(cwd=cwd, home=home)

    assert cfg.license == "MIT"
    assert cfg.source_path == cwd / ".fusion2ros.yaml"
