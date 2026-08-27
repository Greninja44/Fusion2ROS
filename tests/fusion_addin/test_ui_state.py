"""Tests for fusion_addin.ui.state -- the only file under fusion_addin/ui/
with zero `adsk` dependency, so the only one testable here without a live
Fusion process. Everything else in fusion_addin/ui/ is exercised only by
inspection/review, per the project brief.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fusion_addin.ui import state


def test_last_generated_round_trips(tmp_path):
    pkg_dir = tmp_path / "my_robot"
    state.set_last_generated(pkg_dir, "my_robot")

    assert state.get_last_package_dir() == pkg_dir
    assert state.get_last_robot_name() == "my_robot"


def test_last_wsl_ros_ws_src_round_trips():
    state.set_last_wsl_ros_ws_src("~/ros2_ws/src")

    assert state.get_last_wsl_ros_ws_src() == "~/ros2_ws/src"


def test_generate_dialog_state_round_trips(tmp_path, monkeypatch):
    monkeypatch.setattr(state, "_DIALOG_STATE_PATH", tmp_path / ".generate_dialog_state.json")

    assert state.load_generate_dialog_state("my_robot") == {}

    state.save_generate_dialog_state("my_robot", {"include_gazebo": True, "moveit_group_name": "arm"})

    assert state.load_generate_dialog_state("my_robot") == {"include_gazebo": True, "moveit_group_name": "arm"}
    assert state.load_generate_dialog_state("other_robot") == {}


def test_generate_dialog_state_does_not_clobber_other_robots(tmp_path, monkeypatch):
    monkeypatch.setattr(state, "_DIALOG_STATE_PATH", tmp_path / ".generate_dialog_state.json")

    state.save_generate_dialog_state("robot_a", {"include_gazebo": True})
    state.save_generate_dialog_state("robot_b", {"include_nav2": True})

    assert state.load_generate_dialog_state("robot_a") == {"include_gazebo": True}
    assert state.load_generate_dialog_state("robot_b") == {"include_nav2": True}


def test_generate_dialog_state_overwrites_same_robot(tmp_path, monkeypatch):
    monkeypatch.setattr(state, "_DIALOG_STATE_PATH", tmp_path / ".generate_dialog_state.json")

    state.save_generate_dialog_state("robot_a", {"include_gazebo": True})
    state.save_generate_dialog_state("robot_a", {"include_gazebo": False, "include_nav2": True})

    assert state.load_generate_dialog_state("robot_a") == {"include_gazebo": False, "include_nav2": True}


def test_generate_dialog_state_survives_a_fresh_read(tmp_path, monkeypatch):
    # Simulates a real Fusion restart: a NEW read from disk (not relying on
    # any in-process cache) must still see a previous save.
    path = tmp_path / ".generate_dialog_state.json"
    monkeypatch.setattr(state, "_DIALOG_STATE_PATH", path)
    state.save_generate_dialog_state("my_robot", {"include_moveit": True})

    monkeypatch.setattr(state, "_DIALOG_STATE_PATH", path)  # re-point, as if freshly imported
    assert state.load_generate_dialog_state("my_robot") == {"include_moveit": True}


def test_generate_dialog_state_corrupt_file_is_treated_as_empty(tmp_path, monkeypatch):
    path = tmp_path / ".generate_dialog_state.json"
    path.write_text("not valid json{{{", encoding="utf-8")
    monkeypatch.setattr(state, "_DIALOG_STATE_PATH", path)

    assert state.load_generate_dialog_state("my_robot") == {}


def test_defaults_are_none_before_anything_set():
    # Import a fresh copy of the module (module-level globals persist across
    # tests in the same process otherwise) to check the pristine defaults.
    import importlib

    fresh = importlib.reload(state)
    try:
        assert fresh.get_last_package_dir() is None
        assert fresh.get_last_robot_name() is None
        assert fresh.get_last_wsl_ros_ws_src() is None
    finally:
        # Restore whatever the other tests in this module may rely on being
        # set, and leave the module in a clean state for any test order.
        importlib.reload(state)
