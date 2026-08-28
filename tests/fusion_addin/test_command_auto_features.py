"""Tests for the automation features added to fusion_addin.ui.command:

- _apply_auto_detected_drivetrain / _apply_drivetrain_smart_default_checkboxes
  (drivetrain auto-detection + smart Gazebo/Nav2 checkbox defaults)
- _set_build_chain_inputs_visibility (one-click Generate->Build->Launch UI)
- _build_and_maybe_launch_in_wsl (the chain's actual doctor-check ->
  build -> launch logic)

Same "mock adsk.core, use a lightweight fake CommandInputs" technique as
test_command_drivetrain.py -- extended here with a fake DropDownCommandInput
whose listItems/selectedItem/isSelected actually behave like the real,
mutually-exclusive-selection API (test_command_drivetrain.py's own
_FakeDropDown only needed to be read, never mutated, since nothing there
tested code that calls `.isSelected = True`).
"""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

sys.modules.setdefault("adsk", MagicMock())
sys.modules.setdefault("adsk.core", MagicMock())
sys.modules.setdefault("adsk.fusion", MagicMock())

from fusion_addin.ui import command as cmd  # noqa: E402
import fusion_addin.generators.moveit as moveit_module  # noqa: E402


class _FakeStringInput:
    def __init__(self, value=""):
        self.value = value
        self.isVisible = True


class _FakeBoolInput:
    def __init__(self, value=False):
        self.value = value
        self.isVisible = True


class _FakeListItem:
    def __init__(self, name, is_selected, dropdown):
        self.name = name
        self._is_selected = is_selected
        self._dropdown = dropdown

    @property
    def isSelected(self):
        return self._is_selected

    @isSelected.setter
    def isSelected(self, value):
        if value:
            for item in self._dropdown._items:
                item._is_selected = False
        self._is_selected = value


class _FakeListItems:
    def __init__(self, dropdown):
        self._dropdown = dropdown

    def add(self, name, is_selected):
        item = _FakeListItem(name, is_selected, self._dropdown)
        self._dropdown._items.append(item)
        return item

    def __iter__(self):
        return iter(self._dropdown._items)


class _FakeDropDown:
    def __init__(self, options, initial_selection):
        self._items = []
        self.listItems = _FakeListItems(self)
        self.isVisible = True
        for option in options:
            self.listItems.add(option, option == initial_selection)

    @property
    def selectedItem(self):
        for item in self._items:
            if item.isSelected:
                return item
        return None


_DRIVETRAIN_OPTIONS = (cmd._DRIVETRAIN_NONE, cmd._DRIVETRAIN_DIFFERENTIAL, cmd._DRIVETRAIN_MECANUM)
_LAUNCH_OPTIONS = (cmd._LAUNCH_NONE, cmd._LAUNCH_DISPLAY, cmd._LAUNCH_GAZEBO, cmd._LAUNCH_NAV2, cmd._LAUNCH_BRINGUP)


class _FakeInputs:
    def __init__(
        self,
        string_values=None,
        bool_values=None,
        drivetrain_selection=cmd._DRIVETRAIN_NONE,
        launch_selection=cmd._LAUNCH_NONE,
    ):
        self._values = {}
        for input_id in cmd._DIFFERENTIAL_INPUT_IDS + cmd._MECANUM_INPUT_IDS + cmd._SENSOR_DEPENDENT_INPUT_IDS:
            self._values[input_id] = _FakeStringInput((string_values or {}).get(input_id, ""))
        for input_id in ("moveit_group_name", "wsl_ros_ws_src_for_build"):
            self._values[input_id] = _FakeStringInput((string_values or {}).get(input_id, ""))
        for input_id in cmd._PERSISTED_BOOL_FIELD_IDS:
            self._values[input_id] = _FakeBoolInput((bool_values or {}).get(input_id, False))
        self._values["drivetrain_type"] = _FakeDropDown(_DRIVETRAIN_OPTIONS, drivetrain_selection)
        self._values["launch_after_build"] = _FakeDropDown(_LAUNCH_OPTIONS, launch_selection)
        for slot in cmd._SENSOR_SLOT_IDS:
            type_id = cmd._sensor_slot_input_ids(slot)["type"]
            self._values[type_id] = _FakeDropDown(
                (cmd._SENSOR_TYPE_NONE, cmd._SENSOR_TYPE_CAMERA, cmd._SENSOR_TYPE_LIDAR, cmd._SENSOR_TYPE_IMU),
                (string_values or {}).get(type_id, cmd._SENSOR_TYPE_NONE),
            )

    def itemById(self, input_id):
        return self._values[input_id]


# ---------------------------------------------------------------------------
# _apply_drivetrain_smart_default_checkboxes
# ---------------------------------------------------------------------------


def test_smart_defaults_check_gazebo_and_nav2_when_drivetrain_selected():
    inputs = _FakeInputs(
        bool_values={"include_gazebo": False, "include_nav2": False},
        drivetrain_selection=cmd._DRIVETRAIN_DIFFERENTIAL,
    )
    cmd._apply_drivetrain_smart_default_checkboxes(inputs)
    assert inputs.itemById("include_gazebo").value is True
    assert inputs.itemById("include_nav2").value is True


def test_smart_defaults_do_not_uncheck_when_drivetrain_is_none():
    inputs = _FakeInputs(
        bool_values={"include_gazebo": True, "include_nav2": True}, drivetrain_selection=cmd._DRIVETRAIN_NONE
    )
    cmd._apply_drivetrain_smart_default_checkboxes(inputs)
    assert inputs.itemById("include_gazebo").value is True
    assert inputs.itemById("include_nav2").value is True


def test_smart_defaults_leave_none_selection_alone():
    inputs = _FakeInputs(
        bool_values={"include_gazebo": False, "include_nav2": False}, drivetrain_selection=cmd._DRIVETRAIN_NONE
    )
    cmd._apply_drivetrain_smart_default_checkboxes(inputs)
    assert inputs.itemById("include_gazebo").value is False
    assert inputs.itemById("include_nav2").value is False


def test_smart_defaults_do_not_uncheck_already_true_values_for_mecanum():
    inputs = _FakeInputs(
        bool_values={"include_gazebo": True, "include_nav2": False}, drivetrain_selection=cmd._DRIVETRAIN_MECANUM
    )
    cmd._apply_drivetrain_smart_default_checkboxes(inputs)
    assert inputs.itemById("include_gazebo").value is True
    assert inputs.itemById("include_nav2").value is True


# ---------------------------------------------------------------------------
# _apply_auto_detected_drivetrain
# ---------------------------------------------------------------------------


def test_auto_detect_fills_fields_and_switches_dropdown(monkeypatch):
    detected = {
        "type": "differential_drive",
        "left_wheel_joint": "left_wheel_joint",
        "right_wheel_joint": "right_wheel_joint",
        "wheel_separation": 0.4,
        "wheel_radius": 0.1,
    }
    monkeypatch.setattr(cmd, "detect_drivetrain", lambda robot: detected)

    inputs = _FakeInputs(bool_values={"include_gazebo": False, "include_nav2": False})
    cmd._apply_auto_detected_drivetrain(inputs, robot=object())

    assert inputs.itemById("drivetrain_type").selectedItem.name == cmd._DRIVETRAIN_DIFFERENTIAL
    assert inputs.itemById("drivetrain_left_wheel_joint").value == "left_wheel_joint"
    assert inputs.itemById("drivetrain_right_wheel_joint").value == "right_wheel_joint"
    assert inputs.itemById("drivetrain_wheel_separation_m").value == "0.4"
    assert inputs.itemById("drivetrain_wheel_radius_m").value == "0.1"
    # visibility follows the now-Differential selection
    assert inputs.itemById("drivetrain_left_wheel_joint").isVisible is True
    assert inputs.itemById("drivetrain_fl_wheel_joint").isVisible is False
    # smart-default nudge applied too
    assert inputs.itemById("include_gazebo").value is True
    assert inputs.itemById("include_nav2").value is True


def test_auto_detect_does_nothing_when_detection_returns_none(monkeypatch):
    monkeypatch.setattr(cmd, "detect_drivetrain", lambda robot: None)
    inputs = _FakeInputs(bool_values={"include_gazebo": False, "include_nav2": False})

    cmd._apply_auto_detected_drivetrain(inputs, robot=object())

    assert inputs.itemById("drivetrain_type").selectedItem.name == cmd._DRIVETRAIN_NONE
    assert inputs.itemById("drivetrain_left_wheel_joint").value == ""


def test_auto_detect_does_not_overwrite_a_manual_drivetrain_selection(monkeypatch):
    called = []
    monkeypatch.setattr(cmd, "detect_drivetrain", lambda robot: called.append(1) or {})
    inputs = _FakeInputs(drivetrain_selection=cmd._DRIVETRAIN_MECANUM)

    cmd._apply_auto_detected_drivetrain(inputs, robot=object())

    assert not called  # detect_drivetrain never even called -- short-circuited first
    assert inputs.itemById("drivetrain_type").selectedItem.name == cmd._DRIVETRAIN_MECANUM


def test_auto_detect_does_not_overwrite_manually_typed_fields(monkeypatch):
    called = []
    monkeypatch.setattr(cmd, "detect_drivetrain", lambda robot: called.append(1) or {})
    inputs = _FakeInputs(string_values={"drivetrain_left_wheel_joint": "my_own_joint"})

    cmd._apply_auto_detected_drivetrain(inputs, robot=object())

    assert not called
    assert inputs.itemById("drivetrain_left_wheel_joint").value == "my_own_joint"


# ---------------------------------------------------------------------------
# _set_build_chain_inputs_visibility
# ---------------------------------------------------------------------------


def test_build_chain_inputs_hidden_when_checkbox_unchecked():
    inputs = _FakeInputs(bool_values={"build_in_wsl_after_generate": False})
    cmd._set_build_chain_inputs_visibility(inputs)
    for input_id in cmd._BUILD_CHAIN_INPUT_IDS:
        assert inputs.itemById(input_id).isVisible is False


def test_build_chain_inputs_shown_when_checkbox_checked():
    inputs = _FakeInputs(bool_values={"build_in_wsl_after_generate": True})
    cmd._set_build_chain_inputs_visibility(inputs)
    for input_id in cmd._BUILD_CHAIN_INPUT_IDS:
        assert inputs.itemById(input_id).isVisible is True


# ---------------------------------------------------------------------------
# _build_and_maybe_launch_in_wsl
# ---------------------------------------------------------------------------


class _FakeProgressDialog:
    def __init__(self):
        self.message = ""


def test_build_chain_aborts_when_environment_not_ready(monkeypatch):
    monkeypatch.setattr(cmd, "run_environment_checks", lambda **k: ["a-check"])
    monkeypatch.setattr(cmd, "all_critical_passed", lambda checks: False)
    monkeypatch.setattr(cmd, "format_report", lambda checks: "REPORT TEXT")
    called = {"build": False}
    monkeypatch.setattr(cmd, "build_package_in_wsl", lambda *a, **k: called.__setitem__("build", True))

    report = cmd._build_and_maybe_launch_in_wsl(
        _FakeProgressDialog(), Path("/tmp/pkg"), "my_robot", "~/ros2_ws/src", cmd._LAUNCH_NONE, False
    )

    assert not called["build"]
    assert "REPORT TEXT" in report
    assert "isn't ready" in report


def test_build_chain_reports_build_failure(monkeypatch):
    monkeypatch.setattr(cmd, "run_environment_checks", lambda **k: [])
    monkeypatch.setattr(cmd, "all_critical_passed", lambda checks: True)
    monkeypatch.setattr(
        cmd,
        "build_package_in_wsl",
        lambda *a, **k: SimpleNamespace(success=False, returncode=1, stderr="boom", stdout=""),
    )
    launch_called = []
    monkeypatch.setattr(cmd, "launch_ros2_in_wsl", lambda **k: launch_called.append(1))

    report = cmd._build_and_maybe_launch_in_wsl(
        _FakeProgressDialog(), Path("/tmp/pkg"), "my_robot", "~/ros2_ws/src", cmd._LAUNCH_DISPLAY, False
    )

    assert not launch_called
    assert "FAILED" in report
    assert "boom" in report


def test_build_chain_success_without_launch(monkeypatch):
    monkeypatch.setattr(cmd, "run_environment_checks", lambda **k: [])
    monkeypatch.setattr(cmd, "all_critical_passed", lambda checks: True)
    monkeypatch.setattr(
        cmd, "build_package_in_wsl", lambda *a, **k: SimpleNamespace(success=True, stdout="ok", stderr="")
    )
    saved = []
    monkeypatch.setattr(cmd.state, "set_last_wsl_ros_ws_src", lambda v: saved.append(v))

    report = cmd._build_and_maybe_launch_in_wsl(
        _FakeProgressDialog(), Path("/tmp/pkg"), "my_robot", "~/ros2_ws/src", cmd._LAUNCH_NONE, False
    )

    assert saved == ["~/ros2_ws/src"]
    assert "Built successfully" in report
    assert "Launch" not in report


def test_build_chain_success_with_launch(monkeypatch):
    monkeypatch.setattr(cmd, "run_environment_checks", lambda **k: [])
    monkeypatch.setattr(cmd, "all_critical_passed", lambda checks: True)
    monkeypatch.setattr(
        cmd, "build_package_in_wsl", lambda *a, **k: SimpleNamespace(success=True, stdout="", stderr="")
    )
    monkeypatch.setattr(cmd.state, "set_last_wsl_ros_ws_src", lambda v: None)

    launch_kwargs = {}

    def fake_launch(**kwargs):
        launch_kwargs.update(kwargs)
        return SimpleNamespace(success=True, stdout="launched!", stderr="")

    monkeypatch.setattr(cmd, "launch_ros2_in_wsl", fake_launch)

    report = cmd._build_and_maybe_launch_in_wsl(
        _FakeProgressDialog(), Path("/tmp/pkg"), "my_robot", "~/ros2_ws/src", cmd._LAUNCH_GAZEBO, True
    )

    assert launch_kwargs["package_name"] == "my_robot"
    assert launch_kwargs["launch_file"] == "gazebo.launch.py"
    assert "launched" in report.lower()
    assert "launched!" in report


def test_build_chain_reports_launch_failure(monkeypatch):
    monkeypatch.setattr(cmd, "run_environment_checks", lambda **k: [])
    monkeypatch.setattr(cmd, "all_critical_passed", lambda checks: True)
    monkeypatch.setattr(
        cmd, "build_package_in_wsl", lambda *a, **k: SimpleNamespace(success=True, stdout="", stderr="")
    )
    monkeypatch.setattr(cmd.state, "set_last_wsl_ros_ws_src", lambda v: None)
    monkeypatch.setattr(
        cmd, "launch_ros2_in_wsl", lambda **k: SimpleNamespace(success=False, stdout="", stderr="launch broke")
    )

    report = cmd._build_and_maybe_launch_in_wsl(
        _FakeProgressDialog(), Path("/tmp/pkg"), "my_robot", "~/ros2_ws/src", cmd._LAUNCH_NAV2, False
    )

    assert "FAILED" in report
    assert "launch broke" in report


# ---------------------------------------------------------------------------
# _apply_moveit_smart_default_checkbox
# ---------------------------------------------------------------------------


def test_moveit_smart_default_checks_when_suitable(monkeypatch):
    monkeypatch.setattr(moveit_module, "detect_moveit_suitability", lambda robot: [])
    inputs = _FakeInputs(bool_values={"include_moveit": False})

    cmd._apply_moveit_smart_default_checkbox(inputs, robot=object())

    assert inputs.itemById("include_moveit").value is True


def test_moveit_smart_default_leaves_unchecked_when_unsuitable(monkeypatch):
    monkeypatch.setattr(moveit_module, "detect_moveit_suitability", lambda robot: ["branchy robot"])
    inputs = _FakeInputs(bool_values={"include_moveit": False})

    cmd._apply_moveit_smart_default_checkbox(inputs, robot=object())

    assert inputs.itemById("include_moveit").value is False


def test_moveit_smart_default_never_unchecks(monkeypatch):
    monkeypatch.setattr(moveit_module, "detect_moveit_suitability", lambda robot: ["branchy robot"])
    inputs = _FakeInputs(bool_values={"include_moveit": True})

    cmd._apply_moveit_smart_default_checkbox(inputs, robot=object())

    assert inputs.itemById("include_moveit").value is True


def test_moveit_smart_default_skipped_when_drivetrain_selected(monkeypatch):
    called = []
    monkeypatch.setattr(moveit_module, "detect_moveit_suitability", lambda robot: called.append(1) or [])
    inputs = _FakeInputs(bool_values={"include_moveit": False}, drivetrain_selection=cmd._DRIVETRAIN_DIFFERENTIAL)

    cmd._apply_moveit_smart_default_checkbox(inputs, robot=object())

    assert not called
    assert inputs.itemById("include_moveit").value is False


# ---------------------------------------------------------------------------
# _collect_persistable_dialog_state / _apply_persisted_dialog_state
# ---------------------------------------------------------------------------


def test_collect_and_apply_persisted_dialog_state_round_trips():
    inputs = _FakeInputs(
        string_values={
            "drivetrain_left_wheel_joint": "left_j",
            "drivetrain_right_wheel_joint": "right_j",
            "drivetrain_wheel_separation_m": "0.4",
            "drivetrain_wheel_radius_m": "0.1",
            "moveit_group_name": "my_arm",
            "wsl_ros_ws_src_for_build": "~/custom_ws/src",
        },
        bool_values={"include_gazebo": True, "include_nav2": True, "build_in_wsl_after_generate": True},
        drivetrain_selection=cmd._DRIVETRAIN_DIFFERENTIAL,
        launch_selection=cmd._LAUNCH_GAZEBO,
    )

    saved = cmd._collect_persistable_dialog_state(inputs)

    fresh_inputs = _FakeInputs()
    cmd._apply_persisted_dialog_state(fresh_inputs, saved)

    assert fresh_inputs.itemById("include_gazebo").value is True
    assert fresh_inputs.itemById("include_nav2").value is True
    assert fresh_inputs.itemById("build_in_wsl_after_generate").value is True
    assert fresh_inputs.itemById("moveit_group_name").value == "my_arm"
    assert fresh_inputs.itemById("wsl_ros_ws_src_for_build").value == "~/custom_ws/src"
    assert fresh_inputs.itemById("drivetrain_type").selectedItem.name == cmd._DRIVETRAIN_DIFFERENTIAL
    assert fresh_inputs.itemById("drivetrain_left_wheel_joint").value == "left_j"
    assert fresh_inputs.itemById("launch_after_build").selectedItem.name == cmd._LAUNCH_GAZEBO
    # visibility follows the restored selections
    assert fresh_inputs.itemById("drivetrain_left_wheel_joint").isVisible is True
    assert fresh_inputs.itemById("wsl_ros_ws_src_for_build").isVisible is True


def test_apply_persisted_dialog_state_ignores_missing_keys():
    inputs = _FakeInputs()
    # A partial/older save missing several keys must not raise and must
    # leave everything else at its existing default.
    cmd._apply_persisted_dialog_state(inputs, {"include_gazebo": True})
    assert inputs.itemById("include_gazebo").value is True
    assert inputs.itemById("include_nav2").value is False


def test_apply_persisted_dialog_state_ignores_unknown_dropdown_label():
    inputs = _FakeInputs()
    # Simulates a stale save naming an option a newer/older dialog version
    # doesn't have -- must not raise.
    cmd._apply_persisted_dialog_state(inputs, {"drivetrain_type": "Some Removed Option"})
    assert inputs.itemById("drivetrain_type").selectedItem.name == cmd._DRIVETRAIN_NONE


def test_apply_persisted_dialog_state_restores_sensor_slot_visibility():
    # Real bug: a saved sensor slot (type + its parent_link/name/update_rate
    # strings, all persisted fields -- see _PERSISTED_DROPDOWN_FIELD_IDS /
    # _PERSISTED_STRING_FIELD_IDS) was restored data-wise but its inputs
    # stayed hidden, because _apply_persisted_dialog_state refreshed
    # drivetrain/build-chain visibility but never sensor visibility.
    ids = cmd._sensor_slot_input_ids(2)
    saved = {
        ids["type"]: cmd._SENSOR_TYPE_LIDAR,
        ids["parent_link"]: "base_link",
        ids["name"]: "front_lidar",
        ids["update_rate"]: "10",
    }

    fresh_inputs = _FakeInputs()
    cmd._apply_persisted_dialog_state(fresh_inputs, saved)

    assert fresh_inputs.itemById(ids["type"]).selectedItem.name == cmd._SENSOR_TYPE_LIDAR
    assert fresh_inputs.itemById(ids["parent_link"]).value == "base_link"
    assert fresh_inputs.itemById(ids["parent_link"]).isVisible is True
    assert fresh_inputs.itemById(ids["name"]).isVisible is True
    assert fresh_inputs.itemById(ids["update_rate"]).isVisible is True
    # Untouched slots stay hidden.
    other_ids = cmd._sensor_slot_input_ids(1)
    assert fresh_inputs.itemById(other_ids["parent_link"]).isVisible is False


# ---------------------------------------------------------------------------
# _validate_generated_package
# ---------------------------------------------------------------------------


def test_validate_generated_package_reports_structure_and_urdf_problems(monkeypatch, tmp_path):
    monkeypatch.setattr(cmd, "validate_package_structure", lambda pkg_dir: ["missing package.xml"])
    monkeypatch.setattr(cmd, "validate_urdf_file", lambda path: [f"bad xml in {path.name}"])
    urdf_dir = tmp_path / "urdf"
    urdf_dir.mkdir()
    (urdf_dir / "robot.urdf.xacro").write_text("<robot/>", encoding="utf-8")

    problems = cmd._validate_generated_package(tmp_path)

    assert "missing package.xml" in problems
    assert any("robot.urdf.xacro" in p for p in problems)


def test_validate_generated_package_clean_package_returns_empty(monkeypatch, tmp_path):
    monkeypatch.setattr(cmd, "validate_package_structure", lambda pkg_dir: [])
    monkeypatch.setattr(cmd, "validate_urdf_file", lambda path: [])
    urdf_dir = tmp_path / "urdf"
    urdf_dir.mkdir()
    (urdf_dir / "robot.urdf.xacro").write_text("<robot/>", encoding="utf-8")

    assert cmd._validate_generated_package(tmp_path) == []


def test_validate_generated_package_missing_urdf_dir_does_not_raise(monkeypatch, tmp_path):
    monkeypatch.setattr(cmd, "validate_package_structure", lambda pkg_dir: ["no urdf dir"])
    called = []
    monkeypatch.setattr(cmd, "validate_urdf_file", lambda path: called.append(1) or [])

    problems = cmd._validate_generated_package(tmp_path)

    assert problems == ["no urdf dir"]
    assert not called


# ---------------------------------------------------------------------------
# Sensor UI: _set_sensor_inputs_visibility / _build_sensors
# ---------------------------------------------------------------------------


def _set_sensor_slot(inputs, slot, type_label, parent_link="", name="", update_rate=""):
    ids = cmd._sensor_slot_input_ids(slot)
    cmd._select_dropdown_item(inputs, ids["type"], type_label)
    inputs.itemById(ids["parent_link"]).value = parent_link
    inputs.itemById(ids["name"]).value = name
    inputs.itemById(ids["update_rate"]).value = update_rate


def test_sensor_inputs_hidden_by_default():
    inputs = _FakeInputs()
    cmd._set_sensor_inputs_visibility(inputs)
    for slot in cmd._SENSOR_SLOT_IDS:
        ids = cmd._sensor_slot_input_ids(slot)
        assert inputs.itemById(ids["parent_link"]).isVisible is False
        assert inputs.itemById(ids["name"]).isVisible is False
        assert inputs.itemById(ids["update_rate"]).isVisible is False


def test_sensor_inputs_shown_only_for_active_slot():
    inputs = _FakeInputs()
    _set_sensor_slot(inputs, 2, cmd._SENSOR_TYPE_CAMERA)
    cmd._set_sensor_inputs_visibility(inputs)

    ids1 = cmd._sensor_slot_input_ids(1)
    ids2 = cmd._sensor_slot_input_ids(2)
    assert inputs.itemById(ids1["parent_link"]).isVisible is False
    assert inputs.itemById(ids2["parent_link"]).isVisible is True


def test_build_sensors_empty_when_all_slots_none():
    inputs = _FakeInputs()
    assert cmd._build_sensors(inputs) == []


def test_build_sensors_builds_camera_lidar_imu():
    inputs = _FakeInputs()
    _set_sensor_slot(inputs, 1, cmd._SENSOR_TYPE_CAMERA, parent_link="head_link", name="head_cam")
    _set_sensor_slot(inputs, 2, cmd._SENSOR_TYPE_LIDAR, parent_link="base_link", update_rate="20")
    _set_sensor_slot(inputs, 3, cmd._SENSOR_TYPE_IMU, parent_link="imu_link")

    sensors = cmd._build_sensors(inputs)

    by_type = {s.type: s for s in sensors}
    assert set(by_type) == {"camera", "lidar", "imu"}
    assert by_type["camera"].name == "head_cam"
    assert by_type["camera"].parent_link == "head_link"
    assert by_type["lidar"].parent_link == "base_link"
    assert by_type["lidar"].parameters == {"update_rate": 20.0}
    assert by_type["imu"].name == "imu_3"  # auto-named from type + slot
    assert by_type["imu"].parameters == {}


def test_build_sensors_missing_parent_link_raises():
    inputs = _FakeInputs()
    _set_sensor_slot(inputs, 1, cmd._SENSOR_TYPE_CAMERA, parent_link="")

    with pytest.raises(ValueError, match="parent link"):
        cmd._build_sensors(inputs)


def test_build_sensors_non_numeric_update_rate_raises():
    inputs = _FakeInputs()
    _set_sensor_slot(inputs, 1, cmd._SENSOR_TYPE_IMU, parent_link="imu_link", update_rate="fast")

    with pytest.raises(ValueError, match="update rate"):
        cmd._build_sensors(inputs)
