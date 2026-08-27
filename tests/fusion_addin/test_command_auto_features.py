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
    def __init__(self, initial_selection):
        self._items = []
        self.listItems = _FakeListItems(self)
        self.isVisible = True
        self.listItems.add(cmd._DRIVETRAIN_NONE, initial_selection == cmd._DRIVETRAIN_NONE)
        self.listItems.add(cmd._DRIVETRAIN_DIFFERENTIAL, initial_selection == cmd._DRIVETRAIN_DIFFERENTIAL)
        self.listItems.add(cmd._DRIVETRAIN_MECANUM, initial_selection == cmd._DRIVETRAIN_MECANUM)

    @property
    def selectedItem(self):
        for item in self._items:
            if item.isSelected:
                return item
        return None


class _FakeInputs:
    def __init__(self, string_values=None, bool_values=None, drivetrain_selection=cmd._DRIVETRAIN_NONE):
        self._values = {}
        for input_id in cmd._DIFFERENTIAL_INPUT_IDS + cmd._MECANUM_INPUT_IDS:
            self._values[input_id] = _FakeStringInput((string_values or {}).get(input_id, ""))
        for input_id, value in (bool_values or {}).items():
            self._values[input_id] = _FakeBoolInput(value)
        self._values["drivetrain_type"] = _FakeDropDown(drivetrain_selection)
        for input_id in cmd._BUILD_CHAIN_INPUT_IDS:
            self._values.setdefault(input_id, _FakeStringInput(""))

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
