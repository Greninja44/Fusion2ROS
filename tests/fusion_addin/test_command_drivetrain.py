"""Tests for the drivetrain UI helpers in fusion_addin.ui.command --
_build_drivetrain_metadata / _set_drivetrain_inputs_visibility -- using a
lightweight fake CommandInputs, since importing command.py needs adsk.core/
adsk.fusion mocked (same technique as test_fusion_adapter.py).

This covers the real gap found live: robot.metadata["drivetrain"] (the
convention ros2_control.py/nav2.py use to recognize a mobile base) was
previously only ever set by hand in example scripts -- a real Fusion user
had no way to enable Nav2/ros2_control's drivetrain features at all, so
"Nav2" always failed with "robot.metadata['drivetrain'] is not set" even on
a correctly-detected wheeled robot.
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


class _FakeDropDown:
    def __init__(self, selected_name):
        self.selectedItem = SimpleNamespace(name=selected_name)


class _FakeInputs:
    """Minimal stand-in for adsk.core.CommandInputs -- itemById(id) only."""

    def __init__(self, values: dict, drivetrain_type: str):
        self._string_inputs = {k: _FakeStringInput(v) for k, v in values.items()}
        self._string_inputs["drivetrain_type"] = _FakeDropDown(drivetrain_type)

    def itemById(self, input_id):
        return self._string_inputs[input_id]


def _inputs(drivetrain_type, **values):
    return _FakeInputs(values, drivetrain_type)


# ---------------------------------------------------------------------------
# _build_drivetrain_metadata
# ---------------------------------------------------------------------------


def test_none_selection_returns_none():
    inputs = _inputs(cmd._DRIVETRAIN_NONE)
    assert cmd._build_drivetrain_metadata(inputs) is None


def test_differential_drive_builds_correct_dict():
    inputs = _inputs(
        cmd._DRIVETRAIN_DIFFERENTIAL,
        drivetrain_left_wheel_joint="left_wheel_joint",
        drivetrain_right_wheel_joint="right_wheel_joint",
        drivetrain_wheel_separation_m="0.35",
        drivetrain_wheel_radius_m="0.075",
    )
    drivetrain = cmd._build_drivetrain_metadata(inputs)
    assert drivetrain == {
        "type": "differential_drive",
        "left_wheel_joint": "left_wheel_joint",
        "right_wheel_joint": "right_wheel_joint",
        "wheel_separation": pytest.approx(0.35),
        "wheel_radius": pytest.approx(0.075),
    }


def test_differential_drive_missing_wheel_joint_raises():
    inputs = _inputs(
        cmd._DRIVETRAIN_DIFFERENTIAL,
        drivetrain_left_wheel_joint="",
        drivetrain_right_wheel_joint="right_wheel_joint",
        drivetrain_wheel_separation_m="0.35",
        drivetrain_wheel_radius_m="0.075",
    )
    with pytest.raises(ValueError, match="left and right wheel joint"):
        cmd._build_drivetrain_metadata(inputs)


def test_differential_drive_non_numeric_wheel_radius_raises():
    inputs = _inputs(
        cmd._DRIVETRAIN_DIFFERENTIAL,
        drivetrain_left_wheel_joint="left_wheel_joint",
        drivetrain_right_wheel_joint="right_wheel_joint",
        drivetrain_wheel_separation_m="0.35",
        drivetrain_wheel_radius_m="not_a_number",
    )
    with pytest.raises(ValueError, match="Wheel radius"):
        cmd._build_drivetrain_metadata(inputs)


def test_differential_drive_negative_wheel_separation_raises():
    inputs = _inputs(
        cmd._DRIVETRAIN_DIFFERENTIAL,
        drivetrain_left_wheel_joint="left_wheel_joint",
        drivetrain_right_wheel_joint="right_wheel_joint",
        drivetrain_wheel_separation_m="-0.35",
        drivetrain_wheel_radius_m="0.075",
    )
    with pytest.raises(ValueError, match="positive"):
        cmd._build_drivetrain_metadata(inputs)


def test_mecanum_drive_builds_correct_dict():
    inputs = _inputs(
        cmd._DRIVETRAIN_MECANUM,
        drivetrain_fl_wheel_joint="fl",
        drivetrain_fr_wheel_joint="fr",
        drivetrain_bl_wheel_joint="bl",
        drivetrain_br_wheel_joint="br",
        drivetrain_mecanum_wheel_radius_m="0.06",
        drivetrain_mecanum_center_sum_m="0.4",
    )
    drivetrain = cmd._build_drivetrain_metadata(inputs)
    assert drivetrain == {
        "type": "mecanum_drive",
        "front_left_wheel_joint": "fl",
        "front_right_wheel_joint": "fr",
        "back_left_wheel_joint": "bl",
        "back_right_wheel_joint": "br",
        "wheel_radius": pytest.approx(0.06),
        "sum_of_robot_center_projection_on_x_y_axis": pytest.approx(0.4),
    }


def test_mecanum_drive_missing_joint_raises():
    inputs = _inputs(
        cmd._DRIVETRAIN_MECANUM,
        drivetrain_fl_wheel_joint="fl",
        drivetrain_fr_wheel_joint="fr",
        drivetrain_bl_wheel_joint="",
        drivetrain_br_wheel_joint="br",
        drivetrain_mecanum_wheel_radius_m="0.06",
        drivetrain_mecanum_center_sum_m="0.4",
    )
    with pytest.raises(ValueError, match="back left"):
        cmd._build_drivetrain_metadata(inputs)


# ---------------------------------------------------------------------------
# _set_drivetrain_inputs_visibility
# ---------------------------------------------------------------------------


def test_visibility_none_hides_both_groups():
    inputs = _inputs(
        cmd._DRIVETRAIN_NONE,
        **{k: "" for k in cmd._DIFFERENTIAL_INPUT_IDS + cmd._MECANUM_INPUT_IDS},
    )
    cmd._set_drivetrain_inputs_visibility(inputs)
    for input_id in cmd._DIFFERENTIAL_INPUT_IDS + cmd._MECANUM_INPUT_IDS:
        assert inputs.itemById(input_id).isVisible is False


def test_visibility_differential_shows_only_differential_group():
    inputs = _inputs(
        cmd._DRIVETRAIN_DIFFERENTIAL,
        **{k: "" for k in cmd._DIFFERENTIAL_INPUT_IDS + cmd._MECANUM_INPUT_IDS},
    )
    cmd._set_drivetrain_inputs_visibility(inputs)
    for input_id in cmd._DIFFERENTIAL_INPUT_IDS:
        assert inputs.itemById(input_id).isVisible is True
    for input_id in cmd._MECANUM_INPUT_IDS:
        assert inputs.itemById(input_id).isVisible is False


def test_visibility_mecanum_shows_only_mecanum_group():
    inputs = _inputs(
        cmd._DRIVETRAIN_MECANUM,
        **{k: "" for k in cmd._DIFFERENTIAL_INPUT_IDS + cmd._MECANUM_INPUT_IDS},
    )
    cmd._set_drivetrain_inputs_visibility(inputs)
    for input_id in cmd._MECANUM_INPUT_IDS:
        assert inputs.itemById(input_id).isVisible is True
    for input_id in cmd._DIFFERENTIAL_INPUT_IDS:
        assert inputs.itemById(input_id).isVisible is False
