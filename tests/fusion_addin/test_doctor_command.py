"""Tests for the "Check WSL Environment" command handler in
fusion_addin.ui.doctor_command -- same mocked-adsk.core technique as
test_build_command.py (including the same real-empty-class fix for
adsk.core.Command*EventHandler; see that file's own comment for why a bare
MagicMock can't be used as a base class)."""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

sys.modules.setdefault("adsk", MagicMock())
sys.modules.setdefault("adsk.core", MagicMock())
sys.modules.setdefault("adsk.fusion", MagicMock())
sys.modules["adsk"].core.CommandEventHandler = type("CommandEventHandler", (), {})
sys.modules["adsk"].core.CommandCreatedEventHandler = type("CommandCreatedEventHandler", (), {})

from bridge.windows.doctor import DoctorCheck  # noqa: E402
from fusion_addin.ui import doctor_command as dc  # noqa: E402


class _FakeStringInput:
    def __init__(self, value):
        self.value = value


class _FakeBoolInput:
    def __init__(self, value):
        self.value = value


class _FakeInputs:
    def __init__(self, values):
        self._values = values

    def itemById(self, input_id):
        return self._values[input_id]


class _FakeProgressDialog:
    def __init__(self):
        self.show_calls = []

    def show(self, *a):
        self.show_calls.append(a)

    def hide(self):
        pass


class _FakeUi:
    def __init__(self, progress_dialog):
        self._pd = progress_dialog
        self.messages = []

    def createProgressDialog(self):
        return self._pd

    def messageBox(self, text):
        self.messages.append(text)


def test_execute_shows_report_in_message_box(monkeypatch):
    progress_dialog = _FakeProgressDialog()
    ui = _FakeUi(progress_dialog)
    monkeypatch.setattr(dc.adsk.core.Application, "get", lambda: SimpleNamespace(userInterface=ui))

    captured_kwargs = {}

    def fake_run_environment_checks(**kwargs):
        captured_kwargs.update(kwargs)
        return [DoctorCheck("WSL installed", True, "OK"), DoctorCheck("colcon on PATH", False, "not found")]

    monkeypatch.setattr(dc, "run_environment_checks", fake_run_environment_checks)

    inputs = _FakeInputs(
        {
            "wsl_ros_ws_src": _FakeStringInput("~/ros2_ws/src"),
            "check_gazebo": _FakeBoolInput(True),
        }
    )
    args = SimpleNamespace(command=SimpleNamespace(commandInputs=inputs))

    handler = dc.DoctorCommandExecuteHandler()
    handler.notify(args)

    assert len(ui.messages) == 1
    assert "WSL installed" in ui.messages[0]
    assert "colcon on PATH" in ui.messages[0]
    assert captured_kwargs["wsl_ros_ws_src"] == "~/ros2_ws/src"
    assert captured_kwargs["check_gazebo"] is True
    assert len(progress_dialog.show_calls) == 1
    _, _, minimum_value, maximum_value, _ = progress_dialog.show_calls[0]
    assert minimum_value != maximum_value


def test_execute_falls_back_to_default_ws_src_when_blank(monkeypatch):
    progress_dialog = _FakeProgressDialog()
    ui = _FakeUi(progress_dialog)
    monkeypatch.setattr(dc.adsk.core.Application, "get", lambda: SimpleNamespace(userInterface=ui))

    captured_kwargs = {}
    monkeypatch.setattr(
        dc,
        "run_environment_checks",
        lambda **kwargs: captured_kwargs.update(kwargs) or [DoctorCheck("WSL installed", True, "OK")],
    )

    inputs = _FakeInputs({"wsl_ros_ws_src": _FakeStringInput("  "), "check_gazebo": _FakeBoolInput(False)})
    args = SimpleNamespace(command=SimpleNamespace(commandInputs=inputs))

    dc.DoctorCommandExecuteHandler().notify(args)

    assert captured_kwargs["wsl_ros_ws_src"] == dc.DEFAULT_WSL_ROS_WS_SRC
    assert captured_kwargs["check_gazebo"] is False
