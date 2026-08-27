"""Regression test for the real bug found live: BuildCommandExecuteHandler
called progress_dialog.show(..., 0, 0, 0), and Fusion's real ProgressDialog
rejects minimumValue == maximumValue outright:

    RuntimeError: 3 : invalid argument minimumValue or maximumValue

There is no documented indeterminate/marquee mode for ProgressDialog (per
ProgressDialog_show.htm) -- the fix is the same (0, 1, 0) 1-step range
command.py's GenerateCommandExecuteHandler already uses successfully. This
test mocks just enough of adsk.core to exercise the real notify() call path
and assert the min/max values passed to .show() are no longer equal.
"""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

sys.modules.setdefault("adsk", MagicMock())
sys.modules.setdefault("adsk.core", MagicMock())
sys.modules.setdefault("adsk.fusion", MagicMock())

# build_command.py subclasses adsk.core.CommandEventHandler /
# CommandCreatedEventHandler. A bare MagicMock is NOT usable as a base
# class -- Python's PEP 560 __mro_entries__ protocol is invoked via plain
# getattr(), so MagicMock auto-generates it, and the resulting class
# silently degenerates into another MagicMock instead of a real class with
# a working notify() method (confirmed empirically: type(BuildCommand
# ExecuteHandler) came back as MagicMock, and instantiating+calling it did
# nothing at all, no exception, no calls -- this is what earlier tests in
# this project never hit, since they only exercised free functions, never
# instantiated an adsk.core.*EventHandler subclass directly). Real empty
# classes fix it.
sys.modules["adsk"].core.CommandEventHandler = type("CommandEventHandler", (), {})
sys.modules["adsk"].core.CommandCreatedEventHandler = type("CommandCreatedEventHandler", (), {})

from fusion_addin.ui import build_command as bc  # noqa: E402


class _FakeStringInput:
    def __init__(self, value):
        self.value = value


class _FakeInputs:
    def __init__(self, values: dict):
        self._values = {k: _FakeStringInput(v) for k, v in values.items()}

    def itemById(self, input_id):
        return self._values[input_id]


class _FakeProgressDialog:
    def __init__(self):
        self.isCancelButtonShown = None
        self.message = ""
        self.show_calls = []

    def show(self, title, message, minimum_value, maximum_value, delay):
        self.show_calls.append((title, message, minimum_value, maximum_value, delay))

    def hide(self):
        pass


class _FakeUi:
    def __init__(self, progress_dialog):
        self._progress_dialog = progress_dialog
        self.messages = []

    def createProgressDialog(self):
        return self._progress_dialog

    def messageBox(self, text):
        self.messages.append(text)


def test_execute_shows_progress_dialog_with_distinct_min_and_max(monkeypatch):
    progress_dialog = _FakeProgressDialog()
    ui = _FakeUi(progress_dialog)

    monkeypatch.setattr(bc, "is_wsl_available", lambda: True)
    monkeypatch.setattr(bc.adsk.core.Application, "get", lambda: SimpleNamespace(userInterface=ui))
    monkeypatch.setattr(
        bc,
        "build_package_in_wsl",
        lambda *a, **k: SimpleNamespace(success=True, stdout="ok", stderr="", returncode=0),
    )

    inputs = _FakeInputs(
        {
            "windows_package_path": "/home/batman/output/my_robot",
            "wsl_ros_ws_src": "~/ros2_ws/src",
        }
    )
    args = SimpleNamespace(command=SimpleNamespace(commandInputs=inputs))

    handler = bc.BuildCommandExecuteHandler()
    handler.notify(args)

    assert ui.messages == ["Fusion2ROS: BUILD SUCCESS\n\nok"]
    assert len(progress_dialog.show_calls) == 1
    _, _, minimum_value, maximum_value, _ = progress_dialog.show_calls[0]
    assert minimum_value != maximum_value
