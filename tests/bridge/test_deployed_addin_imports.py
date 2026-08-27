"""Regression test for the real live-Fusion bug this project hit: Fusion
loads the deployed FusionAddins/Fusion2ROS/ folder with ONLY that folder's
parent on sys.path, so every absolute import fusion_addin/ makes of a
repo-root package (bridge.windows, robot_model, ros2_tools.validate) must
actually be deployed alongside it, or it's a ModuleNotFoundError raised at
add-in load time -- silently disabling the add-in with no error dialog (see
Fusion2ROS.py's own docstring for exactly why that happens).

This reproduces Fusion's real import context -- a fresh sys.path containing
only the deployed folder's parent, adsk mocked -- against a tmp_path deploy
produced by the real sync_addin functions, so it fails the same way the
live bug did if anyone re-adds an absolute repo-root import without wiring
its sync_*_into() counterpart.
"""

import importlib
import sys
from pathlib import Path
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from bridge.windows.sync_addin import (
    DEFAULT_SOURCE,
    sync_addin_to_fusion,
    sync_bridge_windows_into,
    sync_ros2_tools_validate_into,
    sync_robot_model_into,
)


@pytest.fixture
def deployed_addin(tmp_path):
    dest = tmp_path / "FusionAddins" / "Fusion2ROS"
    sync_addin_to_fusion(DEFAULT_SOURCE, dest)
    sync_bridge_windows_into(dest)
    sync_robot_model_into(dest)
    sync_ros2_tools_validate_into(dest)
    return dest


def test_deployed_addin_imports_cleanly_under_simulated_fusion_syspath(deployed_addin, monkeypatch):
    addins_parent = deployed_addin.parent

    monkeypatch.setattr(sys, "path", [p for p in sys.path if "Fusion2ROS" not in p and p != ""])
    sys.path.insert(0, str(addins_parent))

    monkeypatch.setitem(sys.modules, "adsk", MagicMock())
    monkeypatch.setitem(sys.modules, "adsk.core", MagicMock())
    monkeypatch.setitem(sys.modules, "adsk.fusion", MagicMock())

    for mod_name in list(sys.modules):
        if mod_name == "Fusion2ROS" or mod_name.startswith("Fusion2ROS."):
            del sys.modules[mod_name]

    try:
        module = importlib.import_module("Fusion2ROS.Fusion2ROS")
    finally:
        for mod_name in list(sys.modules):
            if mod_name == "Fusion2ROS" or mod_name.startswith("Fusion2ROS."):
                del sys.modules[mod_name]

    assert hasattr(module, "run")
    assert hasattr(module, "stop")
