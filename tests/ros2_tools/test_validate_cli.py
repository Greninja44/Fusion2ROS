"""Tests for ros2_tools.validate.cli's `main()` -- the actual command-line
entry point (`python3 -m ros2_tools.validate <path>`), as opposed to
test_validate.py which exercises validate_package_structure/
validate_urdf_file directly. Covers exit codes, stderr messages, and the
narrow OSError-to-clear-message wrapping around the initial path probe
(neither validator ever raises for a bad *target*, only for the filesystem
itself misbehaving on the way to it -- see cli.py's own comment).

Plain `python3 -m pytest`, no Fusion, no ROS, no network.
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ros2_tools.validate import cli

VALID_URDF = """<?xml version="1.0"?>
<robot name="demo">
  <link name="base_link"/>
</robot>
"""

INVALID_URDF = "<robot name=\"demo\"><link name=\"a\"/><link name=\"a\"/></robot>"


def test_main_returns_zero_for_valid_urdf(tmp_path, capsys):
    urdf_path = tmp_path / "demo.urdf"
    urdf_path.write_text(VALID_URDF)

    exit_code = cli.main([str(urdf_path)])

    assert exit_code == 0
    assert capsys.readouterr().out == ""


def test_main_returns_one_and_prints_problems_for_invalid_urdf(tmp_path, capsys):
    urdf_path = tmp_path / "bad.urdf"
    urdf_path.write_text(INVALID_URDF)

    exit_code = cli.main([str(urdf_path)])

    assert exit_code == 1
    assert "duplicate link name" in capsys.readouterr().out


def test_main_reports_clear_error_for_missing_path(tmp_path, capsys):
    exit_code = cli.main([str(tmp_path / "does_not_exist.urdf")])

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "does not exist" in err
    assert "does_not_exist.urdf" in err


def test_main_validates_a_package_directory(tmp_path, capsys):
    pkg_dir = tmp_path / "demo_pkg"
    pkg_dir.mkdir()
    # An empty directory is missing package.xml, CMakeLists.txt, urdf/ --
    # validate_package_structure reports each as a separate problem, never
    # raises.
    exit_code = cli.main([str(pkg_dir)])

    assert exit_code == 1
    out = capsys.readouterr().out
    assert "package.xml is missing" in out
    assert "CMakeLists.txt is missing" in out


def test_main_wraps_oserror_from_exists_probe_as_clear_error(tmp_path, monkeypatch, capsys):
    # Simulates the filesystem itself misbehaving (permission error, a
    # dangling network mount, ...) on the very first exists() check --
    # this must become the same kind of one-line stderr message as "does
    # not exist", not a raw traceback.
    target = tmp_path / "unreadable.urdf"

    def _raise(self):
        raise OSError("simulated permission error")

    monkeypatch.setattr(Path, "exists", _raise)

    exit_code = cli.main([str(target)])

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "could not access" in err
    assert "simulated permission error" in err


def test_main_wraps_oserror_from_is_dir_probe_as_clear_error(tmp_path, monkeypatch, capsys):
    target = tmp_path / "weird.urdf"
    target.write_text(VALID_URDF)

    def _raise(self):
        raise OSError("simulated stat failure")

    monkeypatch.setattr(Path, "is_dir", _raise)

    exit_code = cli.main([str(target)])

    assert exit_code == 1
    err = capsys.readouterr().err
    assert "could not read" in err
    assert "simulated stat failure" in err


def test_main_lets_unexpected_exceptions_propagate(tmp_path, monkeypatch):
    # Anything other than OSError escaping the validators is a real bug,
    # not an expected user-facing failure mode -- must NOT be swallowed
    # into a clean exit code.
    urdf_path = tmp_path / "demo.urdf"
    urdf_path.write_text(VALID_URDF)

    def _boom(path):
        raise RuntimeError("unexpected bug")

    monkeypatch.setattr(cli, "validate_urdf_file", _boom)

    with pytest.raises(RuntimeError, match="unexpected bug"):
        cli.main([str(urdf_path)])
