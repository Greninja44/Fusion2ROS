"""Real tests for bridge.windows.doctor, run from inside WSL -- same
WSL-interop technique tests/bridge/test_windows_invoke.py already
established (real `wsl.exe`, reachable from inside WSL too on this
machine). See that file's own docstring for the full explanation.

Never touches ~/ros2_ws -- checks either run read-only commands or
`mkdir -p` a throwaway tmp_path.
"""

import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from bridge.windows.doctor import DoctorCheck, all_critical_passed, format_report, run_environment_checks
from bridge.windows.invoke import DEFAULT_DISTRO, DEFAULT_ROS_SETUP, run_in_wsl

WSL_AVAILABLE = shutil.which("wsl.exe") is not None
ROS_SETUP_AVAILABLE = Path("/opt/ros/lyrical/setup.bash").exists()

requires_wsl_exe = pytest.mark.skipif(not WSL_AVAILABLE, reason="wsl.exe not reachable from this machine")
requires_wsl_and_ros = pytest.mark.skipif(
    not (WSL_AVAILABLE and ROS_SETUP_AVAILABLE), reason="wsl.exe and/or ROS setup not available on this machine"
)


def test_all_critical_passed_true_for_empty_or_all_passing_list():
    assert all_critical_passed([]) is True
    assert all_critical_passed([DoctorCheck("a", True, "ok")]) is True


def test_all_critical_passed_false_for_a_failed_critical_check():
    checks = [DoctorCheck("a", True, "ok"), DoctorCheck("b", False, "bad")]
    assert all_critical_passed(checks) is False


def test_all_critical_passed_ignores_non_critical_failures():
    checks = [DoctorCheck("a", True, "ok"), DoctorCheck("b", False, "advisory only", critical=False)]
    assert all_critical_passed(checks) is True


def test_format_report_shows_ok_and_fail_markers():
    checks = [DoctorCheck("wsl", True, "OK"), DoctorCheck("colcon", False, "not on PATH")]
    report = format_report(checks)
    assert "[OK] wsl" in report
    assert "[FAIL] colcon" in report
    assert "not on PATH" in report


def test_run_environment_checks_reports_wsl_not_available(monkeypatch):
    import bridge.windows.doctor as doctor_module

    monkeypatch.setattr(doctor_module, "is_wsl_available", lambda: False)

    checks = doctor_module.run_environment_checks()

    assert len(checks) == 1
    assert checks[0].name == "WSL installed"
    assert checks[0].passed is False
    assert all_critical_passed(checks) is False


@requires_wsl_exe
def test_run_environment_checks_reports_unreachable_distro():
    checks = run_environment_checks(distro="Definitely-Not-A-Real-Distro-Name")

    by_name = {c.name: c for c in checks}
    assert "WSL installed" in by_name and by_name["WSL installed"].passed
    assert not by_name["Distro 'Definitely-Not-A-Real-Distro-Name' reachable"].passed
    # Checks after an unreachable distro should not have run at all.
    assert len(checks) == 2


@requires_wsl_and_ros
def test_run_environment_checks_against_the_real_default_distro(tmp_path):
    """Real, live run against this machine's actual configured distro/ROS
    setup -- not mocked. Doesn't assert the build-probe check passes: this
    project discovered, live, that colcon/CMake's Python interpreter
    resolution on this exact machine is genuinely inconsistent run to run
    (see doctor.py's module docstring and `_colcon_build_probe`'s comment)
    -- a real failure OR a real success here are both correct, faithful
    reporting, not a test bug. What this test verifies instead: every
    OTHER (fast, deterministic) check passes, the check list and report are
    well-formed, and the one genuinely-live-flaky check still returns a
    normal DoctorCheck rather than raising."""
    ws_src = str(tmp_path / "ros2_ws" / "src")
    checks = run_environment_checks(
        distro=DEFAULT_DISTRO, ros_setup=DEFAULT_ROS_SETUP, wsl_ros_ws_src=ws_src, check_gazebo=True
    )
    by_name = {c.name: c for c in checks}

    assert by_name["WSL installed"].passed
    assert by_name[f"Distro {DEFAULT_DISTRO!r} reachable"].passed
    assert by_name[f"ROS 2 setup script {DEFAULT_ROS_SETUP!r}"].passed
    assert by_name["colcon on PATH"].passed

    build_check = by_name["colcon can actually build a package"]
    assert isinstance(build_check.passed, bool)
    if not build_check.passed:
        assert "colcon build" in build_check.detail

    assert by_name[f"Colcon workspace src/ {ws_src!r}"].passed
    assert Path(ws_src).is_dir()  # mkdir -p really ran, not just claimed to

    gz_direct = run_in_wsl("command -v gz", distro=DEFAULT_DISTRO, timeout=15)
    gz_check = by_name["gz (Gazebo Sim) on PATH"]
    assert gz_check.passed == gz_direct.success
    assert gz_check.critical is False

    # format_report must not raise regardless of pass/fail mix, and must
    # mention every check name.
    report = format_report(checks)
    for name in by_name:
        assert name in report


@requires_wsl_and_ros
def test_check_gazebo_false_by_default_omits_the_gazebo_check(tmp_path):
    checks = run_environment_checks(
        distro=DEFAULT_DISTRO, ros_setup=DEFAULT_ROS_SETUP, wsl_ros_ws_src=str(tmp_path / "src")
    )
    assert not any("Gazebo" in c.name for c in checks)
