"""Real tests for bridge.windows.invoke, run from inside WSL.

This directly contradicts what this project long assumed: bridge.windows
was believed untestable from inside WSL (see bridge/wsl_side's own test
file, which said so) because it's meant to run under WINDOWS Python inside
the Fusion 360 add-in process. But `wsl.exe` -- the one thing that module
actually depends on -- is reachable via genuine Windows/WSL interop from
*inside* WSL too (this machine has real `wsl.exe` on PATH at
/mnt/c/WINDOWS/system32/wsl.exe, and it works correctly when invoked this
way -- confirmed live, not assumed). That's not the same as running under
an actual Windows Python process inside a real Fusion 360 session (still
unverified -- see bridge/windows/__init__.py), but it IS a real, working
wsl.exe doing real cross-boundary work, which is what these functions
compose. Gated on shutil.which("wsl.exe") so this degrades gracefully
(skips) on a machine without that interop (e.g. a Linux-only CI runner).

Never touches ~/ros2_ws -- every workspace here is a throwaway path under
tmp_path or /tmp, matching the discipline the rest of this project follows.
"""

import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from bridge.windows.invoke import (
    WslResult,
    _windows_path_to_wsl_path,
    build_package_in_wsl,
    launch_ros2_in_wsl,
    run_in_wsl,
)

WSL_AVAILABLE = shutil.which("wsl.exe") is not None
ROS_SETUP_AVAILABLE = Path("/opt/ros/lyrical/setup.bash").exists()
COLCON_AVAILABLE = shutil.which("colcon") is not None

requires_wsl_exe = pytest.mark.skipif(not WSL_AVAILABLE, reason="wsl.exe not reachable from this machine")
requires_wsl_and_ros = pytest.mark.skipif(
    not (WSL_AVAILABLE and ROS_SETUP_AVAILABLE and COLCON_AVAILABLE),
    reason="wsl.exe and/or colcon/ROS setup not available on this machine",
)


@requires_wsl_exe
def test_run_in_wsl_executes_a_real_command():
    result = run_in_wsl("echo hello_from_wsl && exit 0", timeout=15)

    assert isinstance(result, WslResult)
    assert result.success is True
    assert result.returncode == 0
    assert "hello_from_wsl" in result.stdout
    # Regression check for the module's own documented UTF-16LE-vs-UTF-8
    # decoding risk: plain ASCII output must decode cleanly, not come back
    # mangled (e.g. null-byte-interleaved) the way misapplied UTF-16LE
    # decoding would produce.
    assert "\x00" not in result.stdout


@requires_wsl_exe
def test_run_in_wsl_reports_nonzero_exit():
    result = run_in_wsl("exit 7", timeout=15)

    assert result.success is False
    assert result.returncode == 7


@requires_wsl_exe
def test_windows_path_to_wsl_path_translates_unc_path():
    # A UNC path pointing into this machine's own WSL distro must resolve
    # to the exact WSL-native equivalent -- checked by actually reading a
    # real file through the translated path, not just string-matching.
    wsl_path = _windows_path_to_wsl_path(
        r"\\wsl.localhost\Ubuntu-26.04\home\batman\Fusion2ROS\README.md", distro="Ubuntu-26.04"
    )
    assert wsl_path == "/home/batman/Fusion2ROS/README.md"

    result = run_in_wsl(f"test -f {wsl_path} && echo FOUND", timeout=15)
    assert "FOUND" in result.stdout


@requires_wsl_and_ros
def test_build_package_in_wsl_end_to_end(tmp_path):
    # Real, end-to-end: generate an actual package, express its path as a
    # Windows-side UNC path (exactly the form the real bridge would pass in
    # production), and build it in a THROWAWAY workspace -- never ~/ros2_ws.
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from examples.sample_arm import build_sample_arm
    from fusion_addin.app import generate_ros_package

    robot = build_sample_arm()
    package_dir = generate_ros_package(robot, {}, tmp_path / "generated")

    # Any WSL-side absolute path is reachable from "Windows" (i.e. via
    # wsl.exe interop) at \\wsl.localhost\<distro>\<same path, backslashed>
    # -- see docs/ARCHITECTURE.md's "Windows/WSL wiring" section. tmp_path
    # lives under /tmp, not /home/batman, so build the UNC form generically
    # from the absolute path rather than assuming a fixed prefix.
    windows_style_path = r"\\wsl.localhost\Ubuntu-26.04" + str(package_dir).replace("/", "\\")

    throwaway_ws_src = f"/tmp/{tmp_path.name}_bridge_ws/src"
    try:
        result = build_package_in_wsl(windows_style_path, throwaway_ws_src, timeout=120)

        assert isinstance(result, WslResult)
        assert result.success is True, f"stdout={result.stdout}\nstderr={result.stderr}"
        assert "Finished" in result.stdout or "finished" in result.stdout.lower()
    finally:
        shutil.rmtree(f"/tmp/{tmp_path.name}_bridge_ws", ignore_errors=True)


@requires_wsl_exe
def test_launch_ros2_in_wsl_detects_immediate_failure(tmp_path):
    # Real, end-to-end verification of the "did the launch die immediately"
    # detection mechanism -- deliberately launches a nonexistent launch
    # file (no package/build needed for this check) and confirms the
    # composed shell script correctly detects the fast failure and relays
    # the real ros2-launch error text.
    result = launch_ros2_in_wsl(
        "definitely_not_a_real_package",
        "nonexistent.launch.py",
        f"/tmp/{tmp_path.name}_launch_test_ws/src",
        settle_seconds=1.5,
        timeout=20,
    )

    assert result.success is False
    assert result.returncode != 0
    assert "likely failed" in result.stderr
