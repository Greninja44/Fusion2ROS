"""Pre-flight environment checks for the WSL/ROS 2 side of the bridge --
run BEFORE "Build in WSL" (or a chained Generate->Build->Launch) so a broken
environment produces one clear, itemized explanation instead of a cryptic
mid-build failure.

Directly motivated by two real failures hit live during this project's own
testing, neither of which `is_wsl_available()` (detect.py) catches, since
that only checks that *some* distro is registered:

1. The Fusion "Build in WSL" dialog's default distro name didn't match what
   was actually registered/running on the machine.
2. `colcon build` failing with `ModuleNotFoundError: No module named
   'catkin_pkg'` -- root-caused to a `uv`-managed Python
   (`~/.local/bin/python3.12`) landing ahead of the system Python on the
   login-shell `PATH` (e.g. via a `uv`-installed `.bashrc` line), so CMake's
   `find_package(Python3)` resolved to an interpreter that has never had
   `catkin_pkg` installed into it -- `colcon build`/`ament_package()` then
   fails for EVERY package, not just this project's, with no indication the
   real cause is a PATH/interpreter mismatch rather than a broken package.

Every check here runs the SAME way `bridge.windows.invoke.run_in_wsl` always
has -- `wsl.exe -d <distro> -- bash -lc "<command>"`, a genuine login shell
-- specifically so check #2's `python3 -c "import catkin_pkg"` probe sees
the exact same PATH resolution a real `colcon build` invocation would (both
`build_package_in_wsl` and this module route through `run_in_wsl` the same
way), rather than whatever interpreter happens to run the calling process.

Pure function of `bridge.windows.invoke`/`detect` -- no Fusion API, no
filesystem writes -- testable the same "WSL interop makes wsl.exe reachable
from inside WSL too" way `tests/bridge/test_windows_invoke.py` already
established (see that file's own docstring).
"""

from __future__ import annotations

import shlex
from dataclasses import dataclass
from typing import List

from .detect import is_wsl_available
from .invoke import DEFAULT_DISTRO, DEFAULT_ROS_SETUP, DEFAULT_WSL_ROS_WS_SRC, WslResult, run_in_wsl

# A minimal, real ament_cmake package (no source, no dependencies beyond
# ament_cmake itself) used only to actually run `colcon build` end to end as
# a probe -- see `_colcon_build_probe`'s docstring for why a real build,
# not a `python3 -c "import catkin_pkg"` guess, is what this checks.
_PROBE_PACKAGE_NAME = "fusion2ros_doctor_probe"
_PROBE_PACKAGE_XML = f"""<?xml version="1.0"?>
<package format="3">
  <name>{_PROBE_PACKAGE_NAME}</name>
  <version>0.0.0</version>
  <description>Throwaway package Fusion2ROS's environment doctor uses to verify colcon/ament_cmake can build a package here at all.</description>
  <maintainer email="doctor@fusion2ros.local">Fusion2ROS Doctor</maintainer>
  <license>N/A</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
_PROBE_CMAKELISTS = f"""cmake_minimum_required(VERSION 3.10)
project({_PROBE_PACKAGE_NAME})
find_package(ament_cmake REQUIRED)
ament_package()
"""


@dataclass
class DoctorCheck:
    """One environment check's outcome.

    `critical` (default True): if False, a failure is reported as advisory
    only (e.g. Gazebo not installed -- irrelevant if the user never checks
    "Include Gazebo") rather than something that should block a build.
    """

    name: str
    passed: bool
    detail: str
    critical: bool = True


def _colcon_build_probe(distro: str, ros_setup: str, timeout: float) -> WslResult:
    """Actually run `colcon build` on a minimal, real, throwaway ament_cmake
    package inside `distro`, in a dedicated `/tmp` workspace removed
    afterwards either way. This is the only fully faithful way to check
    "can this environment build a ROS 2 package at all" -- see the caller's
    comment for why probing a specific Python interpreter directly isn't
    equivalent."""
    ws = "/tmp/fusion2ros_doctor_probe_ws"
    pkg_dir = f"{ws}/src/{_PROBE_PACKAGE_NAME}"
    command = (
        f"rm -rf {shlex.quote(ws)} && mkdir -p {shlex.quote(pkg_dir)} && "
        f"cat > {shlex.quote(pkg_dir)}/package.xml <<'FUSION2ROS_DOCTOR_EOF'\n"
        f"{_PROBE_PACKAGE_XML}"
        f"FUSION2ROS_DOCTOR_EOF\n"
        f"cat > {shlex.quote(pkg_dir)}/CMakeLists.txt <<'FUSION2ROS_DOCTOR_EOF'\n"
        f"{_PROBE_CMAKELISTS}"
        f"FUSION2ROS_DOCTOR_EOF\n"
        f"source {shlex.quote(ros_setup)} && cd {shlex.quote(ws)} && "
        f"colcon build --packages-select {_PROBE_PACKAGE_NAME}; "
        f"status=$?; rm -rf {shlex.quote(ws)}; exit $status"
    )
    return run_in_wsl(command, distro=distro, timeout=timeout)


def run_environment_checks(
    distro: str = DEFAULT_DISTRO,
    ros_setup: str = DEFAULT_ROS_SETUP,
    wsl_ros_ws_src: str = DEFAULT_WSL_ROS_WS_SRC,
    check_gazebo: bool = False,
) -> List[DoctorCheck]:
    """Run every check and return the full list (not just failures) --
    callers can render an all-clear summary just as easily as an itemized
    failure list. Never raises: a check that can't even run (e.g. `wsl.exe`
    itself missing) is reported as a normal failed DoctorCheck, same as any
    other, so a caller never needs a separate try/except around this.
    """
    checks: List[DoctorCheck] = []

    if not is_wsl_available():
        checks.append(
            DoctorCheck(
                "WSL installed",
                False,
                "wsl.exe is not on PATH, or no WSL distro is registered/runnable. "
                "Install WSL and a distro, then try again.",
            )
        )
        # Every later check shells out via wsl.exe -- pointless to keep
        # going once it's confirmed absent.
        return checks
    checks.append(DoctorCheck("WSL installed", True, "wsl.exe found, at least one distro registered."))

    distro_result = run_in_wsl("true", distro=distro, timeout=15)
    checks.append(
        DoctorCheck(
            f"Distro {distro!r} reachable",
            distro_result.success,
            "OK"
            if distro_result.success
            else (
                f"Could not run a command in distro {distro!r} -- it may not be registered "
                f"under that exact name (check `wsl.exe -l -v` for the real name) or not "
                f"currently startable. Detail: {distro_result.stderr.strip()}"
            ),
        )
    )
    if not distro_result.success:
        return checks

    ros_setup_result = run_in_wsl(f"test -f {shlex.quote(ros_setup)}", distro=distro, timeout=15)
    checks.append(
        DoctorCheck(
            f"ROS 2 setup script {ros_setup!r}",
            ros_setup_result.success,
            "OK" if ros_setup_result.success else f"{ros_setup!r} does not exist inside {distro!r}.",
        )
    )

    colcon_result = run_in_wsl(f"source {shlex.quote(ros_setup)} && command -v colcon", distro=distro, timeout=15)
    checks.append(
        DoctorCheck(
            "colcon on PATH",
            colcon_result.success,
            "OK" if colcon_result.success else "`colcon` is not on PATH after sourcing the ROS 2 setup script.",
        )
    )

    # THE real bug found live (see module docstring, cause #2): checked with
    # an ACTUAL colcon build of a throwaway package, not a guess at which
    # interpreter to probe. A `python3 -c "import catkin_pkg"` check was
    # tried first and rejected: it passed even while colcon build itself
    # kept failing, because CMake's own `find_package(Python3)`/
    # `find_program` interpreter resolution (used by ament_cmake's
    # package_xml_2_cmake.py, which is what actually needs catkin_pkg) does
    # not necessarily pick the same interpreter a plain `python3` on PATH
    # resolves to -- confirmed live on this exact machine: a login shell's
    # `python3` was 3.14 (fine), while CMake's build nonetheless invoked a
    # DIFFERENT `python3.12` found earlier on PATH (a uv-managed one
    # lacking catkin_pkg) for reasons never fully pinned down. A real
    # `colcon build` is the only fully faithful reproduction of what a real
    # build will actually do.
    build_probe = _colcon_build_probe(distro, ros_setup, timeout=60)
    checks.append(
        DoctorCheck(
            "colcon can actually build a package",
            build_probe.success,
            "OK"
            if build_probe.success
            else (
                "A real `colcon build` of a minimal throwaway package failed -- every real "
                "package will fail to build too. Usually caused by CMake's Python3 interpreter "
                "resolution picking a different `python3` than the one on PATH (e.g. a "
                "uv/pyenv-managed interpreter lacking `catkin_pkg`) -- check `which -a "
                f"python3*` inside the distro. Detail:\n{build_probe.stderr.strip()}"
            ),
        )
    )

    ws_result = run_in_wsl(f"mkdir -p {shlex.quote(wsl_ros_ws_src)}", distro=distro, timeout=15)
    checks.append(
        DoctorCheck(
            f"Colcon workspace src/ {wsl_ros_ws_src!r}",
            ws_result.success,
            "OK (exists or created)" if ws_result.success else f"Could not create {wsl_ros_ws_src!r}.",
        )
    )

    if check_gazebo:
        gz_result = run_in_wsl("command -v gz", distro=distro, timeout=15)
        checks.append(
            DoctorCheck(
                "gz (Gazebo Sim) on PATH",
                gz_result.success,
                "OK" if gz_result.success else "`gz` is not on PATH -- Gazebo generation will produce a package "
                "that can't actually be simulated on this machine.",
                critical=False,
            )
        )

    return checks


def format_report(checks: List[DoctorCheck]) -> str:
    """Render `checks` as a plain-text report -- one line per check, failed
    critical checks' detail shown in full underneath. Suitable for a Fusion
    `ui.messageBox` call or a plain terminal print."""
    lines = []
    failed_critical = [c for c in checks if not c.passed and c.critical]
    failed_advisory = [c for c in checks if not c.passed and not c.critical]

    if not failed_critical:
        lines.append("All critical checks passed." if checks else "No checks ran.")
    else:
        lines.append(f"{len(failed_critical)} critical check(s) FAILED -- fix these before building:")

    for check in checks:
        marker = "OK" if check.passed else ("FAIL" if check.critical else "warn")
        lines.append(f"  [{marker}] {check.name}")
        if not check.passed:
            lines.append(f"        {check.detail}")

    return "\n".join(lines)


def all_critical_passed(checks: List[DoctorCheck]) -> bool:
    return all(c.passed for c in checks if c.critical)
