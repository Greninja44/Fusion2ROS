"""Run commands inside WSL from Windows Python (the Fusion 360 add-in
process), and compose the copy+build steps of the bridge workflow.

*** UNVERIFIED -- see bridge/windows/__init__.py ***
Written against documented `wsl.exe` command-line behavior only. Never run
against a real `wsl.exe`, a real Windows Python process, or a real Fusion
360 session. Specific ASSUMPTIONS made here, to be the first things checked
if this misbehaves on a real machine:

  * `wsl.exe -d <distro> -- bash -lc "<command>"` is a valid, supported
    invocation form that runs `<command>` in a login shell inside the named
    distro and relays its exit code as `wsl.exe`'s own exit code. This
    matches the documented workflow in docs/ARCHITECTURE.md ("Bridge
    workflow") and public `wsl.exe` docs, but the exact quoting/escaping
    behavior when Windows' subprocess layer, then `wsl.exe`, then `bash -lc`
    each re-parse the command string has NOT been exercised here. Complex
    commands with nested quotes are a known sharp edge for this chain; we
    keep composed commands as simple as possible (plain paths quoted with
    shlex.quote, no embedded quoting of quoting) to minimize the risk.
  * `wsl.exe`'s captured stdout/stderr may be UTF-16LE-encoded when piped
    (see detect.py's `_decode` for the same issue) -- we defensively decode
    the same way here rather than assuming UTF-8.
  * This module intentionally does NOT import anything from
    bridge/wsl_side/ (even though both are part of the same repo) --
    Windows Python and WSL Python are different interpreters/environments,
    so bridge/wsl_side code cannot run in this process. Composed commands
    therefore re-express the *same two steps* (copy, then colcon build) as
    plain shell text sent through `wsl.exe`, rather than literally
    delegating to bridge.wsl_side.build. Keeping the shell text as close as
    possible to what build.py does (same colcon invocation shape, same
    --packages-select requirement) is a deliberate way to keep behavior
    consistent even though the two can't share code across the OS boundary.
  * A Windows-side path such as `\\\\wsl.localhost\\<distro>\\home\\...` is
    assumed to map directly to the WSL-native path by stripping the UNC
    prefix (this is how Windows exposes a WSL distro's own filesystem back
    to Windows -- see docs/ARCHITECTURE.md "Windows/WSL wiring"). A genuine
    Windows-native path (e.g. `C:\\Users\\...\\output\\my_robot`) is instead
    translated with `wslpath -u`, run inside the target distro. This
    UNC-vs-native branch has not been tested against a real `wsl.exe`.
"""

from __future__ import annotations

import posixpath
import shlex
import shutil
import subprocess
from dataclasses import dataclass
from typing import Optional

DEFAULT_DISTRO = "Ubuntu-26.04"
DEFAULT_ROS_SETUP = "/opt/ros/lyrical/setup.bash"
# Matches docs/ARCHITECTURE.md's "Existing ROS workspace" (~/ros2_ws, already
# used as this project's real build/launch target during development).
DEFAULT_WSL_ROS_WS_SRC = "~/ros2_ws/src"


@dataclass
class WslResult:
    """Result of a command run inside WSL via wsl.exe.

    Deliberately mirrors bridge.wsl_side.build.BuildResult's shape
    (success/stdout/stderr/returncode) for a consistent "how did it go"
    interface on both sides of the bridge, without this module importing
    that one (see module docstring: the two Python environments can't share
    code across the OS boundary anyway).
    """

    success: bool
    stdout: str
    stderr: str
    returncode: int


def _decode(raw: bytes) -> str:
    """Best-effort decode of wsl.exe output. Mirrors detect._decode /
    detect._looks_like_utf16le: wsl.exe's own subcommand output
    (--status, -l -v) can be UTF-16LE when piped, but output relayed from a
    command run *inside* the distro (which is everything this module
    captures, via `-- bash -lc "..."`) is that command's own encoding
    (normally UTF-8) passed through unchanged. Detect rather than assume,
    since assuming utf-16-le unconditionally corrupts ordinary UTF-8
    command output (confirmed empirically in this sandbox -- see
    module docstring)."""
    if not raw:
        return ""
    sample = raw[:200]
    odd_bytes = sample[1::2]
    looks_utf16le = len(odd_bytes) >= 2 and sum(1 for b in odd_bytes if b == 0) / len(odd_bytes) > 0.8
    if looks_utf16le:
        try:
            return raw.decode("utf-16-le").replace("\ufeff", "")
        except UnicodeDecodeError:
            pass
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("utf-8", errors="replace")


def run_in_wsl(
    command: str,
    distro: str = DEFAULT_DISTRO,
    timeout: float = 300.0,
) -> WslResult:
    """Run `command` inside WSL as `bash -lc "<command>"`, via wsl.exe.

    Equivalent to (per docs/ARCHITECTURE.md's bridge workflow diagram):
        wsl.exe -d <distro> -- bash -lc "<command>"

    Returns a WslResult even on timeout or launch failure (e.g. wsl.exe not
    on PATH) rather than raising, so callers always get a reportable
    outcome. Callers should check `is_wsl_available()` (detect.py) before
    relying on this in a UI flow, but this function itself stays defensive
    regardless.
    """
    wsl_path = shutil.which("wsl.exe")
    if wsl_path is None:
        return WslResult(success=False, stdout="", stderr="wsl.exe not found on PATH", returncode=-1)

    argv = [wsl_path, "-d", distro, "--", "bash", "-lc", command]
    try:
        proc = subprocess.run(argv, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return WslResult(
            success=False,
            stdout="",
            stderr=f"wsl.exe command timed out after {timeout}s",
            returncode=-1,
        )
    except OSError as exc:
        return WslResult(success=False, stdout="", stderr=f"failed to launch wsl.exe: {exc}", returncode=-1)

    return WslResult(
        success=(proc.returncode == 0),
        stdout=_decode(proc.stdout),
        stderr=_decode(proc.stderr),
        returncode=proc.returncode,
    )


def _windows_path_to_wsl_path(path: str, distro: str) -> str:
    """Translate a Windows-side path string to its WSL-native path.

    Handles two cases (see module docstring for caveats on both):
      * A `\\\\wsl.localhost\\<distro>\\...` (or legacy `\\\\wsl$\\<distro>\\...`)
        UNC path, which already points at a location *inside* the WSL
        filesystem -- strip the UNC prefix and flip slashes.
      * A genuine Windows-native path (e.g. `C:\\...`) -- ask `wslpath -u`
        (run inside the target distro, via run_in_wsl) to translate it.
    """
    normalized = path.replace("/", "\\")
    for prefix_root in ("\\\\wsl.localhost\\", "\\\\wsl$\\"):
        if normalized.lower().startswith(prefix_root.lower()):
            remainder = normalized[len(prefix_root):]
            # remainder is "<distro>\home\batman\...": drop the distro segment.
            parts = remainder.split("\\", 1)
            rest = parts[1] if len(parts) > 1 else ""
            return "/" + rest.replace("\\", "/")

    result = run_in_wsl(f"wslpath -u {shlex.quote(path)}", distro=distro, timeout=15)
    if not result.success:
        raise RuntimeError(f"could not translate Windows path {path!r} to a WSL path: {result.stderr}")
    return result.stdout.strip()


def build_package_in_wsl(
    windows_package_path: str,
    wsl_ros_ws_src: str,
    distro: str = DEFAULT_DISTRO,
    ros_setup: str = DEFAULT_ROS_SETUP,
    package_name: Optional[str] = None,
    timeout: float = 300.0,
) -> WslResult:
    """Copy a generated package into a WSL colcon workspace and build it.

    Composes, remotely (over wsl.exe), the same two steps as
    bridge.wsl_side.build (copy_package_to_workspace, then colcon_build) --
    see this module's docstring for why they're re-expressed as shell text
    here rather than literally calling that module.

    Args:
        windows_package_path: Windows-side path to the generated package,
            e.g. a UNC `\\\\wsl.localhost\\Ubuntu-26.04\\home\\...\\output\\my_robot`
            path, or a native `C:\\...\\output\\my_robot` path. This is the
            *source*, analogous to `package_dir` in
            bridge.wsl_side.build.copy_package_to_workspace.
        wsl_ros_ws_src: WSL-side path string to the workspace's `src/`
            directory, e.g. `~/ros2_ws/src`. The workspace root that colcon
            builds from is taken to be this path's parent (stripping a
            trailing `src` component), matching standard colcon workspace
            layout.
        package_name: Overrides the package name (defaults to the last
            path segment of windows_package_path).

    Returns a single WslResult: if the copy step fails, that failure is
    returned directly (build is not attempted); otherwise the build step's
    result is returned. This is the value a Fusion UI panel should render
    as BUILD SUCCESS (`result.success`) / BUILD FAILED + `result.stderr`.

    THIS COMPOSITION HAS NOT BEEN RUN END TO END. It is a thin,
    best-effort chaining of already-unverified pieces (path translation,
    run_in_wsl) -- treat it as a draft to validate on a real Windows +
    WSL + Fusion 360 machine, not as tested behavior.
    """
    trimmed = windows_package_path.rstrip("/\\")
    inferred_name = trimmed.replace("/", "\\").split("\\")[-1]
    name = package_name or inferred_name
    if not name:
        return WslResult(success=False, stdout="", stderr=f"could not infer package name from {windows_package_path!r}", returncode=-1)

    try:
        wsl_package_path = _windows_path_to_wsl_path(windows_package_path, distro)
    except RuntimeError as exc:
        return WslResult(success=False, stdout="", stderr=str(exc), returncode=-1)

    ws_src = wsl_ros_ws_src.rstrip("/")
    ws_root = posixpath.dirname(ws_src) if posixpath.basename(ws_src) == "src" else ws_src
    dest = posixpath.join(ws_src, name)

    copy_command = (
        f"mkdir -p {shlex.quote(ws_src)} && "
        f"rm -rf {shlex.quote(dest)} && "
        f"cp -aT {shlex.quote(wsl_package_path)} {shlex.quote(dest)}"
    )
    copy_result = run_in_wsl(copy_command, distro=distro, timeout=timeout)
    if not copy_result.success:
        copy_result.stderr = f"[copy step failed] {copy_result.stderr}"
        return copy_result

    build_command = (
        f"source {shlex.quote(ros_setup)} && "
        f"cd {shlex.quote(ws_root)} && "
        f"colcon build --packages-select {shlex.quote(name)}"
    )
    return run_in_wsl(build_command, distro=distro, timeout=timeout)


def launch_ros2_in_wsl(
    package_name: str,
    launch_file: str,
    wsl_ros_ws_src: str,
    distro: str = DEFAULT_DISTRO,
    ros_setup: str = DEFAULT_ROS_SETUP,
    settle_seconds: float = 2.0,
    timeout: float = 30.0,
) -> WslResult:
    """Run `ros2 launch <package_name> <launch_file>` inside WSL, after
    sourcing both the base ROS 2 install (`ros_setup`) and the target colcon
    workspace's own `install/setup.bash` (the one `build_package_in_wsl`
    just built) -- needed because `ros2 launch` can only find a package's
    launch files once its own workspace overlay is sourced, not just the
    base ROS 2 distro. `wsl_ros_ws_src` uses the same "workspace root is
    this path's parent, unless it's already the root" convention as
    `build_package_in_wsl`.

    A GUI launch (e.g. `display.launch.py`, which starts RViz2) blocks until
    the user closes the window, so this deliberately does NOT wait for the
    launched process to exit -- it starts it detached in the background
    (`nohup ... & disown`) and, after `settle_seconds`, checks only whether
    the backgrounded process is *still running* (as a quick, best-effort
    "did it crash on startup" signal -- e.g. package not found, launch file
    missing, a node erroring out immediately) before returning. A True
    `WslResult.success` here means "the launch command started and didn't
    immediately die", NOT "RViz successfully opened a visible window" --
    this Windows-side process has no way to confirm a GUI actually rendered
    (see docs/ARCHITECTURE.md's own note on WSLg screenshot limitations,
    hit in a different context during this project). Captured
    stdout/stderr, on failure, is whatever the process printed in that
    window, redirected to a temp log file and cat'd back.

    THIS HAS NOT BEEN RUN END TO END -- same "unverified, best-effort draft"
    status as build_package_in_wsl (see this module's docstring), plus one
    extra untested assumption of its own: that `$!` (the backgrounded PID)
    survives correctly through this exact `bash -lc "... & disown; ..."`
    chain when invoked non-interactively via `wsl.exe`. Treat the
    settle-and-poll result as a hint, not a guarantee, until checked live.
    """
    ws_src = wsl_ros_ws_src.rstrip("/")
    ws_root = posixpath.dirname(ws_src) if posixpath.basename(ws_src) == "src" else ws_src
    install_setup = posixpath.join(ws_root, "install", "setup.bash")
    # ROS 2 package names are conventionally simple identifiers
    # ([a-z][a-z0-9_]*, per REP 144); no extra sanitizing attempted beyond
    # that convention for this log filename.
    log_file = f"/tmp/fusion2ros_launch_{package_name}.log"

    command = (
        f"source {shlex.quote(ros_setup)} && "
        f"source {shlex.quote(install_setup)} && "
        f"nohup ros2 launch {shlex.quote(package_name)} {shlex.quote(launch_file)} "
        f"> {shlex.quote(log_file)} 2>&1 & "
        f"disown; "
        f"pid=$!; "
        f"sleep {shlex.quote(str(settle_seconds))}; "
        f"if kill -0 \"$pid\" 2>/dev/null; then "
        f"echo 'Fusion2ROS: launch started (pid '\"$pid\"'), still running after {settle_seconds}s.'; "
        f"else "
        f"echo 'Fusion2ROS: launch process exited within {settle_seconds}s -- likely failed.' >&2; "
        f"cat {shlex.quote(log_file)} >&2; "
        f"exit 1; "
        f"fi"
    )
    return run_in_wsl(command, distro=distro, timeout=timeout)
