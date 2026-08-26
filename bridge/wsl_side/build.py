"""WSL-side build steps of the bridge.

This module runs as a normal Linux process inside WSL (invoked, in
production, by the Windows-side bridge over `wsl.exe`; see
bridge/windows/invoke.py). Everything here is real stdlib code, fully
testable and tested for real in this environment -- see
tests/bridge/test_wsl_side.py.

Two responsibilities, matching the two steps in the "Bridge workflow"
diagram in docs/ARCHITECTURE.md:

1. copy_package_to_workspace -- copy a generated ROS 2 package tree
   (output/<robot_name>/ on the Windows side, already transferred into WSL
   by the time this runs) into a colcon workspace's src/ directory.
2. colcon_build -- run `colcon build --packages-select <package>` inside
   that workspace and report back a structured result.

Neither function knows anything about ~/ros2_ws specifically -- both take
the workspace path as an argument, so tests can point them at a throwaway
tmp_path workspace instead.
"""

from __future__ import annotations

import shlex
import shutil
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional


def copy_package_to_workspace(package_dir: Path, ros_ws_src: Path) -> Path:
    """Copy `package_dir` into `ros_ws_src/<package_dir.name>/`.

    - Replaces any existing copy at the destination cleanly (remove, then
      copy), so a stale file left over from a previous generation of the
      package can never survive a re-copy.
    - Does not follow symlinks destructively: symlinks inside package_dir
      are preserved as symlinks in the copy (`symlinks=True`) rather than
      being dereferenced and their target's contents copied in -- and the
      destination-clearing step never walks through a symlink at the
      destination root either (see below).
    - Never touches anything outside `ros_ws_src/<package_dir.name>/`:
      the only path removed or written is the destination directory itself
      (`ros_ws_src` itself is created if missing, but nothing already
      inside it other than the target package directory is touched).

    Returns the destination path.
    """
    package_dir = Path(package_dir)
    ros_ws_src = Path(ros_ws_src)

    if not package_dir.is_dir():
        raise NotADirectoryError(f"package_dir does not exist or is not a directory: {package_dir}")

    package_name = package_dir.name
    if package_name in ("", ".", ".."):
        raise ValueError(f"refusing to copy package with unsafe name: {package_name!r}")

    ros_ws_src.mkdir(parents=True, exist_ok=True)
    dest = ros_ws_src / package_name

    # Clear the destination first, without following a symlink that might
    # point outside ros_ws_src (rmtree on a symlinked directory would
    # delete the *target's* contents, not the link -- so unlink the link
    # itself instead if that's what we find).
    if dest.is_symlink():
        dest.unlink()
    elif dest.exists():
        shutil.rmtree(dest)

    shutil.copytree(package_dir, dest, symlinks=True)
    return dest


@dataclass
class BuildResult:
    """Structured result of a colcon build invocation."""

    success: bool
    stdout: str
    stderr: str
    returncode: int


def colcon_build(
    ros_ws: Path,
    package_name: str,
    ros_setup: Path = Path("/opt/ros/lyrical/setup.bash"),
    timeout: float = 300.0,
    on_output_line: Optional[Callable[[str], None]] = None,
) -> BuildResult:
    """Run `colcon build --packages-select <package_name>` in `ros_ws`.

    Always scopes the build with --packages-select -- this must never build
    the whole workspace unscoped, since ros_ws may contain unrelated
    packages (in production, ~/ros2_ws is a live, working, unrelated
    project with ~10 packages; building it unscoped would be both slow and
    invasive).

    Sources `ros_setup` first so `colcon`/`ament` environment variables are
    set up, matching the documented bridge workflow:
        bash -lc "source <ros_setup> && cd <ros_ws> && colcon build --packages-select <package_name>"

    Runs via `bash -lc` (a login shell) so the invocation matches exactly
    what the Windows-side bridge sends through `wsl.exe ... bash -lc "..."`.

    Returns a BuildResult even on timeout or launch failure (e.g. missing
    `bash`/`colcon`) rather than raising, so callers (ultimately the Fusion
    UI) always get a reportable BUILD SUCCESS / BUILD FAILED outcome.

    on_output_line (default None, opt-in): if given, called once per line
    of stdout/stderr AS THE BUILD RUNS (not just after it finishes) -- a
    colcon build of anything non-trivial takes real wall-clock time, and
    without this a caller (the Fusion "Build in WSL" command, in
    particular) has nothing to show but a frozen-looking dialog until the
    whole thing completes. When set, this switches from the simple
    subprocess.run() blocking call to a threaded Popen + line-reader pair
    (stdout and stderr each pumped on their own thread so neither can fill
    its OS pipe buffer and deadlock the other) -- the DEFAULT (no callback)
    path is untouched and still uses subprocess.run(), so existing callers
    see zero behavior change. Interleaving between stdout/stderr lines as
    delivered to the callback is not guaranteed (they're read on separate
    threads) -- the final BuildResult.stdout/stderr are still each fully
    separate and in their own original order, exactly as before.
    """
    ros_ws = Path(ros_ws)
    command = (
        f"source {shlex.quote(str(ros_setup))} && "
        f"cd {shlex.quote(str(ros_ws))} && "
        f"colcon build --packages-select {shlex.quote(package_name)}"
    )

    if on_output_line is None:
        try:
            proc = subprocess.run(
                ["bash", "-lc", command],
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout if isinstance(exc.stdout, str) else (exc.stdout or b"").decode(errors="replace")
            stderr = exc.stderr if isinstance(exc.stderr, str) else (exc.stderr or b"").decode(errors="replace")
            stderr += f"\n[bridge] colcon build timed out after {timeout}s"
            return BuildResult(success=False, stdout=stdout, stderr=stderr, returncode=-1)
        except OSError as exc:
            return BuildResult(
                success=False, stdout="", stderr=f"[bridge] failed to launch build: {exc}", returncode=-1
            )

        return BuildResult(
            success=(proc.returncode == 0),
            stdout=proc.stdout,
            stderr=proc.stderr,
            returncode=proc.returncode,
        )

    try:
        proc = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
    except OSError as exc:
        return BuildResult(success=False, stdout="", stderr=f"[bridge] failed to launch build: {exc}", returncode=-1)

    stdout_lines: list = []
    stderr_lines: list = []

    def _pump(stream, sink: list) -> None:
        for line in iter(stream.readline, ""):
            sink.append(line)
            on_output_line(line.rstrip("\n"))
        stream.close()

    stdout_thread = threading.Thread(target=_pump, args=(proc.stdout, stdout_lines), daemon=True)
    stderr_thread = threading.Thread(target=_pump, args=(proc.stderr, stderr_lines), daemon=True)
    stdout_thread.start()
    stderr_thread.start()

    timed_out = False
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        timed_out = True
        proc.kill()
        proc.wait()

    # The pump threads see EOF (readline returns "") once the process's
    # pipes close, which happens at/around process exit -- give them a
    # bounded moment to drain rather than joining forever.
    stdout_thread.join(timeout=5)
    stderr_thread.join(timeout=5)

    stdout = "".join(stdout_lines)
    stderr = "".join(stderr_lines)

    if timed_out:
        stderr += f"\n[bridge] colcon build timed out after {timeout}s"
        return BuildResult(success=False, stdout=stdout, stderr=stderr, returncode=-1)

    return BuildResult(
        success=(proc.returncode == 0),
        stdout=stdout,
        stderr=stderr,
        returncode=proc.returncode,
    )
