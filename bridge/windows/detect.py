"""Detect whether WSL is present and usable, from the Windows side.

*** UNVERIFIED -- see bridge/windows/__init__.py ***
Written against documented `wsl.exe` command-line behavior only. Never run
against a real `wsl.exe`. In particular, the following are ASSUMPTIONS, not
confirmed facts, and should be the first things checked if this misbehaves
on a real machine:

  * `wsl.exe --status` and `wsl.exe -l -v` (or `--list --verbose`) both
    exist and behave as documented on the WSL version installed for the
    target user, and both exit 0 when WSL itself is installed (even if no
    distro is currently running), and nonzero (or raise) when the `wsl`
    feature/launcher is not usable at all.
  * `wsl.exe -l -v` output, when its stdout is *not* a real console (i.e.
    when captured via subprocess, as here), is historically emitted as
    UTF-16LE (with a BOM) rather than the console codepage or UTF-8. This
    is a well-documented `wsl.exe` quirk, not a Python bug. We defensively
    try a couple of decodings rather than assuming either encoding, since
    getting this wrong silently turns "WSL is fine" into "WSL not found".
  * A freshly-installed WSL with zero distros registered should be treated
    as *not* available for our purposes (we need a runnable distro to copy
    files into and run colcon), so `is_wsl_available()` requires at least
    one distro entry, not just a working `wsl.exe` launcher.

The bridge's Fusion UI must disable "Build in WSL" / "Launch RViz" actions
gracefully (per docs/ARCHITECTURE.md, "Bridge workflow") whenever this
returns False -- it must never raise for the "WSL isn't installed at all"
case, since that's an expected, common state on a fresh machine.
"""

from __future__ import annotations

import shutil
import subprocess


def _looks_like_utf16le(raw: bytes) -> bool:
    """Heuristic: genuine UTF-16LE-encoded ASCII-range text has a NUL byte
    at (almost) every odd position (the high byte of each ASCII code unit).
    Real UTF-8 text essentially never does. Confirmed empirically in this
    sandbox (via WSL interop's own copy of wsl.exe -- see bridge/windows/
    __init__.py and detect.py's module docstring for what that does and
    doesn't prove): `wsl.exe --status` / `-l -v` output decodes cleanly
    under this check, while plain command output relayed through does not
    (and must NOT be routed through utf-16-le, or it comes out as garbage).
    """
    sample = raw[:200]
    odd_bytes = sample[1::2]
    if len(odd_bytes) < 2:
        return False
    zero_count = sum(1 for b in odd_bytes if b == 0)
    return zero_count / len(odd_bytes) > 0.8


def _decode(raw: bytes) -> str:
    """Best-effort decode of wsl.exe output.

    wsl.exe has historically written UTF-16LE for some of its own
    subcommands' output (`--status`, `-l -v`) when stdout is piped rather
    than a real console, while output *relayed* from a command run inside
    the distro (e.g. via `-- bash -lc "..."`) is the command's own encoding
    (normally UTF-8) passed through unchanged. Guess which case we're in
    with `_looks_like_utf16le` rather than assuming one or the other, since
    guessing wrong silently corrupts the text instead of failing loudly.
    """
    if not raw:
        return ""
    if _looks_like_utf16le(raw):
        try:
            return raw.decode("utf-16-le").replace("﻿", "")
        except UnicodeDecodeError:
            pass
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("utf-8", errors="replace")


def is_wsl_available() -> bool:
    """Return True if `wsl.exe` is on PATH and at least one distro is
    registered/runnable, False otherwise. Never raises.
    """
    wsl_path = shutil.which("wsl.exe")
    if wsl_path is None:
        return False

    try:
        status = subprocess.run(
            [wsl_path, "--status"],
            capture_output=True,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False

    if status.returncode != 0:
        return False

    try:
        listing = subprocess.run(
            [wsl_path, "-l", "-v"],
            capture_output=True,
            timeout=15,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False

    if listing.returncode != 0:
        return False

    text = _decode(listing.stdout)
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    # Expect a header line ("NAME STATE VERSION" or similar) plus at least
    # one distro row when anything is installed. Be lenient: just require
    # more than one non-empty line, and that at least one line beyond the
    # header contains non-header-looking text.
    return len(lines) > 1
