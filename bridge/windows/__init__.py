"""Windows-side half of the bridge.

Meant to run under Windows Python inside the Fusion 360 add-in process, and
shells out to `wsl.exe` -- a Windows-only binary. Originally assumed
`wsl.exe` "does not exist inside WSL itself" and so nothing here could be
run or tested from a WSL session -- WRONG, corrected after actually trying
it: genuine Windows/WSL interop makes the real `wsl.exe`
(/mnt/c/WINDOWS/system32/wsl.exe on this machine) reachable and fully
functional from *inside* WSL too. `invoke.py`'s `run_in_wsl`,
`_windows_path_to_wsl_path` (UNC-path branch), and `build_package_in_wsl`
are now real-tested this way (see tests/bridge/test_windows_invoke.py) --
including a genuine `colcon build` of an actual generated package through
the full copy+build composition.

*** STILL UNVERIFIED: running under an actual Windows Python process
inside a real Fusion 360 add-in ***, as opposed to wsl.exe reached via
WSL-side interop (which is a real, working, but different code path --
same `wsl.exe` binary, different calling process). `detect.py` and the
`wslpath -u` native-Windows-path-translation branch of `invoke.py` remain
untested for the same reason detect.py's own docstring gives (nothing to
detect from inside WSL: WSL is definitionally already available if this
process exists) or because this WSL-side test environment has no reason to
construct a genuine `C:\\...` path. See detect.py's and invoke.py's own
module docstrings for the remaining specific assumptions.

Nothing here imports anything Windows-only at module scope (no `wsl.exe`
probing, no `import winreg`, etc.) -- all such calls happen lazily inside
functions -- so this package still imports cleanly under plain WSL/Linux
Python, e.g. for `tests/ ... -v` collection on this side.
"""
