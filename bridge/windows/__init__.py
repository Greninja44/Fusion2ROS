"""Windows-side half of the bridge.

Runs under Windows Python inside the Fusion 360 add-in process, and shells
out to `wsl.exe` -- a Windows-only binary. `wsl.exe` does not exist inside
WSL itself, so nothing in this sub-package can be run or tested from a WSL
session.

*** UNVERIFIED ***
Everything in bridge/windows/ was written carefully against documented
`wsl.exe` command-line behavior but has NOT been executed or tested. It was
authored inside a WSL sandbox with no access to an actual Windows Python
process, a real `wsl.exe`, or a live Fusion 360 instance. Treat it as a
best-effort draft that needs real verification on a Windows machine with
Fusion 360 and WSL installed before it's trusted in production. See the
module docstrings in detect.py and invoke.py for the specific assumptions
made.

Nothing here imports anything Windows-only at module scope (no `wsl.exe`
probing, no `import winreg`, etc.) -- all such calls happen lazily inside
functions -- so this package still imports cleanly under plain WSL/Linux
Python, e.g. for `tests/ ... -v` collection on this side.
"""
