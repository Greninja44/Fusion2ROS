"""Optional YAML config file for repeated `scripts/generate_from_json.py`
CLI flags -- which optional outputs to include, a default output directory,
and default `package.xml` maintainer/license metadata -- so a user
generating the same fleet of robots over and over doesn't have to repeat
`--ros2-control --gazebo --moveit ...` on every invocation.

Pure stdlib + PyYAML (already a dependency -- see moveit.py, and the tests
that verify generated YAML by parsing it back). No Fusion API calls, no
ROS/rclpy imports -- importable and testable the same way every other
module directly under `fusion_addin/` is (see package.py's module
docstring for why that constraint exists project-wide).

File format (every key optional -- an empty file, or no file at all, is
valid and means "no defaults, use built-in behavior"):

    output_dir: output/
    include:
      ros2_control: true
      gazebo: false
      moveit: true
      nav2: false
    moveit_group: arm
    maintainer:
      name: Jane Doe
      email: jane@example.com
    license: Apache-2.0

Lookup order (see `resolve_config`): an explicit `--config PATH` if given,
else `<cwd>/.fusion2ros.yaml`, else `~/.fusion2ros.yaml`, else no config at
all (every field left unset).

Precedence rule once a config is loaded (see `resolve_value`, used by the
CLI): an explicit CLI flag always wins; otherwise the config file's value
is used; otherwise a built-in default. There is deliberately no way for a
config file to force a value the CLI can't override -- it only ever fills
in what the user didn't ask for explicitly on that particular invocation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

CONFIG_FILENAME = ".fusion2ros.yaml"

_KNOWN_TOP_LEVEL_KEYS = {"output_dir", "include", "moveit_group", "maintainer", "license"}
_KNOWN_INCLUDE_KEYS = {"ros2_control", "gazebo", "moveit", "nav2"}
_KNOWN_MAINTAINER_KEYS = {"name", "email"}


class ConfigError(Exception):
    """A config file was found but is malformed (bad YAML, wrong shape, an
    unknown key, a value of the wrong type) -- an expected, actionable
    user-facing problem, not a bug, same spirit as
    robot_model.errors.ValidationError."""


@dataclass
class Fusion2ROSConfig:
    """All fields default to None, meaning "not set by this config" --
    `resolve_value` treats None as "fall through to the next thing in the
    precedence chain", never as a real value of "off"/"empty string"."""

    output_dir: Optional[str] = None
    include_ros2_control: Optional[bool] = None
    include_gazebo: Optional[bool] = None
    include_moveit: Optional[bool] = None
    include_nav2: Optional[bool] = None
    moveit_group_name: Optional[str] = None
    maintainer_name: Optional[str] = None
    maintainer_email: Optional[str] = None
    license: Optional[str] = None
    source_path: Optional[Path] = None  # None means "no file was loaded"


def default_config_search_paths(cwd: Optional[Path] = None, home: Optional[Path] = None) -> List[Path]:
    """Where `resolve_config` looks when no `--config PATH` is given, in
    order: the current directory first (a per-project config, likely
    checked into that project's own repo), then the user's home directory
    (a personal, cross-project default). `cwd`/`home` are only for tests --
    real callers should leave both as None (Path.cwd() / Path.home())."""
    cwd = Path(cwd) if cwd is not None else Path.cwd()
    home = Path(home) if home is not None else Path.home()
    return [cwd / CONFIG_FILENAME, home / CONFIG_FILENAME]


def find_config_file(cwd: Optional[Path] = None, home: Optional[Path] = None) -> Optional[Path]:
    """Returns the first config file that exists among
    `default_config_search_paths`, or None if neither exists."""
    for candidate in default_config_search_paths(cwd, home):
        if candidate.is_file():
            return candidate
    return None


def _expect_str(value: Any, key: str, path: Path) -> str:
    if not isinstance(value, str):
        raise ConfigError(f"config file '{path}': '{key}' must be a string, got {value!r}")
    return value


def _expect_bool(value: Any, key: str, path: Path) -> bool:
    # Deliberately not `bool(value)` -- YAML's own `true`/`false` already
    # parse as real Python bools, so anything that *isn't* a bool here
    # (e.g. the string "yes", or 1) is much more likely a typo than
    # deliberate, and should be reported rather than silently coerced.
    if not isinstance(value, bool):
        raise ConfigError(f"config file '{path}': '{key}' must be true/false, got {value!r}")
    return value


def load_config_file(path: Path) -> Fusion2ROSConfig:
    """Parse one YAML config file into a Fusion2ROSConfig. Never raises a
    raw yaml.YAMLError/OSError/TypeError -- everything becomes a
    ConfigError with a one-line, actionable message naming the file and
    the offending key."""
    import yaml  # local import, like moveit.py's -- keep this module (and

    # everything that merely imports it, e.g. generate_from_json.py)
    # importable even in a stripped-down environment that lacks PyYAML,
    # as long as nobody actually points it at a config file.

    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ConfigError(f"could not read config file '{path}': {exc}") from None

    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f"config file '{path}' is not valid YAML: {exc}") from None

    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ConfigError(
            f"config file '{path}' must be a YAML mapping at the top level, got {type(raw).__name__}"
        )

    unknown = set(raw) - _KNOWN_TOP_LEVEL_KEYS
    if unknown:
        raise ConfigError(
            f"config file '{path}' has unknown key(s): {', '.join(sorted(unknown))} "
            f"(expected some of: {', '.join(sorted(_KNOWN_TOP_LEVEL_KEYS))})"
        )

    cfg = Fusion2ROSConfig(source_path=path)

    if "output_dir" in raw:
        cfg.output_dir = _expect_str(raw["output_dir"], "output_dir", path)

    if "include" in raw:
        include = raw["include"]
        if not isinstance(include, dict):
            raise ConfigError(
                f"config file '{path}': 'include' must be a mapping, got {type(include).__name__}"
            )
        unknown_include = set(include) - _KNOWN_INCLUDE_KEYS
        if unknown_include:
            raise ConfigError(
                f"config file '{path}': unknown include key(s): {', '.join(sorted(unknown_include))} "
                f"(expected some of: {', '.join(sorted(_KNOWN_INCLUDE_KEYS))})"
            )
        for yaml_key, attr in (
            ("ros2_control", "include_ros2_control"),
            ("gazebo", "include_gazebo"),
            ("moveit", "include_moveit"),
            ("nav2", "include_nav2"),
        ):
            if yaml_key in include:
                setattr(cfg, attr, _expect_bool(include[yaml_key], f"include.{yaml_key}", path))

    if "moveit_group" in raw:
        cfg.moveit_group_name = _expect_str(raw["moveit_group"], "moveit_group", path)

    if "maintainer" in raw:
        maintainer = raw["maintainer"]
        if not isinstance(maintainer, dict):
            raise ConfigError(
                f"config file '{path}': 'maintainer' must be a mapping, got {type(maintainer).__name__}"
            )
        unknown_maintainer = set(maintainer) - _KNOWN_MAINTAINER_KEYS
        if unknown_maintainer:
            raise ConfigError(
                f"config file '{path}': unknown maintainer key(s): {', '.join(sorted(unknown_maintainer))} "
                f"(expected: {', '.join(sorted(_KNOWN_MAINTAINER_KEYS))})"
            )
        if "name" in maintainer:
            cfg.maintainer_name = _expect_str(maintainer["name"], "maintainer.name", path)
        if "email" in maintainer:
            cfg.maintainer_email = _expect_str(maintainer["email"], "maintainer.email", path)

    if "license" in raw:
        cfg.license = _expect_str(raw["license"], "license", path)

    return cfg


def resolve_config(
    explicit_path: Optional[Path] = None,
    cwd: Optional[Path] = None,
    home: Optional[Path] = None,
) -> Fusion2ROSConfig:
    """Load the config file that applies to this run.

    - `explicit_path` (the CLI's `--config PATH`): must exist, or this
      raises ConfigError -- naming a config file explicitly and having it
      silently ignored (e.g. a typo'd path) would be far more confusing
      than a clear error.
    - otherwise, the first of `default_config_search_paths` that exists.
    - if none exists, returns an all-None Fusion2ROSConfig (source_path is
      None) -- not an error: no config file at all is the common case, and
      every field's None already means "fall through to the built-in
      default" via `resolve_value`.
    """
    if explicit_path is not None:
        explicit_path = Path(explicit_path)
        if not explicit_path.is_file():
            raise ConfigError(f"--config file does not exist: '{explicit_path}'")
        return load_config_file(explicit_path)

    found = find_config_file(cwd, home)
    if found is None:
        return Fusion2ROSConfig()
    return load_config_file(found)


def resolve_value(cli_value: Any, config_value: Any, builtin_default: Any) -> Any:
    """The one precedence rule this module exists to express: an explicit
    CLI value (not None) wins; else the config file's value (not None)
    wins; else `builtin_default`. Used once per setting by the CLI so the
    rule stays in exactly one place rather than being reimplemented ad hoc
    per flag."""
    if cli_value is not None:
        return cli_value
    if config_value is not None:
        return config_value
    return builtin_default
