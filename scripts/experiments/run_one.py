#!/usr/bin/env python3
"""Run one reproducible HGA-SP experiment.

The solver is invoked with an argv list (never through a shell), so instance,
depot-configuration, and output paths may safely contain spaces.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import platform
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


SCHEMA_VERSION = 2
SUCCESS_STATUS = "success"
CPP_INT_MAX = 2_147_483_647
SP_SEED_MAX = 2_000_000_000

_ENVIRONMENT_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_SENSITIVE_ENVIRONMENT_NAME = re.compile(
    r"(?:TOKEN|SECRET|PASSWORD|PASSWD|LICENSE|ACCESS_?ID)",
    re.IGNORECASE,
)
_MANAGED_ENVIRONMENT_KEYS = frozenset(
    {
        "GUROBI_HOME",
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
        "HGASP_SP_THREADS",
        "HGASP_SP_TRIGGER_FREQUENCY",
        "HGASP_SP_TIME_LIMIT_SECONDS",
        "HGASP_SP_MIP_GAP",
        "HGASP_SP_SEED",
        "HGASP_SP_MAX_POOL_SIZE",
        "HGASP_SP_OUTPUT_FLAG",
        "HGASP_SP_MIP_FOCUS",
        "HGASP_SP_HEURISTICS",
        "HGASP_SP_REPRESENTATION_ROUTE_LIMIT",
        "HGASP_LOG_DIR",
        "HGASP_LOG_CONSOLE",
        "HGASP_SP_LOG_FILE",
    }
)
_ACTIVE_PROCESS_LOCK = threading.Lock()
_ACTIVE_PROCESSES: set[subprocess.Popen] = set()


class ExperimentError(RuntimeError):
    """Raised when an experiment cannot be configured safely."""


def utc_now() -> str:
    """Return a stable UTC timestamp suitable for experiment metadata."""

    return (
        dt.datetime.now(dt.timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def json_bytes(value: Any) -> bytes:
    """Serialize JSON deterministically for storage and hashing."""

    return (
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    ).encode("utf-8")


def write_json(path: Path, value: Any) -> None:
    """Atomically write a JSON document."""

    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json_bytes(value)
    temporary_name: Optional[str] = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", dir=str(path.parent), prefix=f".{path.name}.", delete=False
        ) as temporary:
            temporary.write(payload)
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_name = temporary.name
        os.replace(temporary_name, path)
    finally:
        if temporary_name is not None:
            try:
                Path(temporary_name).unlink()
            except FileNotFoundError:
                pass


def load_json(path: Path) -> Dict[str, Any]:
    def reject_non_finite(value: str) -> None:
        raise ExperimentError(
            f"non-finite JSON number {value!r} is not allowed in {path}"
        )

    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream, parse_constant=reject_non_finite)
    except (OSError, json.JSONDecodeError) as error:
        raise ExperimentError(f"cannot read JSON file {path}: {error}") from error
    if not isinstance(value, dict):
        raise ExperimentError(f"expected a JSON object in {path}")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as error:
        raise ExperimentError(f"cannot hash {path}: {error}") from error
    return digest.hexdigest()


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def resolve_path(value: str, base: Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def safe_identifier(value: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9._-]+", "_", value).strip("._")
    if not normalized:
        raise ExperimentError(f"cannot derive a safe identifier from {value!r}")
    return normalized


def require_project_venv(
    repo_root: Path, current_prefix: Optional[Path] = None
) -> None:
    """Reject accidental execution with a base or unrelated Python."""

    expected = (repo_root / ".venv").resolve()
    active = Path(current_prefix or sys.prefix).resolve()
    if active != expected:
        launcher = expected / (
            "Scripts/python.exe" if os.name == "nt" else "bin/python"
        )
        raise ExperimentError(
            "experiment runners must use the repository virtual environment; "
            f"run {launcher} instead of {sys.executable}"
        )


def available_cpu_count() -> int:
    """Return the CPU count available to this process, respecting affinity."""

    process_cpu_count = getattr(os, "process_cpu_count", None)
    if callable(process_cpu_count):
        count = process_cpu_count()
        if count:
            return int(count)
    if hasattr(os, "sched_getaffinity"):
        try:
            affinity = os.sched_getaffinity(0)
        except OSError:
            pass
        else:
            if affinity:
                return len(affinity)
    return os.cpu_count() or 1


def require_integer(
    value: Any,
    field: str,
    minimum: Optional[int] = None,
    maximum: Optional[int] = None,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ExperimentError(f"{field} must be an integer")
    if minimum is not None and value < minimum:
        raise ExperimentError(f"{field} must be at least {minimum}")
    if maximum is not None and value > maximum:
        raise ExperimentError(f"{field} must be at most {maximum}")
    return value


def validate_environment_mapping(configured: Any) -> Dict[str, str]:
    if not isinstance(configured, Mapping):
        raise ExperimentError("environment must be a JSON object")

    validated: Dict[str, str] = {}
    for key, value in configured.items():
        if not isinstance(key, str) or not isinstance(value, str):
            raise ExperimentError(
                "environment must map string names to string values"
            )
        if not _ENVIRONMENT_NAME.fullmatch(key):
            raise ExperimentError(f"invalid environment variable name: {key!r}")
        if "\x00" in value:
            raise ExperimentError(
                f"environment variable {key!r} contains a NUL byte"
            )
        if _SENSITIVE_ENVIRONMENT_NAME.search(key):
            raise ExperimentError(
                f"environment variable {key!r} looks credential-bearing and "
                "would be written to run.json; export it in the parent "
                "environment instead"
            )
        if key in _MANAGED_ENVIRONMENT_KEYS or key.startswith("HGASP_SP_"):
            raise ExperimentError(
                f"environment variable {key!r} is managed by the runner; "
                "set the corresponding manifest field instead"
            )
        validated[key] = value
    return validated


def _run_capture(command: Sequence[str], cwd: Path) -> Tuple[int, str]:
    try:
        completed = subprocess.run(
            list(command),
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=10,
        )
    except (OSError, subprocess.TimeoutExpired):
        return 1, ""
    return completed.returncode, completed.stdout.strip()


def collect_git_provenance(repo_root: Path) -> Dict[str, Any]:
    """Read Git state, including repositories using .git-hga-sp."""

    alternate_git_dir = repo_root / ".git-hga-sp"
    regular_git_entry = repo_root / ".git"

    if alternate_git_dir.is_dir():
        base_command = [
            "git",
            f"--git-dir={alternate_git_dir}",
            f"--work-tree={repo_root}",
        ]
        metadata_dir: Optional[str] = str(alternate_git_dir)
    elif regular_git_entry.exists():
        base_command = ["git", "-C", str(repo_root)]
        metadata_dir = str(regular_git_entry)
    else:
        return {
            "available": False,
            "commit": None,
            "dirty": None,
            "metadata_dir": None,
            "error": "no .git or .git-hga-sp metadata found",
        }

    commit_code, commit_output = _run_capture(
        [*base_command, "rev-parse", "--verify", "HEAD"], repo_root
    )
    status_code, status_output = _run_capture(
        [*base_command, "status", "--porcelain", "--untracked-files=normal"],
        repo_root,
    )
    if commit_code != 0 or status_code != 0:
        return {
            "available": False,
            "commit": commit_output or None,
            "dirty": None,
            "metadata_dir": metadata_dir,
            "error": "Git metadata could not be queried",
        }
    return {
        "available": True,
        "commit": commit_output.splitlines()[0],
        "dirty": bool(status_output),
        "metadata_dir": metadata_dir,
        "error": None,
    }


def _gurobi_version_from_header(home: Path) -> Optional[Dict[str, Any]]:
    roots = [home]
    if (home / "linux64").is_dir():
        roots.append(home / "linux64")
    for root in roots:
        header = root / "include" / "gurobi_c.h"
        if not header.is_file():
            continue
        try:
            contents = header.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        components: List[str] = []
        for name in ("MAJOR", "MINOR", "TECHNICAL"):
            match = re.search(
                rf"^\s*#define\s+GRB_VERSION_{name}\s+(\d+)\s*$",
                contents,
                flags=re.MULTILINE,
            )
            if match is None:
                break
            components.append(match.group(1))
        if len(components) == 3:
            return {
                "version": ".".join(components),
                "home": str(root.resolve()),
                "source": str(header.resolve()),
            }
    return None


def detect_gurobi(configured_home: Optional[str], repo_root: Path) -> Dict[str, Any]:
    """Record the installed Gurobi version without starting an optimization."""

    candidate_homes: List[Path] = []
    if configured_home:
        candidate_homes.append(resolve_path(configured_home, repo_root))
    environment_home = os.environ.get("GUROBI_HOME")
    if environment_home:
        candidate_homes.append(resolve_path(environment_home, repo_root))
    candidate_homes.append(Path("/opt/gurobi-current"))
    try:
        candidate_homes.extend(
            sorted(Path("/opt").glob("gurobi*"), reverse=True)
        )
    except OSError:
        pass

    seen: set = set()
    for candidate in candidate_homes:
        key = str(candidate)
        if key in seen:
            continue
        seen.add(key)
        detected = _gurobi_version_from_header(candidate)
        if detected is not None:
            return detected

    executable_candidates: List[str] = []
    on_path = shutil.which("gurobi_cl")
    if on_path:
        executable_candidates.append(on_path)
    for home in candidate_homes:
        executable_candidates.append(str(home / "bin" / "gurobi_cl"))
        executable_candidates.append(str(home / "linux64" / "bin" / "gurobi_cl"))

    for executable in executable_candidates:
        if not Path(executable).is_file():
            continue
        return_code, output = _run_capture([executable, "--version"], repo_root)
        if return_code == 0 and output:
            match = re.search(r"version\s+([0-9]+(?:\.[0-9]+)+)", output, re.I)
            return {
                "version": match.group(1) if match else output.splitlines()[0],
                "home": str(Path(executable).resolve().parent.parent),
                "source": str(Path(executable).resolve()),
            }

    return {"version": None, "home": configured_home, "source": "not detected"}


def collect_platform_provenance() -> Dict[str, Any]:
    return {
        "system": platform.system(),
        "release": platform.release(),
        "version": platform.version(),
        "machine": platform.machine(),
        "processor": platform.processor() or None,
        "python_version": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "logical_cpu_count": os.cpu_count(),
    }


def build_solver_command(config: Mapping[str, Any]) -> List[str]:
    """Build the HGA-SP positional CLI as a safe argv list."""

    command = [str(config["binary"]), str(config["instance"])]
    depot_config = config.get("depot_config")
    if depot_config:
        command.append(str(depot_config))
    command.extend(
        [
            str(config["solution"]),
            format(float(config.get("best_known", 0.0)), ".17g"),
            "1" if bool(config.get("is_optimal", False)) else "0",
            str(int(config["limits"]["time_seconds"])),
            str(int(config["limits"]["iterations"])),
            str(int(config["seed"])),
        ]
    )
    return command


def _validate_run_config(config: Mapping[str, Any]) -> None:
    binary = Path(str(config["binary"]))
    instance = Path(str(config["instance"]))
    if not binary.is_file():
        raise ExperimentError(f"solver binary does not exist: {binary}")
    if not os.access(binary, os.X_OK):
        raise ExperimentError(f"solver binary is not executable: {binary}")
    if not instance.is_file():
        raise ExperimentError(f"instance does not exist: {instance}")
    depot = config.get("depot_config")
    if depot and not Path(str(depot)).is_file():
        raise ExperimentError(f"depot configuration does not exist: {depot}")

    limits = config.get("limits")
    resources = config.get("resources")
    sp = config.get("sp")
    if (
        not isinstance(limits, Mapping)
        or not isinstance(resources, Mapping)
        or not isinstance(sp, Mapping)
    ):
        raise ExperimentError("run config requires limits, resources, and sp objects")
    require_integer(
        config.get("seed"), "seed", minimum=0, maximum=CPP_INT_MAX
    )
    best_known = config.get("best_known", 0.0)
    if (
        isinstance(best_known, bool)
        or not isinstance(best_known, (int, float))
        or not math.isfinite(float(best_known))
    ):
        raise ExperimentError("best_known must be a finite number")
    require_integer(
        limits.get("time_seconds"),
        "limits.time_seconds",
        minimum=0,
        maximum=CPP_INT_MAX,
    )
    require_integer(
        limits.get("iterations"),
        "limits.iterations",
        minimum=1,
        maximum=CPP_INT_MAX,
    )
    timeout = limits.get("process_timeout_seconds")
    if timeout is not None and (
        isinstance(timeout, bool)
        or not isinstance(timeout, (int, float))
        or not math.isfinite(float(timeout))
        or timeout <= 0
    ):
        raise ExperimentError(
            "limits.process_timeout_seconds must be finite and positive"
        )
    threads = require_integer(
        resources.get("gurobi_threads"),
        "resources.gurobi_threads",
        minimum=1,
        maximum=CPP_INT_MAX,
    )
    available_cpus = available_cpu_count()
    if threads > available_cpus:
        raise ExperimentError(
            f"resources.gurobi_threads ({threads}) exceeds "
            f"{available_cpus} available logical CPUs"
        )
    validate_environment_mapping(config.get("environment", {}))
    validate_sp_config(sp)


def validate_sp_config(sp: Mapping[str, Any]) -> None:
    boolean_fields = ("representation_route_limit",)
    for field in boolean_fields:
        if not isinstance(sp.get(field), bool):
            raise ExperimentError(f"sp.{field} must be boolean")

    integer_ranges = {
        "trigger_frequency": (1, CPP_INT_MAX),
        "seed": (0, SP_SEED_MAX),
        "max_pool_size": (1, CPP_INT_MAX),
        "output_flag": (0, 1),
        "mip_focus": (0, 3),
    }
    for field, (minimum, maximum) in integer_ranges.items():
        value = sp.get(field)
        if isinstance(value, bool) or not isinstance(value, int):
            raise ExperimentError(f"sp.{field} must be an integer")
        if value < minimum or (maximum is not None and value > maximum):
            upper = f" and at most {maximum}" if maximum is not None else ""
            raise ExperimentError(
                f"sp.{field} must be at least {minimum}{upper}"
            )

    numeric_ranges = {
        "time_limit_seconds": (0.0, None, False),
        "mip_gap": (0.0, 1.0, True),
        "heuristics": (0.0, 1.0, True),
    }
    for field, (minimum, maximum, include_minimum) in numeric_ranges.items():
        value = sp.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ExperimentError(f"sp.{field} must be numeric")
        if not math.isfinite(float(value)):
            raise ExperimentError(f"sp.{field} must be finite")
        if (value < minimum if include_minimum else value <= minimum) or (
            maximum is not None and value > maximum
        ):
            lower_relation = "at least" if include_minimum else "greater than"
            upper = f" and at most {maximum}" if maximum is not None else ""
            raise ExperimentError(
                f"sp.{field} must be {lower_relation} {minimum}{upper}"
            )


def _read_previous_run(path: Path) -> Optional[Dict[str, Any]]:
    if not path.is_file():
        return None
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _terminate_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        if os.name == "posix":
            os.killpg(process.pid, signal.SIGTERM)
        else:
            process.terminate()
        process.wait(timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        try:
            if os.name == "posix":
                os.killpg(process.pid, signal.SIGKILL)
            else:
                process.kill()
        except OSError:
            pass
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass


def _register_process(process: subprocess.Popen) -> None:
    with _ACTIVE_PROCESS_LOCK:
        _ACTIVE_PROCESSES.add(process)


def _unregister_process(process: subprocess.Popen) -> None:
    with _ACTIVE_PROCESS_LOCK:
        _ACTIVE_PROCESSES.discard(process)


def active_process_count() -> int:
    with _ACTIVE_PROCESS_LOCK:
        return sum(process.poll() is None for process in _ACTIVE_PROCESSES)


def terminate_active_processes() -> None:
    """Terminate every solver process currently owned by this runner."""

    with _ACTIVE_PROCESS_LOCK:
        processes = list(_ACTIVE_PROCESSES)
    for process in processes:
        _terminate_process(process)


def _environment_overrides(config: Mapping[str, Any]) -> Dict[str, str]:
    overrides = validate_environment_mapping(config.get("environment", {}))

    # Avoid accidental nested parallelism in common numerical runtimes.
    threads = str(int(config["resources"]["gurobi_threads"]))
    sp = config["sp"]
    overrides.update(
        {
            "OMP_NUM_THREADS": threads,
            "OPENBLAS_NUM_THREADS": threads,
            "MKL_NUM_THREADS": threads,
            "NUMEXPR_NUM_THREADS": threads,
            "HGASP_SP_THREADS": threads,
            "HGASP_SP_TRIGGER_FREQUENCY": str(sp["trigger_frequency"]),
            "HGASP_SP_TIME_LIMIT_SECONDS": format(
                float(sp["time_limit_seconds"]), ".15g"
            ),
            "HGASP_SP_MIP_GAP": format(float(sp["mip_gap"]), ".15g"),
            "HGASP_SP_SEED": str(sp["seed"]),
            "HGASP_SP_MAX_POOL_SIZE": str(sp["max_pool_size"]),
            "HGASP_SP_OUTPUT_FLAG": str(sp["output_flag"]),
            "HGASP_SP_MIP_FOCUS": str(sp["mip_focus"]),
            "HGASP_SP_HEURISTICS": format(float(sp["heuristics"]), ".15g"),
            "HGASP_SP_REPRESENTATION_ROUTE_LIMIT": (
                "1" if sp["representation_route_limit"] else "0"
            ),
        }
    )
    if config.get("gurobi_home"):
        overrides["GUROBI_HOME"] = str(config["gurobi_home"])
    return overrides


def execute_run(
    config: Dict[str, Any],
    run_dir: Path,
    repo_root: Path,
    force_successful: bool = False,
    shared_provenance: Optional[Mapping[str, Any]] = None,
    cancellation_event: Optional[threading.Event] = None,
) -> Dict[str, Any]:
    """Execute one run and return its final run.json document."""

    if cancellation_event is not None and cancellation_event.is_set():
        raise ExperimentError("run cancelled before launch")
    _validate_run_config(config)
    if shared_provenance is None:
        source_provenance: Dict[str, Any] = {
            "git": collect_git_provenance(repo_root),
            "platform": collect_platform_provenance(),
            "gurobi": detect_gurobi(config.get("gurobi_home"), repo_root),
        }
    else:
        source_provenance = {
            "git": shared_provenance["git"],
            "platform": shared_provenance["platform"],
            "gurobi": shared_provenance["gurobi"],
        }
    run_dir.mkdir(parents=True, exist_ok=True)

    config_path = run_dir / "config.resolved.json"
    run_path = run_dir / "run.json"
    solution_path = Path(str(config["solution"]))
    config_payload = json_bytes(config)
    config_digest = sha256_bytes(config_payload)
    hashes: Dict[str, Optional[str]] = {
        "binary_sha256": sha256_file(Path(str(config["binary"]))),
        "instance_sha256": sha256_file(Path(str(config["instance"]))),
        "depot_config_sha256": None,
        "config_sha256": config_digest,
    }
    if config.get("depot_config"):
        hashes["depot_config_sha256"] = sha256_file(
            Path(str(config["depot_config"]))
        )

    previous = _read_previous_run(run_path)
    if previous and previous.get("status") == SUCCESS_STATUS:
        previous_hashes = previous.get("provenance", {}).get("hashes", {})
        same_inputs = all(
            previous_hashes.get(name) == digest
            for name, digest in hashes.items()
        )
        has_solution = solution_path.is_file() and solution_path.stat().st_size > 0
        if same_inputs and has_solution and not force_successful:
            skipped = dict(previous)
            skipped["skipped"] = True
            return skipped
        if not same_inputs and not force_successful:
            raise ExperimentError(
                f"{run_dir} contains a successful run with a different "
                "configuration; choose a new experiment name or pass "
                "--force-successful"
            )

    attempt = int(previous.get("attempt", 0)) + 1 if previous else 1
    attempt_log_dir = run_dir / "logs" / f"attempt_{attempt:03d}"
    while attempt_log_dir.exists():
        attempt += 1
        attempt_log_dir = run_dir / "logs" / f"attempt_{attempt:03d}"
    attempt_log_dir.mkdir(parents=True, exist_ok=False)
    stdout_path = attempt_log_dir / "stdout.log"
    stderr_path = attempt_log_dir / "stderr.log"
    hgasp_log_path = attempt_log_dir / "hgasp.log"
    gurobi_log_path = attempt_log_dir / "gurobi.log"
    config_path.write_bytes(config_payload)

    command = build_solver_command(config)
    environment_overrides = _environment_overrides(config)
    environment_overrides.update(
        {
            "HGASP_LOG_DIR": str(attempt_log_dir),
            "HGASP_LOG_CONSOLE": "1",
            "HGASP_SP_LOG_FILE": str(gurobi_log_path),
        }
    )
    process_environment = os.environ.copy()
    process_environment.update(environment_overrides)

    provenance = {
        "git": source_provenance["git"],
        "hashes": hashes,
        "platform": source_provenance["platform"],
        "gurobi": source_provenance["gurobi"],
    }
    result: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "experiment_name": config["experiment_name"],
        "run_id": config["run_id"],
        "instance_id": config["instance_id"],
        "seed": int(config["seed"]),
        "attempt": attempt,
        "status": "running",
        "started_at": utc_now(),
        "finished_at": None,
        "duration_seconds": None,
        "return_code": None,
        "timed_out": False,
        "command": command,
        "cwd": str(repo_root),
        "environment_overrides": environment_overrides,
        "files": {
            "resolved_config": str(config_path),
            "log_directory": str(attempt_log_dir),
            "stdout": str(stdout_path),
            "stderr": str(stderr_path),
            "hgasp": str(hgasp_log_path),
            "gurobi": str(gurobi_log_path),
            "solution": str(solution_path),
        },
        "solution_exists": False,
        "solution_size_bytes": 0,
        "provenance": provenance,
        "error": None,
    }
    write_json(run_path, result)

    # A failed prior attempt must not make a later failed launch look successful.
    try:
        solution_path.unlink()
    except FileNotFoundError:
        pass
    except OSError as error:
        raise ExperimentError(f"cannot replace prior solution {solution_path}: {error}")

    start = time.monotonic()
    process: Optional[subprocess.Popen] = None
    timeout = config["limits"].get("process_timeout_seconds")
    try:
        with stdout_path.open("wb") as stdout_stream, stderr_path.open(
            "wb"
        ) as stderr_stream:
            process = subprocess.Popen(
                command,
                cwd=str(repo_root),
                env=process_environment,
                stdout=stdout_stream,
                stderr=stderr_stream,
                start_new_session=(os.name == "posix"),
            )
            _register_process(process)
            try:
                if cancellation_event is not None and cancellation_event.is_set():
                    _terminate_process(process)
                    result["return_code"] = process.returncode
                else:
                    result["return_code"] = process.wait(
                        timeout=float(timeout) if timeout is not None else None
                    )
            except subprocess.TimeoutExpired:
                result["timed_out"] = True
                _terminate_process(process)
                result["return_code"] = process.returncode
    except KeyboardInterrupt:
        if process is not None:
            _terminate_process(process)
            result["return_code"] = process.returncode
        result["status"] = "interrupted"
        result["error"] = "interrupted by user"
        raise
    except OSError as error:
        result["status"] = "launch_error"
        result["error"] = str(error)
    finally:
        if process is not None:
            _unregister_process(process)
        result["finished_at"] = utc_now()
        result["duration_seconds"] = round(time.monotonic() - start, 6)
        result["solution_exists"] = solution_path.is_file()
        if result["solution_exists"]:
            result["solution_size_bytes"] = solution_path.stat().st_size

        if result["status"] == "running":
            if cancellation_event is not None and cancellation_event.is_set():
                result["status"] = "interrupted"
                result["error"] = "cancelled by matrix runner"
            elif result["timed_out"]:
                result["status"] = "timed_out"
                result["error"] = "external process timeout expired"
            elif result["return_code"] != 0:
                result["status"] = "failed"
                result["error"] = (
                    f"solver exited with return code {result['return_code']}"
                )
            elif not result["solution_exists"] or result["solution_size_bytes"] == 0:
                result["status"] = "failed"
                result["error"] = "solver exited successfully but wrote no solution"
            else:
                result["status"] = SUCCESS_STATUS
        write_json(run_path, result)

    return result


def parse_environment(values: Iterable[str]) -> Dict[str, str]:
    environment: Dict[str, str] = {}
    for value in values:
        if "=" not in value:
            raise ExperimentError(f"environment override must be KEY=VALUE: {value}")
        key, item = value.split("=", 1)
        if not key:
            raise ExperimentError("environment override key cannot be empty")
        if key in environment:
            raise ExperimentError(
                f"environment override {key!r} was provided more than once"
            )
        environment[key] = item
    return validate_environment_mapping(environment)


def make_direct_config(arguments: argparse.Namespace) -> Tuple[Dict[str, Any], Path, Path]:
    repo_root = resolve_path(arguments.repo_root, Path.cwd())
    binary = resolve_path(arguments.binary, repo_root)
    instance = resolve_path(arguments.instance, repo_root)
    depot = (
        str(resolve_path(arguments.depot_config, repo_root))
        if arguments.depot_config
        else None
    )
    run_dir = resolve_path(arguments.output_dir, repo_root)
    instance_id = safe_identifier(arguments.instance_id or instance.stem)
    solution = run_dir / "solution.txt"
    config: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "experiment_name": arguments.experiment_name,
        "run_id": f"{instance_id}__seed_{arguments.seed}",
        "instance_id": instance_id,
        "binary": str(binary),
        "instance": str(instance),
        "depot_config": depot,
        "solution": str(solution),
        "seed": arguments.seed,
        "best_known": arguments.best_known,
        "is_optimal": arguments.is_optimal,
        "limits": {
            "time_seconds": arguments.time_limit,
            "iterations": arguments.iteration_limit,
            "process_timeout_seconds": arguments.process_timeout,
        },
        "resources": {"gurobi_threads": arguments.gurobi_threads},
        "sp": {
            "trigger_frequency": arguments.sp_trigger_frequency,
            "time_limit_seconds": arguments.sp_time_limit,
            "mip_gap": arguments.sp_mip_gap,
            "seed": arguments.sp_seed,
            "max_pool_size": arguments.sp_max_pool_size,
            "output_flag": arguments.sp_output_flag,
            "mip_focus": arguments.sp_mip_focus,
            "heuristics": arguments.sp_heuristics,
            "representation_route_limit": (
                arguments.sp_representation_route_limit
            ),
        },
        "environment": parse_environment(arguments.env),
        "gurobi_home": (
            str(resolve_path(arguments.gurobi_home, repo_root))
            if arguments.gurobi_home
            else None
        ),
    }
    return config, run_dir, repo_root


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run one HGA-SP seed in an isolated directory and record "
            "reproducibility metadata."
        )
    )
    parser.add_argument("--binary", required=True, help="HGA-SP executable")
    parser.add_argument("--instance", required=True, help="CLRP instance file")
    parser.add_argument(
        "--depot-config",
        help="optional depot-configuration file; paths with spaces are supported",
    )
    parser.add_argument(
        "--output-dir", required=True, help="independent directory for this run"
    )
    parser.add_argument("--seed", required=True, type=int)
    parser.add_argument("--time-limit", required=True, type=int, metavar="SECONDS")
    parser.add_argument("--iteration-limit", required=True, type=int)
    parser.add_argument("--best-known", type=float, default=0.0)
    parser.add_argument("--is-optimal", action="store_true")
    parser.add_argument("--gurobi-threads", type=int, default=1)
    parser.add_argument("--sp-trigger-frequency", type=int, default=50001)
    parser.add_argument("--sp-time-limit", type=float, default=360.0)
    parser.add_argument("--sp-mip-gap", type=float, default=0.001)
    parser.add_argument("--sp-seed", type=int, default=0)
    parser.add_argument("--sp-max-pool-size", type=int, default=50000)
    parser.add_argument("--sp-output-flag", type=int, choices=(0, 1), default=0)
    parser.add_argument("--sp-mip-focus", type=int, choices=range(4), default=1)
    parser.add_argument("--sp-heuristics", type=float, default=0.15)
    representation_limit = parser.add_mutually_exclusive_group()
    representation_limit.add_argument(
        "--sp-representation-route-limit",
        dest="sp_representation_route_limit",
        action="store_true",
        default=True,
    )
    representation_limit.add_argument(
        "--no-sp-representation-route-limit",
        dest="sp_representation_route_limit",
        action="store_false",
    )
    parser.add_argument(
        "--process-timeout",
        type=float,
        help="optional external wall-clock timeout in seconds",
    )
    parser.add_argument(
        "--repo-root",
        default=".",
        help="working tree and base for relative paths (default: current directory)",
    )
    parser.add_argument("--experiment-name", default="single")
    parser.add_argument("--instance-id")
    parser.add_argument("--gurobi-home")
    parser.add_argument(
        "--env",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="environment override; repeat as needed",
    )
    parser.add_argument(
        "--force-successful",
        action="store_true",
        help="replace a previously successful run in the same directory",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    try:
        require_project_venv(Path(__file__).resolve().parents[2])
    except ExperimentError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    parser = build_parser()
    arguments = parser.parse_args(argv)
    try:
        config, run_dir, repo_root = make_direct_config(arguments)
        result = execute_run(
            config,
            run_dir,
            repo_root,
            force_successful=arguments.force_successful,
        )
    except ExperimentError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130

    if result.get("skipped"):
        print(f"SKIP {result['run_id']}: successful result already exists")
    else:
        print(
            f"{result['status'].upper()} {result['run_id']} "
            f"({result['duration_seconds']:.3f}s)"
        )
    return 0 if result["status"] == SUCCESS_STATUS else 1


if __name__ == "__main__":
    sys.exit(main())
