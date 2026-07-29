#!/usr/bin/env python3
"""Run one reproducible HGA-SP experiment.

The solver is invoked with an argv list (never through a shell), so instance,
depot-configuration, and output paths may safely contain spaces.
"""

from __future__ import annotations

import argparse
import contextlib
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
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple


VALIDATION_DIR = Path(__file__).resolve().parents[1] / "validation"
if str(VALIDATION_DIR) not in sys.path:
    sys.path.insert(0, str(VALIDATION_DIR))

import solution_validator  # noqa: E402


SCHEMA_VERSION = 3
LEGACY_SCHEMA_VERSION = 2
SUCCESS_STATUS = "success"
PROBLEM_MODES = frozenset(("clrp", "clrp-tw"))
CPP_INT_MAX = 2_147_483_647
SP_SEED_MAX = 2_000_000_000
GUROBI_THREADS_MAX = 1024
SP_OPTIONAL_DEFAULTS: Dict[str, Any] = {
    "require_success": False,
    "work_limit": 0.0,
    "local_enabled": True,
    "local_trigger_frequency": 10000,
    "local_time_limit_seconds": 30.0,
    "adaptive_trigger_enabled": True,
    "adaptive_minimum_interval": 5000,
    "adaptive_local_new_columns": 250,
    "adaptive_local_stagnation": 5000,
    "adaptive_global_stagnation": 20000,
    "adaptive_utility_ema_weight": 0.25,
    "adaptive_high_utility_threshold": 1.0e-3,
    "lp_dual_feedback_enabled": True,
    "lp_time_limit_seconds": 10.0,
    "persistent_lp_enabled": True,
    "solution_pool_size": 5,
    "solution_pool_gap": 0.02,
    "solution_pool_search_mode": 2,
    "diverse_injection_count": 3,
    "adaptive_search_allocation_enabled": True,
    "adaptive_population_exploration": 0.25,
    "adaptive_population_diversity_weight": 0.10,
    "adaptive_population_column_weight": 0.05,
    "adaptive_search_ema_weight": 0.20,
    "adaptive_mutation_minimum_percent": 5,
    "adaptive_mutation_maximum_percent": 40,
    "wls_token_duration_minutes": 0,
    "wls_token_refresh": 0.9,
    "client_log_level": 1,
    "server_timeout_seconds": 30,
    "environment_max_attempts": 3,
    "environment_retry_base_milliseconds": 250,
}

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
        "HGASP_SP_TRIGGER_OFFSET",
        "HGASP_SP_TIME_LIMIT_SECONDS",
        "HGASP_SP_LOCAL_ENABLED",
        "HGASP_SP_LOCAL_TRIGGER_FREQUENCY",
        "HGASP_SP_LOCAL_TIME_LIMIT_SECONDS",
        "HGASP_SP_ADAPTIVE_TRIGGER_ENABLED",
        "HGASP_SP_ADAPTIVE_MINIMUM_INTERVAL",
        "HGASP_SP_ADAPTIVE_LOCAL_NEW_COLUMNS",
        "HGASP_SP_ADAPTIVE_LOCAL_STAGNATION",
        "HGASP_SP_ADAPTIVE_GLOBAL_STAGNATION",
        "HGASP_SP_ADAPTIVE_UTILITY_EMA_WEIGHT",
        "HGASP_SP_ADAPTIVE_HIGH_UTILITY_THRESHOLD",
        "HGASP_SP_LP_DUAL_FEEDBACK_ENABLED",
        "HGASP_SP_LP_TIME_LIMIT_SECONDS",
        "HGASP_SP_PERSISTENT_LP_ENABLED",
        "HGASP_SP_SOLUTION_POOL_SIZE",
        "HGASP_SP_SOLUTION_POOL_GAP",
        "HGASP_SP_SOLUTION_POOL_SEARCH_MODE",
        "HGASP_SP_DIVERSE_INJECTION_COUNT",
        "HGASP_ADAPTIVE_SEARCH_ALLOCATION_ENABLED",
        "HGASP_ADAPTIVE_POPULATION_EXPLORATION",
        "HGASP_ADAPTIVE_POPULATION_DIVERSITY_WEIGHT",
        "HGASP_ADAPTIVE_POPULATION_COLUMN_WEIGHT",
        "HGASP_ADAPTIVE_SEARCH_EMA_WEIGHT",
        "HGASP_ADAPTIVE_MUTATION_MINIMUM_PERCENT",
        "HGASP_ADAPTIVE_MUTATION_MAXIMUM_PERCENT",
        "HGASP_SP_WORK_LIMIT",
        "HGASP_SP_MIP_GAP",
        "HGASP_SP_SEED",
        "HGASP_SP_MAX_POOL_SIZE",
        "HGASP_SP_OUTPUT_FLAG",
        "HGASP_SP_MIP_FOCUS",
        "HGASP_SP_HEURISTICS",
        "HGASP_SP_WLS_TOKEN_DURATION_MINUTES",
        "HGASP_SP_WLS_TOKEN_REFRESH",
        "HGASP_SP_CLIENT_LOG_LEVEL",
        "HGASP_SP_SERVER_TIMEOUT_SECONDS",
        "HGASP_SP_ENVIRONMENT_MAX_ATTEMPTS",
        "HGASP_SP_ENVIRONMENT_RETRY_BASE_MILLISECONDS",
        "HGASP_SP_REPRESENTATION_ROUTE_LIMIT",
        "HGASP_LOG_DIR",
        "HGASP_LOG_CONSOLE",
        "HGASP_LOG_SEGMENT_BYTES",
        "HGASP_LOG_BACKUPS",
        "HGASP_LOG_MESSAGE_MAX_BYTES",
        "HGASP_SP_LOG_FILE",
    }
)
_ACTIVE_PROCESS_LOCK = threading.Lock()
_ACTIVE_PROCESSES: Dict[subprocess.Popen, "_ManagedProcess"] = {}
SUPERVISOR_PATH = Path(__file__).with_name("process_supervisor.py")
SUPERVISOR_HEARTBEAT_SECONDS = 5.0
SUPERVISOR_TERM_GRACE_SECONDS = 2.0
SUPERVISOR_KILL_GRACE_SECONDS = 2.0


class CrossProcessFileLock:
    """Non-blocking lock whose descriptor can be inherited by a supervisor."""

    def __init__(self, path: Path, description: str) -> None:
        self.path = path
        self.description = description
        self.fd: Optional[int] = None

    def acquire(self) -> "CrossProcessFileLock":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        flags = os.O_RDWR | os.O_CREAT
        if hasattr(os, "O_CLOEXEC"):
            flags |= os.O_CLOEXEC
        descriptor = os.open(self.path, flags, 0o600)
        try:
            if os.name == "posix":
                import fcntl

                try:
                    fcntl.flock(
                        descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB
                    )
                except BlockingIOError as error:
                    raise ExperimentError(
                        f"{self.description} is already active: {self.path}"
                    ) from error
            else:
                import msvcrt

                try:
                    os.lseek(descriptor, 0, os.SEEK_SET)
                    msvcrt.locking(descriptor, msvcrt.LK_NBLCK, 1)
                except OSError as error:
                    raise ExperimentError(
                        f"{self.description} is already active: {self.path}"
                    ) from error
            owner = (
                json.dumps(
                    {"pid": os.getpid(), "acquired_at": utc_now()},
                    sort_keys=True,
                )
                + "\n"
            ).encode("utf-8")
            os.ftruncate(descriptor, 0)
            os.write(descriptor, owner)
            os.fsync(descriptor)
            self.fd = descriptor
            return self
        except Exception:
            os.close(descriptor)
            raise

    def close(self) -> None:
        if self.fd is None:
            return
        descriptor = self.fd
        self.fd = None
        try:
            if os.name == "posix":
                import fcntl

                fcntl.flock(descriptor, fcntl.LOCK_UN)
            else:
                import msvcrt

                os.lseek(descriptor, 0, os.SEEK_SET)
                msvcrt.locking(descriptor, msvcrt.LK_UNLCK, 1)
        finally:
            os.close(descriptor)

    def __enter__(self) -> "CrossProcessFileLock":
        return self.acquire()

    def __exit__(self, _type: Any, _value: Any, _traceback: Any) -> None:
        self.close()


@dataclass
class _ManagedProcess:
    """Bookkeeping for one solver process and its isolated process group."""

    process: subprocess.Popen
    process_group_id: Optional[int]
    wait_lock: threading.Lock
    reaped: threading.Event
    termination_lock: threading.Lock
    termination_requested_at: Optional[float] = None
    kill_requested: bool = False
    cpu_user_seconds: Optional[float] = None
    cpu_system_seconds: Optional[float] = None


@dataclass
class SignalState:
    """First catchable termination signal observed by a runner."""

    signum: Optional[int] = None
    name: Optional[str] = None


class _RunCancelled(RuntimeError):
    """Internal control flow used when the matrix cancellation event is set."""


class ExperimentError(RuntimeError):
    """Raised when an experiment cannot be configured safely."""


@dataclass(frozen=True)
class CpuTopology:
    """CPU resources available to the runner after process affinity is applied."""

    logical_cpu_ids: Tuple[int, ...]
    physical_core_cpu_ids: Tuple[Tuple[int, ...], ...]
    source: str

    @property
    def logical_cpu_count(self) -> int:
        return len(self.logical_cpu_ids)

    @property
    def physical_core_count(self) -> int:
        return len(self.physical_core_cpu_ids)

    @property
    def physical_core_representatives(self) -> Tuple[int, ...]:
        return tuple(core[0] for core in self.physical_core_cpu_ids)

    @property
    def threads_per_core(self) -> int:
        return max((len(core) for core in self.physical_core_cpu_ids), default=1)

    @property
    def smt_available(self) -> bool:
        return any(len(core) > 1 for core in self.physical_core_cpu_ids)

    def scheduling_slots(self, allow_smt: bool) -> Tuple[Tuple[int, ...], ...]:
        primary = [(core[0],) for core in self.physical_core_cpu_ids]
        if allow_smt:
            # Consume one hardware thread from every physical core before
            # allocating any SMT sibling.
            siblings = [
                (cpu,)
                for core in self.physical_core_cpu_ids
                for cpu in core[1:]
            ]
            return tuple(primary + siblings)
        return tuple(primary)

    def as_metadata(self) -> Dict[str, Any]:
        return {
            "source": self.source,
            "available_logical_cpu_count": self.logical_cpu_count,
            "available_logical_cpu_ids": list(self.logical_cpu_ids),
            "available_physical_core_count": self.physical_core_count,
            "physical_core_representatives": list(
                self.physical_core_representatives
            ),
            "physical_core_affinity_groups": [
                list(core) for core in self.physical_core_cpu_ids
            ],
            "threads_per_core": self.threads_per_core,
            "smt_available": self.smt_available,
        }


def utc_now() -> str:
    """Return a stable UTC timestamp suitable for experiment metadata."""

    return (
        dt.datetime.now(dt.timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


@contextlib.contextmanager
def cancellation_signal_handlers(
    cancellation_event: threading.Event,
):
    """Translate SIGINT/SIGTERM/SIGHUP into cooperative runner cancellation.

    Python dispatches signal handlers only in the main thread.  Worker threads
    observe ``cancellation_event`` and terminate the process groups they own.
    The handler deliberately does not raise asynchronously.  This both closes
    the Popen-to-registration race and avoids blocking termination signals in
    a child through an inherited pthread signal mask.  Repeated signals during
    cleanup are coalesced so they cannot interrupt child reaping.
    """

    state = SignalState()
    if threading.current_thread() is not threading.main_thread():
        yield state
        return

    handled = tuple(
        candidate
        for candidate in (
            signal.SIGINT,
            signal.SIGTERM,
            getattr(signal, "SIGHUP", None),
        )
        if candidate is not None
    )
    previous = {signum: signal.getsignal(signum) for signum in handled}

    def request_cancellation(signum: int, _frame: Any) -> None:
        cancellation_event.set()
        if state.signum is not None:
            return
        state.signum = int(signum)
        try:
            state.name = signal.Signals(signum).name
        except ValueError:
            state.name = f"signal-{signum}"

    try:
        for signum in handled:
            signal.signal(signum, request_cancellation)
        yield state
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


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


def available_cpu_ids() -> Tuple[int, ...]:
    """Return logical CPU IDs available to this process."""

    if hasattr(os, "sched_getaffinity"):
        try:
            affinity = os.sched_getaffinity(0)
        except OSError:
            pass
        else:
            if affinity:
                return tuple(sorted(int(cpu) for cpu in affinity))

    process_cpu_count = getattr(os, "process_cpu_count", None)
    if callable(process_cpu_count):
        count = process_cpu_count()
        if count:
            return tuple(range(int(count)))
    return tuple(range(os.cpu_count() or 1))


def available_cpu_count() -> int:
    """Return the logical CPU count available to this process."""

    return len(available_cpu_ids())


def detect_cpu_topology(
    logical_cpu_ids: Optional[Sequence[int]] = None,
    sysfs_cpu_root: Path = Path("/sys/devices/system/cpu"),
    system_name: Optional[str] = None,
) -> CpuTopology:
    """Group affinity-visible Linux CPUs by physical package and core."""

    logical = tuple(
        sorted(
            set(
                int(cpu)
                for cpu in (
                    available_cpu_ids()
                    if logical_cpu_ids is None
                    else logical_cpu_ids
                )
            )
        )
    )
    if not logical:
        raise ExperimentError("the process has no available logical CPUs")

    detected_system = system_name or platform.system()
    if detected_system == "Linux":
        grouped: Dict[Tuple[int, int], List[int]] = {}
        topology_error: Optional[Exception] = None
        try:
            for cpu in logical:
                topology_root = sysfs_cpu_root / f"cpu{cpu}" / "topology"
                package = int(
                    (topology_root / "physical_package_id")
                    .read_text(encoding="ascii")
                    .strip()
                )
                core = int(
                    (topology_root / "core_id")
                    .read_text(encoding="ascii")
                    .strip()
                )
                grouped.setdefault((package, core), []).append(cpu)
        except (OSError, ValueError) as error:
            topology_error = error
            grouped.clear()
        if grouped:
            physical_cores = tuple(
                tuple(sorted(cpus))
                for _, cpus in sorted(
                    grouped.items(), key=lambda item: min(item[1])
                )
            )
            return CpuTopology(logical, physical_cores, "linux_sysfs")
        raise ExperimentError(
            "cannot detect affinity-visible physical CPU topology from "
            f"{sysfs_cpu_root}: {topology_error or 'no core data'}"
        )

    # Outside Linux, preserve direct-run portability. The matrix runner will
    # reject affinity binding when the platform cannot apply it.
    return CpuTopology(
        logical,
        tuple((cpu,) for cpu in logical),
        "logical_cpu_fallback",
    )


@contextlib.contextmanager
def inherited_cpu_affinity(
    cpu_ids: Optional[Sequence[int]],
) -> Iterable[Tuple[int, ...]]:
    """Temporarily bind the calling thread so a spawned child inherits it."""

    if not hasattr(os, "sched_getaffinity") or not hasattr(
        os, "sched_setaffinity"
    ):
        if cpu_ids is not None:
            raise ExperimentError(
                "CPU affinity binding is unavailable on this platform"
            )
        yield available_cpu_ids()
        return

    original = tuple(sorted(int(cpu) for cpu in os.sched_getaffinity(0)))
    if cpu_ids is None:
        yield original
        return

    requested = tuple(sorted(set(int(cpu) for cpu in cpu_ids)))
    if not requested:
        raise ExperimentError("CPU affinity cannot be empty")
    unavailable = sorted(set(requested) - set(original))
    if unavailable:
        raise ExperimentError(
            "requested CPU affinity includes unavailable logical CPU(s): "
            + ", ".join(str(cpu) for cpu in unavailable)
        )

    try:
        os.sched_setaffinity(0, requested)
        effective = tuple(sorted(int(cpu) for cpu in os.sched_getaffinity(0)))
        if effective != requested:
            raise ExperimentError(
                f"requested CPU affinity {requested} became {effective}"
            )
        yield effective
    finally:
        os.sched_setaffinity(0, original)


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


def collect_platform_provenance(
    cpu_topology: Optional[CpuTopology] = None,
) -> Dict[str, Any]:
    topology = cpu_topology or detect_cpu_topology()
    processor = platform.processor().strip()
    if not processor and platform.system() == "Linux":
        try:
            cpuinfo_candidates: Dict[str, str] = {}
            for line in Path("/proc/cpuinfo").read_text(
                encoding="utf-8", errors="replace"
            ).splitlines():
                key, separator, value = line.partition(":")
                normalized_key = key.strip().casefold()
                if separator and normalized_key in {
                    "model name",
                    "hardware",
                    "processor",
                } and value.strip():
                    cpuinfo_candidates.setdefault(
                        normalized_key, value.strip()
                    )
            processor = (
                cpuinfo_candidates.get("model name")
                or cpuinfo_candidates.get("hardware")
                or (
                    cpuinfo_candidates.get("processor", "")
                    if not cpuinfo_candidates.get("processor", "").isdigit()
                    else ""
                )
            )
        except OSError:
            pass
    processor = processor or "unknown"
    machine = platform.machine() or "unknown"
    hardware_identity = {
        "machine": machine.casefold(),
        "processor": " ".join(processor.casefold().split()),
        "available_physical_core_count": topology.physical_core_count,
        "threads_per_core": topology.threads_per_core,
    }
    hardware_payload = json.dumps(
        hardware_identity,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    hardware_id = "sha256:" + hashlib.sha256(hardware_payload).hexdigest()
    result = {
        "system": platform.system(),
        "release": platform.release(),
        "version": platform.version(),
        "machine": machine,
        "processor": processor,
        # Stable within an effective hardware topology. The unhashed inputs
        # remain alongside the identifier so the fingerprint is auditable.
        "hardware_id": hardware_id,
        "hardware_identity": hardware_identity,
        "python_version": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "logical_cpu_count": os.cpu_count(),
    }
    result.update(topology.as_metadata())
    return result


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
    command.extend(
        ["--problem-mode", str(config.get("problem_mode", "clrp"))]
    )
    return command


def _validate_run_config(
    config: Mapping[str, Any], run_dir: Optional[Path] = None
) -> None:
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
    problem_mode = config.get("problem_mode", "clrp")
    if problem_mode not in PROBLEM_MODES:
        raise ExperimentError(
            "problem_mode must be either 'clrp' or 'clrp-tw'"
        )
    solution = Path(str(config["solution"]))
    expected_solution_json = solution.with_name("solution.json")
    solution_json = Path(
        str(config.get("solution_json") or expected_solution_json)
    )
    normalized_solution = solution.resolve()
    normalized_solution_json = solution_json.resolve()
    if normalized_solution == normalized_solution_json:
        raise ExperimentError("solution and solution_json must be distinct paths")
    if normalized_solution_json != expected_solution_json.resolve():
        raise ExperimentError(
            "solution_json must be the solution.txt sibling named solution.json"
        )
    protected_inputs = {
        "binary": binary.resolve(),
        "instance": instance.resolve(),
    }
    if depot:
        protected_inputs["depot_config"] = Path(str(depot)).resolve()
    for output_name, output_path in (
        ("solution", normalized_solution),
        ("solution_json", normalized_solution_json),
    ):
        for input_name, input_path in protected_inputs.items():
            if output_path == input_path:
                raise ExperimentError(
                    f"{output_name} must not overwrite {input_name}"
                )
    if run_dir is not None:
        normalized_run_dir = run_dir.resolve()
        expected_solution = normalized_run_dir / "solution.txt"
        expected_json = normalized_run_dir / "solution.json"
        if normalized_solution != expected_solution:
            raise ExperimentError(
                "solution must be run_dir/solution.txt"
            )
        if normalized_solution_json != expected_json:
            raise ExperimentError(
                "solution_json must be run_dir/solution.json"
            )

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
        or float(best_known) < 0
    ):
        raise ExperimentError("best_known must be finite and nonnegative")
    is_optimal = config.get("is_optimal", False)
    if not isinstance(is_optimal, bool):
        raise ExperimentError("is_optimal must be boolean")
    if is_optimal and float(best_known) <= 0:
        raise ExperimentError(
            "is_optimal=true requires a positive best_known value"
        )
    stop_at_reference = config.get("stop_at_reference", True)
    if not isinstance(stop_at_reference, bool):
        raise ExperimentError("stop_at_reference must be boolean")
    if not stop_at_reference and is_optimal:
        raise ExperimentError(
            "is_optimal must be false when stop_at_reference=false"
        )
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
        maximum=GUROBI_THREADS_MAX,
    )
    available_cpus = available_cpu_count()
    if threads > available_cpus:
        raise ExperimentError(
            f"resources.gurobi_threads ({threads}) exceeds "
            f"{available_cpus} available logical CPUs"
        )
    validate_environment_mapping(config.get("environment", {}))
    validate_sp_config(sp, allow_deterministic_trigger_offset=False)
    validation = config.get("validation", {"required": False})
    if not isinstance(validation, Mapping):
        raise ExperimentError("validation must be an object")
    unknown_validation = sorted(
        set(validation)
        - {"required", "absolute_tolerance", "relative_tolerance"}
    )
    if unknown_validation:
        raise ExperimentError(
            "validation contains unknown field(s): "
            + ", ".join(unknown_validation)
        )
    if not isinstance(validation.get("required", False), bool):
        raise ExperimentError("validation.required must be boolean")
    if (
        problem_mode == "clrp-tw"
        and not validation.get("required", False)
    ):
        raise ExperimentError(
            "clrp-tw runs must set validation.required=true"
        )
    for name, default in (
        (
            "absolute_tolerance",
            solution_validator.DEFAULT_ABSOLUTE_TOLERANCE,
        ),
        (
            "relative_tolerance",
            solution_validator.DEFAULT_RELATIVE_TOLERANCE,
        ),
    ):
        value = validation.get(name, default)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or value < 0
        ):
            raise ExperimentError(
                f"validation.{name} must be finite and nonnegative"
            )


def validate_sp_config(
    sp: Mapping[str, Any],
    allow_deterministic_trigger_offset: bool = True,
) -> None:
    if not isinstance(sp.get("representation_route_limit"), bool):
        raise ExperimentError("sp.representation_route_limit must be boolean")
    for boolean_field in (
        "local_enabled",
        "adaptive_trigger_enabled",
        "lp_dual_feedback_enabled",
        "persistent_lp_enabled",
        "adaptive_search_allocation_enabled",
    ):
        if not isinstance(
            sp.get(boolean_field, SP_OPTIONAL_DEFAULTS[boolean_field]),
            bool,
        ):
            raise ExperimentError(f"sp.{boolean_field} must be boolean")
    if not isinstance(
        sp.get("require_success", SP_OPTIONAL_DEFAULTS["require_success"]),
        bool,
    ):
        raise ExperimentError("sp.require_success must be boolean")

    integer_ranges = {
        "trigger_frequency": (1, CPP_INT_MAX),
        "seed": (0, SP_SEED_MAX),
        "max_pool_size": (1, CPP_INT_MAX),
        "output_flag": (0, 1),
        "mip_focus": (0, 3),
        "wls_token_duration_minutes": (0, SP_SEED_MAX),
        "client_log_level": (0, 3),
        "server_timeout_seconds": (1, SP_SEED_MAX),
        "environment_max_attempts": (1, 10),
        "environment_retry_base_milliseconds": (0, 60000),
        "local_trigger_frequency": (1, CPP_INT_MAX),
        "adaptive_minimum_interval": (1, CPP_INT_MAX),
        "adaptive_local_new_columns": (1, CPP_INT_MAX),
        "adaptive_local_stagnation": (1, CPP_INT_MAX),
        "adaptive_global_stagnation": (1, CPP_INT_MAX),
        "solution_pool_size": (1, 1000),
        "solution_pool_search_mode": (0, 2),
        "diverse_injection_count": (1, 1000),
        "adaptive_mutation_minimum_percent": (0, 100),
        "adaptive_mutation_maximum_percent": (0, 100),
    }
    for field, (minimum, maximum) in integer_ranges.items():
        value = sp.get(field, SP_OPTIONAL_DEFAULTS.get(field))
        if isinstance(value, bool) or not isinstance(value, int):
            raise ExperimentError(f"sp.{field} must be an integer")
        if value < minimum or (maximum is not None and value > maximum):
            upper = f" and at most {maximum}" if maximum is not None else ""
            raise ExperimentError(
                f"sp.{field} must be at least {minimum}{upper}"
            )

    trigger_offset = sp.get("trigger_offset", 0)
    if trigger_offset == "deterministic":
        if not allow_deterministic_trigger_offset:
            raise ExperimentError(
                "sp.trigger_offset must be resolved to an integer for one run"
            )
    elif isinstance(trigger_offset, bool) or not isinstance(
        trigger_offset, int
    ):
        raise ExperimentError(
            "sp.trigger_offset must be an integer or 'deterministic'"
        )
    elif trigger_offset < 0 or trigger_offset >= sp["trigger_frequency"]:
        raise ExperimentError(
            "sp.trigger_offset must be at least 0 and less than "
            "sp.trigger_frequency"
        )

    numeric_ranges = {
        "time_limit_seconds": (0.0, None, False),
        "work_limit": (0.0, None, True),
        "mip_gap": (0.0, 1.0, True),
        "heuristics": (0.0, 1.0, True),
        "wls_token_refresh": (0.0, 1.0, True),
        "local_time_limit_seconds": (0.0, None, False),
        "adaptive_utility_ema_weight": (0.0, 1.0, False),
        "adaptive_high_utility_threshold": (0.0, None, False),
        "lp_time_limit_seconds": (0.0, None, False),
        "solution_pool_gap": (0.0, None, True),
        "adaptive_population_exploration": (0.0, None, True),
        "adaptive_population_diversity_weight": (0.0, None, True),
        "adaptive_population_column_weight": (0.0, None, True),
        "adaptive_search_ema_weight": (0.0, 1.0, False),
    }
    for field, (minimum, maximum, include_minimum) in numeric_ranges.items():
        value = sp.get(field, SP_OPTIONAL_DEFAULTS.get(field))
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
    if (
        sp.get(
            "adaptive_global_stagnation",
            SP_OPTIONAL_DEFAULTS["adaptive_global_stagnation"],
        )
        < sp.get(
            "adaptive_local_stagnation",
            SP_OPTIONAL_DEFAULTS["adaptive_local_stagnation"],
        )
    ):
        raise ExperimentError(
            "sp.adaptive_global_stagnation must be at least "
            "sp.adaptive_local_stagnation"
        )
    if (
        sp.get(
            "diverse_injection_count",
            SP_OPTIONAL_DEFAULTS["diverse_injection_count"],
        )
        > sp.get(
            "solution_pool_size",
            SP_OPTIONAL_DEFAULTS["solution_pool_size"],
        )
    ):
        raise ExperimentError(
            "sp.diverse_injection_count must not exceed "
            "sp.solution_pool_size"
        )
    if (
        sp.get(
            "adaptive_mutation_minimum_percent",
            SP_OPTIONAL_DEFAULTS[
                "adaptive_mutation_minimum_percent"
            ],
        )
        > sp.get(
            "adaptive_mutation_maximum_percent",
            SP_OPTIONAL_DEFAULTS[
                "adaptive_mutation_maximum_percent"
            ],
        )
    ):
        raise ExperimentError(
            "sp.adaptive_mutation_minimum_percent must not exceed "
            "sp.adaptive_mutation_maximum_percent"
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


def inspect_sp_success_logs(paths: Sequence[Path]) -> Dict[str, Any]:
    """Require every observed SP result to have a success marker and no error."""

    result_pattern = re.compile(rb"\[SP Result\] status=([A-Z_]+)")
    failure_status_names = {
        "ERROR",
        "GUROBI_ERROR",
        "INFEASIBLE",
        "INVALID_SOLUTION",
        "NO_SOLUTION",
    }
    checked_logs: List[str] = []
    read_errors: List[str] = []
    result_statuses: List[str] = []
    success_count = 0

    for path in paths:
        checked_logs.append(str(path))
        if not path.is_file():
            continue
        try:
            content = path.read_bytes()
            result_statuses.extend(
                match.group(1).decode("ascii")
                for match in result_pattern.finditer(content)
            )
            success_count += content.count(b"[SP Success]")
        except OSError as error:
            read_errors.append(f"{path}: {error}")

    result_count = len(result_statuses)
    failure_statuses = [
        status for status in result_statuses if status in failure_status_names
    ]
    result_status_marker_found = result_count > 0
    success_marker_found = success_count > 0
    observed = (
        result_count > 0
        and success_count == result_count
        and not failure_statuses
        and not read_errors
    )
    return {
        "result_status_marker_found": result_status_marker_found,
        "success_marker_found": success_marker_found,
        "result_count": result_count,
        "success_count": success_count,
        "result_statuses": result_statuses,
        "failure_statuses": failure_statuses,
        "checked": True,
        "observed": observed,
        "checked_logs": checked_logs,
        "read_errors": read_errors,
    }


def inspect_sp_success_attempt(
    hgasp_log_path: Path,
    stdout_path: Path,
) -> Dict[str, Any]:
    """Prefer canonical rotated HGA logs, using stdout only as a fallback."""

    rotated: List[Tuple[int, Path]] = []
    for candidate in hgasp_log_path.parent.glob(
        f"{hgasp_log_path.name}.*"
    ):
        try:
            index = int(candidate.suffix[1:])
        except ValueError:
            continue
        rotated.append((index, candidate))
    canonical_paths = [
        path for _index, path in sorted(rotated, reverse=True)
    ]
    canonical_paths.append(hgasp_log_path)
    evidence = inspect_sp_success_logs(canonical_paths)
    if evidence["result_count"] > 0 or evidence["read_errors"]:
        return evidence
    # Fake/test solvers and failures before RunLogger initialization may only
    # write stdout.  Do not scan it when canonical evidence exists, otherwise
    # console teeing could double-count the same SP event.
    return inspect_sp_success_logs((stdout_path,))


def _managed_process(process: subprocess.Popen) -> Optional[_ManagedProcess]:
    with _ACTIVE_PROCESS_LOCK:
        return _ACTIVE_PROCESSES.get(process)


def _register_process(process: subprocess.Popen) -> _ManagedProcess:
    process_group_id: Optional[int] = None
    if os.name == "posix":
        try:
            process_group_id = os.getpgid(process.pid)
        except OSError:
            # ``start_new_session=True`` makes the child PID its PGID.  The
            # fallback remains useful when a very short-lived test process
            # exits between Popen and getpgid.
            process_group_id = process.pid
    managed = _ManagedProcess(
        process=process,
        process_group_id=process_group_id,
        wait_lock=threading.Lock(),
        reaped=threading.Event(),
        termination_lock=threading.Lock(),
    )
    with _ACTIVE_PROCESS_LOCK:
        if process in _ACTIVE_PROCESSES:
            raise ExperimentError(
                f"solver process {process.pid} was registered more than once"
            )
        _ACTIVE_PROCESSES[process] = managed
    return managed


def _unregister_process(process: subprocess.Popen) -> None:
    with _ACTIVE_PROCESS_LOCK:
        _ACTIVE_PROCESSES.pop(process, None)


def active_process_count() -> int:
    with _ACTIVE_PROCESS_LOCK:
        managed_processes = tuple(_ACTIVE_PROCESSES.values())
    return sum(
        not _managed_process_is_fully_stopped(managed)
        for managed in managed_processes
    )


def _record_wait_status(
    managed: _ManagedProcess,
    status: int,
    usage: Optional[Any],
) -> int:
    try:
        return_code = os.waitstatus_to_exitcode(status)
    except AttributeError:
        if os.WIFSIGNALED(status):
            return_code = -os.WTERMSIG(status)
        else:
            return_code = os.WEXITSTATUS(status)
    managed.process.returncode = return_code
    if usage is not None:
        managed.cpu_user_seconds = float(usage.ru_utime)
        managed.cpu_system_seconds = float(usage.ru_stime)
    managed.reaped.set()
    return return_code


def _wait_for_registered_process(
    process: subprocess.Popen,
    timeout: Optional[float] = None,
    cancellation_event: Optional[threading.Event] = None,
) -> int:
    """Wait for one exact child and capture CPU use without global deltas.

    On POSIX, ``wait4(pid, ...)`` attributes user/system CPU to this child
    (including its waited-for descendants) and therefore remains correct when
    many solver processes run concurrently.  ``RUSAGE_CHILDREN`` deltas are
    intentionally not used because they cannot be assigned safely per worker.
    """

    managed = _managed_process(process)
    if managed is None:
        raise ExperimentError(
            f"solver process {process.pid} is not registered"
        )
    if managed.reaped.is_set():
        assert process.returncode is not None
        return int(process.returncode)

    start = time.monotonic()
    deadline = None if timeout is None else start + max(0.0, float(timeout))
    lock_timeout = -1.0 if deadline is None else max(0.0, deadline - start)
    acquired = managed.wait_lock.acquire(
        timeout=lock_timeout
    ) if lock_timeout >= 0 else managed.wait_lock.acquire()
    if not acquired:
        raise subprocess.TimeoutExpired(process.args, float(timeout or 0.0))
    try:
        if managed.reaped.is_set():
            assert process.returncode is not None
            return int(process.returncode)

        if os.name == "posix" and hasattr(os, "wait4"):
            while True:
                if cancellation_event is not None and cancellation_event.is_set():
                    raise _RunCancelled("solver run was cancelled")
                try:
                    pid, status, usage = os.wait4(process.pid, os.WNOHANG)
                except ChildProcessError as error:
                    if managed.reaped.is_set() and process.returncode is not None:
                        return int(process.returncode)
                    raise ExperimentError(
                        f"solver process {process.pid} was reaped unexpectedly"
                    ) from error
                if pid == process.pid:
                    return _record_wait_status(managed, status, usage)
                now = time.monotonic()
                if deadline is not None and now >= deadline:
                    raise subprocess.TimeoutExpired(process.args, float(timeout))
                wait_seconds = 0.05
                if deadline is not None:
                    wait_seconds = min(wait_seconds, max(0.0, deadline - now))
                if cancellation_event is not None:
                    cancellation_event.wait(wait_seconds)
                else:
                    time.sleep(wait_seconds)

        # Portable fallback.  CPU attribution is unavailable here, but
        # cancellation remains responsive.
        while True:
            if cancellation_event is not None and cancellation_event.is_set():
                raise _RunCancelled("solver run was cancelled")
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                raise subprocess.TimeoutExpired(process.args, float(timeout))
            wait_seconds = 0.05
            if deadline is not None:
                wait_seconds = min(wait_seconds, max(0.0, deadline - now))
            try:
                return_code = process.wait(timeout=wait_seconds)
            except subprocess.TimeoutExpired:
                continue
            managed.reaped.set()
            return int(return_code)
    finally:
        managed.wait_lock.release()


def _signal_process_group(
    managed: _ManagedProcess,
    signum: int,
) -> None:
    try:
        if os.name == "posix" and managed.process_group_id is not None:
            os.killpg(managed.process_group_id, signum)
        else:
            managed.process.send_signal(signum)
    except (OSError, ProcessLookupError):
        pass


def _request_process_group_termination(managed: _ManagedProcess) -> float:
    """Send TERM once and return the shared grace-period start time."""

    with managed.termination_lock:
        if managed.termination_requested_at is None:
            managed.termination_requested_at = time.monotonic()
            _signal_process_group(managed, signal.SIGTERM)
        return managed.termination_requested_at


def _request_process_group_kill(managed: _ManagedProcess) -> None:
    with managed.termination_lock:
        if managed.kill_requested:
            return
        managed.kill_requested = True
        _signal_process_group(
            managed, getattr(signal, "SIGKILL", signal.SIGTERM)
        )


def _process_group_has_live_members(process_group_id: Optional[int]) -> bool:
    if process_group_id is None or os.name != "posix":
        return False

    if sys.platform.startswith("linux"):
        # killpg(..., 0) also reports zombie-only groups.  /proc lets cleanup
        # distinguish those already-dead members from a descendant that is
        # still executing after its group leader has been reaped.
        proc_root = Path("/proc")
        try:
            entries = tuple(proc_root.iterdir())
        except OSError:
            return _process_group_may_exist(process_group_id)
        inspection_incomplete = False
        for entry in entries:
            if not entry.name.isdigit():
                continue
            try:
                content = (entry / "stat").read_text(encoding="utf-8")
            except FileNotFoundError:
                continue
            except (OSError, UnicodeError):
                inspection_incomplete = True
                continue
            closing_parenthesis = content.rfind(")")
            if closing_parenthesis < 0:
                inspection_incomplete = True
                continue
            fields = content[closing_parenthesis + 2 :].split()
            if len(fields) < 3:
                inspection_incomplete = True
                continue
            state = fields[0]
            try:
                member_group = int(fields[2])
            except ValueError:
                inspection_incomplete = True
                continue
            if member_group == process_group_id and state != "Z":
                return True
        # If any proc entry could not be inspected, absence was not proved.
        # The signal-0 fallback is deliberately conservative: it may regard a
        # zombie-only group as present, but it cannot silently declare a live
        # solver descendant cleaned up.
        return (
            _process_group_may_exist(process_group_id)
            if inspection_incomplete
            else False
        )

    return _process_group_may_exist(process_group_id)


def _process_group_may_exist(process_group_id: int) -> bool:
    """Conservatively probe a process group when detailed inspection fails."""

    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        # Unknown probe failures do not prove that the group is gone.
        return True
    return True


def _managed_process_is_fully_stopped(managed: _ManagedProcess) -> bool:
    """Return true only after the leader is reaped and its group is quiescent."""

    return (
        managed.reaped.is_set()
        and not _process_group_has_live_members(managed.process_group_id)
    )


def _wait_for_process_group_exit(
    managed: _ManagedProcess,
    deadline: float,
) -> bool:
    while _process_group_has_live_members(managed.process_group_id):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(0.05, remaining))
    return True


def _terminate_process(process: subprocess.Popen) -> bool:
    """Terminate, converge, and reap one precisely registered process group.

    The return value is an explicit cleanup confirmation.  Callers must retain
    an unconfirmed registration and must not report a successful run.
    """

    managed = _managed_process(process)
    if managed is None:
        return True
    if _managed_process_is_fully_stopped(managed):
        return True
    termination_started = _request_process_group_termination(managed)
    term_deadline = termination_started + 5.0

    if not managed.reaped.is_set():
        try:
            _wait_for_registered_process(
                process,
                timeout=max(0.0, term_deadline - time.monotonic()),
            )
        except subprocess.TimeoutExpired:
            pass
        except ExperimentError:
            pass

    group_exited = _wait_for_process_group_exit(managed, term_deadline)
    if not group_exited:
        _request_process_group_kill(managed)

    kill_deadline = time.monotonic() + 5.0
    if not managed.reaped.is_set():
        try:
            _wait_for_registered_process(
                process,
                timeout=max(0.0, kill_deadline - time.monotonic()),
            )
        except (subprocess.TimeoutExpired, ExperimentError):
            pass
    group_exited = _wait_for_process_group_exit(managed, kill_deadline)
    return managed.reaped.is_set() and group_exited


def process_cpu_times(
    process: Optional[subprocess.Popen],
) -> Tuple[Optional[float], Optional[float]]:
    if process is None:
        return None, None
    managed = _managed_process(process)
    if managed is None:
        return None, None
    return managed.cpu_user_seconds, managed.cpu_system_seconds


def _try_reap_registered_process(managed: _ManagedProcess) -> None:
    """Reap one leader without waiting for a lock held by its worker."""

    if managed.reaped.is_set() or not managed.wait_lock.acquire(blocking=False):
        return
    try:
        if managed.reaped.is_set():
            return
        if os.name == "posix" and hasattr(os, "wait4"):
            try:
                pid, status, usage = os.wait4(
                    managed.process.pid, os.WNOHANG
                )
            except ChildProcessError:
                return
            if pid == managed.process.pid:
                _record_wait_status(managed, status, usage)
            return
        return_code = managed.process.poll()
        if return_code is not None:
            managed.reaped.set()
    finally:
        managed.wait_lock.release()


def _wait_managed_processes_until(
    managed_processes: Sequence[_ManagedProcess],
    deadline: float,
) -> bool:
    while True:
        for managed in managed_processes:
            _try_reap_registered_process(managed)
        if all(
            _managed_process_is_fully_stopped(managed)
            for managed in managed_processes
        ):
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(0.05, remaining))


def terminate_active_processes() -> bool:
    """Terminate all owned groups with one shared TERM/KILL time budget."""

    with _ACTIVE_PROCESS_LOCK:
        managed_processes = list(_ACTIVE_PROCESSES.values())
    managed_processes = [
        managed
        for managed in managed_processes
        if not _managed_process_is_fully_stopped(managed)
    ]
    # Signal the whole snapshot first.  With many workers this avoids waiting
    # up to the grace period for one process while every other solver keeps
    # consuming resources.
    for managed in managed_processes:
        _request_process_group_termination(managed)
    term_deadline = time.monotonic() + 5.0
    if not _wait_managed_processes_until(managed_processes, term_deadline):
        remaining = [
            managed
            for managed in managed_processes
            if not _managed_process_is_fully_stopped(managed)
        ]
        for managed in remaining:
            _request_process_group_kill(managed)
        cleanup_confirmed = _wait_managed_processes_until(
            remaining, time.monotonic() + 5.0
        )
    else:
        cleanup_confirmed = True
    return cleanup_confirmed and active_process_count() == 0


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
            "HGASP_SP_TRIGGER_OFFSET": str(sp.get("trigger_offset", 0)),
            "HGASP_SP_TIME_LIMIT_SECONDS": format(
                float(sp["time_limit_seconds"]), ".15g"
            ),
            "HGASP_SP_LOCAL_ENABLED": (
                "1" if sp.get("local_enabled", True) else "0"
            ),
            "HGASP_SP_LOCAL_TRIGGER_FREQUENCY": str(
                sp.get("local_trigger_frequency", 10000)
            ),
            "HGASP_SP_LOCAL_TIME_LIMIT_SECONDS": format(
                float(sp.get("local_time_limit_seconds", 30.0)),
                ".15g",
            ),
            "HGASP_SP_ADAPTIVE_TRIGGER_ENABLED": (
                "1"
                if sp.get("adaptive_trigger_enabled", True)
                else "0"
            ),
            "HGASP_SP_ADAPTIVE_MINIMUM_INTERVAL": str(
                sp.get("adaptive_minimum_interval", 5000)
            ),
            "HGASP_SP_ADAPTIVE_LOCAL_NEW_COLUMNS": str(
                sp.get("adaptive_local_new_columns", 250)
            ),
            "HGASP_SP_ADAPTIVE_LOCAL_STAGNATION": str(
                sp.get("adaptive_local_stagnation", 5000)
            ),
            "HGASP_SP_ADAPTIVE_GLOBAL_STAGNATION": str(
                sp.get("adaptive_global_stagnation", 20000)
            ),
            "HGASP_SP_ADAPTIVE_UTILITY_EMA_WEIGHT": format(
                float(sp.get("adaptive_utility_ema_weight", 0.25)),
                ".15g",
            ),
            "HGASP_SP_ADAPTIVE_HIGH_UTILITY_THRESHOLD": format(
                float(
                    sp.get(
                        "adaptive_high_utility_threshold",
                        1.0e-3,
                    )
                ),
                ".15g",
            ),
            "HGASP_SP_LP_DUAL_FEEDBACK_ENABLED": (
                "1"
                if sp.get("lp_dual_feedback_enabled", True)
                else "0"
            ),
            "HGASP_SP_LP_TIME_LIMIT_SECONDS": format(
                float(sp.get("lp_time_limit_seconds", 10.0)),
                ".15g",
            ),
            "HGASP_SP_PERSISTENT_LP_ENABLED": (
                "1" if sp.get("persistent_lp_enabled", True) else "0"
            ),
            "HGASP_SP_SOLUTION_POOL_SIZE": str(
                sp.get("solution_pool_size", 5)
            ),
            "HGASP_SP_SOLUTION_POOL_GAP": format(
                float(sp.get("solution_pool_gap", 0.02)), ".15g"
            ),
            "HGASP_SP_SOLUTION_POOL_SEARCH_MODE": str(
                sp.get("solution_pool_search_mode", 2)
            ),
            "HGASP_SP_DIVERSE_INJECTION_COUNT": str(
                sp.get("diverse_injection_count", 3)
            ),
            "HGASP_ADAPTIVE_SEARCH_ALLOCATION_ENABLED": (
                "1"
                if sp.get("adaptive_search_allocation_enabled", True)
                else "0"
            ),
            "HGASP_ADAPTIVE_POPULATION_EXPLORATION": format(
                float(sp.get("adaptive_population_exploration", 0.25)),
                ".15g",
            ),
            "HGASP_ADAPTIVE_POPULATION_DIVERSITY_WEIGHT": format(
                float(
                    sp.get(
                        "adaptive_population_diversity_weight", 0.10
                    )
                ),
                ".15g",
            ),
            "HGASP_ADAPTIVE_POPULATION_COLUMN_WEIGHT": format(
                float(
                    sp.get("adaptive_population_column_weight", 0.05)
                ),
                ".15g",
            ),
            "HGASP_ADAPTIVE_SEARCH_EMA_WEIGHT": format(
                float(sp.get("adaptive_search_ema_weight", 0.20)),
                ".15g",
            ),
            "HGASP_ADAPTIVE_MUTATION_MINIMUM_PERCENT": str(
                sp.get("adaptive_mutation_minimum_percent", 5)
            ),
            "HGASP_ADAPTIVE_MUTATION_MAXIMUM_PERCENT": str(
                sp.get("adaptive_mutation_maximum_percent", 40)
            ),
            "HGASP_SP_WORK_LIMIT": format(
                float(sp.get("work_limit", 0.0)), ".15g"
            ),
            "HGASP_SP_MIP_GAP": format(float(sp["mip_gap"]), ".15g"),
            "HGASP_SP_SEED": str(sp["seed"]),
            "HGASP_SP_MAX_POOL_SIZE": str(sp["max_pool_size"]),
            "HGASP_SP_OUTPUT_FLAG": str(sp["output_flag"]),
            "HGASP_SP_MIP_FOCUS": str(sp["mip_focus"]),
            "HGASP_SP_HEURISTICS": format(float(sp["heuristics"]), ".15g"),
            "HGASP_SP_WLS_TOKEN_DURATION_MINUTES": str(
                sp.get("wls_token_duration_minutes", 0)
            ),
            "HGASP_SP_WLS_TOKEN_REFRESH": format(
                float(sp.get("wls_token_refresh", 0.9)), ".15g"
            ),
            "HGASP_SP_CLIENT_LOG_LEVEL": str(
                sp.get("client_log_level", 1)
            ),
            "HGASP_SP_SERVER_TIMEOUT_SECONDS": str(
                sp.get("server_timeout_seconds", 30)
            ),
            "HGASP_SP_ENVIRONMENT_MAX_ATTEMPTS": str(
                sp.get("environment_max_attempts", 3)
            ),
            "HGASP_SP_ENVIRONMENT_RETRY_BASE_MILLISECONDS": str(
                sp.get("environment_retry_base_milliseconds", 250)
            ),
            "HGASP_SP_REPRESENTATION_ROUTE_LIMIT": (
                "1" if sp["representation_route_limit"] else "0"
            ),
        }
    )
    if config.get("gurobi_home"):
        overrides["GUROBI_HOME"] = str(config["gurobi_home"])
    return overrides


def _execute_run_locked(
    config: Dict[str, Any],
    run_dir: Path,
    repo_root: Path,
    force_successful: bool = False,
    shared_provenance: Optional[Mapping[str, Any]] = None,
    cancellation_event: Optional[threading.Event] = None,
    cpu_affinity: Optional[Sequence[int]] = None,
    affinity_mode: str = "unbound",
    inherited_lock_fds: Sequence[int] = (),
) -> Dict[str, Any]:
    """Execute one run and return its final run.json document."""

    if cancellation_event is not None and cancellation_event.is_set():
        raise ExperimentError("run cancelled before launch")
    _validate_run_config(config, run_dir)
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
    solution_json_path = Path(
        str(config.get("solution_json") or solution_path.with_name("solution.json"))
    )
    validation_config = dict(config.get("validation", {"required": False}))
    validation_required = bool(validation_config.get("required", False))
    sp_success_required = bool(
        config["sp"].get(
            "require_success", SP_OPTIONAL_DEFAULTS["require_success"]
        )
    )
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
        expected_solution_hash = previous_hashes.get("solution_sha256")
        has_solution = (
            solution_path.is_file()
            and solution_path.stat().st_size > 0
            and isinstance(expected_solution_hash, str)
            and sha256_file(solution_path) == expected_solution_hash
        )
        previous_has_solution_json = bool(
            previous.get("solution_json_exists", False)
        )
        if validation_required or previous_has_solution_json:
            expected_solution_json_hash = previous_hashes.get(
                "solution_json_sha256"
            )
            has_solution = (
                has_solution
                and solution_json_path.is_file()
                and solution_json_path.stat().st_size > 0
                and isinstance(expected_solution_json_hash, str)
                and sha256_file(solution_json_path)
                == expected_solution_json_hash
            )
        if same_inputs and has_solution and validation_required:
            try:
                validation_result = solution_validator.validate_solution(
                    Path(str(config["instance"])),
                    solution_json_path,
                    str(config.get("problem_mode", "clrp")),
                    absolute_tolerance=float(
                        validation_config.get(
                            "absolute_tolerance",
                            solution_validator.DEFAULT_ABSOLUTE_TOLERANCE,
                        )
                    ),
                    relative_tolerance=float(
                        validation_config.get(
                            "relative_tolerance",
                            solution_validator.DEFAULT_RELATIVE_TOLERANCE,
                        )
                    ),
                )
                has_solution = validation_result.valid
            except solution_validator.ValidationInputError:
                has_solution = False
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
    process_status_path = attempt_log_dir / "process.status.json"
    config_path.write_bytes(config_payload)

    command = build_solver_command(config)
    environment_overrides = _environment_overrides(config)
    environment_overrides.update(
        {
            "HGASP_LOG_DIR": str(attempt_log_dir),
            "HGASP_LOG_CONSOLE": "0",
            "HGASP_LOG_SEGMENT_BYTES": str(8 * 1024 * 1024),
            "HGASP_LOG_BACKUPS": "2",
            "HGASP_LOG_MESSAGE_MAX_BYTES": "4096",
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
    requested_affinity = (
        sorted(set(int(cpu) for cpu in cpu_affinity))
        if cpu_affinity is not None
        else None
    )
    result: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "experiment_name": config["experiment_name"],
        "run_id": config["run_id"],
        "instance_id": config["instance_id"],
        "problem_mode": config.get("problem_mode", "clrp"),
        "reference": config.get("reference"),
        "best_known": float(config.get("best_known", 0.0)),
        "is_optimal": bool(config.get("is_optimal", False)),
        "stop_at_reference": bool(config.get("stop_at_reference", True)),
        "seed": int(config["seed"]),
        "attempt": attempt,
        "status": "running",
        "process_status": "running",
        "started_at": utc_now(),
        "finished_at": None,
        "duration_seconds": None,
        "duration_basis": "process_cpu",
        "duration_scope": "whole_child_process",
        "cpu_time_seconds": None,
        "cpu_user_seconds": None,
        "cpu_system_seconds": None,
        "wall_duration_seconds": None,
        "return_code": None,
        "timed_out": False,
        "supervisor_pid": None,
        "supervisor_process_group_id": None,
        "solver_pid": None,
        "solver_process_group_id": None,
        "solver_start_ticks": None,
        "process_health": "launching",
        "command": command,
        "cwd": str(repo_root),
        "environment_overrides": environment_overrides,
        "cpu_affinity": {
            "mode": affinity_mode,
            "requested_logical_cpu_ids": requested_affinity,
            "effective_logical_cpu_ids": None,
        },
        "process_group_cleanup": {
            "status": "not_started",
            "confirmed": True,
            "process_group_id": None,
            "error": None,
        },
        "files": {
            "resolved_config": str(config_path),
            "log_directory": str(attempt_log_dir),
            "stdout": str(stdout_path),
            "stderr": str(stderr_path),
            "hgasp": str(hgasp_log_path),
            "events": str(attempt_log_dir / "events.jsonl"),
            "algorithm_status": str(
                attempt_log_dir / "algorithm.status.json"
            ),
            "gurobi": str(gurobi_log_path),
            "process_status": str(process_status_path),
            "solution": str(solution_path),
            "solution_json": str(solution_json_path),
        },
        "solution_exists": False,
        "solution_size_bytes": 0,
        "solution_json_exists": False,
        "solution_json_size_bytes": 0,
        "validation_required": validation_required,
        "validation_status": "pending" if validation_required else "not_required",
        "validation": None,
        "sp_success_assertion": {
            "required": sp_success_required,
            "checked": False,
            "observed": False,
            "result_status_marker_found": False,
            "success_marker_found": False,
            "checked_logs": [],
            "read_errors": [],
        },
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
    try:
        solution_json_path.unlink()
    except FileNotFoundError:
        pass
    except OSError as error:
        raise ExperimentError(
            f"cannot replace prior JSON solution {solution_json_path}: {error}"
        )

    wall_start = time.monotonic()
    process: Optional[subprocess.Popen] = None
    control_read_fd: Optional[int] = None
    control_write_fd: Optional[int] = None
    timeout = config["limits"].get("process_timeout_seconds")
    try:
        with stdout_path.open("wb") as stdout_stream, stderr_path.open(
            "wb"
        ) as stderr_stream:
            with inherited_cpu_affinity(cpu_affinity) as launcher_affinity:
                launch_command = command
                popen_options: Dict[str, Any] = {
                    "cwd": str(repo_root),
                    "env": process_environment,
                    "stdout": stdout_stream,
                    "stderr": stderr_stream,
                    "start_new_session": (os.name == "posix"),
                }
                if os.name == "posix":
                    if hasattr(os, "pipe2") and hasattr(os, "O_CLOEXEC"):
                        control_read_fd, control_write_fd = os.pipe2(
                            os.O_CLOEXEC
                        )
                    else:
                        control_read_fd, control_write_fd = os.pipe()
                        os.set_inheritable(control_read_fd, False)
                        os.set_inheritable(control_write_fd, False)
                    launch_command = [
                        sys.executable,
                        str(SUPERVISOR_PATH),
                        "--control-fd",
                        str(control_read_fd),
                        "--status-file",
                        str(process_status_path),
                        "--heartbeat-seconds",
                        format(SUPERVISOR_HEARTBEAT_SECONDS, ".15g"),
                        "--term-grace-seconds",
                        format(SUPERVISOR_TERM_GRACE_SECONDS, ".15g"),
                        "--kill-grace-seconds",
                        format(SUPERVISOR_KILL_GRACE_SECONDS, ".15g"),
                        "--",
                        *command,
                    ]
                    pass_fds = tuple(
                        sorted(
                            {
                                int(control_read_fd),
                                *(
                                    int(fd)
                                    for fd in inherited_lock_fds
                                    if int(fd) >= 0
                                ),
                            }
                        )
                    )
                    popen_options["pass_fds"] = pass_fds
                process = subprocess.Popen(
                    launch_command,
                    **popen_options,
                )
                if control_read_fd is not None:
                    os.close(control_read_fd)
                    control_read_fd = None
                # Register immediately after Popen, before affinity inspection
                # or any operation that could race cancellation.  Signal
                # handlers only set the shared event, so they cannot unwind
                # this small critical section.
                managed = _register_process(process)
                result["process_group_cleanup"] = {
                    "status": "pending",
                    "confirmed": False,
                    "process_group_id": managed.process_group_id,
                    "error": None,
                }
                result["supervisor_pid"] = process.pid
                result["supervisor_process_group_id"] = (
                    managed.process_group_id
                )
                effective_affinity = launcher_affinity
                if hasattr(os, "sched_getaffinity"):
                    try:
                        effective_affinity = tuple(
                            sorted(
                                int(cpu)
                                for cpu in os.sched_getaffinity(process.pid)
                            )
                        )
                    except (OSError, ProcessLookupError):
                        pass
                result["cpu_affinity"]["effective_logical_cpu_ids"] = list(
                    effective_affinity
                )
                # Persist the live supervisor identity immediately.  A hard
                # parent death after this point leaves an actionable record,
                # while the control-pipe EOF independently stops the solver.
                write_json(run_path, result)
            try:
                if cancellation_event is not None and cancellation_event.is_set():
                    if control_write_fd is not None:
                        os.close(control_write_fd)
                        control_write_fd = None
                    _terminate_process(process)
                    result["return_code"] = process.returncode
                else:
                    result["return_code"] = _wait_for_registered_process(
                        process,
                        timeout=float(timeout) if timeout is not None else None,
                        cancellation_event=cancellation_event,
                    )
            except _RunCancelled:
                if control_write_fd is not None:
                    os.close(control_write_fd)
                    control_write_fd = None
                _terminate_process(process)
                result["return_code"] = process.returncode
            except subprocess.TimeoutExpired:
                result["timed_out"] = True
                if control_write_fd is not None:
                    os.close(control_write_fd)
                    control_write_fd = None
                _terminate_process(process)
                result["return_code"] = process.returncode
    except KeyboardInterrupt:
        if process is not None:
            _terminate_process(process)
            result["return_code"] = process.returncode
        result["status"] = "interrupted"
        result["process_status"] = "interrupted"
        result["error"] = "interrupted by user"
        raise
    except OSError as error:
        result["status"] = "launch_error"
        result["process_status"] = "launch_error"
        result["error"] = str(error)
    finally:
        for descriptor_name in ("control_read_fd", "control_write_fd"):
            descriptor = (
                control_read_fd
                if descriptor_name == "control_read_fd"
                else control_write_fd
            )
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
                if descriptor_name == "control_read_fd":
                    control_read_fd = None
                else:
                    control_write_fd = None
        cleanup_confirmed = True
        if process is not None:
            managed = _managed_process(process)
            if managed is not None:
                if not _managed_process_is_fully_stopped(managed):
                    cleanup_confirmed = _terminate_process(process)
                    result["return_code"] = process.returncode
                else:
                    cleanup_confirmed = True
                if cleanup_confirmed:
                    result["process_group_cleanup"] = {
                        "status": "complete",
                        "confirmed": True,
                        "process_group_id": managed.process_group_id,
                        "error": None,
                    }
                else:
                    cleanup_error = (
                        "solver process-group cleanup could not be confirmed "
                        "after TERM and KILL"
                    )
                    result["process_group_cleanup"] = {
                        "status": "failed",
                        "confirmed": False,
                        "process_group_id": managed.process_group_id,
                        "error": cleanup_error,
                    }
                    if result["status"] == "running":
                        result["status"] = "failed"
                        result["process_status"] = "cleanup_failed"
                        result["error"] = cleanup_error
            else:
                cleanup_confirmed = False
                cleanup_error = (
                    "solver process-group cleanup could not be confirmed "
                    "because its registration is missing"
                )
                result["process_group_cleanup"] = {
                    "status": "failed",
                    "confirmed": False,
                    "process_group_id": None,
                    "error": cleanup_error,
                }
                if result["status"] == "running":
                    result["status"] = "failed"
                    result["process_status"] = "cleanup_failed"
                    result["error"] = cleanup_error
        status_document = _read_previous_run(process_status_path)
        if status_document is not None:
            result["solver_pid"] = status_document.get("solver_pid")
            result["solver_process_group_id"] = status_document.get(
                "solver_process_group_id"
            )
            result["solver_start_ticks"] = status_document.get(
                "solver_start_ticks"
            )
            result["process_health"] = status_document.get(
                "health", result["process_health"]
            )
            solver_group_id = status_document.get(
                "solver_process_group_id"
            )
            if (
                isinstance(solver_group_id, int)
                and result["process_group_cleanup"]["status"] != "not_started"
            ):
                result["process_group_cleanup"][
                    "process_group_id"
                ] = solver_group_id
        cpu_user_seconds, cpu_system_seconds = process_cpu_times(process)
        if status_document is not None:
            solver_cpu_user = status_document.get("cpu_user_seconds")
            solver_cpu_system = status_document.get("cpu_system_seconds")
            if isinstance(solver_cpu_user, (int, float)) and isinstance(
                solver_cpu_system, (int, float)
            ):
                cpu_user_seconds = float(solver_cpu_user)
                cpu_system_seconds = float(solver_cpu_system)
        if process is not None and cleanup_confirmed:
            _unregister_process(process)
        result["finished_at"] = utc_now()
        wall_duration_seconds = round(time.monotonic() - wall_start, 6)
        result["wall_duration_seconds"] = wall_duration_seconds
        if cpu_user_seconds is not None and cpu_system_seconds is not None:
            result["cpu_user_seconds"] = round(cpu_user_seconds, 6)
            result["cpu_system_seconds"] = round(cpu_system_seconds, 6)
            result["cpu_time_seconds"] = round(
                cpu_user_seconds + cpu_system_seconds, 6
            )
            result["duration_seconds"] = result["cpu_time_seconds"]
        else:
            # wait4 is unavailable on some non-POSIX platforms.  Preserve a
            # usable duration while making the fallback explicit in metadata.
            result["duration_basis"] = "wall_clock_fallback"
            result["duration_seconds"] = wall_duration_seconds
        result["solution_exists"] = solution_path.is_file()
        if result["solution_exists"]:
            result["solution_size_bytes"] = solution_path.stat().st_size
            if result["solution_size_bytes"] > 0:
                hashes["solution_sha256"] = sha256_file(solution_path)
        result["solution_json_exists"] = solution_json_path.is_file()
        if result["solution_json_exists"]:
            result["solution_json_size_bytes"] = solution_json_path.stat().st_size
            if result["solution_json_size_bytes"] > 0:
                hashes["solution_json_sha256"] = sha256_file(
                    solution_json_path
                )
        if sp_success_required:
            sp_success_evidence = inspect_sp_success_attempt(
                hgasp_log_path, stdout_path
            )
            result["sp_success_assertion"] = {
                "required": True,
                **sp_success_evidence,
            }
        else:
            sp_success_evidence = result["sp_success_assertion"]

        if result["status"] == "running":
            if cancellation_event is not None and cancellation_event.is_set():
                result["status"] = "interrupted"
                result["process_status"] = "interrupted"
                result["error"] = "cancelled by matrix runner"
            elif result["timed_out"]:
                result["status"] = "timed_out"
                result["process_status"] = "timed_out"
                result["error"] = "external process timeout expired"
            elif result["return_code"] != 0:
                result["status"] = "failed"
                result["process_status"] = "failed"
                result["error"] = (
                    f"solver exited with return code {result['return_code']}"
                )
            elif (
                sp_success_required
                and not sp_success_evidence["observed"]
            ):
                missing = []
                if not sp_success_evidence["result_status_marker_found"]:
                    missing.append("'[SP Result] status='")
                if not sp_success_evidence["success_marker_found"]:
                    missing.append("'[SP Success]'")
                detail = ""
                if sp_success_evidence["read_errors"]:
                    detail = "; log read errors: " + "; ".join(
                        sp_success_evidence["read_errors"]
                    )
                result["status"] = "failed"
                result["process_status"] = "sp_assertion_failed"
                strict_detail = ""
                if sp_success_evidence.get("failure_statuses"):
                    strict_detail += (
                        "; failing SP statuses: "
                        + ", ".join(sp_success_evidence["failure_statuses"])
                    )
                result_count = sp_success_evidence.get("result_count", 0)
                success_count = sp_success_evidence.get("success_count", 0)
                if result_count != success_count:
                    strict_detail += (
                        f"; SP result/success count mismatch "
                        f"{result_count}/{success_count}"
                    )
                result["error"] = (
                    "required SP success assertion failed: missing "
                    + " and ".join(missing)
                    + " from this attempt's stdout.log/hgasp.log"
                    + detail
                    + strict_detail
                )
            elif not result["solution_exists"] or result["solution_size_bytes"] == 0:
                result["status"] = "failed"
                result["process_status"] = "missing_solution"
                result["error"] = "solver exited successfully but wrote no solution"
            else:
                result["process_status"] = "passed"
                should_validate = (
                    validation_required
                    or (
                        result["solution_json_exists"]
                        and result["solution_json_size_bytes"] > 0
                    )
                )
                if should_validate:
                    if (
                        not result["solution_json_exists"]
                        or result["solution_json_size_bytes"] == 0
                    ):
                        result["status"] = "failed"
                        result["validation_status"] = "missing"
                        result["error"] = (
                            "independent validation is required but "
                            "solution.json is missing"
                        )
                    else:
                        try:
                            validation_result = (
                                solution_validator.validate_solution(
                                    Path(str(config["instance"])),
                                    solution_json_path,
                                    str(config.get("problem_mode", "clrp")),
                                    absolute_tolerance=float(
                                        validation_config.get(
                                            "absolute_tolerance",
                                            solution_validator.DEFAULT_ABSOLUTE_TOLERANCE,
                                        )
                                    ),
                                    relative_tolerance=float(
                                        validation_config.get(
                                            "relative_tolerance",
                                            solution_validator.DEFAULT_RELATIVE_TOLERANCE,
                                        )
                                    ),
                                )
                            )
                            result["validation"] = validation_result.as_dict()
                            result["validation_status"] = (
                                "passed" if validation_result.valid else "failed"
                            )
                            if validation_result.valid:
                                result["status"] = SUCCESS_STATUS
                            else:
                                result["status"] = "failed"
                                first_error = (
                                    validation_result.errors[0]
                                    if validation_result.errors
                                    else "unspecified validation failure"
                                )
                                result["error"] = (
                                    "independent solution validation failed: "
                                    + first_error
                                )
                        except solution_validator.ValidationInputError as error:
                            result["status"] = "failed"
                            result["validation_status"] = "error"
                            result["validation"] = {
                                "status": "error",
                                "valid": False,
                                "errors": [str(error)],
                                "warnings": [],
                                "recomputed": {},
                            }
                            result["error"] = (
                                "independent solution validation could not run: "
                                + str(error)
                            )
                else:
                    result["status"] = SUCCESS_STATUS
        # A signal can arrive while validation or hashing is running, after
        # the earlier status decision.  Cancellation remains authoritative
        # until the final atomic metadata write.
        if (
            cancellation_event is not None
            and cancellation_event.is_set()
            and result["status"] != "interrupted"
        ):
            result["status"] = "interrupted"
            result["process_status"] = "interrupted"
            result["error"] = "cancelled by experiment runner"
        write_json(run_path, result)

    return result


def execute_run(
    config: Dict[str, Any],
    run_dir: Path,
    repo_root: Path,
    force_successful: bool = False,
    shared_provenance: Optional[Mapping[str, Any]] = None,
    cancellation_event: Optional[threading.Event] = None,
    cpu_affinity: Optional[Sequence[int]] = None,
    affinity_mode: str = "unbound",
    inherited_lock_fds: Sequence[int] = (),
) -> Dict[str, Any]:
    """Execute one run while exclusively owning its output directory."""

    run_dir.mkdir(parents=True, exist_ok=True)
    with CrossProcessFileLock(
        run_dir / ".run.lock", f"run {config.get('run_id', run_dir.name)}"
    ) as run_lock:
        assert run_lock.fd is not None
        return _execute_run_locked(
            config,
            run_dir,
            repo_root,
            force_successful=force_successful,
            shared_provenance=shared_provenance,
            cancellation_event=cancellation_event,
            cpu_affinity=cpu_affinity,
            affinity_mode=affinity_mode,
            inherited_lock_fds=(
                *tuple(int(fd) for fd in inherited_lock_fds),
                run_lock.fd,
            ),
        )


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
        "problem_mode": arguments.problem_mode,
        "binary": str(binary),
        "instance": str(instance),
        "depot_config": depot,
        "solution": str(solution),
        "solution_json": str(run_dir / "solution.json"),
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
            "trigger_offset": arguments.sp_trigger_offset,
            "time_limit_seconds": arguments.sp_time_limit,
            "work_limit": arguments.sp_work_limit,
            "mip_gap": arguments.sp_mip_gap,
            "seed": arguments.sp_seed,
            "max_pool_size": arguments.sp_max_pool_size,
            "output_flag": arguments.sp_output_flag,
            "mip_focus": arguments.sp_mip_focus,
            "heuristics": arguments.sp_heuristics,
            "wls_token_duration_minutes": (
                arguments.sp_wls_token_duration_minutes
            ),
            "wls_token_refresh": arguments.sp_wls_token_refresh,
            "client_log_level": arguments.sp_client_log_level,
            "server_timeout_seconds": arguments.sp_server_timeout,
            "environment_max_attempts": (
                arguments.sp_environment_max_attempts
            ),
            "environment_retry_base_milliseconds": (
                arguments.sp_environment_retry_base_ms
            ),
            "representation_route_limit": (
                arguments.sp_representation_route_limit
            ),
            "require_success": arguments.require_sp_success,
        },
        "environment": parse_environment(arguments.env),
        "validation": {
            "required": (
                arguments.require_validation
                or arguments.problem_mode == "clrp-tw"
            ),
            "absolute_tolerance": arguments.validation_absolute_tolerance,
            "relative_tolerance": arguments.validation_relative_tolerance,
        },
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
        "--problem-mode",
        choices=sorted(PROBLEM_MODES),
        default="clrp",
        help="mathematical problem mode (default: clrp)",
    )
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
    parser.add_argument(
        "--require-validation",
        action="store_true",
        help="require an independently valid solution.json before success",
    )
    parser.add_argument(
        "--validation-absolute-tolerance",
        type=float,
        default=solution_validator.DEFAULT_ABSOLUTE_TOLERANCE,
    )
    parser.add_argument(
        "--validation-relative-tolerance",
        type=float,
        default=solution_validator.DEFAULT_RELATIVE_TOLERANCE,
    )
    parser.add_argument("--gurobi-threads", type=int, default=1)
    parser.add_argument("--sp-trigger-frequency", type=int, default=50001)
    parser.add_argument("--sp-trigger-offset", type=int, default=0)
    parser.add_argument("--sp-time-limit", type=float, default=360.0)
    parser.add_argument(
        "--sp-work-limit",
        type=float,
        default=0.0,
        help="deterministic Gurobi work-unit limit; 0 disables it",
    )
    parser.add_argument("--sp-mip-gap", type=float, default=0.001)
    parser.add_argument("--sp-seed", type=int, default=0)
    parser.add_argument("--sp-max-pool-size", type=int, default=50000)
    parser.add_argument("--sp-output-flag", type=int, choices=(0, 1), default=0)
    parser.add_argument("--sp-mip-focus", type=int, choices=range(4), default=1)
    parser.add_argument("--sp-heuristics", type=float, default=0.15)
    parser.add_argument(
        "--require-sp-success",
        action="store_true",
        help=(
            "fail unless this attempt logs both an SP result and "
            "an SP success marker"
        ),
    )
    parser.add_argument(
        "--sp-wls-token-duration-minutes",
        type=int,
        default=0,
        help="WLS token duration override; 0 keeps Gurobi automatic policy",
    )
    parser.add_argument("--sp-wls-token-refresh", type=float, default=0.9)
    parser.add_argument(
        "--sp-client-log-level",
        type=int,
        choices=range(4),
        default=1,
        help="Compute Server/WLS client log level (1 records errors only)",
    )
    parser.add_argument("--sp-server-timeout", type=int, default=30)
    parser.add_argument(
        "--sp-environment-max-attempts",
        type=int,
        default=3,
    )
    parser.add_argument(
        "--sp-environment-retry-base-ms",
        type=int,
        default=250,
    )
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
    cancellation_event = threading.Event()
    signal_state = SignalState()
    with cancellation_signal_handlers(cancellation_event) as signal_state:
        try:
            config, run_dir, repo_root = make_direct_config(arguments)
            result = execute_run(
                config,
                run_dir,
                repo_root,
                force_successful=arguments.force_successful,
                cancellation_event=cancellation_event,
            )
        except ExperimentError as error:
            if signal_state.signum is not None:
                cleanup_confirmed = terminate_active_processes()
                print(
                    f"interrupted by {signal_state.name}; "
                    + (
                        "solver process group cleanup was confirmed"
                        if cleanup_confirmed
                        else "solver process group cleanup could not be "
                        "confirmed after TERM and KILL"
                    ),
                    file=sys.stderr,
                )
                return 128 + signal_state.signum
            print(f"error: {error}", file=sys.stderr)
            return 2
        except KeyboardInterrupt:
            cancellation_event.set()
            cleanup_confirmed = terminate_active_processes()
            print(
                "interrupted; "
                + (
                    "solver process group cleanup was confirmed"
                    if cleanup_confirmed
                    else "solver process group cleanup could not be "
                    "confirmed after TERM and KILL"
                ),
                file=sys.stderr,
            )
            return 130

    if signal_state.signum is not None:
        cleanup_confirmed = terminate_active_processes()
        print(
            f"interrupted by {signal_state.name}; "
            + (
                "solver process group cleanup was confirmed"
                if cleanup_confirmed
                else "solver process group cleanup could not be "
                "confirmed after TERM and KILL"
            ),
            file=sys.stderr,
        )
        return 128 + signal_state.signum

    cleanup_confirmed = terminate_active_processes()
    if not cleanup_confirmed:
        print(
            "error: one or more solver process groups could not be "
            "confirmed stopped after TERM and KILL",
            file=sys.stderr,
        )
        return 1

    if result.get("skipped"):
        print(f"SKIP {result['run_id']}: successful result already exists")
    else:
        cpu_time = result.get("cpu_time_seconds")
        timing = (
            f"{cpu_time:.3f} CPU s, "
            f"{result['wall_duration_seconds']:.3f} wall s"
            if isinstance(cpu_time, (int, float))
            else f"{result['wall_duration_seconds']:.3f} wall s"
        )
        print(
            f"{result['status'].upper()} {result['run_id']} "
            f"({timing})"
        )
    return 0 if result["status"] == SUCCESS_STATUS else 1


if __name__ == "__main__":
    sys.exit(main())
