#!/usr/bin/env python3
"""Supervise one solver process independently of the experiment runner.

The runner passes the read end of a private control pipe.  EOF means that the
runner disappeared, including an uncatchable SIGKILL.  The supervisor then
terminates the solver's isolated process group, escalates to SIGKILL after a
bounded grace period, and reaps every adopted descendant it can observe.

This module deliberately has no dependency on ``run_one``.  It must remain a
small, single-threaded process so it can safely own the solver lifecycle.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import os
import selectors
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Optional, Sequence


PR_SET_CHILD_SUBREAPER = 36
PR_SET_PDEATHSIG = 1
HEARTBEAT_SCHEMA_VERSION = 1


def _utc_now() -> str:
    import datetime as dt

    return (
        dt.datetime.now(dt.timezone.utc)
        .isoformat(timespec="milliseconds")
        .replace("+00:00", "Z")
    )


def _write_json_atomic(path: Path, value: Dict[str, Any]) -> None:
    """Replace one bounded status document without growing an append log."""

    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n"
    ).encode("utf-8")
    temporary_name: Optional[str] = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            dir=str(path.parent),
            prefix=f".{path.name}.",
            delete=False,
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


def _enable_linux_subreaper() -> bool:
    if not sys.platform.startswith("linux"):
        return False
    try:
        libc = ctypes.CDLL(None, use_errno=True)
        result = libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0)
        return result == 0
    except (AttributeError, OSError):
        return False


def _linux_solver_parent_death_guard(expected_parent_pid: int) -> Any:
    """Return a pre-exec guard; safe because this supervisor is single-threaded."""

    def configure() -> None:
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(PR_SET_PDEATHSIG, signal.SIGKILL, 0, 0, 0) != 0:
            os._exit(125)
        # The parent may have died between fork and prctl.
        if os.getppid() != expected_parent_pid:
            os._exit(125)

    return configure


def _read_linux_process(pid: int) -> Optional[Dict[str, Any]]:
    if not sys.platform.startswith("linux"):
        return None
    try:
        content = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except (FileNotFoundError, OSError, UnicodeError):
        return None
    closing = content.rfind(")")
    if closing < 0:
        return None
    fields = content[closing + 2 :].split()
    if len(fields) < 22:
        return None
    try:
        clock_ticks = float(os.sysconf("SC_CLK_TCK"))
        page_size = int(os.sysconf("SC_PAGE_SIZE"))
        return {
            "pid": pid,
            "state": fields[0],
            "process_group_id": int(fields[2]),
            "cpu_seconds": (
                float(int(fields[11]) + int(fields[12])) / clock_ticks
            ),
            "start_ticks": int(fields[19]),
            "rss_bytes": max(0, int(fields[21])) * page_size,
        }
    except (OSError, ValueError):
        return None


def _group_snapshot(process_group_id: int) -> Dict[str, Any]:
    """Return bounded aggregate health for one solver process group."""

    if not sys.platform.startswith("linux"):
        try:
            os.killpg(process_group_id, 0)
            alive = True
        except ProcessLookupError:
            alive = False
        except (PermissionError, OSError):
            alive = True
        return {
            "alive": alive,
            "member_count": None,
            "cpu_seconds": None,
            "rss_bytes": None,
            "states": [],
        }

    members = []
    try:
        entries = Path("/proc").iterdir()
    except OSError:
        entries = ()
    for entry in entries:
        if not entry.name.isdigit():
            continue
        item = _read_linux_process(int(entry.name))
        if (
            item is not None
            and item["process_group_id"] == process_group_id
            and item["state"] != "Z"
        ):
            members.append(item)
    return {
        "alive": bool(members),
        "member_count": len(members),
        "cpu_seconds": round(
            sum(float(item["cpu_seconds"]) for item in members), 6
        ),
        "rss_bytes": sum(int(item["rss_bytes"]) for item in members),
        "states": sorted({str(item["state"]) for item in members}),
    }


def _signal_group(process_group_id: int, signum: int) -> None:
    try:
        os.killpg(process_group_id, signum)
    except (ProcessLookupError, PermissionError, OSError):
        pass


def _reap_adopted_children() -> None:
    if os.name != "posix":
        return
    while True:
        try:
            pid, _status = os.waitpid(-1, os.WNOHANG)
        except ChildProcessError:
            return
        except OSError:
            return
        if pid <= 0:
            return


def _wait_group_exit(
    process_group_id: int,
    deadline: float,
    heartbeat: "Heartbeat",
    health: str,
) -> bool:
    while True:
        snapshot = _group_snapshot(process_group_id)
        heartbeat.write(health, snapshot=snapshot)
        if not snapshot["alive"]:
            _reap_adopted_children()
            return True
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        time.sleep(min(0.05, remaining))


class Heartbeat:
    def __init__(
        self,
        path: Path,
        supervisor_pid: int,
        solver_pid: int,
        solver_pgid: int,
        solver_start_ticks: Optional[int],
        subreaper_enabled: bool,
    ) -> None:
        self.path = path
        self.supervisor_pid = supervisor_pid
        self.solver_pid = solver_pid
        self.solver_pgid = solver_pgid
        self.solver_start_ticks = solver_start_ticks
        self.subreaper_enabled = subreaper_enabled
        self.sequence = 0
        self.started_monotonic = time.monotonic()
        self.maximum_cpu_seconds = 0.0
        self.solver_cpu_user_seconds: Optional[float] = None
        self.solver_cpu_system_seconds: Optional[float] = None

    def write(
        self,
        health: str,
        *,
        snapshot: Optional[Dict[str, Any]] = None,
        reason: Optional[str] = None,
        solver_return_code: Optional[int] = None,
        solver_cpu_user_seconds: Optional[float] = None,
        solver_cpu_system_seconds: Optional[float] = None,
    ) -> None:
        self.sequence += 1
        current = snapshot or _group_snapshot(self.solver_pgid)
        current_cpu = current.get("cpu_seconds")
        if isinstance(current_cpu, (int, float)):
            self.maximum_cpu_seconds = max(
                self.maximum_cpu_seconds, float(current_cpu)
            )
        if solver_cpu_user_seconds is not None:
            self.solver_cpu_user_seconds = float(solver_cpu_user_seconds)
        if solver_cpu_system_seconds is not None:
            self.solver_cpu_system_seconds = float(
                solver_cpu_system_seconds
            )
        if (
            self.solver_cpu_user_seconds is not None
            and self.solver_cpu_system_seconds is not None
        ):
            self.maximum_cpu_seconds = max(
                self.maximum_cpu_seconds,
                self.solver_cpu_user_seconds
                + self.solver_cpu_system_seconds,
            )
        leader = _read_linux_process(self.solver_pid)
        value: Dict[str, Any] = {
            "schema_version": HEARTBEAT_SCHEMA_VERSION,
            "updated_at": _utc_now(),
            "sequence": self.sequence,
            "elapsed_wall_seconds": round(
                time.monotonic() - self.started_monotonic, 3
            ),
            "health": health,
            "reason": reason,
            "supervisor_pid": self.supervisor_pid,
            "solver_pid": self.solver_pid,
            "solver_process_group_id": self.solver_pgid,
            "solver_start_ticks": self.solver_start_ticks,
            "solver_state": leader["state"] if leader else None,
            "cpu_seconds": round(self.maximum_cpu_seconds, 6),
            "cpu_user_seconds": self.solver_cpu_user_seconds,
            "cpu_system_seconds": self.solver_cpu_system_seconds,
            "rss_bytes": current["rss_bytes"],
            "live_member_count": current["member_count"],
            "process_states": current["states"],
            "solver_return_code": solver_return_code,
            "subreaper_enabled": self.subreaper_enabled,
        }
        _write_json_atomic(self.path, value)


def _poll_solver(
    solver: subprocess.Popen,
) -> tuple[Optional[int], Optional[Any]]:
    """Poll the exact solver child while retaining its wait4 CPU usage."""

    if solver.returncode is not None:
        return int(solver.returncode), None
    if os.name == "posix" and hasattr(os, "wait4"):
        try:
            pid, status, usage = os.wait4(solver.pid, os.WNOHANG)
        except ChildProcessError:
            return solver.returncode, None
        if pid == solver.pid:
            try:
                return_code = os.waitstatus_to_exitcode(status)
            except AttributeError:
                return_code = (
                    -os.WTERMSIG(status)
                    if os.WIFSIGNALED(status)
                    else os.WEXITSTATUS(status)
                )
            solver.returncode = int(return_code)
            return int(return_code), usage
        return None, None
    return_code = solver.poll()
    return (
        (int(return_code), None)
        if return_code is not None
        else (None, None)
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--control-fd", required=True, type=int)
    parser.add_argument("--status-file", required=True)
    parser.add_argument("--heartbeat-seconds", type=float, default=5.0)
    parser.add_argument("--term-grace-seconds", type=float, default=2.0)
    parser.add_argument("--kill-grace-seconds", type=float, default=2.0)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    return parser


def _normalized_exit_code(return_code: int) -> int:
    if return_code < 0:
        return min(255, 128 + abs(return_code))
    return min(255, return_code)


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = _build_parser().parse_args(argv)
    if (
        not arguments.command
        or arguments.command[0] != "--"
        or len(arguments.command) < 2
    ):
        print("process_supervisor: missing solver command", file=sys.stderr)
        return 2
    if (
        not math.isfinite(arguments.heartbeat_seconds)
        or arguments.heartbeat_seconds <= 0
        or not math.isfinite(arguments.term_grace_seconds)
        or arguments.term_grace_seconds < 0
        or not math.isfinite(arguments.kill_grace_seconds)
        or arguments.kill_grace_seconds < 0
    ):
        print("process_supervisor: invalid timing argument", file=sys.stderr)
        return 2

    control_fd = int(arguments.control_fd)
    status_path = Path(arguments.status_file)
    command = arguments.command[1:]
    stop_reason: Optional[str] = None

    def request_stop(signum: int, _frame: Any) -> None:
        nonlocal stop_reason
        if stop_reason is None:
            try:
                stop_reason = signal.Signals(signum).name
            except ValueError:
                stop_reason = f"signal-{signum}"

    for signum in (
        signal.SIGINT,
        signal.SIGTERM,
        getattr(signal, "SIGHUP", signal.SIGTERM),
    ):
        signal.signal(signum, request_stop)

    subreaper_enabled = _enable_linux_subreaper()
    solver_options: Dict[str, Any] = {"start_new_session": True}
    if sys.platform.startswith("linux"):
        # ``preexec_fn`` is forbidden in the multi-threaded matrix runner but
        # is safe here: the dedicated supervisor is deliberately single-threaded.
        solver_options["preexec_fn"] = _linux_solver_parent_death_guard(
            os.getpid()
        )
    try:
        solver = subprocess.Popen(command, **solver_options)
    except OSError as error:
        _write_json_atomic(
            status_path,
            {
                "schema_version": HEARTBEAT_SCHEMA_VERSION,
                "updated_at": _utc_now(),
                "health": "launch_error",
                "reason": str(error),
                "supervisor_pid": os.getpid(),
                "solver_pid": None,
                "solver_process_group_id": None,
            },
        )
        return 127

    solver_pgid = solver.pid
    leader = _read_linux_process(solver.pid)
    heartbeat = Heartbeat(
        status_path,
        os.getpid(),
        solver.pid,
        solver_pgid,
        leader["start_ticks"] if leader else None,
        subreaper_enabled,
    )
    heartbeat.write("running")

    selector = selectors.DefaultSelector()
    try:
        os.set_blocking(control_fd, False)
    except (AttributeError, OSError):
        pass
    selector.register(control_fd, selectors.EVENT_READ)
    next_heartbeat = time.monotonic() + arguments.heartbeat_seconds
    return_code: Optional[int] = None
    solver_usage: Optional[Any] = None
    try:
        while True:
            return_code, observed_usage = _poll_solver(solver)
            if observed_usage is not None:
                solver_usage = observed_usage
            if return_code is not None:
                break
            if stop_reason is not None:
                break
            # SIGCHLD may be transparently restarted around select().  Poll
            # the direct child at a small bounded interval rather than making
            # solver completion wait for the heartbeat period.
            timeout = min(
                0.1, max(0.0, next_heartbeat - time.monotonic())
            )
            events = selector.select(timeout)
            if events:
                try:
                    value = os.read(control_fd, 1)
                except BlockingIOError:
                    value = b"x"
                except OSError:
                    value = b""
                if not value:
                    stop_reason = "parent_control_eof"
                    break
            if time.monotonic() >= next_heartbeat:
                heartbeat.write("running")
                next_heartbeat = (
                    time.monotonic() + arguments.heartbeat_seconds
                )
    finally:
        selector.close()
        try:
            os.close(control_fd)
        except OSError:
            pass

    if return_code is None:
        heartbeat.write("terminating", reason=stop_reason)
        _signal_group(solver_pgid, signal.SIGTERM)
        term_deadline = time.monotonic() + arguments.term_grace_seconds
        if not _wait_group_exit(
            solver_pgid, term_deadline, heartbeat, "terminating"
        ):
            heartbeat.write("killing", reason=stop_reason)
            _signal_group(solver_pgid, signal.SIGKILL)
            kill_deadline = time.monotonic() + arguments.kill_grace_seconds
            _wait_group_exit(
                solver_pgid, kill_deadline, heartbeat, "killing"
            )
        try:
            return_code, observed_usage = _poll_solver(solver)
            if observed_usage is not None:
                solver_usage = observed_usage
        except ChildProcessError:
            return_code = solver.returncode
    else:
        # A solver may leave descendants behind after its leader exits.
        snapshot = _group_snapshot(solver_pgid)
        if snapshot["alive"]:
            _signal_group(solver_pgid, signal.SIGTERM)
            if not _wait_group_exit(
                solver_pgid,
                time.monotonic() + arguments.term_grace_seconds,
                heartbeat,
                "cleaning_descendants",
            ):
                _signal_group(solver_pgid, signal.SIGKILL)
                _wait_group_exit(
                    solver_pgid,
                    time.monotonic() + arguments.kill_grace_seconds,
                    heartbeat,
                    "cleaning_descendants",
                )

    _reap_adopted_children()
    final_code = int(return_code if return_code is not None else 1)
    final_snapshot = _group_snapshot(solver_pgid)
    heartbeat.write(
        "complete" if not final_snapshot["alive"] else "cleanup_failed",
        snapshot=final_snapshot,
        reason=stop_reason,
        solver_return_code=final_code,
        solver_cpu_user_seconds=(
            float(solver_usage.ru_utime)
            if solver_usage is not None
            else None
        ),
        solver_cpu_system_seconds=(
            float(solver_usage.ru_stime)
            if solver_usage is not None
            else None
        ),
    )
    return _normalized_exit_code(final_code)


if __name__ == "__main__":
    sys.exit(main())
