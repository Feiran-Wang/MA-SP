#!/usr/bin/env python3
"""Profile HGA/SP interaction from solver stdout logs.

New logs declare ``time_basis=process_cpu`` for SP timings. Historical logs
without that marker used wall time for SP, so the parser labels their basis
explicitly and never combines unlike clocks.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path


ITERATION = re.compile(
    r"It: (\d+)--G-Fit:\s*([0-9.]+)"
    r"\s+--R-Fit:\s*([0-9.]+)"
    r"\s+Curr Sol--([0-9.]+)"
    r"\s+IT NoImpro--(\d+)"
    r"\s+Time comsuming--([0-9.]+)"
)
SP_MONITOR = re.compile(
    r"\[SP MONITOR\]\s+"
    r"(?:scope=\S+\s+)?"
    r"(?:iteration=|Iteration\s+)(\d+)"
    r"\s+\|\s+RoutePool Size:\s*(\d+)"
)
SP_RESULT_LINE = re.compile(
    r"^\[SP Result\]\s+(?P<body>.*)$", re.MULTILINE
)
SP_FIELD = re.compile(r"([a-z_]+)=(\"[^\"]*\"|\S+)")


def optional(value: float | int | None) -> object:
    return "" if value is None else value


def numeric_field(fields: dict[str, str], name: str) -> float | None:
    raw = fields.get(name)
    if raw is None:
        return None
    if raw.endswith("s"):
        raw = raw[:-1]
    try:
        value = float(raw)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def parse_sp_results(text: str) -> list[dict[str, object]]:
    """Parse every SP result; incumbent/bound/gap fields are optional."""

    results: list[dict[str, object]] = []
    for line in SP_RESULT_LINE.finditer(text):
        fields = {
            key: value.strip('"')
            for key, value in SP_FIELD.findall(line.group("body"))
        }
        explicit_basis = fields.get("time_basis")
        results.append(
            {
                "status": fields.get("status", "UNKNOWN"),
                "solve": numeric_field(fields, "solve"),
                "total": numeric_field(fields, "total"),
                "objective": numeric_field(fields, "objective"),
                "bound": numeric_field(fields, "best_bound"),
                "gap": numeric_field(fields, "gap"),
                "time_basis": (
                    explicit_basis
                    if explicit_basis is not None
                    else "legacy_wall"
                ),
            }
        )
    return results


def read_rotated_text(path: Path) -> str:
    """Read bounded log segments from oldest to newest.

    ``RunLogger`` names retained segments ``hgasp.log.N``.  Reading them in
    descending suffix order reconstructs the surviving chronological window
    without assuming that the unbounded historical beginning still exists.
    """

    segments: list[Path] = []
    for candidate in path.parent.glob(path.name + ".*"):
        suffix = candidate.name.removeprefix(path.name + ".")
        if suffix.isdigit():
            segments.append(candidate)
    segments.sort(
        key=lambda candidate: int(
            candidate.name.removeprefix(path.name + ".")
        ),
        reverse=True,
    )
    if path.is_file():
        segments.append(path)
    return "".join(
        segment.read_text(encoding="utf-8", errors="replace")
        for segment in segments
    )


def canonical_run_logs(experiment: Path) -> list[Path]:
    """Select the actual/latest attempt once for every instance/seed cell."""

    selected: list[Path] = []
    for run_path in sorted(experiment.glob("runs/*/seed_*/run.json")):
        run_dir = run_path.parent
        candidate: Path | None = None
        try:
            record = json.loads(run_path.read_text(encoding="utf-8"))
            files = record.get("files", {})
            configured = files.get("hgasp")
            if isinstance(configured, str):
                configured_path = Path(configured)
                if configured_path.is_file():
                    candidate = configured_path
        except (OSError, json.JSONDecodeError, TypeError):
            candidate = None

        if candidate is None:
            attempts = sorted(
                run_dir.glob("logs/attempt_*/hgasp.log"),
                reverse=True,
            )
            if not attempts:
                attempts = sorted(
                    run_dir.glob("logs/attempt_*/stdout.log"),
                    reverse=True,
                )
            candidate = attempts[0] if attempts else None
        if candidate is not None:
            selected.append(candidate)
    return selected


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    rows: list[dict[str, object]] = []
    for log in canonical_run_logs(args.experiment):
        text = read_rotated_text(log)
        points = [
            {
                "iteration": int(match.group(1)),
                "global": float(match.group(2)),
                "restart": float(match.group(3)),
                "current": float(match.group(4)),
                "no_improve": int(match.group(5)),
                "time": float(match.group(6)),
            }
            for match in ITERATION.finditer(text)
        ]
        monitors = [
            (int(match.group(1)), int(match.group(2)))
            for match in SP_MONITOR.finditer(text)
        ]
        results = parse_sp_results(text)
        candidates = [point["global"] for point in points]
        candidates.extend(
            result["objective"]
            for result in results
            if result["objective"] is not None
        )
        incumbent = min(candidates) if candidates else None
        best_points = [
            point
            for point in points
            if incumbent is not None and point["global"] == incumbent
        ]
        first_best = best_points[0] if best_points else None
        last = points[-1] if points else None
        cpu_sp_total = sum(
            result["total"]
            for result in results
            if result["time_basis"] == "process_cpu"
            and result["total"] is not None
        )
        legacy_wall_sp_total = sum(
            result["total"]
            for result in results
            if result["time_basis"] == "legacy_wall"
            and result["total"] is not None
        )
        sp_time_bases = {
            result["time_basis"] for result in results
        }
        sp_time_basis = (
            next(iter(sp_time_bases))
            if len(sp_time_bases) == 1
            else ("none" if not sp_time_bases else "mixed")
        )
        comparable_sp_total = (
            sum(
                result["total"]
                for result in results
                if result["total"] is not None
            )
            if len(sp_time_bases) == 1
            else None
        )

        first_monitor = monitors[0] if monitors else None
        first_result = results[0] if results else None
        pre_sp = None
        if first_monitor is not None:
            preceding = [
                point
                for point in points
                if point["iteration"] <= first_monitor[0]
            ]
            if preceding:
                pre_sp = preceding[-1]["global"]

        run_dir = log.parents[2]
        instance = run_dir.parent.name
        seed = int(run_dir.name.removeprefix("seed_"))
        elapsed_cpu = last["time"] if last else None
        rows.append(
            {
                "instance": instance,
                "seed": seed,
                "last_iteration": optional(
                    int(last["iteration"]) if last else None
                ),
                "elapsed_cpu_seconds": optional(elapsed_cpu),
                "initial_incumbent": optional(
                    points[0]["global"] if points else None
                ),
                "current_incumbent": optional(incumbent),
                "absolute_improvement": optional(
                    points[0]["global"] - incumbent
                    if points and incumbent is not None
                    else None
                ),
                "first_best_iteration": optional(
                    int(first_best["iteration"]) if first_best else None
                ),
                "first_best_cpu_seconds": optional(
                    first_best["time"] if first_best else None
                ),
                "last_no_improve_iterations": optional(
                    int(last["no_improve"]) if last else None
                ),
                "restart_count": text.count("Executing Global Restart"),
                "sp_calls": len(results),
                "sp_results_with_objective": sum(
                    result["objective"] is not None for result in results
                ),
                "sp_results_with_bound": sum(
                    result["bound"] is not None for result in results
                ),
                "sp_results_with_gap": sum(
                    result["gap"] is not None for result in results
                ),
                "sp_successes": text.count("[SP Success]"),
                "sp_statuses": ";".join(
                    result["status"] for result in results
                ),
                "sp_time_basis": sp_time_basis,
                "sp_total_seconds": optional(comparable_sp_total),
                "sp_cpu_total_seconds": cpu_sp_total,
                "sp_legacy_wall_total_seconds": legacy_wall_sp_total,
                "raw_iterations_per_cpu_second": optional(
                    last["iteration"] / elapsed_cpu
                    if last is not None and elapsed_cpu is not None
                    and elapsed_cpu > 0
                    else None
                ),
                "first_sp_iteration": optional(
                    first_monitor[0] if first_monitor else None
                ),
                "first_sp_pool_size": optional(
                    first_monitor[1] if first_monitor else None
                ),
                "pre_sp_incumbent": optional(pre_sp),
                "first_sp_objective": optional(
                    first_result["objective"] if first_result else None
                ),
                "first_sp_delta": optional(
                    first_result["objective"] - pre_sp
                    if first_result is not None
                    and first_result["objective"] is not None
                    and pre_sp is not None
                    else None
                ),
                "first_sp_solve_seconds": optional(
                    first_result["solve"] if first_result else None
                ),
                "first_sp_gap": optional(
                    first_result["gap"] if first_result else None
                ),
                "first_sp_time_basis": (
                    first_result["time_basis"] if first_result else ""
                ),
                "source_log": str(log),
            }
        )

    if not rows:
        raise SystemExit(
            f"no run logs found below {args.experiment}"
        )
    fieldnames = list(rows[0])
    stream = None
    try:
        if args.output is None:
            stream = __import__("sys").stdout
        else:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            stream = args.output.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    finally:
        if args.output is not None and stream is not None:
            stream.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
