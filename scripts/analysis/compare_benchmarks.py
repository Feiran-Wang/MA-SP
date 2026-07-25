#!/usr/bin/env python3
"""Compare experiment runs with the Set S reference solutions."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean
from typing import Iterable, TextIO


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EXPERIMENTS = REPO_ROOT / "artifacts" / "experiments"
DEFAULT_BENCHMARKS = REPO_ROOT / "solutions" / "set S"
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
OBJECTIVE_PATTERNS = (
    re.compile(rf"The objective is\s*:?\s*({NUMBER})", re.IGNORECASE),
    re.compile(rf"The actual cost is\s*:?\s*({NUMBER})", re.IGNORECASE),
)
RUNNING_TIME_PATTERN = re.compile(
    rf"The running time is\s*:?\s*({NUMBER})", re.IGNORECASE
)
CSV_HEADER = (
    "instance",
    "successful_runs",
    "discovered_runs",
    "best_objective",
    "average_objective",
    "benchmark",
    "best_gap_percent",
    "average_gap_percent",
    "avg_cpu_time_seconds",
    "avg_wall_time_seconds",
)


@dataclass(frozen=True)
class RunObservation:
    """One usable solver run."""

    instance: str
    objective: float
    solver_time_seconds: float | None
    wall_time_seconds: float | None


@dataclass(frozen=True)
class ComparisonRow:
    """Aggregate statistics for one instance."""

    instance: str
    successful_runs: int
    discovered_runs: int
    best_objective: float | None
    average_objective: float | None
    benchmark: float | None
    best_gap_percent: float | None
    average_gap_percent: float | None
    avg_cpu_time_seconds: float | None
    avg_wall_time_seconds: float | None


def require_project_venv() -> None:
    """Reject accidental execution from a base Python environment."""

    expected = (REPO_ROOT / ".venv").resolve()
    if Path(sys.prefix).resolve() != expected:
        raise SystemExit(
            "Run this script with the repository virtual environment: "
            ".venv/bin/python scripts/analysis/compare_benchmarks.py"
        )


def canonical_instance_name(value: str) -> str:
    """Match ``100-5-1c`` with the reference file ``10051c.txt``."""

    return re.sub(r"[^a-z0-9]", "", value.casefold())


def instance_sort_key(instance: str) -> tuple[int, int, str]:
    parts = instance.split("-")
    try:
        return int(parts[0]), int(parts[1]), "-".join(parts[2:])
    except (ValueError, IndexError):
        return 0, 0, instance


def finite_number(value: object, *, nonnegative: bool = False) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    if not math.isfinite(result) or (nonnegative and result < 0.0):
        return None
    return result


def parse_solution(path: Path) -> tuple[float | None, float | None]:
    try:
        content = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None, None
    objective = None
    for pattern in OBJECTIVE_PATTERNS:
        match = pattern.search(content)
        if match is not None:
            objective = finite_number(float(match.group(1)))
            if objective is not None:
                break
    time_match = RUNNING_TIME_PATTERN.search(content)
    solver_time = (
        finite_number(float(time_match.group(1)), nonnegative=True)
        if time_match is not None
        else None
    )
    return objective, solver_time


def parse_objective(path: Path) -> float | None:
    """Read only the objective, as required for reference solutions."""

    return parse_solution(path)[0]


def load_json_object(path: Path) -> dict[str, object] | None:
    def reject_non_finite(value: str) -> None:
        raise ValueError(f"non-finite JSON number {value}")

    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream, parse_constant=reject_non_finite)
    except (OSError, json.JSONDecodeError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def discover_runs(
    experiments_directory: Path,
) -> tuple[dict[str, list[RunObservation]], dict[str, int], list[str]]:
    """Collect valid successful runs and counts for every discovered instance."""

    observations: dict[str, list[RunObservation]] = {}
    discovered: dict[str, int] = {}
    warnings: list[str] = []

    for metadata_path in sorted(experiments_directory.rglob("run.json")):
        metadata = load_json_object(metadata_path)
        if metadata is None:
            warnings.append(f"{metadata_path}: invalid run.json")
            continue

        instance = metadata.get("instance_id")
        if not isinstance(instance, str) or not instance.strip():
            warnings.append(f"{metadata_path}: missing instance_id")
            continue
        instance = instance.strip()
        discovered[instance] = discovered.get(instance, 0) + 1

        if metadata.get("status") != "success":
            continue

        solution_path = metadata_path.with_name("solution.txt")
        objective, solver_time = parse_solution(solution_path)
        if objective is None:
            warnings.append(
                f"{metadata_path}: successful run has no finite objective"
            )
            continue

        wall_time = finite_number(
            metadata.get("duration_seconds"), nonnegative=True
        )
        observations.setdefault(instance, []).append(
            RunObservation(instance, objective, solver_time, wall_time)
        )

    return observations, discovered, warnings


def load_benchmarks(
    benchmark_directory: Path,
) -> tuple[dict[str, float], list[str]]:
    benchmarks: dict[str, float] = {}
    warnings: list[str] = []
    for path in sorted(benchmark_directory.glob("*.txt")):
        key = canonical_instance_name(path.stem)
        objective = parse_objective(path)
        if not key or objective is None or objective <= 0.0:
            warnings.append(f"{path}: missing positive finite benchmark")
            continue
        if key in benchmarks and not math.isclose(
            benchmarks[key], objective, rel_tol=0.0, abs_tol=1e-9
        ):
            warnings.append(f"{path}: conflicting duplicate benchmark")
            continue
        benchmarks[key] = objective
    return benchmarks, warnings


def comparison_rows(
    observations: dict[str, list[RunObservation]],
    discovered: dict[str, int],
    benchmarks: dict[str, float],
) -> list[ComparisonRow]:
    rows: list[ComparisonRow] = []
    instances = set(discovered) | set(observations)
    for instance in sorted(instances, key=instance_sort_key):
        runs = observations.get(instance, [])
        objectives = [run.objective for run in runs]
        solver_times = [
            run.solver_time_seconds
            for run in runs
            if run.solver_time_seconds is not None
        ]
        wall_times = [
            run.wall_time_seconds
            for run in runs
            if run.wall_time_seconds is not None
        ]
        best = min(objectives) if objectives else None
        average = fmean(objectives) if objectives else None
        benchmark = benchmarks.get(canonical_instance_name(instance))
        if benchmark is None or best is None or average is None:
            best_gap = None
            average_gap = None
        else:
            best_gap = 100.0 * (best - benchmark) / benchmark
            average_gap = 100.0 * (average - benchmark) / benchmark
        rows.append(
            ComparisonRow(
                instance=instance,
                successful_runs=len(runs),
                discovered_runs=discovered.get(instance, len(runs)),
                best_objective=best,
                average_objective=average,
                benchmark=benchmark,
                best_gap_percent=best_gap,
                average_gap_percent=average_gap,
                avg_cpu_time_seconds=(
                    fmean(solver_times) if solver_times else None
                ),
                avg_wall_time_seconds=fmean(wall_times) if wall_times else None,
            )
        )
    return rows


def format_number(value: float | None) -> str:
    return "" if value is None else f"{value:.6f}"


def write_csv(rows: Iterable[ComparisonRow], stream: TextIO) -> None:
    writer = csv.writer(stream)
    writer.writerow(CSV_HEADER)
    for row in rows:
        writer.writerow(
            (
                row.instance,
                row.successful_runs,
                row.discovered_runs,
                format_number(row.best_objective),
                format_number(row.average_objective),
                format_number(row.benchmark),
                format_number(row.best_gap_percent),
                format_number(row.average_gap_percent),
                format_number(row.avg_cpu_time_seconds),
                format_number(row.avg_wall_time_seconds),
            )
        )


def main(argv: list[str] | None = None) -> int:
    require_project_venv()
    parser = argparse.ArgumentParser(
        description=(
            "Compare successful HGA-SP experiment runs with Set S benchmarks."
        )
    )
    parser.add_argument(
        "--experiments",
        type=Path,
        default=DEFAULT_EXPERIMENTS,
        help="directory recursively containing run.json files",
    )
    parser.add_argument(
        "--benchmarks",
        type=Path,
        default=DEFAULT_BENCHMARKS,
        help="directory containing Set S benchmark solution files",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV destination; stdout is used when omitted",
    )
    arguments = parser.parse_args(argv)

    experiments_directory = arguments.experiments.resolve()
    benchmark_directory = arguments.benchmarks.resolve()
    if not experiments_directory.is_dir():
        parser.error(
            f"experiment directory does not exist: {experiments_directory}"
        )
    if not benchmark_directory.is_dir():
        parser.error(
            f"benchmark directory does not exist: {benchmark_directory}"
        )

    observations, discovered, run_warnings = discover_runs(
        experiments_directory
    )
    benchmarks, benchmark_warnings = load_benchmarks(benchmark_directory)
    rows = comparison_rows(observations, discovered, benchmarks)
    for warning in (*run_warnings, *benchmark_warnings):
        print(f"warning: {warning}", file=sys.stderr)

    if arguments.output is None:
        write_csv(rows, sys.stdout)
    else:
        output_path = arguments.output.resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", newline="", encoding="utf-8") as stream:
            write_csv(rows, stream)
        print(
            f"Wrote {len(rows)} instance comparisons to {output_path}",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
