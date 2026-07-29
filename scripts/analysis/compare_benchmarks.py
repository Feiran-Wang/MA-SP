#!/usr/bin/env python3
"""Compare experiment runs with typed CLRP/CLRP-TW reference results."""

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
TIME_BASIS_PATTERN = re.compile(
    r"The time basis is\s*:?\s*([A-Za-z0-9_.-]+)", re.IGNORECASE
)
TIME_SCOPE_PATTERN = re.compile(
    r"The time scope is\s*:?\s*([A-Za-z0-9_.-]+)", re.IGNORECASE
)
SEARCH_TIME_BASIS = "process_cpu"
SEARCH_TIME_SCOPE = "search_after_instance_loading"
CSV_HEADER = (
    "instance",
    "successful_runs",
    "discovered_runs",
    "best_objective",
    "average_objective",
    "baseline_name",
    "benchmark",
    "reference_status",
    "lower_bound",
    "best_gap_percent",
    "average_gap_percent",
    "best_gap_to_lower_bound_percent",
    "average_gap_to_lower_bound_percent",
    "avg_search_cpu_time_seconds",
    "search_time_basis",
    "search_time_scope",
    "run_hardware_id",
    "avg_operational_child_cpu_time_seconds",
    "avg_wall_time_seconds",
    "reference_time_seconds",
    "reference_time_basis",
    "reference_time_scope",
    "reference_hardware_id",
    "time_comparison_status",
    "average_time_gap_percent",
    "time_speedup",
)


@dataclass(frozen=True)
class ParsedSolution:
    """Human-readable solution fields with explicit scientific time semantics."""

    objective: float | None
    reported_time_seconds: float | None
    search_cpu_time_seconds: float | None
    time_basis: str | None
    time_scope: str | None


@dataclass(frozen=True)
class RunObservation:
    """One usable solver run."""

    instance: str
    objective: float
    search_cpu_time_seconds: float | None
    search_time_scope: str | None
    operational_child_cpu_time_seconds: float | None
    wall_time_seconds: float | None
    hardware_id: str | None


@dataclass(frozen=True)
class ReferenceResult:
    """A typed literature reference; unknown is distinct from numeric zero."""

    status: str
    value: float | None
    name: str | None = None
    lower_bound: float | None = None
    source: str | None = None
    distance_policy: str | None = None
    time_seconds: float | None = None
    time_basis: str | None = None
    time_scope: str | None = None
    hardware_id: str | None = None


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
    avg_search_cpu_time_seconds: float | None
    search_time_basis: str | None
    search_time_scope: str | None
    run_hardware_id: str | None
    avg_operational_child_cpu_time_seconds: float | None
    avg_wall_time_seconds: float | None
    baseline_name: str | None = None
    reference_status: str | None = None
    lower_bound: float | None = None
    best_gap_to_lower_bound_percent: float | None = None
    average_gap_to_lower_bound_percent: float | None = None
    reference_time_seconds: float | None = None
    reference_time_basis: str | None = None
    reference_time_scope: str | None = None
    reference_hardware_id: str | None = None
    time_comparison_status: str | None = None
    average_time_gap_percent: float | None = None
    time_speedup: float | None = None


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


def parse_solution_record(path: Path) -> ParsedSolution:
    try:
        content = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ParsedSolution(None, None, None, None, None)
    objective = None
    for pattern in OBJECTIVE_PATTERNS:
        match = pattern.search(content)
        if match is not None:
            objective = finite_number(float(match.group(1)))
            if objective is not None:
                break
    time_match = RUNNING_TIME_PATTERN.search(content)
    reported_time = (
        finite_number(float(time_match.group(1)), nonnegative=True)
        if time_match is not None
        else None
    )
    basis_match = TIME_BASIS_PATTERN.search(content)
    scope_match = TIME_SCOPE_PATTERN.search(content)
    time_basis = (
        basis_match.group(1).casefold() if basis_match is not None else None
    )
    time_scope = (
        scope_match.group(1).casefold() if scope_match is not None else None
    )
    search_cpu_time = (
        reported_time
        if time_basis == SEARCH_TIME_BASIS
        and time_scope == SEARCH_TIME_SCOPE
        else None
    )
    return ParsedSolution(
        objective,
        reported_time,
        search_cpu_time,
        time_basis,
        time_scope,
    )


def parse_solution(path: Path) -> tuple[float | None, float | None]:
    """Backward-compatible objective/search-CPU tuple."""

    parsed = parse_solution_record(path)
    return parsed.objective, parsed.search_cpu_time_seconds


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
        solution = parse_solution_record(solution_path)
        if solution.objective is None:
            warnings.append(
                f"{metadata_path}: successful run has no finite objective"
            )
            continue

        if (
            solution.reported_time_seconds is not None
            and solution.search_cpu_time_seconds is None
        ):
            warnings.append(
                f"{solution_path}: running time omitted from scientific "
                "aggregation because process_cpu/search_after_instance_loading "
                "was not declared"
            )

        # wait4(pid) covers the complete solver child, including startup,
        # parsing, and shutdown. Keep it as an operational diagnostic; it must
        # never replace the search-scoped CPU value emitted by solution.txt.
        operational_child_cpu_time = finite_number(
            metadata.get("cpu_time_seconds"), nonnegative=True
        )
        wall_time = finite_number(
            metadata.get("wall_duration_seconds"), nonnegative=True
        )
        if wall_time is None and "cpu_time_seconds" not in metadata:
            wall_time = finite_number(
                metadata.get("duration_seconds"), nonnegative=True
            )
        provenance = metadata.get("provenance")
        platform = (
            provenance.get("platform")
            if isinstance(provenance, dict)
            else None
        )
        hardware_id = (
            platform.get("hardware_id")
            if isinstance(platform, dict)
            and isinstance(platform.get("hardware_id"), str)
            and platform.get("hardware_id").strip()
            else None
        )
        observations.setdefault(instance, []).append(
            RunObservation(
                instance,
                solution.objective,
                solution.search_cpu_time_seconds,
                solution.time_scope
                if solution.search_cpu_time_seconds is not None
                else None,
                operational_child_cpu_time,
                wall_time,
                hardware_id,
            )
        )

    return observations, discovered, warnings


def load_benchmarks(
    benchmark_directory: Path,
) -> tuple[dict[str, float | ReferenceResult], list[str]]:
    benchmarks: dict[str, float | ReferenceResult] = {}
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
    reference_csv = benchmark_directory / "reference_results.csv"
    if reference_csv.is_file():
        untyped_reference_times = 0
        try:
            with reference_csv.open(newline="", encoding="utf-8") as stream:
                reader = csv.DictReader(stream)
                required = {"instance_id", "status"}
                if reader.fieldnames is None or not required.issubset(
                    reader.fieldnames
                ):
                    warnings.append(
                        f"{reference_csv}: requires instance_id,status columns"
                    )
                else:
                    for line_number, row in enumerate(reader, 2):
                        instance = (row.get("instance_id") or "").strip()
                        status = (row.get("status") or "").strip().casefold()
                        if not instance or status not in (
                            "unknown",
                            "bks",
                            "optimal",
                        ):
                            warnings.append(
                                f"{reference_csv}:{line_number}: invalid "
                                "instance_id or status"
                            )
                            continue
                        value = _optional_positive_csv_number(
                            row.get("value"), reference_csv, line_number,
                            "value", warnings
                        )
                        lower_bound = _optional_positive_csv_number(
                            row.get("lower_bound"), reference_csv, line_number,
                            "lower_bound", warnings
                        )
                        time_seconds = _optional_positive_csv_number(
                            row.get("time_seconds"), reference_csv, line_number,
                            "time_seconds", warnings
                        )
                        if status != "unknown" and value is None:
                            warnings.append(
                                f"{reference_csv}:{line_number}: "
                                f"{status} requires a positive value"
                            )
                            continue
                        if status == "unknown" and value is not None:
                            warnings.append(
                                f"{reference_csv}:{line_number}: "
                                "unknown reference must not have a value"
                            )
                            continue
                        if instance in benchmarks:
                            warnings.append(
                                f"{reference_csv}:{line_number}: duplicate "
                                f"exact instance ID {instance}"
                            )
                            continue
                        benchmarks[instance] = ReferenceResult(
                            status=status,
                            value=value,
                            name=(row.get("baseline_name") or "").strip()
                            or None,
                            lower_bound=lower_bound,
                            source=(row.get("source") or "").strip() or None,
                            distance_policy=(
                                (row.get("distance_policy") or "").strip()
                                or None
                            ),
                            time_seconds=time_seconds,
                            time_basis=(
                                (row.get("time_basis") or "").strip().casefold()
                                or None
                            ),
                            time_scope=(
                                (row.get("time_scope") or "").strip().casefold()
                                or None
                            ),
                            hardware_id=(
                                (row.get("hardware_id") or "").strip() or None
                            ),
                        )
                        reference = benchmarks[instance]
                        if (
                            isinstance(reference, ReferenceResult)
                            and reference.time_seconds is not None
                            and (
                                reference.time_basis is None
                                or reference.time_scope is None
                                or reference.hardware_id is None
                            )
                        ):
                            untyped_reference_times += 1
            if untyped_reference_times:
                warnings.append(
                    f"{reference_csv}: {untyped_reference_times} "
                    "time_seconds value(s) are descriptive only; "
                    "time_basis, time_scope, and hardware_id are required "
                    "for a time-gap comparison"
                )
        except OSError as error:
            warnings.append(f"{reference_csv}: {error}")
    return benchmarks, warnings


def _optional_positive_csv_number(
    raw: str | None,
    path: Path,
    line_number: int,
    field: str,
    warnings: list[str],
) -> float | None:
    if raw is None or not raw.strip():
        return None
    try:
        value = float(raw)
    except ValueError:
        warnings.append(f"{path}:{line_number}: {field} is not numeric")
        return None
    if not math.isfinite(value) or value <= 0:
        warnings.append(
            f"{path}:{line_number}: {field} must be positive and finite"
        )
        return None
    return value


def comparison_rows(
    observations: dict[str, list[RunObservation]],
    discovered: dict[str, int],
    benchmarks: dict[str, float | ReferenceResult],
) -> list[ComparisonRow]:
    rows: list[ComparisonRow] = []
    instances = set(discovered) | set(observations)
    for instance in sorted(instances, key=instance_sort_key):
        runs = observations.get(instance, [])
        objectives = [run.objective for run in runs]
        search_cpu_times = [
            run.search_cpu_time_seconds
            for run in runs
            if run.search_cpu_time_seconds is not None
        ]
        operational_child_cpu_times = [
            run.operational_child_cpu_time_seconds
            for run in runs
            if run.operational_child_cpu_time_seconds is not None
        ]
        wall_times = [
            run.wall_time_seconds
            for run in runs
            if run.wall_time_seconds is not None
        ]
        best = min(objectives) if objectives else None
        average = fmean(objectives) if objectives else None
        raw_reference = benchmarks.get(instance)
        if raw_reference is None:
            raw_reference = benchmarks.get(canonical_instance_name(instance))
        if isinstance(raw_reference, ReferenceResult):
            benchmark = raw_reference.value
            baseline_name = raw_reference.name
            reference_status = raw_reference.status
            lower_bound = raw_reference.lower_bound
            reference_time = raw_reference.time_seconds
            reference_time_basis = raw_reference.time_basis
            reference_time_scope = raw_reference.time_scope
            reference_hardware_id = raw_reference.hardware_id
        else:
            benchmark = raw_reference
            baseline_name = None
            reference_status = "legacy" if raw_reference is not None else None
            lower_bound = None
            reference_time = None
            reference_time_basis = None
            reference_time_scope = None
            reference_hardware_id = None
        if benchmark is None or best is None or average is None:
            best_gap = None
            average_gap = None
        else:
            best_gap = 100.0 * (best - benchmark) / benchmark
            average_gap = 100.0 * (average - benchmark) / benchmark
        if lower_bound is None or best is None or average is None:
            best_lb_gap = None
            average_lb_gap = None
        else:
            best_lb_gap = 100.0 * (best - lower_bound) / lower_bound
            average_lb_gap = 100.0 * (average - lower_bound) / lower_bound
        search_scopes = {
            run.search_time_scope
            for run in runs
            if run.search_cpu_time_seconds is not None
        }
        search_scope = (
            next(iter(search_scopes)) if len(search_scopes) == 1 else None
        )
        run_hardware_ids = {
            run.hardware_id
            for run in runs
            if run.search_cpu_time_seconds is not None
        }
        run_hardware_id = (
            next(iter(run_hardware_ids))
            if len(run_hardware_ids) == 1
            and None not in run_hardware_ids
            else None
        )
        average_search_cpu_time = (
            fmean(search_cpu_times) if search_cpu_times else None
        )
        if reference_time is None:
            time_comparison_status = "no_reference_time"
        elif average_search_cpu_time is None:
            time_comparison_status = "no_search_cpu_time"
        elif average_search_cpu_time <= 0.0:
            time_comparison_status = "nonpositive_search_cpu_time"
        elif search_scope is None:
            time_comparison_status = "mixed_search_time_scope"
        elif (
            reference_time_basis is None
            or reference_time_scope is None
            or reference_hardware_id is None
        ):
            time_comparison_status = "reference_time_not_comparable"
        elif run_hardware_id is None:
            time_comparison_status = "run_hardware_not_declared"
        elif reference_time_basis != SEARCH_TIME_BASIS:
            time_comparison_status = "time_basis_mismatch"
        elif reference_time_scope != search_scope:
            time_comparison_status = "time_scope_mismatch"
        elif reference_hardware_id != run_hardware_id:
            time_comparison_status = "hardware_mismatch"
        else:
            time_comparison_status = "compatible"

        if time_comparison_status != "compatible":
            average_time_gap = None
            time_speedup = None
        else:
            average_time_gap = (
                100.0
                * (average_search_cpu_time - reference_time)
                / reference_time
            )
            time_speedup = reference_time / average_search_cpu_time
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
                avg_search_cpu_time_seconds=average_search_cpu_time,
                search_time_basis=(
                    SEARCH_TIME_BASIS
                    if average_search_cpu_time is not None
                    else None
                ),
                search_time_scope=search_scope,
                run_hardware_id=run_hardware_id,
                avg_operational_child_cpu_time_seconds=(
                    fmean(operational_child_cpu_times)
                    if operational_child_cpu_times
                    else None
                ),
                avg_wall_time_seconds=fmean(wall_times) if wall_times else None,
                baseline_name=baseline_name,
                reference_status=reference_status,
                lower_bound=lower_bound,
                best_gap_to_lower_bound_percent=best_lb_gap,
                average_gap_to_lower_bound_percent=average_lb_gap,
                reference_time_seconds=reference_time,
                reference_time_basis=reference_time_basis,
                reference_time_scope=reference_time_scope,
                reference_hardware_id=reference_hardware_id,
                time_comparison_status=time_comparison_status,
                average_time_gap_percent=average_time_gap,
                time_speedup=time_speedup,
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
                row.baseline_name or "",
                format_number(row.benchmark),
                row.reference_status or "",
                format_number(row.lower_bound),
                format_number(row.best_gap_percent),
                format_number(row.average_gap_percent),
                format_number(row.best_gap_to_lower_bound_percent),
                format_number(row.average_gap_to_lower_bound_percent),
                format_number(row.avg_search_cpu_time_seconds),
                row.search_time_basis or "",
                row.search_time_scope or "",
                row.run_hardware_id or "",
                format_number(
                    row.avg_operational_child_cpu_time_seconds
                ),
                format_number(row.avg_wall_time_seconds),
                format_number(row.reference_time_seconds),
                row.reference_time_basis or "",
                row.reference_time_scope or "",
                row.reference_hardware_id or "",
                row.time_comparison_status or "",
                format_number(row.average_time_gap_percent),
                format_number(row.time_speedup),
            )
        )


def main(argv: list[str] | None = None) -> int:
    require_project_venv()
    parser = argparse.ArgumentParser(
        description=(
            "Compare successful HGA-SP experiment runs with typed CLRP or "
            "CLRP-TW references."
        )
    )
    parser.add_argument(
        "--experiments",
        type=Path,
        action="append",
        help="directory recursively containing run.json files",
    )
    parser.add_argument(
        "--benchmarks",
        type=Path,
        default=DEFAULT_BENCHMARKS,
        help=(
            "directory containing legacy benchmark TXT files and/or "
            "reference_results.csv"
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV destination; stdout is used when omitted",
    )
    arguments = parser.parse_args(argv)

    experiment_arguments = arguments.experiments or [DEFAULT_EXPERIMENTS]
    experiment_directories = [path.resolve() for path in experiment_arguments]
    benchmark_directory = arguments.benchmarks.resolve()
    for experiments_directory in experiment_directories:
        if not experiments_directory.is_dir():
            parser.error(
                "experiment directory does not exist: "
                f"{experiments_directory}"
            )
    if not benchmark_directory.is_dir():
        parser.error(
            f"benchmark directory does not exist: {benchmark_directory}"
        )

    observations: dict[str, list[RunObservation]] = {}
    discovered: dict[str, int] = {}
    run_warnings: list[str] = []
    for experiments_directory in experiment_directories:
        directory_observations, directory_discovered, directory_warnings = (
            discover_runs(experiments_directory)
        )
        for instance, runs in directory_observations.items():
            observations.setdefault(instance, []).extend(runs)
        for instance, count in directory_discovered.items():
            discovered[instance] = discovered.get(instance, 0) + count
        run_warnings.extend(directory_warnings)
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
