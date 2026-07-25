#!/usr/bin/env python3
"""Summarize historical Set S text results into a deterministic CSV file."""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS = REPO_ROOT / "results" / "set S"
RESULT_NAME = re.compile(r"^(?P<instance>.+)_(?P<seed>\d+)\.txt$")
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
OBJECTIVE = re.compile(rf"The objective is:\s*({NUMBER})")
RUNNING_TIME = re.compile(rf"The running time is:\s*({NUMBER})")


def require_project_venv() -> None:
    expected = (REPO_ROOT / ".venv").resolve()
    if Path(sys.prefix).resolve() != expected:
        raise SystemExit(
            "Run this script with the repository virtual environment: "
            ".venv/bin/python scripts/analysis/summarize_results.py"
        )


def instance_sort_key(instance: str) -> tuple[int, int, str]:
    parts = instance.split("-")
    try:
        return int(parts[0]), int(parts[1]), "-".join(parts[2:])
    except (ValueError, IndexError):
        return 0, 0, instance


def collect_results(
    results_directory: Path,
) -> dict[str, dict[int, tuple[float, float | None]]]:
    collected: dict[str, dict[int, tuple[float, float | None]]] = {}
    for path in sorted(results_directory.glob("*.txt")):
        match = RESULT_NAME.fullmatch(path.name)
        if match is None:
            continue
        content = path.read_text(encoding="utf-8", errors="replace")
        objective_match = OBJECTIVE.search(content)
        if objective_match is None:
            continue
        time_match = RUNNING_TIME.search(content)
        collected.setdefault(match.group("instance"), {})[
            int(match.group("seed"))
        ] = (
            float(objective_match.group(1)),
            float(time_match.group(1)) if time_match else None,
        )
    return collected


def rows(
    collected: dict[str, dict[int, tuple[float, float | None]]],
    expected_runs: int,
) -> Iterable[list[str]]:
    for instance in sorted(collected, key=instance_sort_key):
        runs = collected[instance]
        objectives = [value[0] for value in runs.values()]
        times = [value[1] for value in runs.values() if value[1] is not None]
        yield [
            instance,
            f"{min(objectives):.2f}",
            f"{sum(objectives) / len(objectives):.2f}",
            f"{sum(times) / len(times):.2f}" if times else "",
            f"{len(runs)}/{expected_runs}",
        ]


def main() -> int:
    require_project_venv()
    parser = argparse.ArgumentParser(
        description="Summarize historical Set S result text files."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_RESULTS,
        help="result directory (default: results/set S)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV destination (default: <input>/summary.csv)",
    )
    parser.add_argument("--expected-runs", type=int, default=20)
    arguments = parser.parse_args()

    input_directory = arguments.input.resolve()
    if not input_directory.is_dir():
        parser.error(f"result directory does not exist: {input_directory}")
    if arguments.expected_runs <= 0:
        parser.error("--expected-runs must be positive")
    output_path = (
        arguments.output.resolve()
        if arguments.output
        else input_directory / "summary.csv"
    )

    collected = collect_results(input_directory)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["Instance", "Best Obj", "Avg Obj", "Avg Time (s)", "Progress"]
        )
        writer.writerows(rows(collected, arguments.expected_runs))

    print(
        f"Wrote {len(collected)} instance summaries to {output_path}",
        file=sys.stdout,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
