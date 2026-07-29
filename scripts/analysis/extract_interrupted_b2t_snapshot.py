#!/usr/bin/env python3
"""Extract provisional incumbents from an interrupted B2T comparison run."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path


ITERATION = re.compile(r"It: (\d+)--G-Fit:\s*([0-9.]+)")
SP_RESULT = re.compile(
    r"\[SP Result\] status=([A-Z_]+).*?objective=([0-9.]+)"
)
FAILURE_MARKERS = (
    "GUROBI_ERROR",
    "INVALID_SOLUTION",
    "[SP Result] status=ERROR",
)


def canonical_log(run_dir: Path) -> Path:
    run_path = run_dir / "run.json"
    if run_path.is_file():
        try:
            record = json.loads(run_path.read_text(encoding="utf-8"))
            configured = record.get("files", {}).get("hgasp")
            if isinstance(configured, str) and Path(configured).is_file():
                return Path(configured)
        except (OSError, json.JSONDecodeError, TypeError):
            pass
    attempts = sorted(
        run_dir.glob("logs/attempt_*/hgasp.log"),
        reverse=True,
    )
    if not attempts:
        attempts = sorted(
            run_dir.glob("logs/attempt_*/stdout.log"),
            reverse=True,
        )
    if not attempts:
        raise FileNotFoundError(f"no solver log below {run_dir}")
    return attempts[0]


def read_rotated_text(path: Path) -> str:
    segments = []
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
    segments.append(path)
    return "".join(
        segment.read_text(encoding="utf-8", errors="replace")
        for segment in segments
        if segment.is_file()
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("experiment", type=Path)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    with args.baseline.open(newline="", encoding="utf-8") as stream:
        references = {
            row["instance_id"]: float(row["b2t_objective"])
            for row in csv.DictReader(stream)
            if row["instance_id"].startswith("C")
        }

    rows: list[dict[str, object]] = []
    for instance, b2t in sorted(references.items()):
        for seed in (0, 1):
            run_dir = (
                args.experiment
                / "runs"
                / instance
                / f"seed_{seed}"
            )
            log = canonical_log(run_dir)
            text = read_rotated_text(log)
            iterations = [
                (int(iteration), float(value))
                for iteration, value in ITERATION.findall(text)
            ]
            sp_results = [
                (status, float(value))
                for status, value in SP_RESULT.findall(text)
            ]
            candidates = [value for _, value in iterations]
            candidates.extend(value for _, value in sp_results)
            incumbent = min(candidates) if candidates else None
            rows.append(
                {
                    "instance": instance,
                    "seed": seed,
                    "run_status": "interrupted",
                    "last_logged_iteration": max(
                        (iteration for iteration, _ in iterations),
                        default="",
                    ),
                    "current_incumbent": incumbent,
                    "b2t_bks": b2t,
                    "gap_to_b2t_percent": (
                        100.0 * (incumbent - b2t) / b2t
                        if incumbent is not None
                        else None
                    ),
                    "sp_result_count": len(sp_results),
                    "sp_success_count": text.count("[SP Success]"),
                    "sp_statuses": ";".join(
                        status for status, _ in sp_results
                    ),
                    "gurobi_error_observed": any(
                        marker in text for marker in FAILURE_MARKERS
                    ),
                    "independently_validated": False,
                    "source_log": str(log),
                }
            )

    fieldnames = list(rows[0])
    if args.output is None:
        stream = __import__("sys").stdout
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    else:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
