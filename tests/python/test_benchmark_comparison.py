from __future__ import annotations

import contextlib
import csv
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
ANALYSIS_DIR = REPO_ROOT / "scripts" / "analysis"
sys.path.insert(0, str(ANALYSIS_DIR))

import compare_benchmarks as comparison  # noqa: E402


def write_run(
    root: Path,
    instance: str,
    seed: int,
    *,
    status: str = "success",
    objective: str | None = "100",
    solver_time: str | None = None,
    duration: object = 2.0,
) -> None:
    run_directory = root / "runs" / instance / f"seed_{seed}"
    run_directory.mkdir(parents=True)
    (run_directory / "run.json").write_text(
        json.dumps(
            {
                "instance_id": instance,
                "status": status,
                "duration_seconds": duration,
            }
        ),
        encoding="utf-8",
    )
    if objective is not None:
        time_line = (
            f"The running time is:\n{solver_time}\n"
            if solver_time is not None
            else ""
        )
        (run_directory / "solution.txt").write_text(
            f"The objective is:\n{objective}\n{time_line}", encoding="utf-8"
        )


class CollectionTests(unittest.TestCase):
    def test_aggregate_gaps_time_and_invalid_runs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            experiments = root / "artifacts"
            benchmarks = root / "benchmarks"
            benchmarks.mkdir()
            (benchmarks / "10051c.txt").write_text(
                "The objective is:\n100\n", encoding="utf-8"
            )

            write_run(
                experiments,
                "100-5-1c",
                0,
                objective="100",
                solver_time="1",
                duration=2,
            )
            write_run(
                experiments,
                "100-5-1c",
                1,
                objective="110",
                solver_time="3",
                duration=4,
            )
            write_run(
                experiments, "100-5-1c", 2, status="failed", objective="90"
            )
            write_run(experiments, "100-5-1c", 3, objective=None)
            write_run(
                experiments, "100-5-1c", 4, objective="nan", duration=1
            )
            write_run(
                experiments, "100-5-2c", 0, status="failed", objective="90"
            )

            observations, discovered, warnings = comparison.discover_runs(
                experiments
            )
            reference, reference_warnings = comparison.load_benchmarks(
                benchmarks
            )
            rows = comparison.comparison_rows(
                observations, discovered, reference
            )

            self.assertEqual(len(rows), 2)
            row = rows[0]
            self.assertEqual(row.successful_runs, 2)
            self.assertEqual(row.discovered_runs, 5)
            self.assertEqual(row.best_objective, 100.0)
            self.assertEqual(row.average_objective, 105.0)
            self.assertEqual(row.benchmark, 100.0)
            self.assertAlmostEqual(row.best_gap_percent, 0.0)
            self.assertAlmostEqual(row.average_gap_percent, 5.0)
            self.assertAlmostEqual(row.avg_cpu_time_seconds, 2.0)
            self.assertAlmostEqual(row.avg_wall_time_seconds, 3.0)
            self.assertEqual(len(warnings), 2)
            self.assertEqual(reference_warnings, [])
            failed_row = rows[1]
            self.assertEqual(failed_row.successful_runs, 0)
            self.assertEqual(failed_row.discovered_runs, 1)
            self.assertIsNone(failed_row.best_objective)

    def test_missing_benchmark_and_nonfinite_duration(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_run(root, "tiny", 0, objective="7", duration=None)
            observations, discovered, warnings = comparison.discover_runs(root)
            rows = comparison.comparison_rows(
                observations, discovered, benchmarks={}
            )
            self.assertEqual(warnings, [])
            self.assertEqual(rows[0].benchmark, None)
            self.assertEqual(rows[0].best_gap_percent, None)
            self.assertEqual(rows[0].avg_cpu_time_seconds, None)
            self.assertEqual(rows[0].avg_wall_time_seconds, None)

            metadata = next(root.rglob("run.json"))
            value = json.loads(metadata.read_text(encoding="utf-8"))
            value["duration_seconds"] = math.inf
            metadata.write_text(
                json.dumps(value, allow_nan=True), encoding="utf-8"
            )
            observations, _, warnings = comparison.discover_runs(root)
            self.assertEqual(observations, {})
            self.assertEqual(len(warnings), 1)


class CommandLineTests(unittest.TestCase):
    def test_cli_writes_deterministic_csv(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            experiments = root / "experiments"
            benchmarks = root / "solutions"
            benchmarks.mkdir()
            (benchmarks / "10051c.txt").write_text(
                "The actual cost is 100\n", encoding="utf-8"
            )
            write_run(
                experiments,
                "100-5-1c",
                0,
                objective="101",
                solver_time="2.5",
                duration=3,
            )
            output = root / "report" / "comparison.csv"

            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                status = comparison.main(
                    [
                        "--experiments",
                        str(experiments),
                        "--benchmarks",
                        str(benchmarks),
                        "--output",
                        str(output),
                    ]
                )
            self.assertEqual(status, 0, stderr.getvalue())
            with output.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["instance"], "100-5-1c")
            self.assertEqual(rows[0]["successful_runs"], "1")
            self.assertEqual(rows[0]["best_gap_percent"], "1.000000")
            self.assertEqual(rows[0]["avg_cpu_time_seconds"], "2.500000")
            self.assertEqual(rows[0]["avg_wall_time_seconds"], "3.000000")


if __name__ == "__main__":
    unittest.main()
