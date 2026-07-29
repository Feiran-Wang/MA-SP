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
import profile_hgasp_logs as profiler  # noqa: E402


def write_run(
    root: Path,
    instance: str,
    seed: int,
    *,
    status: str = "success",
    objective: str | None = "100",
    solver_time: str | None = None,
    duration: object = 2.0,
    time_basis: str | None = comparison.SEARCH_TIME_BASIS,
    time_scope: str | None = comparison.SEARCH_TIME_SCOPE,
    hardware_id: str | None = "test-cpu",
    operational_child_cpu: float | None = None,
) -> None:
    run_directory = root / "runs" / instance / f"seed_{seed}"
    run_directory.mkdir(parents=True)
    metadata: dict[str, object] = {
        "instance_id": instance,
        "status": status,
        "duration_seconds": duration,
        "provenance": {
            "platform": {"hardware_id": hardware_id}
            if hardware_id is not None
            else {}
        },
    }
    if operational_child_cpu is not None:
        metadata["cpu_time_seconds"] = operational_child_cpu
    (run_directory / "run.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )
    if objective is not None:
        time_line = (
            (
                f"The running time is:\n{solver_time}\n"
                + (
                    f"The time basis is:\n{time_basis}\n"
                    if time_basis is not None
                    else ""
                )
                + (
                    f"The time scope is:\n{time_scope}\n"
                    if time_scope is not None
                    else ""
                )
            )
            if solver_time is not None
            else ""
        )
        (run_directory / "solution.txt").write_text(
            f"The objective is:\n{objective}\n{time_line}", encoding="utf-8"
        )


class CollectionTests(unittest.TestCase):
    def test_search_cpu_is_not_overwritten_by_operational_child_cpu(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_run(
                root,
                "tiny",
                0,
                objective="7",
                solver_time="99",
                duration=1.25,
                operational_child_cpu=1.25,
            )
            metadata_path = next(root.rglob("run.json"))
            metadata = json.loads(
                metadata_path.read_text(encoding="utf-8")
            )
            metadata.update(
                {
                    "duration_seconds": 1.25,
                    "duration_basis": "process_cpu",
                    "wall_duration_seconds": 3.5,
                }
            )
            metadata_path.write_text(
                json.dumps(metadata), encoding="utf-8"
            )

            observations, discovered, warnings = comparison.discover_runs(root)
            rows = comparison.comparison_rows(
                observations, discovered, benchmarks={}
            )

        self.assertEqual(warnings, [])
        self.assertAlmostEqual(rows[0].avg_search_cpu_time_seconds, 99.0)
        self.assertAlmostEqual(
            rows[0].avg_operational_child_cpu_time_seconds, 1.25
        )
        self.assertEqual(
            rows[0].search_time_scope,
            comparison.SEARCH_TIME_SCOPE,
        )
        self.assertAlmostEqual(rows[0].avg_wall_time_seconds, 3.5)

    def test_typed_exact_reference_and_lower_bound(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            experiments = root / "experiments"
            references = root / "references"
            references.mkdir()
            (references / "reference_results.csv").write_text(
                "instance_id,status,value,lower_bound,source,"
                "distance_policy,time_seconds,baseline_name,time_basis,"
                "time_scope,hardware_id\n"
                "R101_s1_3_10,bks,105,100,paper,euclidean-floor,10,"
                "18-Farham,process_cpu,search_after_instance_loading,"
                "test-cpu\n"
                "R102_s1_3_10,unknown,,,paper,euclidean-floor,,,,,\n",
                encoding="utf-8",
            )
            write_run(
                experiments,
                "R101_s1_3_10",
                0,
                objective="110",
                solver_time="2",
            )
            observations, discovered, warnings = comparison.discover_runs(
                experiments
            )
            loaded, reference_warnings = comparison.load_benchmarks(references)
            rows = comparison.comparison_rows(
                observations, discovered, loaded
            )
        self.assertEqual(warnings, [])
        self.assertEqual(reference_warnings, [])
        self.assertEqual(rows[0].reference_status, "bks")
        self.assertEqual(rows[0].baseline_name, "18-Farham")
        self.assertEqual(rows[0].benchmark, 105.0)
        self.assertEqual(rows[0].lower_bound, 100.0)
        self.assertAlmostEqual(rows[0].best_gap_percent, 100 * 5 / 105)
        self.assertAlmostEqual(rows[0].best_gap_to_lower_bound_percent, 10.0)
        self.assertEqual(rows[0].reference_time_seconds, 10.0)
        self.assertAlmostEqual(rows[0].average_time_gap_percent, -80.0)
        self.assertAlmostEqual(rows[0].time_speedup, 5.0)
        self.assertEqual(rows[0].time_comparison_status, "compatible")

    def test_untyped_reference_time_is_descriptive_only(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            experiments = root / "experiments"
            references = root / "references"
            references.mkdir()
            (references / "reference_results.csv").write_text(
                "instance_id,status,value,time_seconds\n"
                "tiny,optimal,7,10\n",
                encoding="utf-8",
            )
            write_run(
                experiments,
                "tiny",
                0,
                objective="7",
                solver_time="2",
            )
            observations, discovered, run_warnings = (
                comparison.discover_runs(experiments)
            )
            loaded, reference_warnings = comparison.load_benchmarks(references)
            rows = comparison.comparison_rows(
                observations, discovered, loaded
            )
        self.assertEqual(run_warnings, [])
        self.assertEqual(len(reference_warnings), 1)
        self.assertEqual(
            rows[0].time_comparison_status,
            "reference_time_not_comparable",
        )
        self.assertIsNone(rows[0].average_time_gap_percent)
        self.assertIsNone(rows[0].time_speedup)
        self.assertEqual(rows[0].reference_time_seconds, 10.0)

    def test_legacy_unscoped_solution_time_is_not_aggregated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_run(
                root,
                "tiny",
                0,
                objective="7",
                solver_time="99",
                time_basis=None,
                time_scope=None,
                operational_child_cpu=3.0,
            )
            observations, discovered, warnings = comparison.discover_runs(root)
            rows = comparison.comparison_rows(
                observations, discovered, benchmarks={}
            )
        self.assertEqual(len(warnings), 1)
        self.assertEqual(rows[0].best_objective, 7.0)
        self.assertIsNone(rows[0].avg_search_cpu_time_seconds)
        self.assertEqual(
            rows[0].avg_operational_child_cpu_time_seconds, 3.0
        )

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
            self.assertAlmostEqual(row.avg_search_cpu_time_seconds, 2.0)
            self.assertAlmostEqual(row.avg_wall_time_seconds, 3.0)
            self.assertIsNone(row.reference_time_seconds)
            self.assertIsNone(row.average_time_gap_percent)
            self.assertIsNone(row.time_speedup)
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
            self.assertEqual(rows[0].avg_search_cpu_time_seconds, None)
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
            (root / "empty-experiments").mkdir()
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
                        "--experiments",
                        str(root / "empty-experiments"),
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
            self.assertEqual(
                rows[0]["avg_search_cpu_time_seconds"], "2.500000"
            )
            self.assertEqual(
                rows[0]["search_time_scope"],
                comparison.SEARCH_TIME_SCOPE,
            )
            self.assertEqual(rows[0]["avg_wall_time_seconds"], "3.000000")
            self.assertEqual(rows[0]["reference_time_seconds"], "")
            self.assertEqual(rows[0]["average_time_gap_percent"], "")
            self.assertEqual(rows[0]["time_speedup"], "")


class SpLogProfileTests(unittest.TestCase):
    def test_all_sp_statuses_are_counted_with_optional_solution_fields(
        self,
    ) -> None:
        text = "\n".join(
            (
                "[SP Result] status=OPTIMAL solutions=1 "
                "time_basis=process_cpu solve=2.0s total=2.5s "
                "objective=100 best_bound=100 gap=0",
                "[SP Result] status=INFEASIBLE solutions=0 "
                "time_basis=process_cpu solve=1.0s total=1.2s",
                "[SP Result] status=TIME_LIMIT solutions=0 "
                "solve=3.0s total=3.5s diagnostic=\"no incumbent\"",
            )
        )
        results = profiler.parse_sp_results(text)

        self.assertEqual(len(results), 3)
        self.assertEqual(
            [result["status"] for result in results],
            ["OPTIMAL", "INFEASIBLE", "TIME_LIMIT"],
        )
        self.assertEqual(results[1]["objective"], None)
        self.assertEqual(results[1]["bound"], None)
        self.assertEqual(results[1]["gap"], None)
        self.assertEqual(results[2]["time_basis"], "legacy_wall")


if __name__ == "__main__":
    unittest.main()
