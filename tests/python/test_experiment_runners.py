from __future__ import annotations

import concurrent.futures
import contextlib
import copy
import io
import json
import os
import stat
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_DIR = REPO_ROOT / "scripts" / "experiments"
sys.path.insert(0, str(RUNNER_DIR))

import run_matrix  # noqa: E402
import run_one  # noqa: E402


def sp_config() -> dict:
    return {
        "trigger_frequency": 10,
        "time_limit_seconds": 1.0,
        "mip_gap": 0.0,
        "seed": 0,
        "max_pool_size": 100,
        "output_flag": 0,
        "mip_focus": 0,
        "heuristics": 0.1,
        "representation_route_limit": True,
    }


def make_executable(path: Path, body: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#!/bin/sh\nset -eu\n" + body, encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def run_config(root: Path, binary: Path, instance: Path, run_dir: Path) -> dict:
    return {
        "schema_version": run_one.SCHEMA_VERSION,
        "experiment_name": "test",
        "run_id": "instance__seed_0",
        "instance_id": "instance",
        "binary": str(binary),
        "instance": str(instance),
        "depot_config": None,
        "solution": str(run_dir / "solution.txt"),
        "seed": 0,
        "best_known": 0.0,
        "is_optimal": False,
        "limits": {
            "time_seconds": 1,
            "iterations": 1,
            "process_timeout_seconds": 10,
        },
        "resources": {"gurobi_threads": 1},
        "sp": sp_config(),
        "environment": {"EXPERIMENT_LABEL": "value with spaces"},
        "gurobi_home": None,
    }


def provenance() -> dict:
    return {
        "git": {"available": False},
        "platform": {"logical_cpu_count": 1},
        "gurobi": {"version": None},
    }


def manifest() -> dict:
    return {
        "schema_version": run_one.SCHEMA_VERSION,
        "name": "test_matrix",
        "binary": "bin/solver",
        "output_root": "artifacts",
        "instances": [{"path": "instance.json"}],
        "seeds": [0],
        "limits": {"time_seconds": 1, "iterations": 1},
        "resources": {"workers": 1, "gurobi_threads": 1},
        "sp": sp_config(),
        "environment": {},
    }


class ValidationTests(unittest.TestCase):
    def test_zero_time_limit_means_unlimited(self) -> None:
        value = manifest()
        value["limits"]["time_seconds"] = 0
        run_matrix.validate_manifest(value)

    def test_project_venv_is_required(self) -> None:
        run_one.require_project_venv(REPO_ROOT)
        with self.assertRaisesRegex(run_one.ExperimentError, r"\.venv"):
            run_one.require_project_venv(
                REPO_ROOT, current_prefix=REPO_ROOT / "another-venv"
            )

    def test_seed_boundaries_match_cpp_int(self) -> None:
        value = manifest()
        value["seeds"] = [0, run_one.CPP_INT_MAX]
        run_matrix.validate_manifest(value)

        for invalid in (-1, run_one.CPP_INT_MAX + 1):
            changed = copy.deepcopy(value)
            changed["seeds"] = [invalid]
            with self.assertRaises(run_one.ExperimentError):
                run_matrix.validate_manifest(changed)

    def test_cpu_oversubscription_respects_available_affinity(self) -> None:
        value = manifest()
        value["resources"] = {"workers": 2, "gurobi_threads": 2}
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path = root / "manifest.json"
            manifest_path.write_text("{}", encoding="utf-8")
            with mock.patch.object(run_one, "available_cpu_count", return_value=3):
                with self.assertRaisesRegex(
                    run_one.ExperimentError, "exceeding 3"
                ):
                    run_matrix.resolve_manifest(
                        value, manifest_path, root, None, None, None
                    )

    def test_environment_mapping_is_explicit_and_safe(self) -> None:
        config = {
            "environment": {"EXPERIMENT_LABEL": "alpha beta"},
            "resources": {"gurobi_threads": 2},
            "sp": sp_config(),
            "gurobi_home": "/gurobi root",
        }
        overrides = run_one._environment_overrides(config)
        self.assertEqual(overrides["EXPERIMENT_LABEL"], "alpha beta")
        self.assertEqual(overrides["HGASP_SP_THREADS"], "2")
        self.assertEqual(overrides["GUROBI_HOME"], "/gurobi root")

        for key in (
            "HGASP_SP_SEED",
            "HGASP_LOG_DIR",
            "HGASP_SP_LOG_FILE",
            "MY_API_TOKEN",
            "GUROBI_SECRET",
        ):
            with self.assertRaises(run_one.ExperimentError):
                run_one.validate_environment_mapping({key: "do-not-record"})


class ExecutionTests(unittest.TestCase):
    def test_paths_with_spaces_and_missing_solution_recovery(self) -> None:
        with tempfile.TemporaryDirectory(prefix="hgasp runner ") as temporary:
            root = Path(temporary)
            binary = root / "binary dir" / "fake solver"
            instance = root / "instance dir" / "tiny instance.json"
            instance.parent.mkdir(parents=True)
            instance.write_text("{}", encoding="utf-8")
            make_executable(binary, 'printf "valid solution\\n" > "$2"\n')
            run_dir = root / "output dir" / "seed 0"
            config = run_config(root, binary, instance, run_dir)

            first = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )
            self.assertEqual(first["status"], "success")
            self.assertEqual(first["command"][0], str(binary))
            self.assertEqual(first["command"][1], str(instance))
            self.assertEqual(first["attempt"], 1)
            first_logs = run_dir / "logs" / "attempt_001"
            self.assertEqual(
                Path(first["files"]["log_directory"]),
                first_logs,
            )
            self.assertTrue((first_logs / "stdout.log").is_file())
            self.assertTrue((first_logs / "stderr.log").is_file())
            self.assertEqual(
                first["environment_overrides"]["HGASP_LOG_DIR"],
                str(first_logs),
            )
            self.assertEqual(
                first["environment_overrides"]["HGASP_SP_LOG_FILE"],
                str(first_logs / "gurobi.log"),
            )

            Path(config["solution"]).unlink()
            second = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )
            self.assertEqual(second["status"], "success")
            self.assertEqual(second["attempt"], 2)
            second_logs = run_dir / "logs" / "attempt_002"
            self.assertTrue((second_logs / "stdout.log").is_file())
            self.assertTrue((second_logs / "stderr.log").is_file())
            self.assertTrue((first_logs / "stdout.log").is_file())

            skipped = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )
            self.assertTrue(skipped["skipped"])

    def test_dry_run_creates_no_artifacts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="hgasp dry run ") as temporary:
            root = Path(temporary)
            binary = root / "bin with spaces" / "solver"
            instance = root / "inputs with spaces" / "instance.json"
            depot = root / "inputs with spaces" / "depot config.txt"
            instance.parent.mkdir(parents=True)
            instance.write_text("{}", encoding="utf-8")
            depot.write_text("depot", encoding="utf-8")
            make_executable(binary, "exit 0\n")
            output_root = root / "artifacts with spaces"
            value = manifest()
            value["binary"] = str(binary)
            value["output_root"] = str(output_root)
            value["instances"] = [
                {"path": str(instance), "depot_config": str(depot)}
            ]
            manifest_path = root / "manifest with spaces.json"
            manifest_path.write_text(json.dumps(value), encoding="utf-8")

            stdout = io.StringIO()
            stderr = io.StringIO()
            with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(
                stderr
            ):
                result = run_matrix.main([str(manifest_path), "--dry-run"])
            self.assertEqual(result, 0, stderr.getvalue())
            self.assertIn(str(instance), stdout.getvalue())
            self.assertFalse(output_root.exists())

    def test_failed_attempt_is_retried(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(binary, "exit 7\n")
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)

            failed = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )
            self.assertEqual(failed["status"], "failed")
            self.assertEqual(failed["attempt"], 1)

            make_executable(binary, 'printf "recovered\\n" > "$2"\n')
            recovered = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )
            self.assertEqual(recovered["status"], "success")
            self.assertEqual(recovered["attempt"], 2)

    def test_cancellation_terminates_active_solver(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "slow-solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(binary, "sleep 30\n")
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)
            cancellation = threading.Event()

            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                future = pool.submit(
                    run_one.execute_run,
                    config,
                    run_dir,
                    root,
                    False,
                    provenance(),
                    cancellation,
                )
                deadline = time.monotonic() + 3
                while (
                    run_one.active_process_count() == 0
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.01)
                self.assertEqual(run_one.active_process_count(), 1)
                cancellation.set()
                run_one.terminate_active_processes()
                result = future.result(timeout=5)

            self.assertEqual(result["status"], "interrupted")
            self.assertEqual(run_one.active_process_count(), 0)


if __name__ == "__main__":
    unittest.main()
