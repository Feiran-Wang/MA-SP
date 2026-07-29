from __future__ import annotations

import concurrent.futures
import contextlib
import copy
import io
import json
import os
import signal
import stat
import subprocess
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
        "trigger_offset": 0,
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
        "problem_mode": "clrp",
        "name": "test_matrix",
        "binary": "bin/solver",
        "output_root": "artifacts",
        "instances": [{"path": "instance.json"}],
        "seeds": [0],
        "limits": {"time_seconds": 1, "iterations": 1},
        "resources": {"workers": 1, "gurobi_threads": 1},
        "sp": sp_config(),
        "validation": {"required": False},
        "environment": {},
    }


def test_topology(
    physical_cores: tuple[tuple[int, ...], ...],
) -> run_one.CpuTopology:
    logical = tuple(cpu for core in physical_cores for cpu in core)
    return run_one.CpuTopology(logical, physical_cores, "test")


class ValidationTests(unittest.TestCase):
    def test_schema_v2_is_accepted_as_clrp(self) -> None:
        value = manifest()
        value["schema_version"] = 2
        del value["problem_mode"]
        run_matrix.validate_manifest(value)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path = root / "manifest.json"
            manifest_path.write_text("{}", encoding="utf-8")
            with mock.patch.object(
                run_one,
                "detect_cpu_topology",
                return_value=test_topology(((0,),)),
            ):
                resolved = run_matrix.resolve_manifest(
                    value, manifest_path, root, None, None, None
                )
        self.assertEqual(resolved["problem_mode"], "clrp")
        self.assertEqual(resolved["source_schema_version"], 2)

    def test_clrp_tw_requires_independent_validation(self) -> None:
        value = manifest()
        value["problem_mode"] = "clrp-tw"
        with self.assertRaisesRegex(
            run_one.ExperimentError, "validation.required=true"
        ):
            run_matrix.validate_manifest(value)
        value["validation"]["required"] = True
        value["instances"] = [
            {
                "path": "instance.txt",
                "reference": {"status": "unknown", "value": None},
            }
        ]
        run_matrix.validate_manifest(value)

    def test_run_config_cannot_bypass_tw_validation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.txt"
            make_executable(binary, "exit 0\n")
            instance.write_text("placeholder", encoding="utf-8")
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)
            config["problem_mode"] = "clrp-tw"
            config["validation"] = {"required": False}
            with self.assertRaisesRegex(
                run_one.ExperimentError, "validation.required=true"
            ):
                run_one._validate_run_config(config, run_dir)

    def test_output_paths_are_coherent_and_cannot_overwrite_inputs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            make_executable(binary, "exit 0\n")
            instance.write_text("{}", encoding="utf-8")
            run_dir = root / "run"

            wrong_json = run_config(root, binary, instance, run_dir)
            wrong_json["solution_json"] = str(run_dir / "different.json")
            with self.assertRaisesRegex(
                run_one.ExperimentError, "sibling named solution.json"
            ):
                run_one._validate_run_config(wrong_json, run_dir)

            overwrites_input = run_config(root, binary, instance, run_dir)
            overwrites_input["solution"] = str(instance)
            overwrites_input["solution_json"] = str(
                instance.with_name("solution.json")
            )
            with self.assertRaisesRegex(
                run_one.ExperimentError, "must not overwrite instance"
            ):
                run_one._validate_run_config(overwrites_input)

            outside_run = run_config(root, binary, instance, run_dir)
            outside_run["solution"] = str(root / "solution.txt")
            outside_run["solution_json"] = str(root / "solution.json")
            with self.assertRaisesRegex(
                run_one.ExperimentError, "run_dir/solution.txt"
            ):
                run_one._validate_run_config(outside_run, run_dir)

    def test_default_validation_tolerance_matches_cpp_tw_evaluator(self) -> None:
        value = manifest()
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path = root / "manifest.json"
            manifest_path.write_text("{}", encoding="utf-8")
            with mock.patch.object(
                run_one,
                "detect_cpu_topology",
                return_value=test_topology(((0,),)),
            ):
                resolved = run_matrix.resolve_manifest(
                    value, manifest_path, root, None, None, None
                )
        self.assertEqual(
            resolved["validation"]["absolute_tolerance"], 1e-9
        )

    def test_run_target_fields_are_strictly_validated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            make_executable(binary, "exit 0\n")
            instance.write_text("{}", encoding="utf-8")
            run_dir = root / "run"

            for field, invalid, message in (
                ("best_known", -1.0, "finite and nonnegative"),
                ("is_optimal", 1, "must be boolean"),
            ):
                with self.subTest(field=field):
                    config = run_config(root, binary, instance, run_dir)
                    config[field] = invalid
                    with self.assertRaisesRegex(
                        run_one.ExperimentError, message
                    ):
                        run_one._validate_run_config(config, run_dir)

            config = run_config(root, binary, instance, run_dir)
            config["is_optimal"] = True
            with self.assertRaisesRegex(
                run_one.ExperimentError, "positive best_known"
            ):
                run_one._validate_run_config(config, run_dir)

            config = run_config(root, binary, instance, run_dir)
            config["best_known"] = 123.0
            config["is_optimal"] = True
            config["stop_at_reference"] = False
            with self.assertRaisesRegex(
                run_one.ExperimentError,
                "is_optimal must be false",
            ):
                run_one._validate_run_config(config, run_dir)

    def test_typed_reference_normalizes_without_fake_bks(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            instance = root / "instance.txt"
            instance.write_text("test", encoding="utf-8")
            normalized = run_matrix._normalize_instance(
                {
                    "path": str(instance),
                    "reference": {"status": "unknown", "value": None},
                },
                0,
                root,
            )
        self.assertEqual(normalized["reference"]["status"], "unknown")
        self.assertEqual(normalized["best_known"], 0.0)
        self.assertFalse(normalized["is_optimal"])
        self.assertTrue(normalized["stop_at_reference"])

    def test_optimal_reference_can_remain_metadata_without_early_stop(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            instance = root / "instance.txt"
            instance.write_text("test", encoding="utf-8")
            normalized = run_matrix._normalize_instance(
                {
                    "path": str(instance),
                    "reference": {"status": "optimal", "value": 123.0},
                    "stop_at_reference": False,
                },
                0,
                root,
            )
        self.assertEqual(normalized["reference"]["status"], "optimal")
        self.assertEqual(normalized["best_known"], 123.0)
        self.assertFalse(normalized["is_optimal"])
        self.assertFalse(normalized["stop_at_reference"])

    def test_stop_at_reference_controls_expanded_solver_argv(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            instance = root / "instance.txt"
            instance.write_text("test", encoding="utf-8")
            manifest_path = root / "manifest.json"
            manifest_path.write_text("{}", encoding="utf-8")

            def expanded(stop_at_reference: bool | None) -> tuple[dict, list[str]]:
                value = manifest()
                instance_config = {
                    "path": str(instance),
                    "reference": {"status": "optimal", "value": 123.0},
                }
                if stop_at_reference is not None:
                    instance_config["stop_at_reference"] = stop_at_reference
                value["instances"] = [instance_config]
                with mock.patch.object(
                    run_one,
                    "detect_cpu_topology",
                    return_value=test_topology(((0,),)),
                ):
                    resolved = run_matrix.resolve_manifest(
                        value, manifest_path, root, None, None, None
                    )
                config, _ = run_matrix.expand_runs(
                    resolved, root / "experiment"
                )[0]
                return config, run_one.build_solver_command(config)

            no_stop_config, no_stop_command = expanded(False)
            default_config, default_command = expanded(None)

        self.assertEqual(no_stop_config["reference"]["status"], "optimal")
        self.assertFalse(no_stop_config["is_optimal"])
        no_stop_solution_index = no_stop_command.index(
            no_stop_config["solution"]
        )
        self.assertEqual(no_stop_command[no_stop_solution_index + 2], "0")

        self.assertEqual(default_config["reference"]["status"], "optimal")
        self.assertTrue(default_config["is_optimal"])
        default_solution_index = default_command.index(
            default_config["solution"]
        )
        self.assertEqual(default_command[default_solution_index + 2], "1")

    def test_solver_command_appends_explicit_problem_mode(self) -> None:
        config = {
            "binary": "/solver",
            "instance": "/instance",
            "solution": "/solution",
            "best_known": 0,
            "is_optimal": False,
            "limits": {"time_seconds": 1, "iterations": 2},
            "seed": 3,
            "problem_mode": "clrp-tw",
        }
        command = run_one.build_solver_command(config)
        self.assertEqual(command[-2:], ["--problem-mode", "clrp-tw"])

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

    def test_gurobi_thread_limit_matches_supported_parameter_range(self) -> None:
        value = manifest()
        value["resources"]["gurobi_threads"] = (
            run_one.GUROBI_THREADS_MAX + 1
        )
        with self.assertRaisesRegex(run_one.ExperimentError, "at most 1024"):
            run_matrix.validate_manifest(value)

    def test_trigger_offset_must_be_within_frequency(self) -> None:
        value = manifest()
        value["sp"]["trigger_offset"] = value["sp"]["trigger_frequency"]
        with self.assertRaisesRegex(
            run_one.ExperimentError, "less than sp.trigger_frequency"
        ):
            run_matrix.validate_manifest(value)

    def test_cpu_topology_groups_affinity_visible_siblings(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            layout = {
                0: (0, 0),
                1: (0, 0),
                2: (0, 1),
                3: (0, 1),
            }
            for cpu, (package, core) in layout.items():
                topology_dir = root / f"cpu{cpu}" / "topology"
                topology_dir.mkdir(parents=True)
                (topology_dir / "physical_package_id").write_text(
                    str(package), encoding="ascii"
                )
                (topology_dir / "core_id").write_text(
                    str(core), encoding="ascii"
                )

            topology = run_one.detect_cpu_topology(
                logical_cpu_ids=(1, 2, 3),
                sysfs_cpu_root=root,
                system_name="Linux",
            )

        self.assertEqual(topology.logical_cpu_ids, (1, 2, 3))
        self.assertEqual(topology.physical_core_cpu_ids, ((1,), (2, 3)))
        self.assertEqual(topology.physical_core_representatives, (1, 2))
        self.assertEqual(topology.logical_cpu_count, 3)
        self.assertEqual(topology.physical_core_count, 2)
        self.assertTrue(topology.smt_available)
        metadata = run_one.collect_platform_provenance(topology)
        self.assertEqual(metadata["available_logical_cpu_ids"], [1, 2, 3])
        self.assertEqual(metadata["available_physical_core_count"], 2)
        self.assertEqual(
            metadata["physical_core_affinity_groups"], [[1], [2, 3]]
        )
        self.assertRegex(metadata["hardware_id"], r"^sha256:[0-9a-f]{64}$")
        self.assertEqual(
            metadata["hardware_identity"]["available_physical_core_count"], 2
        )
        self.assertEqual(metadata["hardware_identity"]["threads_per_core"], 2)
        self.assertEqual(
            run_one.collect_platform_provenance(topology)["hardware_id"],
            metadata["hardware_id"],
        )

    def test_default_worker_limit_uses_physical_cores(self) -> None:
        value = manifest()
        value["resources"] = {"workers": 3, "gurobi_threads": 1}
        topology = test_topology(((0, 1), (2, 3)))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path = root / "manifest.json"
            manifest_path.write_text("{}", encoding="utf-8")
            with mock.patch.object(
                run_one, "detect_cpu_topology", return_value=topology
            ):
                with self.assertRaisesRegex(
                    run_one.ExperimentError, "available physical cores"
                ):
                    run_matrix.resolve_manifest(
                        value, manifest_path, root, None, None, None
                    )

    def test_allow_smt_is_explicit_and_uses_logical_cpu_capacity(self) -> None:
        value = manifest()
        value["resources"] = {
            "workers": 3,
            "gurobi_threads": 1,
            "allow_smt": True,
        }
        topology = test_topology(((0, 1), (2, 3)))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path = root / "manifest.json"
            manifest_path.write_text("{}", encoding="utf-8")
            with mock.patch.object(
                run_one, "detect_cpu_topology", return_value=topology
            ):
                resolved = run_matrix.resolve_manifest(
                    value, manifest_path, root, None, None, None
                )
        self.assertTrue(resolved["resources"]["allow_smt"])

    def test_thread_budget_consumes_distinct_physical_cores(self) -> None:
        topology = test_topology(
            ((0, 1), (2, 3), (4, 5), (6, 7))
        )
        pool = run_matrix.build_cpu_affinity_pool(
            {"workers": 2, "gurobi_threads": 2, "allow_smt": False},
            topology,
        )
        first = pool.acquire()
        second = pool.acquire()
        try:
            self.assertEqual(first, (0, 2))
            self.assertEqual(second, (4, 6))
            self.assertTrue(set(first).isdisjoint(second))
        finally:
            pool.release(first)
            pool.release(second)

        with self.assertRaisesRegex(
            run_one.ExperimentError, "available physical cores"
        ):
            run_matrix.build_cpu_affinity_pool(
                {"workers": 3, "gurobi_threads": 2, "allow_smt": False},
                topology,
            )

    def test_affinity_pool_never_leases_a_cpu_twice(self) -> None:
        pool = run_matrix.CpuAffinityPool(
            ((0,), (2,)), "physical_core_representatives"
        )
        first = pool.acquire()
        second = pool.acquire()
        try:
            self.assertTrue(set(first).isdisjoint(second))
        finally:
            pool.release(first)
            pool.release(second)
        reacquired = pool.acquire()
        self.assertIn(reacquired, ((0,), (2,)))
        pool.release(reacquired)

    def test_affinity_pool_is_safe_under_threadpool_concurrency(self) -> None:
        pool = run_matrix.CpuAffinityPool(
            ((0,), (2,)), "physical_core_representatives"
        )
        active: set[tuple[int, ...]] = set()
        lock = threading.Lock()

        def use_slot() -> tuple[int, ...]:
            slot = pool.acquire()
            try:
                with lock:
                    self.assertNotIn(slot, active)
                    active.add(slot)
                time.sleep(0.01)
                with lock:
                    active.remove(slot)
                return slot
            finally:
                pool.release(slot)

        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            used = list(executor.map(lambda _: use_slot(), range(8)))
        self.assertEqual(set(used), {(0,), (2,)})

    def test_deterministic_trigger_offset_is_stable_and_preserves_calls(
        self,
    ) -> None:
        frequency = 10
        iterations = 23
        first = run_matrix.deterministic_trigger_offset(
            "instance__seed_7", frequency, iterations
        )
        second = run_matrix.deterministic_trigger_offset(
            "instance__seed_7", frequency, iterations
        )
        self.assertEqual(first, second)
        self.assertGreaterEqual(first, 0)
        self.assertLessEqual(first, 3)
        self.assertEqual(
            (iterations - first) // frequency,
            iterations // frequency,
        )

    def test_matrix_defaults_trigger_offset_to_deterministic(self) -> None:
        value = manifest()
        del value["sp"]["trigger_offset"]
        value["seeds"] = [0, 1]
        topology = test_topology(((0, 1), (2, 3)))
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest_path = root / "manifest.json"
            manifest_path.write_text("{}", encoding="utf-8")
            with mock.patch.object(
                run_one, "detect_cpu_topology", return_value=topology
            ):
                resolved = run_matrix.resolve_manifest(
                    value, manifest_path, root, None, None, None
                )
            runs = run_matrix.expand_runs(resolved, root / "runs")

        self.assertEqual(resolved["sp"]["trigger_offset"], "deterministic")
        self.assertFalse(resolved["sp"]["require_success"])
        for config, _ in runs:
            offset = config["sp"]["trigger_offset"]
            self.assertIsInstance(offset, int)
            self.assertGreaterEqual(offset, 0)
            self.assertLess(offset, config["sp"]["trigger_frequency"])

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
        self.assertEqual(overrides["HGASP_SP_TRIGGER_OFFSET"], "0")
        self.assertEqual(overrides["HGASP_SP_WORK_LIMIT"], "0")
        self.assertEqual(overrides["HGASP_SP_LOCAL_ENABLED"], "1")
        self.assertEqual(
            overrides["HGASP_SP_LOCAL_TRIGGER_FREQUENCY"], "10000"
        )
        self.assertEqual(
            overrides["HGASP_SP_LOCAL_TIME_LIMIT_SECONDS"], "30"
        )
        self.assertEqual(
            overrides["HGASP_SP_ADAPTIVE_TRIGGER_ENABLED"], "1"
        )
        self.assertEqual(
            overrides["HGASP_SP_ADAPTIVE_LOCAL_NEW_COLUMNS"], "250"
        )
        self.assertEqual(
            overrides["HGASP_SP_LP_DUAL_FEEDBACK_ENABLED"], "1"
        )
        self.assertEqual(
            overrides["HGASP_SP_PERSISTENT_LP_ENABLED"], "1"
        )
        self.assertEqual(
            overrides["HGASP_SP_SOLUTION_POOL_SIZE"], "5"
        )
        self.assertEqual(
            overrides["HGASP_SP_DIVERSE_INJECTION_COUNT"], "3"
        )
        self.assertEqual(
            overrides["HGASP_ADAPTIVE_SEARCH_ALLOCATION_ENABLED"],
            "1",
        )
        self.assertEqual(
            overrides["HGASP_ADAPTIVE_MUTATION_MAXIMUM_PERCENT"],
            "40",
        )
        self.assertEqual(overrides["HGASP_SP_CLIENT_LOG_LEVEL"], "1")
        self.assertEqual(
            overrides["HGASP_SP_ENVIRONMENT_MAX_ATTEMPTS"], "3"
        )
        self.assertEqual(
            overrides["HGASP_SP_ENVIRONMENT_RETRY_BASE_MILLISECONDS"],
            "250",
        )
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

    def test_optional_gurobi_environment_controls_are_validated(self) -> None:
        value = sp_config()
        value.update(run_one.SP_OPTIONAL_DEFAULTS)
        run_one.validate_sp_config(value)

        invalid_cases = {
            "work_limit": -1.0,
            "wls_token_duration_minutes": -1,
            "wls_token_refresh": 1.1,
            "client_log_level": 4,
            "server_timeout_seconds": 0,
            "environment_max_attempts": 11,
            "environment_retry_base_milliseconds": 60001,
        }
        for field, invalid in invalid_cases.items():
            with self.subTest(field=field):
                changed = dict(value)
                changed[field] = invalid
                with self.assertRaises(run_one.ExperimentError):
                    run_one.validate_sp_config(changed)

        invalid_boolean = sp_config()
        invalid_boolean["require_success"] = 1
        with self.assertRaisesRegex(
            run_one.ExperimentError, "sp.require_success must be boolean"
        ):
            run_one.validate_sp_config(invalid_boolean)


class ExecutionTests(unittest.TestCase):
    def test_signal_handlers_request_idempotent_cancellation(self) -> None:
        handled = [signal.SIGINT, signal.SIGTERM]
        if hasattr(signal, "SIGHUP"):
            handled.append(signal.SIGHUP)
        for signum in handled:
            with self.subTest(signal=signal.Signals(signum).name):
                cancellation = threading.Event()
                previous = signal.getsignal(signum)

                with run_one.cancellation_signal_handlers(
                    cancellation
                ) as state:
                    handler = signal.getsignal(signum)
                    self.assertTrue(callable(handler))
                    handler(signum, None)
                    self.assertTrue(cancellation.is_set())
                    self.assertEqual(
                        state.name, signal.Signals(signum).name
                    )
                    # A repeated signal must not break child reaping.
                    handler(signum, None)

                self.assertEqual(signal.getsignal(signum), previous)

    def test_run_records_exact_child_cpu_and_separate_wall_time(self) -> None:
        if os.name != "posix" or not hasattr(os, "wait4"):
            self.skipTest("per-child wait4 resource usage is unavailable")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "cpu-solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    "i=0\n"
                    "while [ \"$i\" -lt 20000 ]; do i=$((i + 1)); done\n"
                    'printf "valid solution\\n" > "$2"\n'
                ),
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)

            result = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )
            process_status = json.loads(
                Path(result["files"]["process_status"]).read_text()
            )

        self.assertEqual(result["status"], "success")
        self.assertEqual(result["duration_basis"], "process_cpu")
        self.assertEqual(result["duration_scope"], "whole_child_process")
        self.assertIsInstance(result["cpu_user_seconds"], float)
        self.assertIsInstance(result["cpu_system_seconds"], float)
        self.assertGreater(result["cpu_time_seconds"], 0.0)
        self.assertEqual(
            result["duration_seconds"], result["cpu_time_seconds"]
        )
        self.assertGreaterEqual(result["wall_duration_seconds"], 0.0)
        self.assertGreater(process_status["cpu_seconds"], 0.0)
        self.assertAlmostEqual(
            result["cpu_time_seconds"],
            process_status["cpu_seconds"],
            delta=0.01,
        )

    def test_required_sp_success_accepts_markers_across_attempt_logs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    'printf "[SP Result] status=OPTIMAL\\n'
                    '[SP Success] injected\\n" '
                    '> "$HGASP_LOG_DIR/hgasp.log"\n'
                    'printf "valid solution\\n" > "$2"\n'
                ),
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)
            config["sp"]["require_success"] = True

            result = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )

        self.assertEqual(result["status"], "success")
        assertion = result["sp_success_assertion"]
        self.assertTrue(assertion["required"])
        self.assertTrue(assertion["observed"])
        self.assertTrue(assertion["result_status_marker_found"])
        self.assertTrue(assertion["success_marker_found"])

    def test_sp_success_scans_rotated_canonical_logs_without_stdout_duplicates(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            hgasp = root / "hgasp.log"
            rotated = root / "hgasp.log.1"
            stdout = root / "stdout.log"
            first = b"[SP Result] status=OPTIMAL\n[SP Success]\n"
            second = b"[SP Result] status=TIME_LIMIT\n[SP Success]\n"
            rotated.write_bytes(first)
            hgasp.write_bytes(second)
            stdout.write_bytes(first + second)

            evidence = run_one.inspect_sp_success_attempt(hgasp, stdout)

        self.assertTrue(evidence["observed"])
        self.assertEqual(evidence["result_count"], 2)
        self.assertEqual(evidence["success_count"], 2)
        self.assertEqual(
            evidence["result_statuses"], ["OPTIMAL", "TIME_LIMIT"]
        )
        self.assertNotIn(str(stdout), evidence["checked_logs"])

    def test_required_sp_success_fails_when_sp_never_runs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                'printf "valid solution\\n" > "$2"\n',
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)
            config["sp"]["require_success"] = True

            result = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["process_status"], "sp_assertion_failed")
        self.assertFalse(result["sp_success_assertion"]["observed"])
        self.assertIn("[SP Result] status=", result["error"])
        self.assertIn("[SP Success]", result["error"])

    def test_required_sp_success_rejects_any_gurobi_error(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    'printf "[SP Result] status=GUROBI_ERROR\\n"\n'
                    'printf "[SP Result] status=OPTIMAL\\n"\n'
                    'printf "[SP Success] valid later call\\n"\n'
                    'printf "valid solution\\n" > "$2"\n'
                ),
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)
            config["sp"]["require_success"] = True

            result = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )

        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["process_status"], "sp_assertion_failed")
        assertion = result["sp_success_assertion"]
        self.assertFalse(assertion["observed"])
        self.assertEqual(assertion["failure_statuses"], ["GUROBI_ERROR"])
        self.assertEqual(assertion["result_count"], 2)
        self.assertEqual(assertion["success_count"], 1)

    def test_required_validation_rejects_missing_solution_json(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.txt"
            instance.write_text("placeholder", encoding="utf-8")
            make_executable(binary, 'printf "legacy solution\\n" > "$2"\n')
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)
            config["problem_mode"] = "clrp-tw"
            config["validation"] = {"required": True}

            result = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["validation_status"], "missing")
        self.assertIn("solution.json is missing", result["error"])

    def test_solver_inherits_and_records_assigned_cpu_affinity(self) -> None:
        if not hasattr(os, "sched_getaffinity") or not hasattr(
            os, "sched_setaffinity"
        ):
            self.skipTest("Linux CPU affinity is unavailable")

        original_affinity = set(os.sched_getaffinity(0))
        target_cpu = min(original_affinity)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "affinity-solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    "awk '/^Cpus_allowed_list:/ {print $2}' "
                    '/proc/self/status > "$2"\n'
                ),
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)

            result = run_one.execute_run(
                config,
                run_dir,
                root,
                shared_provenance=provenance(),
                cpu_affinity=(target_cpu,),
                affinity_mode="physical_core_representatives",
            )

            self.assertEqual(result["status"], "success")
            self.assertEqual(
                result["cpu_affinity"]["requested_logical_cpu_ids"],
                [target_cpu],
            )
            self.assertEqual(
                result["cpu_affinity"]["effective_logical_cpu_ids"],
                [target_cpu],
            )
            self.assertEqual(
                Path(config["solution"]).read_text(encoding="utf-8").strip(),
                str(target_cpu),
            )
        self.assertEqual(set(os.sched_getaffinity(0)), original_affinity)

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

            # A non-empty but altered artifact must never be trusted by resume.
            Path(config["solution"]).write_text(
                "tampered solution\n", encoding="utf-8"
            )
            recovered_from_tamper = run_one.execute_run(
                config, run_dir, root, shared_provenance=provenance()
            )
            self.assertEqual(recovered_from_tamper["status"], "success")
            self.assertEqual(recovered_from_tamper["attempt"], 3)
            self.assertNotIn("skipped", recovered_from_tamper)
            self.assertEqual(
                recovered_from_tamper["provenance"]["hashes"][
                    "solution_sha256"
                ],
                run_one.sha256_file(Path(config["solution"])),
            )

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

    def test_running_state_and_bounded_heartbeat_are_persisted(self) -> None:
        if os.name != "posix":
            self.skipTest("POSIX supervisor control pipes are unavailable")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "slow-solver"
            instance = root / "instance.json"
            pid_file = root / "solver.pid"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    'printf "%s\\n" "$$" > "$PID_FILE"\n'
                    "sleep 1\n"
                    'printf "valid solution\\n" > "$2"\n'
                ),
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)
            config["environment"] = {"PID_FILE": str(pid_file)}

            with mock.patch.object(
                run_one, "SUPERVISOR_HEARTBEAT_SECONDS", 0.05
            ), concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                future = pool.submit(
                    run_one.execute_run,
                    config,
                    run_dir,
                    root,
                    False,
                    provenance(),
                )
                run_path = run_dir / "run.json"
                deadline = time.monotonic() + 3
                live = None
                heartbeat = None
                while time.monotonic() < deadline:
                    if run_path.is_file():
                        live = json.loads(run_path.read_text())
                        status_name = live.get("files", {}).get(
                            "process_status"
                        )
                        if status_name and Path(status_name).is_file():
                            heartbeat = json.loads(
                                Path(status_name).read_text()
                            )
                            if heartbeat.get("sequence", 0) >= 2:
                                break
                    time.sleep(0.02)

                self.assertIsNotNone(live)
                self.assertEqual(live["status"], "running")
                self.assertIsInstance(live["supervisor_pid"], int)
                self.assertIsInstance(
                    live["supervisor_process_group_id"], int
                )
                self.assertIsInstance(
                    live["cpu_affinity"]["effective_logical_cpu_ids"], list
                )
                self.assertIsNotNone(heartbeat)
                self.assertEqual(heartbeat["health"], "running")
                self.assertIsInstance(heartbeat["solver_pid"], int)
                self.assertIsInstance(
                    heartbeat["solver_process_group_id"], int
                )
                self.assertIn("cpu_seconds", heartbeat)
                self.assertIn("rss_bytes", heartbeat)
                status_path = Path(live["files"]["process_status"])
                first_size = status_path.stat().st_size
                time.sleep(0.15)
                second_size = status_path.stat().st_size
                self.assertLess(first_size, 4096)
                self.assertLess(second_size, 4096)
                result = future.result(timeout=5)

            self.assertEqual(result["status"], "success")
            self.assertEqual(result["process_health"], "complete")

    def test_run_lock_rejects_concurrent_writer(self) -> None:
        if os.name != "posix":
            self.skipTest("flock test requires POSIX")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "slow-solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                "sleep 1\n"
                'printf "valid solution\\n" > "$2"\n',
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)

            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                future = pool.submit(
                    run_one.execute_run,
                    config,
                    run_dir,
                    root,
                    False,
                    provenance(),
                )
                deadline = time.monotonic() + 3
                while (
                    not (run_dir / "run.json").is_file()
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.02)
                with self.assertRaisesRegex(
                    run_one.ExperimentError, "already active"
                ):
                    run_one.execute_run(
                        config,
                        run_dir,
                        root,
                        shared_provenance=provenance(),
                    )
                result = future.result(timeout=5)

            self.assertEqual(result["status"], "success")
            self.assertEqual(result["attempt"], 1)

    def test_sigkill_runner_triggers_supervisor_cleanup(self) -> None:
        if os.name != "posix":
            self.skipTest("SIGKILL supervisor test requires POSIX")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            pid_file = root / "solver-pids.txt"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    "trap '' TERM\n"
                    "sh -c 'trap \"\" TERM; "
                    "while :; do sleep 1; done' &\n"
                    "child=$!\n"
                    'printf "%s %s\\n" "$$" "$child" > "$PID_FILE"\n'
                    'wait "$child"\n'
                ),
            )
            output_root = root / "output"
            value = manifest()
            value["binary"] = str(binary)
            value["output_root"] = str(output_root)
            value["instances"] = [{"path": str(instance)}]
            value["environment"] = {"PID_FILE": str(pid_file)}
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(value), encoding="utf-8")

            runner = subprocess.Popen(
                [
                    sys.executable,
                    str(RUNNER_DIR / "run_matrix.py"),
                    str(manifest_path),
                ],
                cwd=str(REPO_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            deadline = time.monotonic() + 10
            while not pid_file.is_file() and time.monotonic() < deadline:
                if runner.poll() is not None:
                    break
                time.sleep(0.02)
            self.assertTrue(pid_file.is_file())
            solver_pid, _child_pid = (
                int(value) for value in pid_file.read_text().split()
            )

            runner.kill()
            runner.communicate(timeout=5)
            deadline = time.monotonic() + 8
            while (
                run_one._process_group_has_live_members(solver_pid)
                and time.monotonic() < deadline
            ):
                time.sleep(0.05)
            self.assertFalse(
                run_one._process_group_has_live_members(solver_pid)
            )

            experiment_root = output_root / "test_matrix"
            run_dir = (
                experiment_root / "runs" / "instance" / "seed_0"
            )
            status_path = next(
                (run_dir / "logs").glob("attempt_*/process.status.json")
            )
            deadline = time.monotonic() + 5
            status = json.loads(status_path.read_text())
            while (
                status.get("health") not in ("complete", "cleanup_failed")
                and time.monotonic() < deadline
            ):
                time.sleep(0.05)
                status = json.loads(status_path.read_text())
            self.assertEqual(status["health"], "complete")
            self.assertEqual(status["reason"], "parent_control_eof")
            deadline = time.monotonic() + 5
            while True:
                try:
                    with run_one.CrossProcessFileLock(
                        experiment_root / ".matrix.lock", "recovery check"
                    ), run_one.CrossProcessFileLock(
                        run_dir / ".run.lock", "run recovery check"
                    ):
                        break
                except run_one.ExperimentError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.05)

    def test_matrix_lock_rejects_duplicate_invocation(self) -> None:
        if os.name != "posix":
            self.skipTest("flock test requires POSIX")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            pid_file = root / "launches.txt"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    'printf "%s\\n" "$$" >> "$PID_FILE"\n'
                    "sleep 1\n"
                    'printf "valid solution\\n" > "$2"\n'
                ),
            )
            output_root = root / "output"
            value = manifest()
            value["binary"] = str(binary)
            value["output_root"] = str(output_root)
            value["instances"] = [{"path": str(instance)}]
            value["environment"] = {"PID_FILE": str(pid_file)}
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(value), encoding="utf-8")
            command = [
                sys.executable,
                str(RUNNER_DIR / "run_matrix.py"),
                str(manifest_path),
            ]

            first = subprocess.Popen(
                command,
                cwd=str(REPO_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            deadline = time.monotonic() + 5
            while not pid_file.is_file() and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertTrue(pid_file.is_file())
            duplicate = subprocess.run(
                command,
                cwd=str(REPO_ROOT),
                capture_output=True,
                text=True,
                timeout=5,
                check=False,
            )
            stdout, stderr = first.communicate(timeout=10)

            self.assertEqual(duplicate.returncode, 2)
            self.assertIn("already active", duplicate.stderr)
            self.assertEqual(first.returncode, 0, stdout + stderr)
            self.assertEqual(len(pid_file.read_text().splitlines()), 1)
            matrix_status_path = (
                output_root / "test_matrix" / "matrix.status.json"
            )
            matrix_status = json.loads(matrix_status_path.read_text())
            self.assertEqual(matrix_status["future_states"]["done"], 1)
            self.assertLess(matrix_status_path.stat().st_size, 16384)

    def test_normal_leader_exit_cleans_background_process_group_member(
        self,
    ) -> None:
        if os.name != "posix":
            self.skipTest("POSIX process-group signals are unavailable")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            pid_file = root / "solver-pids.txt"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    "sleep 30 &\n"
                    "child=$!\n"
                    'printf "%s %s\\n" "$$" "$child" > "$PID_FILE"\n'
                    'printf "valid solution\\n" > "$2"\n'
                    "exit 0\n"
                ),
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)
            config["environment"] = {"PID_FILE": str(pid_file)}

            result = run_one.execute_run(
                config,
                run_dir,
                root,
                shared_provenance=provenance(),
            )

            leader_pid, child_pid = (
                int(value) for value in pid_file.read_text().split()
            )
            self.assertEqual(result["status"], "success")
            self.assertEqual(
                result["process_group_cleanup"]["status"], "complete"
            )
            self.assertTrue(
                result["process_group_cleanup"]["confirmed"]
            )
            self.assertEqual(
                result["process_group_cleanup"]["process_group_id"],
                leader_pid,
            )
            self.assertEqual(run_one.active_process_count(), 0)
            self.assertFalse(
                run_one._process_group_has_live_members(leader_pid)
            )

            # A zombie has already stopped and consumes no solver resources;
            # the container's init process may reap it shortly afterwards.
            child_stat = Path(f"/proc/{child_pid}/stat")
            if child_stat.is_file():
                self.assertEqual(
                    child_stat.read_text(encoding="utf-8").split()[2],
                    "Z",
                )

    def test_unconfirmed_group_cleanup_cannot_report_success(self) -> None:
        if os.name != "posix":
            self.skipTest("POSIX process-group signals are unavailable")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    "sleep 30 &\n"
                    'printf "valid solution\\n" > "$2"\n'
                    "exit 0\n"
                ),
            )
            run_dir = root / "run"
            config = run_config(root, binary, instance, run_dir)

            with mock.patch.object(
                run_one,
                "_managed_process_is_fully_stopped",
                return_value=False,
            ), mock.patch.object(
                run_one,
                "_wait_for_process_group_exit",
                return_value=False,
            ):
                result = run_one.execute_run(
                    config,
                    run_dir,
                    root,
                    shared_provenance=provenance(),
                )

            self.assertEqual(result["status"], "failed")
            self.assertEqual(result["process_status"], "cleanup_failed")
            self.assertEqual(
                result["process_group_cleanup"]["status"], "failed"
            )
            self.assertFalse(
                result["process_group_cleanup"]["confirmed"]
            )
            self.assertIn("TERM and KILL", result["error"])

            # The mocked failed confirmation deliberately retains the
            # registration for a later matrix cleanup pass. The real KILL has
            # already stopped the temporary child, so release the test-only
            # bookkeeping entry after checking that behavior.
            with run_one._ACTIVE_PROCESS_LOCK:
                retained = tuple(run_one._ACTIVE_PROCESSES.values())
            self.assertEqual(len(retained), 1)
            self.assertTrue(
                run_one._wait_for_process_group_exit(
                    retained[0], time.monotonic() + 2.0
                )
            )
            run_one._unregister_process(retained[0].process)

    def test_sigterm_matrix_writes_summary_and_leaves_no_live_solver(self) -> None:
        if os.name != "posix":
            self.skipTest("POSIX process-group signals are unavailable")
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            binary = root / "solver"
            instance = root / "instance.json"
            pid_file = root / "solver-pids.txt"
            term_file = root / "solver-received-term.txt"
            instance.write_text("{}", encoding="utf-8")
            make_executable(
                binary,
                (
                    "trap 'printf received > \"$TERM_FILE\"; exit 42' TERM\n"
                    "sh -c 'trap \"\" TERM; "
                    "while :; do sleep 1; done' &\n"
                    "child=$!\n"
                    'printf "%s %s\\n" "$$" "$child" > "$PID_FILE"\n'
                    'wait "$child"\n'
                ),
            )
            output_root = root / "output"
            value = manifest()
            value["binary"] = str(binary)
            value["output_root"] = str(output_root)
            value["instances"] = [{"path": str(instance)}]
            value["environment"] = {
                "PID_FILE": str(pid_file),
                "TERM_FILE": str(term_file),
            }
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(value), encoding="utf-8")

            runner = subprocess.Popen(
                [
                    sys.executable,
                    str(RUNNER_DIR / "run_matrix.py"),
                    str(manifest_path),
                ],
                cwd=str(REPO_ROOT),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            deadline = time.monotonic() + 10
            while not pid_file.is_file() and time.monotonic() < deadline:
                if runner.poll() is not None:
                    break
                time.sleep(0.02)
            self.assertTrue(
                pid_file.is_file(),
                runner.communicate(timeout=2)[1]
                if runner.poll() is not None
                else "solver did not launch",
            )
            solver_pids = [
                int(value) for value in pid_file.read_text().split()
            ]

            runner.send_signal(signal.SIGTERM)
            stdout, stderr = runner.communicate(timeout=15)

            self.assertEqual(runner.returncode, 128 + signal.SIGTERM, stderr)
            summary_path = output_root / "test_matrix" / "summary.json"
            self.assertTrue(summary_path.is_file(), stdout + stderr)
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(summary["interrupted"])
            self.assertEqual(summary["interruption"]["signal"], "SIGTERM")
            self.assertEqual(
                summary["active_solver_processes_after_cleanup"], 0
            )
            self.assertTrue(summary["process_group_cleanup_confirmed"])
            self.assertIsNone(summary["process_group_cleanup_error"])
            run_path = (
                output_root
                / "test_matrix"
                / "runs"
                / "instance"
                / "seed_0"
                / "run.json"
            )
            run_result = json.loads(run_path.read_text(encoding="utf-8"))
            self.assertEqual(run_result["status"], "interrupted")
            self.assertEqual(
                term_file.read_text(encoding="utf-8"), "received"
            )

            # A zombie is already dead and consumes no solver resources; it can
            # briefly remain until the container's init process reaps it.
            for solver_pid in solver_pids:
                proc_stat = Path(f"/proc/{solver_pid}/stat")
                if proc_stat.is_file():
                    state = proc_stat.read_text(encoding="utf-8").split()[2]
                    self.assertEqual(state, "Z")
            self.assertFalse(
                run_one._process_group_has_live_members(solver_pids[0])
            )


if __name__ == "__main__":
    unittest.main()
