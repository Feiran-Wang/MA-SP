#!/usr/bin/env python3
"""Expand and execute a JSON HGA-SP experiment manifest."""

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import hashlib
import json
import math
import os
import queue
import shlex
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import run_one


MATRIX_STATUS_INTERVAL_SECONDS = 30.0
MATRIX_STATUS_ACTIVE_SAMPLE_LIMIT = 20


class CpuAffinityPool:
    """Thread-safe leases for non-overlapping solver CPU affinities."""

    def __init__(
        self,
        slots: Sequence[Sequence[int]],
        mode: str,
    ) -> None:
        normalized = tuple(
            tuple(sorted(set(int(cpu) for cpu in slot))) for slot in slots
        )
        if not normalized or any(not slot for slot in normalized):
            raise run_one.ExperimentError(
                "CPU affinity scheduler requires non-empty slots"
            )
        flattened = [cpu for slot in normalized for cpu in slot]
        if len(set(flattened)) != len(flattened):
            raise run_one.ExperimentError(
                "CPU affinity scheduler slots overlap"
            )

        self.mode = mode
        self._slots = frozenset(normalized)
        self._leased: set[Tuple[int, ...]] = set()
        self._lock = threading.Lock()
        self._available: queue.Queue[Tuple[int, ...]] = queue.Queue()
        for slot in normalized:
            self._available.put(slot)

    @property
    def size(self) -> int:
        return len(self._slots)

    def acquire(self) -> Tuple[int, ...]:
        slot = self._available.get()
        with self._lock:
            if slot in self._leased:
                raise run_one.ExperimentError(
                    f"CPU affinity slot was leased twice: {slot}"
                )
            self._leased.add(slot)
        return slot

    def release(self, slot: Sequence[int]) -> None:
        normalized = tuple(sorted(set(int(cpu) for cpu in slot)))
        with self._lock:
            if normalized not in self._leased:
                raise run_one.ExperimentError(
                    f"CPU affinity slot was not leased: {normalized}"
                )
            self._leased.remove(normalized)
        self._available.put(normalized)


def build_cpu_affinity_pool(
    resources: Mapping[str, Any],
    topology: run_one.CpuTopology,
) -> CpuAffinityPool:
    workers = int(resources["workers"])
    allow_smt = bool(resources.get("allow_smt", False))
    cpu_units = topology.scheduling_slots(allow_smt)
    gurobi_threads = int(resources["gurobi_threads"])
    required_units = workers * gurobi_threads
    capacity_name = "logical CPUs with SMT" if allow_smt else "physical cores"
    if required_units > len(cpu_units):
        opt_in = (
            ""
            if allow_smt
            else "; set resources.allow_smt=true explicitly to use SMT siblings"
        )
        raise run_one.ExperimentError(
            f"workers ({workers}) * gurobi_threads ({gurobi_threads}) = "
            f"{required_units}, exceeding {len(cpu_units)} available "
            f"{capacity_name}{opt_in}"
        )

    selected_units = cpu_units[:required_units]
    slots = []
    for start in range(0, required_units, gurobi_threads):
        affinity = tuple(
            cpu
            for unit in selected_units[start : start + gurobi_threads]
            for cpu in unit
        )
        slots.append(affinity)

    return CpuAffinityPool(
        slots,
        (
            "logical_cpu_smt"
            if allow_smt
            else "physical_core_representatives"
        ),
    )


def _require_integer(
    value: Any,
    field: str,
    minimum: Optional[int] = None,
    maximum: Optional[int] = None,
) -> int:
    return run_one.require_integer(value, field, minimum, maximum)


def deterministic_trigger_offset(
    run_id: str,
    trigger_frequency: int,
    iteration_limit: int,
) -> int:
    """Return a stable offset without reducing the scheduled SP call count."""

    maximum_offset = min(
        trigger_frequency - 1,
        iteration_limit % trigger_frequency,
    )
    if maximum_offset <= 0:
        return 0
    digest = hashlib.sha256(run_id.encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "big") % (maximum_offset + 1)


def _validate_keys(
    value: Mapping[str, Any],
    field: str,
    allowed: Sequence[str],
    required: Sequence[str] = (),
) -> None:
    unknown = sorted(set(value) - set(allowed))
    if unknown:
        raise run_one.ExperimentError(
            f"{field} contains unknown field(s): {', '.join(unknown)}"
        )
    missing = sorted(set(required) - set(value))
    if missing:
        raise run_one.ExperimentError(
            f"{field} is missing required field(s): {', '.join(missing)}"
        )


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    _validate_keys(
        manifest,
        "manifest",
        (
            "schema_version",
            "problem_mode",
            "name",
            "binary",
            "output_root",
            "gurobi_home",
            "instances",
            "seeds",
            "limits",
            "resources",
            "sp",
            "validation",
            "environment",
            "fail_fast",
        ),
        (
            "schema_version",
            "name",
            "binary",
            "output_root",
            "instances",
            "seeds",
            "limits",
            "resources",
            "sp",
        ),
    )
    schema_version = manifest.get("schema_version")
    if schema_version not in (
        run_one.LEGACY_SCHEMA_VERSION,
        run_one.SCHEMA_VERSION,
    ):
        raise run_one.ExperimentError(
            f"schema_version must be {run_one.LEGACY_SCHEMA_VERSION} "
            f"or {run_one.SCHEMA_VERSION}"
        )
    if schema_version == run_one.SCHEMA_VERSION:
        if "problem_mode" not in manifest:
            raise run_one.ExperimentError(
                "schema v3 requires top-level problem_mode"
            )
    elif manifest.get("problem_mode", "clrp") != "clrp":
        raise run_one.ExperimentError(
            "schema v2 manifests are interpreted as problem_mode='clrp'"
        )
    if manifest.get("problem_mode", "clrp") not in run_one.PROBLEM_MODES:
        raise run_one.ExperimentError(
            "problem_mode must be either 'clrp' or 'clrp-tw'"
        )
    for field in ("name", "binary", "output_root"):
        if not isinstance(manifest.get(field), str) or not manifest[field]:
            raise run_one.ExperimentError(f"{field} must be a non-empty string")
    if run_one.safe_identifier(manifest["name"]) != manifest["name"]:
        raise run_one.ExperimentError(
            "name must contain only letters, digits, '.', '_', or '-'"
        )
    if "fail_fast" in manifest and not isinstance(
        manifest["fail_fast"], bool
    ):
        raise run_one.ExperimentError("fail_fast must be boolean")
    if "gurobi_home" in manifest and (
        not isinstance(manifest["gurobi_home"], str)
        or not manifest["gurobi_home"]
    ):
        raise run_one.ExperimentError("gurobi_home must be a non-empty string")

    instances = manifest.get("instances")
    if not isinstance(instances, list) or not instances:
        raise run_one.ExperimentError("instances must be a non-empty array")
    for index, instance in enumerate(instances):
        if isinstance(instance, str):
            if not instance:
                raise run_one.ExperimentError(
                    f"instances[{index}] cannot be empty"
                )
            continue
        if not isinstance(instance, Mapping):
            raise run_one.ExperimentError(
                f"instances[{index}] must be a path string or object"
            )
        _validate_keys(
            instance,
            f"instances[{index}]",
            (
                "id",
                "path",
                "depot_config",
                "best_known",
                "is_optimal",
                "reference",
                "stop_at_reference",
            ),
            ("path",),
        )
        if not isinstance(instance.get("path"), str) or not instance["path"]:
            raise run_one.ExperimentError(
                f"instances[{index}].path must be a non-empty string"
            )
        if "id" in instance:
            identifier = instance["id"]
            if (
                not isinstance(identifier, str)
                or run_one.safe_identifier(identifier) != identifier
            ):
                raise run_one.ExperimentError(
                    f"instances[{index}].id must contain only "
                    "letters, digits, '.', '_', or '-'"
                )
        if "depot_config" in instance and (
            not isinstance(instance["depot_config"], str)
            or not instance["depot_config"]
        ):
            raise run_one.ExperimentError(
                f"instances[{index}].depot_config must be a string"
            )
        if "is_optimal" in instance and not isinstance(
            instance["is_optimal"], bool
        ):
            raise run_one.ExperimentError(
                f"instances[{index}].is_optimal must be boolean"
            )
        if "stop_at_reference" in instance and not isinstance(
            instance["stop_at_reference"], bool
        ):
            raise run_one.ExperimentError(
                f"instances[{index}].stop_at_reference must be boolean"
            )
        if "best_known" in instance and (
            isinstance(instance["best_known"], bool)
            or not isinstance(instance["best_known"], (int, float))
            or not math.isfinite(float(instance["best_known"]))
            or float(instance["best_known"]) < 0
        ):
            raise run_one.ExperimentError(
                f"instances[{index}].best_known must be finite and nonnegative"
            )
        if instance.get("is_optimal", False) and float(
            instance.get("best_known", 0)
        ) <= 0:
            raise run_one.ExperimentError(
                f"instances[{index}].is_optimal=true requires a positive "
                "best_known value"
            )
        reference = instance.get("reference")
        if reference is not None:
            if "best_known" in instance or "is_optimal" in instance:
                raise run_one.ExperimentError(
                    f"instances[{index}] cannot combine reference with "
                    "legacy best_known/is_optimal"
                )
            if not isinstance(reference, Mapping):
                raise run_one.ExperimentError(
                    f"instances[{index}].reference must be an object"
                )
            _validate_keys(
                reference,
                f"instances[{index}].reference",
                (
                    "status",
                    "value",
                    "lower_bound",
                    "source",
                    "distance_policy",
                ),
                ("status",),
            )
            status = reference.get("status")
            if status not in ("unknown", "bks", "optimal"):
                raise run_one.ExperimentError(
                    f"instances[{index}].reference.status must be "
                    "unknown, bks, or optimal"
                )
            reference_value = reference.get("value")
            if status == "unknown":
                if reference_value is not None:
                    raise run_one.ExperimentError(
                        f"instances[{index}].reference.value must be null "
                        "when status is unknown"
                    )
            elif (
                isinstance(reference_value, bool)
                or not isinstance(reference_value, (int, float))
                or not math.isfinite(float(reference_value))
                or float(reference_value) <= 0
            ):
                raise run_one.ExperimentError(
                    f"instances[{index}].reference.value must be a "
                    "positive finite number"
                )
            lower_bound = reference.get("lower_bound")
            if lower_bound is not None and (
                isinstance(lower_bound, bool)
                or not isinstance(lower_bound, (int, float))
                or not math.isfinite(float(lower_bound))
                or float(lower_bound) <= 0
            ):
                raise run_one.ExperimentError(
                    f"instances[{index}].reference.lower_bound must be a "
                    "positive finite number or null"
                )
            if (
                lower_bound is not None
                and reference_value is not None
                and float(lower_bound) > float(reference_value) + 1e-9
            ):
                raise run_one.ExperimentError(
                    f"instances[{index}].reference.lower_bound cannot exceed "
                    "reference.value"
                )
            for field in ("source", "distance_policy"):
                if field in reference and (
                    not isinstance(reference[field], str)
                    or not reference[field].strip()
                ):
                    raise run_one.ExperimentError(
                        f"instances[{index}].reference.{field} must be "
                        "a non-empty string"
                    )

    seeds = manifest.get("seeds")
    if not isinstance(seeds, list) or not seeds:
        raise run_one.ExperimentError("seeds must be a non-empty array")
    parsed_seeds = [
        _require_integer(
            seed, f"seeds[{index}]", 0, run_one.CPP_INT_MAX
        )
        for index, seed in enumerate(seeds)
    ]
    if len(set(parsed_seeds)) != len(parsed_seeds):
        raise run_one.ExperimentError("seeds must not contain duplicates")

    limits = manifest.get("limits")
    resources = manifest.get("resources")
    sp = manifest.get("sp")
    if not isinstance(limits, Mapping):
        raise run_one.ExperimentError("limits must be an object")
    if not isinstance(resources, Mapping):
        raise run_one.ExperimentError("resources must be an object")
    if not isinstance(sp, Mapping):
        raise run_one.ExperimentError("sp must be an object")
    _validate_keys(
        limits,
        "limits",
        ("time_seconds", "iterations", "process_timeout_seconds"),
        ("time_seconds", "iterations"),
    )
    _validate_keys(
        resources,
        "resources",
        ("workers", "gurobi_threads", "allow_smt"),
        ("workers", "gurobi_threads"),
    )
    _validate_keys(
        sp,
        "sp",
        (
            "trigger_frequency",
            "trigger_offset",
            "time_limit_seconds",
            "local_enabled",
            "local_trigger_frequency",
            "local_time_limit_seconds",
            "adaptive_trigger_enabled",
            "adaptive_minimum_interval",
            "adaptive_local_new_columns",
            "adaptive_local_stagnation",
            "adaptive_global_stagnation",
            "adaptive_utility_ema_weight",
            "adaptive_high_utility_threshold",
            "lp_dual_feedback_enabled",
            "lp_time_limit_seconds",
            "persistent_lp_enabled",
            "solution_pool_size",
            "solution_pool_gap",
            "solution_pool_search_mode",
            "diverse_injection_count",
            "adaptive_search_allocation_enabled",
            "adaptive_population_exploration",
            "adaptive_population_diversity_weight",
            "adaptive_population_column_weight",
            "adaptive_search_ema_weight",
            "adaptive_mutation_minimum_percent",
            "adaptive_mutation_maximum_percent",
            "work_limit",
            "mip_gap",
            "seed",
            "max_pool_size",
            "output_flag",
            "mip_focus",
            "heuristics",
            "wls_token_duration_minutes",
            "wls_token_refresh",
            "client_log_level",
            "server_timeout_seconds",
            "environment_max_attempts",
            "environment_retry_base_milliseconds",
            "representation_route_limit",
            "require_success",
        ),
        (
            "trigger_frequency",
            "time_limit_seconds",
            "mip_gap",
            "seed",
            "max_pool_size",
            "output_flag",
            "mip_focus",
            "heuristics",
            "representation_route_limit",
        ),
    )
    _require_integer(
        limits.get("time_seconds"),
        "limits.time_seconds",
        0,
        run_one.CPP_INT_MAX,
    )
    _require_integer(
        limits.get("iterations"),
        "limits.iterations",
        1,
        run_one.CPP_INT_MAX,
    )
    process_timeout = limits.get("process_timeout_seconds")
    if process_timeout is not None and (
        isinstance(process_timeout, bool)
        or not isinstance(process_timeout, (int, float))
        or not math.isfinite(float(process_timeout))
        or process_timeout <= 0
    ):
        raise run_one.ExperimentError(
            "limits.process_timeout_seconds must be finite and positive"
        )
    _require_integer(
        resources.get("workers"),
        "resources.workers",
        1,
        run_one.CPP_INT_MAX,
    )
    _require_integer(
        resources.get("gurobi_threads"),
        "resources.gurobi_threads",
        1,
        run_one.GUROBI_THREADS_MAX,
    )
    if "allow_smt" in resources and not isinstance(
        resources["allow_smt"], bool
    ):
        raise run_one.ExperimentError("resources.allow_smt must be boolean")
    run_one.validate_sp_config(sp)
    run_one.validate_environment_mapping(manifest.get("environment", {}))
    validation = manifest.get("validation", {"required": False})
    if not isinstance(validation, Mapping):
        raise run_one.ExperimentError("validation must be an object")
    _validate_keys(
        validation,
        "validation",
        ("required", "absolute_tolerance", "relative_tolerance"),
    )
    if not isinstance(validation.get("required", False), bool):
        raise run_one.ExperimentError("validation.required must be boolean")
    for field in ("absolute_tolerance", "relative_tolerance"):
        if field in validation and (
            isinstance(validation[field], bool)
            or not isinstance(validation[field], (int, float))
            or not math.isfinite(float(validation[field]))
            or float(validation[field]) < 0
        ):
            raise run_one.ExperimentError(
                f"validation.{field} must be finite and nonnegative"
            )
    if (
        manifest.get("problem_mode", "clrp") == "clrp-tw"
        and not validation.get("required", False)
    ):
        raise run_one.ExperimentError(
            "clrp-tw manifests must set validation.required=true"
        )


def _normalize_instance(
    raw: Any, index: int, repo_root: Path
) -> Dict[str, Any]:
    if isinstance(raw, str):
        path = run_one.resolve_path(raw, repo_root)
        value: Dict[str, Any] = {"path": str(path)}
    else:
        value = dict(raw)
        path = run_one.resolve_path(value["path"], repo_root)
        value["path"] = str(path)

    value["id"] = run_one.safe_identifier(value.get("id") or path.stem)
    value["depot_config"] = (
        str(run_one.resolve_path(value["depot_config"], repo_root))
        if value.get("depot_config")
        else None
    )
    raw_reference = value.get("reference")
    stop_at_reference = bool(value.get("stop_at_reference", True))
    if raw_reference is None:
        best_known = float(value.get("best_known", 0.0))
        reference_is_optimal = bool(value.get("is_optimal", False))
        value["reference"] = {
            "status": "optimal" if reference_is_optimal else (
                "bks" if best_known > 0 else "unknown"
            ),
            "value": best_known if best_known > 0 else None,
            "lower_bound": None,
            "source": "legacy manifest fields",
        }
    else:
        value["reference"] = dict(raw_reference)
        status = value["reference"]["status"]
        best_known = float(value["reference"].get("value") or 0.0)
        reference_is_optimal = status == "optimal"
    # The current C++ positional interface still accepts a numeric incumbent
    # and optimality flag.  Unknown references are mapped to its neutral 0/false
    # values, while the typed reference remains in run metadata.
    value["best_known"] = best_known
    value["is_optimal"] = reference_is_optimal and stop_at_reference
    value["stop_at_reference"] = stop_at_reference
    value["manifest_index"] = index
    return value


def resolve_manifest(
    manifest: Dict[str, Any],
    manifest_path: Path,
    repo_root: Path,
    workers_override: Optional[int],
    threads_override: Optional[int],
    output_override: Optional[str],
    allow_smt_override: Optional[bool] = None,
) -> Dict[str, Any]:
    validate_manifest(manifest)
    resolved = copy.deepcopy(manifest)
    resolved["source_schema_version"] = manifest["schema_version"]
    resolved["schema_version"] = run_one.SCHEMA_VERSION
    resolved["problem_mode"] = manifest.get("problem_mode", "clrp")
    resolved["manifest_path"] = str(manifest_path.resolve())
    resolved["repo_root"] = str(repo_root)
    resolved["binary"] = str(run_one.resolve_path(manifest["binary"], repo_root))
    resolved["output_root"] = str(
        run_one.resolve_path(output_override or manifest["output_root"], repo_root)
    )
    resolved["instances"] = [
        _normalize_instance(raw, index, repo_root)
        for index, raw in enumerate(manifest["instances"])
    ]
    if workers_override is not None:
        resolved["resources"]["workers"] = workers_override
    if threads_override is not None:
        resolved["resources"]["gurobi_threads"] = threads_override
    if allow_smt_override is not None:
        resolved["resources"]["allow_smt"] = allow_smt_override
    else:
        resolved["resources"].setdefault("allow_smt", False)
    if resolved.get("gurobi_home"):
        resolved["gurobi_home"] = str(
            run_one.resolve_path(resolved["gurobi_home"], repo_root)
        )
    resolved["sp"].setdefault("trigger_offset", "deterministic")
    for field, default in run_one.SP_OPTIONAL_DEFAULTS.items():
        resolved["sp"].setdefault(field, default)
    resolved.setdefault("validation", {})
    resolved["validation"].setdefault("required", False)
    resolved["validation"].setdefault(
        "absolute_tolerance",
        run_one.solution_validator.DEFAULT_ABSOLUTE_TOLERANCE,
    )
    resolved["validation"].setdefault(
        "relative_tolerance",
        run_one.solution_validator.DEFAULT_RELATIVE_TOLERANCE,
    )

    workers = _require_integer(
        resolved["resources"]["workers"],
        "resources.workers",
        1,
        run_one.CPP_INT_MAX,
    )
    gurobi_threads = _require_integer(
        resolved["resources"]["gurobi_threads"],
        "resources.gurobi_threads",
        1,
        run_one.GUROBI_THREADS_MAX,
    )
    allow_smt = resolved["resources"]["allow_smt"]
    if not isinstance(allow_smt, bool):
        raise run_one.ExperimentError("resources.allow_smt must be boolean")
    build_cpu_affinity_pool(
        {
            "workers": workers,
            "gurobi_threads": gurobi_threads,
            "allow_smt": allow_smt,
        },
        run_one.detect_cpu_topology(),
    )

    identifiers = [instance["id"] for instance in resolved["instances"]]
    if len(set(identifiers)) != len(identifiers):
        raise run_one.ExperimentError(
            "instance IDs must be unique because they define run directories"
        )
    return resolved


def expand_runs(
    manifest: Mapping[str, Any], experiment_root: Path
) -> List[Tuple[Dict[str, Any], Path]]:
    runs: List[Tuple[Dict[str, Any], Path]] = []
    for instance in manifest["instances"]:
        for seed in manifest["seeds"]:
            run_id = f"{instance['id']}__seed_{seed}"
            run_dir = experiment_root / "runs" / instance["id"] / f"seed_{seed}"
            sp = dict(manifest["sp"])
            if sp["trigger_offset"] == "deterministic":
                sp["trigger_offset"] = deterministic_trigger_offset(
                    run_id,
                    int(sp["trigger_frequency"]),
                    int(manifest["limits"]["iterations"]),
                )
            config: Dict[str, Any] = {
                "schema_version": run_one.SCHEMA_VERSION,
                "experiment_name": manifest["name"],
                "run_id": run_id,
                "instance_id": instance["id"],
                "problem_mode": manifest["problem_mode"],
                "binary": manifest["binary"],
                "instance": instance["path"],
                "depot_config": instance["depot_config"],
                "solution": str((run_dir / "solution.txt").resolve()),
                "solution_json": str((run_dir / "solution.json").resolve()),
                "seed": seed,
                "best_known": instance["best_known"],
                "is_optimal": instance["is_optimal"],
                "reference": dict(instance["reference"]),
                "stop_at_reference": instance["stop_at_reference"],
                "limits": dict(manifest["limits"]),
                "resources": {
                    "gurobi_threads": manifest["resources"]["gurobi_threads"],
                    "allow_smt": manifest["resources"]["allow_smt"],
                },
                "sp": sp,
                "environment": dict(manifest.get("environment", {})),
                "validation": dict(manifest["validation"]),
                "gurobi_home": manifest.get("gurobi_home"),
                "manifest": {
                    "path": manifest["manifest_path"],
                    "sha256": run_one.sha256_file(
                        Path(manifest["manifest_path"])
                    ),
                },
            }
            runs.append((config, run_dir))
    return runs


def _execute_job(
    job: Tuple[Dict[str, Any], Path],
    repo_root: Path,
    force_successful: bool,
    shared_provenance: Mapping[str, Any],
    cancellation_event: threading.Event,
    affinity_pool: CpuAffinityPool,
    inherited_lock_fds: Sequence[int] = (),
) -> Dict[str, Any]:
    config, run_dir = job
    affinity = affinity_pool.acquire()
    try:
        return run_one.execute_run(
            config,
            run_dir,
            repo_root,
            force_successful=force_successful,
            shared_provenance=shared_provenance,
            cancellation_event=cancellation_event,
            cpu_affinity=affinity,
            affinity_mode=affinity_pool.mode,
            inherited_lock_fds=inherited_lock_fds,
        )
    finally:
        affinity_pool.release(affinity)


def _emit_matrix_status(
    experiment_root: Path,
    jobs: Sequence[Tuple[Dict[str, Any], Path]],
    futures: Mapping[concurrent.futures.Future, str],
    counts: Mapping[str, int],
) -> None:
    """Atomically replace one bounded aggregate status and print one line."""

    future_states = {
        "queued": sum(not future.running() and not future.done() for future in futures),
        "running": sum(future.running() and not future.done() for future in futures),
        "done": sum(future.done() for future in futures),
    }
    health_counts: Dict[str, int] = {}
    total_rss_bytes = 0
    total_cpu_seconds = 0.0
    heartbeat_count = 0
    oldest_heartbeat_age = 0.0
    active_sample: List[str] = []
    now = time.time()
    for config, run_dir in jobs:
        run_path = run_dir / "run.json"
        try:
            run_document = run_one.load_json(run_path)
        except (run_one.ExperimentError, OSError):
            continue
        if run_document.get("status") != "running":
            continue
        if len(active_sample) < MATRIX_STATUS_ACTIVE_SAMPLE_LIMIT:
            active_sample.append(str(config["run_id"]))
        status_name = run_document.get("files", {}).get("process_status")
        if not isinstance(status_name, str):
            continue
        status_path = Path(status_name)
        try:
            status_document = run_one.load_json(status_path)
            age = max(0.0, now - status_path.stat().st_mtime)
        except (run_one.ExperimentError, OSError):
            continue
        heartbeat_count += 1
        oldest_heartbeat_age = max(oldest_heartbeat_age, age)
        health = str(status_document.get("health", "unknown"))
        health_counts[health] = health_counts.get(health, 0) + 1
        rss = status_document.get("rss_bytes")
        cpu = status_document.get("cpu_seconds")
        if isinstance(rss, int) and rss >= 0:
            total_rss_bytes += rss
        if isinstance(cpu, (int, float)) and math.isfinite(float(cpu)):
            total_cpu_seconds += float(cpu)

    status = {
        "schema_version": run_one.SCHEMA_VERSION,
        "updated_at": run_one.utc_now(),
        "total_runs": len(jobs),
        "future_states": future_states,
        "result_counts": dict(counts),
        "active_solver_processes": run_one.active_process_count(),
        "heartbeat_count": heartbeat_count,
        "health_counts": health_counts,
        "oldest_heartbeat_age_seconds": round(oldest_heartbeat_age, 3),
        "aggregate_solver_cpu_seconds": round(total_cpu_seconds, 3),
        "aggregate_solver_rss_bytes": total_rss_bytes,
        "active_run_sample": sorted(active_sample),
        "active_run_sample_truncated": (
            future_states["running"] > MATRIX_STATUS_ACTIVE_SAMPLE_LIMIT
        ),
    }
    run_one.write_json(experiment_root / "matrix.status.json", status)
    print(
        "STATUS "
        f"done={future_states['done']}/{len(jobs)} "
        f"running={future_states['running']} "
        f"queued={future_states['queued']} "
        f"heartbeat={heartbeat_count} "
        f"oldest={oldest_heartbeat_age:.1f}s",
        flush=True,
    )


def validate_input_paths(manifest: Mapping[str, Any]) -> None:
    binary = Path(str(manifest["binary"]))
    if not binary.is_file() or not os.access(binary, os.X_OK):
        raise run_one.ExperimentError(
            f"solver binary is not executable: {binary}"
        )
    for instance in manifest["instances"]:
        if not Path(instance["path"]).is_file():
            raise run_one.ExperimentError(
                f"instance does not exist: {instance['path']}"
            )
        depot_config = instance["depot_config"]
        if depot_config and not Path(depot_config).is_file():
            raise run_one.ExperimentError(
                f"depot configuration does not exist: {depot_config}"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run all instance/seed combinations from a JSON manifest with "
            "bounded parallelism."
        )
    )
    parser.add_argument("manifest", help="JSON experiment manifest")
    parser.add_argument(
        "--repo-root",
        help=(
            "base for relative manifest paths "
            "(default: repository containing this script)"
        ),
    )
    parser.add_argument("--output-root", help="override manifest output_root")
    parser.add_argument("--workers", type=int, help="override parallel workers")
    parser.add_argument(
        "--gurobi-threads", type=int, help="override Gurobi threads per run"
    )
    parser.add_argument(
        "--allow-smt",
        action="store_true",
        default=None,
        help=(
            "explicitly allow separate workers on SMT siblings; by default "
            "workers are limited to physical cores"
        ),
    )
    parser.add_argument(
        "--force-successful",
        action="store_true",
        help="replace successful results instead of skipping them",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="validate and print the run matrix without creating files",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    try:
        run_one.require_project_venv(Path(__file__).resolve().parents[2])
    except run_one.ExperimentError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    arguments = build_parser().parse_args(argv)
    manifest_path = Path(arguments.manifest).expanduser().resolve()
    default_repo_root = Path(__file__).resolve().parents[2]
    repo_root = (
        run_one.resolve_path(arguments.repo_root, Path.cwd())
        if arguments.repo_root
        else default_repo_root
    )

    try:
        original = run_one.load_json(manifest_path)
        resolved = resolve_manifest(
            original,
            manifest_path,
            repo_root,
            arguments.workers,
            arguments.gurobi_threads,
            arguments.output_root,
            arguments.allow_smt,
        )
        experiment_root = (
            Path(resolved["output_root"]) / run_one.safe_identifier(resolved["name"])
        )
        jobs = expand_runs(resolved, experiment_root)
        validate_input_paths(resolved)
        cpu_topology = run_one.detect_cpu_topology()
        affinity_pool = build_cpu_affinity_pool(
            resolved["resources"], cpu_topology
        )
    except run_one.ExperimentError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if arguments.dry_run:
        print(
            f"{len(jobs)} runs; workers={resolved['resources']['workers']}; "
            f"gurobi_threads={resolved['resources']['gurobi_threads']}; "
            f"allow_smt={str(resolved['resources']['allow_smt']).lower()}; "
            f"physical_cores={cpu_topology.physical_core_count}; "
            f"logical_cpus={cpu_topology.logical_cpu_count}"
        )
        for config, run_dir in jobs:
            print(f"{config['run_id']} -> {run_dir}")
            print(f"  {shlex.join(run_one.build_solver_command(config))}")
        return 0

    # Capture source provenance before this invocation writes any artifacts.
    shared_provenance = {
        "git": run_one.collect_git_provenance(repo_root),
        "platform": run_one.collect_platform_provenance(cpu_topology),
        "gurobi": run_one.detect_gurobi(resolved.get("gurobi_home"), repo_root),
    }

    experiment_root.mkdir(parents=True, exist_ok=True)
    try:
        experiment_lock = run_one.CrossProcessFileLock(
            experiment_root / ".matrix.lock",
            f"experiment {resolved['name']}",
        ).acquire()
    except run_one.ExperimentError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    assert experiment_lock.fd is not None
    run_one.write_json(experiment_root / "manifest.original.json", original)
    run_one.write_json(experiment_root / "manifest.resolved.json", resolved)

    counts = {"success": 0, "skipped": 0, "failed": 0}
    failures: List[Dict[str, Any]] = []
    fail_fast = bool(resolved.get("fail_fast", False))
    aborted = False
    interruption: Optional[Dict[str, Any]] = None
    started_at = run_one.utc_now()
    workers = resolved["resources"]["workers"]

    cancellation_event = threading.Event()
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=workers)
    futures: Dict[concurrent.futures.Future, str] = {}
    signal_state = run_one.SignalState()
    with run_one.cancellation_signal_handlers(
        cancellation_event
    ) as signal_state:
        try:
            for job in jobs:
                if cancellation_event.is_set():
                    break
                future = executor.submit(
                    _execute_job,
                    job,
                    repo_root,
                    arguments.force_successful,
                    shared_provenance,
                    cancellation_event,
                    affinity_pool,
                    (experiment_lock.fd,),
                )
                futures[future] = job[0]["run_id"]

            pending_futures = set(futures)
            next_status = time.monotonic()
            while pending_futures:
                if signal_state.signum is not None:
                    aborted = True
                    break
                now = time.monotonic()
                if now >= next_status:
                    _emit_matrix_status(
                        experiment_root, jobs, futures, counts
                    )
                    next_status = now + MATRIX_STATUS_INTERVAL_SECONDS
                wait_timeout = max(
                    0.0,
                    min(
                        MATRIX_STATUS_INTERVAL_SECONDS,
                        next_status - time.monotonic(),
                    ),
                )
                done, pending_futures = concurrent.futures.wait(
                    pending_futures,
                    timeout=wait_timeout,
                    return_when=concurrent.futures.FIRST_COMPLETED,
                )
                for future in done:
                    run_id = futures[future]
                    try:
                        result = future.result()
                    except run_one.ExperimentError as error:
                        counts["failed"] += 1
                        failures.append(
                            {"run_id": run_id, "error": str(error)}
                        )
                        print(f"ERROR {run_id}: {error}", file=sys.stderr)
                        if fail_fast:
                            aborted = True
                            cancellation_event.set()
                        continue
                    except Exception as error:
                        counts["failed"] += 1
                        failures.append(
                            {
                                "run_id": run_id,
                                "error": (
                                    f"{type(error).__name__}: {error}"
                                ),
                            }
                        )
                        print(f"ERROR {run_id}: {error}", file=sys.stderr)
                        if fail_fast:
                            aborted = True
                            cancellation_event.set()
                        continue

                    if result.get("skipped"):
                        counts["skipped"] += 1
                        print(f"SKIP {run_id}")
                    elif result["status"] == run_one.SUCCESS_STATUS:
                        counts["success"] += 1
                        cpu_time = result.get("cpu_time_seconds")
                        timing = (
                            f"{cpu_time:.3f} CPU s"
                            if isinstance(cpu_time, (int, float))
                            else (
                                f"{result['wall_duration_seconds']:.3f} "
                                "wall s"
                            )
                        )
                        print(f"OK   {run_id} ({timing})")
                    else:
                        counts["failed"] += 1
                        failures.append(
                            {
                                "run_id": run_id,
                                "error": result.get("error"),
                            }
                        )
                        print(
                            f"FAIL {run_id}: {result.get('error')}",
                            file=sys.stderr,
                        )
                        if fail_fast:
                            aborted = True
                            cancellation_event.set()
                if aborted:
                    for pending in pending_futures:
                        pending.cancel()
                    run_one.terminate_active_processes()
                    break
        except KeyboardInterrupt:
            aborted = True
            interruption = {
                "signal": "SIGINT",
                "signal_number": int(getattr(signal, "SIGINT", 2)),
            }
            cancellation_event.set()
            print(
                "interrupted; stopping solver process groups",
                file=sys.stderr,
            )
        finally:
            if signal_state.signum is not None and interruption is None:
                aborted = True
                interruption = {
                    "signal": signal_state.name,
                    "signal_number": signal_state.signum,
                }
                print(
                    f"interrupted by {signal_state.name}; stopping solver "
                    "process groups",
                    file=sys.stderr,
                )
            if cancellation_event.is_set():
                for future in futures:
                    future.cancel()
                run_one.terminate_active_processes()
            executor.shutdown(
                wait=True,
                cancel_futures=(aborted or cancellation_event.is_set()),
            )
            # A worker can register a child just after the first cancellation
            # snapshot.  The post-join pass is idempotent and closes that race.
            run_one.terminate_active_processes()

    if signal_state.signum is not None and interruption is None:
        aborted = True
        interruption = {
            "signal": signal_state.name,
            "signal_number": signal_state.signum,
        }
        run_one.terminate_active_processes()

    cleanup_confirmed = run_one.terminate_active_processes()
    active_processes_after_cleanup = run_one.active_process_count()
    cleanup_error = None
    if not cleanup_confirmed:
        aborted = True
        cleanup_error = (
            "one or more solver process groups could not be confirmed stopped "
            "after TERM and KILL"
        )
        print(f"error: {cleanup_error}", file=sys.stderr)

    _emit_matrix_status(experiment_root, jobs, futures, counts)
    summary = {
        "schema_version": run_one.SCHEMA_VERSION,
        "experiment_name": resolved["name"],
        "problem_mode": resolved["problem_mode"],
        "started_at": started_at,
        "finished_at": run_one.utc_now(),
        "total_runs": len(jobs),
        "counts": counts,
        "failures": failures,
        "fail_fast": fail_fast,
        "aborted": aborted,
        "interrupted": interruption is not None,
        "interruption": interruption,
        "process_group_cleanup_confirmed": cleanup_confirmed,
        "process_group_cleanup_error": cleanup_error,
        "active_solver_processes_after_cleanup": (
            active_processes_after_cleanup
            if cleanup_confirmed or active_processes_after_cleanup > 0
            else None
        ),
    }
    run_one.write_json(experiment_root / "summary.json", summary)
    if interruption is not None:
        print(
            "interrupted summary written; "
            + (
                "all registered solver process groups were confirmed stopped"
                if cleanup_confirmed
                else "one or more solver process groups could not be "
                "confirmed stopped after TERM and KILL"
            ),
            file=sys.stderr,
        )
        experiment_lock.close()
        return 128 + int(interruption["signal_number"])
    print(
        f"completed: {counts['success']} successful, "
        f"{counts['skipped']} skipped, {counts['failed']} failed"
    )
    exit_code = 1 if counts["failed"] or not cleanup_confirmed else 0
    experiment_lock.close()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
