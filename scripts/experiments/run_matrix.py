#!/usr/bin/env python3
"""Expand and execute a JSON HGA-SP experiment manifest."""

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import json
import math
import os
import shlex
import sys
import threading
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import run_one


def _require_integer(
    value: Any,
    field: str,
    minimum: Optional[int] = None,
    maximum: Optional[int] = None,
) -> int:
    return run_one.require_integer(value, field, minimum, maximum)


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
            "name",
            "binary",
            "output_root",
            "gurobi_home",
            "instances",
            "seeds",
            "limits",
            "resources",
            "sp",
            "environment",
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
    if manifest.get("schema_version") != run_one.SCHEMA_VERSION:
        raise run_one.ExperimentError(
            f"schema_version must be {run_one.SCHEMA_VERSION}"
        )
    for field in ("name", "binary", "output_root"):
        if not isinstance(manifest.get(field), str) or not manifest[field]:
            raise run_one.ExperimentError(f"{field} must be a non-empty string")
    if run_one.safe_identifier(manifest["name"]) != manifest["name"]:
        raise run_one.ExperimentError(
            "name must contain only letters, digits, '.', '_', or '-'"
        )
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
            ("id", "path", "depot_config", "best_known", "is_optimal"),
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
        if "best_known" in instance and (
            isinstance(instance["best_known"], bool)
            or not isinstance(instance["best_known"], (int, float))
            or not math.isfinite(float(instance["best_known"]))
        ):
            raise run_one.ExperimentError(
                f"instances[{index}].best_known must be finite and numeric"
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
        ("workers", "gurobi_threads"),
        ("workers", "gurobi_threads"),
    )
    _validate_keys(
        sp,
        "sp",
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
        run_one.CPP_INT_MAX,
    )
    run_one.validate_sp_config(sp)
    run_one.validate_environment_mapping(manifest.get("environment", {}))


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
    value["best_known"] = float(value.get("best_known", 0.0))
    value["is_optimal"] = bool(value.get("is_optimal", False))
    value["manifest_index"] = index
    return value


def resolve_manifest(
    manifest: Dict[str, Any],
    manifest_path: Path,
    repo_root: Path,
    workers_override: Optional[int],
    threads_override: Optional[int],
    output_override: Optional[str],
) -> Dict[str, Any]:
    validate_manifest(manifest)
    resolved = copy.deepcopy(manifest)
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
    if resolved.get("gurobi_home"):
        resolved["gurobi_home"] = str(
            run_one.resolve_path(resolved["gurobi_home"], repo_root)
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
        run_one.CPP_INT_MAX,
    )
    cpu_count = run_one.available_cpu_count()
    if workers * gurobi_threads > cpu_count:
        raise run_one.ExperimentError(
            f"workers ({workers}) * gurobi_threads ({gurobi_threads}) = "
            f"{workers * gurobi_threads}, exceeding {cpu_count} logical CPUs"
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
            config: Dict[str, Any] = {
                "schema_version": run_one.SCHEMA_VERSION,
                "experiment_name": manifest["name"],
                "run_id": run_id,
                "instance_id": instance["id"],
                "binary": manifest["binary"],
                "instance": instance["path"],
                "depot_config": instance["depot_config"],
                "solution": str((run_dir / "solution.txt").resolve()),
                "seed": seed,
                "best_known": instance["best_known"],
                "is_optimal": instance["is_optimal"],
                "limits": dict(manifest["limits"]),
                "resources": {
                    "gurobi_threads": manifest["resources"]["gurobi_threads"]
                },
                "sp": dict(manifest["sp"]),
                "environment": dict(manifest.get("environment", {})),
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
) -> Dict[str, Any]:
    config, run_dir = job
    return run_one.execute_run(
        config,
        run_dir,
        repo_root,
        force_successful=force_successful,
        shared_provenance=shared_provenance,
        cancellation_event=cancellation_event,
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
        )
        experiment_root = (
            Path(resolved["output_root"]) / run_one.safe_identifier(resolved["name"])
        )
        jobs = expand_runs(resolved, experiment_root)
        validate_input_paths(resolved)
    except run_one.ExperimentError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    if arguments.dry_run:
        print(
            f"{len(jobs)} runs; workers={resolved['resources']['workers']}; "
            f"gurobi_threads={resolved['resources']['gurobi_threads']}"
        )
        for config, run_dir in jobs:
            print(f"{config['run_id']} -> {run_dir}")
            print(f"  {shlex.join(run_one.build_solver_command(config))}")
        return 0

    # Capture source provenance before this invocation writes any artifacts.
    shared_provenance = {
        "git": run_one.collect_git_provenance(repo_root),
        "platform": run_one.collect_platform_provenance(),
        "gurobi": run_one.detect_gurobi(resolved.get("gurobi_home"), repo_root),
    }

    experiment_root.mkdir(parents=True, exist_ok=True)
    run_one.write_json(experiment_root / "manifest.original.json", original)
    run_one.write_json(experiment_root / "manifest.resolved.json", resolved)

    counts = {"success": 0, "skipped": 0, "failed": 0}
    failures: List[Dict[str, Any]] = []
    started_at = run_one.utc_now()
    workers = resolved["resources"]["workers"]

    cancellation_event = threading.Event()
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=workers)
    futures: Dict[concurrent.futures.Future, str] = {}
    try:
        futures = {
            executor.submit(
                _execute_job,
                job,
                repo_root,
                arguments.force_successful,
                shared_provenance,
                cancellation_event,
            ): job[0]["run_id"]
            for job in jobs
        }
        for future in concurrent.futures.as_completed(futures):
            run_id = futures[future]
            try:
                result = future.result()
            except run_one.ExperimentError as error:
                counts["failed"] += 1
                failures.append({"run_id": run_id, "error": str(error)})
                print(f"ERROR {run_id}: {error}", file=sys.stderr)
                continue
            except Exception as error:  # keep the remaining matrix running
                counts["failed"] += 1
                failures.append(
                    {
                        "run_id": run_id,
                        "error": f"{type(error).__name__}: {error}",
                    }
                )
                print(f"ERROR {run_id}: {error}", file=sys.stderr)
                continue

            if result.get("skipped"):
                counts["skipped"] += 1
                print(f"SKIP {run_id}")
            elif result["status"] == run_one.SUCCESS_STATUS:
                counts["success"] += 1
                print(
                    f"OK   {run_id} "
                    f"({result['duration_seconds']:.3f}s)"
                )
            else:
                counts["failed"] += 1
                failures.append(
                    {"run_id": run_id, "error": result.get("error")}
                )
                print(
                    f"FAIL {run_id}: {result.get('error')}",
                    file=sys.stderr,
                )
    except KeyboardInterrupt:
        cancellation_event.set()
        for future in futures:
            future.cancel()
        run_one.terminate_active_processes()
        executor.shutdown(wait=True, cancel_futures=True)
        print(
            "interrupted; running solver processes were stopped",
            file=sys.stderr,
        )
        return 130
    else:
        executor.shutdown(wait=True)

    summary = {
        "schema_version": run_one.SCHEMA_VERSION,
        "experiment_name": resolved["name"],
        "started_at": started_at,
        "finished_at": run_one.utc_now(),
        "total_runs": len(jobs),
        "counts": counts,
        "failures": failures,
    }
    run_one.write_json(experiment_root / "summary.json", summary)
    print(
        f"completed: {counts['success']} successful, "
        f"{counts['skipped']} skipped, {counts['failed']} failed"
    )
    return 1 if counts["failed"] else 0


if __name__ == "__main__":
    sys.exit(main())
