# Experiment protocol

This protocol defines the minimum reproducibility standard for HGA-SP
computational studies. The automation entry points are
`scripts/experiments/run_one.py` and `scripts/experiments/run_matrix.py`.

## 1. Freeze the study definition

Create a new JSON manifest under `experiments/manifests/` for each benchmark,
parameter study, or ablation. Give it a new stable `name`; never reuse a
published experiment name for a different configuration.

The manifest must fix:

- the exact executable;
- ordered instance metadata and optional depot configurations;
- all random seeds;
- solver time and iteration limits;
- whether SP is enabled, its trigger policy, pool limit, and complete Gurobi
  parameter set;
- process-level safety timeout, when used;
- maximum worker count and Gurobi threads per worker;
- non-secret environment overrides.

All relative paths are interpreted from the repository root, regardless of the
caller's current directory. Instance IDs must be unique because they determine
artifact directories.

## 2. Validate the solver and environment

Before launching a matrix:

```bash
cmake --preset release-gurobi
cmake --build --preset release-gurobi
ctest --preset release-gurobi
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/smoke.json \
  --dry-run
```

If `.venv` does not yet exist, create it once with
`python3 -m venv .venv`. The experiment entry points reject base/system Python
and unrelated virtual environments.

The CTest Gurobi smoke test is the authoritative runtime check. The
experiment runner separately detects and records the Gurobi version from its
installation header or `gurobi_cl --version`; version discovery itself does
not solve a model.

Use a clean Git worktree for publication runs. A dirty tree is permitted for
development experiments, but is explicitly recorded. The runner recognizes
both `.git` and `.git-hga-sp` metadata directories.

## 3. Control resources

One matrix worker is one HGA-SP process. The manifest is rejected when:

```text
workers × gurobi_threads > os.cpu_count()
```

Common OpenMP and BLAS thread variables are pinned to `gurobi_threads` to
reduce hidden oversubscription. The runner sends this value through
`HGASP_SP_THREADS`, together with all values from the manifest's `sp` object
through named `HGASP_SP_*` variables. These exact effective overrides are
recorded in `run.json`. For comparable paper results, keep the same CPU model,
worker count, Gurobi thread count, solver version, and limits across variants.

The SP seed is independent from the HGA run seed. Declare both explicitly.

`limits.time_seconds` and `limits.iterations` are algorithm inputs. A
`time_seconds` value of `0` disables the global CPU-time limit; the iteration
limit remains active.
`limits.process_timeout_seconds` is an optional outer guard. If used, it
should exceed the algorithm time limit enough to cover initialization,
checkpoint/output work, and normal shutdown.

## 4. Run and resume

Launch a declared study with:

```bash
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/<study>.json
```

The runner expands the Cartesian product of instances and seeds and uses a
dedicated directory for every cell. It invokes HGA-SP via `subprocess.Popen`
with a list of arguments and `shell=False`; depot paths containing spaces are
safe.

A run is operationally successful only when the process exits with code zero
and writes a non-empty `solution.txt`. This does not replace mathematical
solution validation. Before publishing results, independently verify customer
coverage, vehicle capacity, depot capacity, and the recomputed objective.

Rerunning the same command:

- skips a successful cell whose solution and input hashes still match;
- retries failed, timed-out, interrupted, and incomplete cells;
- refuses to overwrite a successful cell whose configuration or inputs
  changed, unless `--force-successful` is explicitly supplied.

Prefer a new experiment name over `--force-successful` so previous evidence
remains intact.

## 5. Preserve raw evidence

Each run directory contains:

- `config.resolved.json`: absolute paths and all effective per-run values;
- `run.json`: status, timing, seed, exact command, hashes, platform, Git, and
  Gurobi provenance;
- `solution.txt`: the solver result;
- `logs/attempt_NNN/stdout.log` and `stderr.log`: separate process streams;
- `logs/attempt_NNN/hgasp.log`: the solver's combined run log;
- `logs/attempt_NNN/gurobi.log`: the isolated SP log when Gurobi is invoked.

Every retry receives the next `attempt_NNN` directory. Previous failed,
timed-out, or interrupted logs are never truncated.

At experiment level, preserve `manifest.original.json`,
`manifest.resolved.json`, and `summary.json`. Do not edit these artifacts by
hand. Archive the entire experiment directory before aggregation.

SHA-256 values are recorded for the executable, instance, depot
configuration, and resolved run config. The full environment is deliberately
not captured because it may contain credentials; only declared and
runner-controlled overrides are stored.

## 6. Analyze without mutating raw runs

Analysis scripts should read run directories and write derived tables or plots
elsewhere. At minimum, report:

- number of planned, successful, skipped, failed, and timed-out runs;
- objective distribution by instance and algorithm variant;
- seed count and treatment of failures;
- wall-clock and iteration stopping rules;
- Gurobi version and per-run thread setting;
- Git commit and whether any included run was dirty.

For Set S experiments, generate the benchmark comparison with:

```bash
.venv/bin/python scripts/analysis/compare_benchmarks.py \
  --experiments artifacts/experiments/small_scale_10_runs \
  --benchmarks "solutions/set S" \
  --output artifacts/experiments/small_scale_10_runs/benchmark_comparison.csv
```

The CSV reports discovered and successful run counts, best and average
objectives, reference objective, best and average percentage gaps, mean solver
time, and mean wall-clock time.

Never silently discard failures or substitute extra seeds. Any excluded run
and rerun policy must be stated in the paper's computational protocol.
