# HGA-SP experiments

Experiment definitions are JSON manifests. The runner uses only the Python
standard library and starts the solver with an argument array, not a shell
command. Paths such as `instances/depot configurations/...` therefore remain a
single argument.

## Quick start

Build and test the Gurobi-backed executable first:

```bash
cmake --preset release-gurobi
cmake --build --preset release-gurobi
ctest --preset release-gurobi
```

Create the project-local Python environment once. The runners deliberately
reject base/system Python:

```bash
python3 -m venv .venv
```

Validate and inspect the smoke matrix without running the algorithm:

```bash
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/smoke.json \
  --dry-run
```

Run it:

```bash
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/smoke.json
```

The manifest format is documented by
`experiments/schemas/experiment.schema.json`. Relative binary, instance,
depot-configuration, and output paths are resolved from the repository root.

For a single ad hoc run:

```bash
.venv/bin/python scripts/experiments/run_one.py \
  --binary build/release-gurobi/hgasp \
  --instance instances/set_S/100-5-1c.json \
  --depot-config "instances/depot configurations/S/depot_configuratons_100-5-1c.txt" \
  --output-dir artifacts/experiments/manual/runs/100-5-1c/seed_0 \
  --seed 0 \
  --time-limit 60 \
  --iteration-limit 1000
```

## Matrix and resource behavior

`instances × seeds` defines the run matrix. Every matrix cell has its own
directory:

```text
artifacts/experiments/<name>/
├── manifest.original.json
├── manifest.resolved.json
├── summary.json
└── runs/
    └── <instance-id>/
        └── seed_<seed>/
            ├── config.resolved.json
            ├── run.json
            ├── solution.txt
            └── logs/
                ├── attempt_001/
                │   ├── stdout.log
                │   ├── stderr.log
                │   ├── hgasp.log
                │   └── gurobi.log
                └── attempt_002/ ...
```

Each retry receives a new attempt directory, so earlier failure evidence is
preserved. The runner explicitly assigns unique HGA-SP and Gurobi log paths
for every matrix cell and attempt.

`resources.workers` bounds concurrent processes and
`resources.gurobi_threads` is the per-process CPU budget. The runner rejects a
manifest when:

```text
workers × gurobi_threads > logical CPU count
```

It also pins common OpenMP and numerical-library thread environment variables
to the same per-run value. `resources.gurobi_threads` is passed to the solver
as `HGASP_SP_THREADS`; the current smoke profile uses one.

The required `sp` object makes every matheuristic setting part of the resolved
configuration and its hash. The runner exports it through this explicit
contract:

| Manifest value | Solver environment variable |
|---|---|
| `sp.trigger_frequency` | `HGASP_SP_TRIGGER_FREQUENCY` |
| `sp.time_limit_seconds` | `HGASP_SP_TIME_LIMIT_SECONDS` |
| `sp.mip_gap` | `HGASP_SP_MIP_GAP` |
| `sp.seed` | `HGASP_SP_SEED` |
| `sp.max_pool_size` | `HGASP_SP_MAX_POOL_SIZE` |
| `sp.output_flag` | `HGASP_SP_OUTPUT_FLAG` |
| `sp.mip_focus` | `HGASP_SP_MIP_FOCUS` |
| `sp.heuristics` | `HGASP_SP_HEURISTICS` |
| `sp.representation_route_limit` | `HGASP_SP_REPRESENTATION_ROUTE_LIMIT` |
| `resources.gurobi_threads` | `HGASP_SP_THREADS` |

Boolean settings are encoded as `1` or `0`. These effective values appear in
`run.json` under `environment_overrides`.

## Resume and retry rules

- A successful run is skipped only when its non-empty solution and its binary,
  instance, depot configuration, and resolved-config hashes still match.
- A failed, timed-out, interrupted, or incomplete run is retried in the same
  matrix-cell directory, with `attempt` incremented in `run.json`.
- A successful directory with changed inputs is protected. Use a new
  experiment name, or explicitly pass `--force-successful`.
- Matrix execution continues after an individual failure and returns a
  non-zero status if any run failed.

The external `process_timeout_seconds` is a safety limit for a hung process. It
is separate from `time_seconds`, which is passed to HGA-SP. Set
`time_seconds` to `0` when an experiment must have no global CPU-time limit.

## Provenance

Each `run.json` records UTC start/end times, duration, seed, exact argv, return
code, selected environment overrides, Git commit and dirty state, SHA-256
hashes of all inputs, platform/Python details, and the detected Gurobi version.
Git discovery supports both conventional `.git` metadata and this workspace's
`.git-hga-sp` layout. Credential paths and the complete process environment
are not copied into artifacts.

See `docs/experiment_protocol.md` before producing paper tables.

## Benchmark comparison

`scripts/analysis/compare_benchmarks.py` recursively reads successful
`run.json`/`solution.txt` pairs from an experiment directory, matches instance
IDs with the reference solutions in `solutions/set S/`, and reports the best
and average objective gaps together with mean process CPU and wall-clock
times.

```bash
.venv/bin/python scripts/analysis/compare_benchmarks.py \
  --experiments artifacts/experiments/small_scale_10_runs \
  --benchmarks "solutions/set S" \
  --output artifacts/experiments/small_scale_10_runs/benchmark_comparison.csv
```

The CSV retains both successful and discovered run counts, so failed or
incomplete cells remain visible in the comparison.
