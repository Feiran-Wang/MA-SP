# HGA-SP experiments

Experiment definitions are JSON manifests. Schema v3 supports two explicit
mathematical modes:

- `problem_mode: "clrp"` solves the original Set S CLRP;
- `problem_mode: "clrp-tw"` solves Farham Set F with hard time windows.

CLRP-TW is validated as ordinary CLRP feasibility plus customer time windows,
service times, and depot closing times. Schema v2 manifests remain readable
and are normalized to `problem_mode: "clrp"`; new manifests should use v3.

The runner uses only the Python
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

The corresponding CLRP-TW smoke manifest is:

```bash
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/set_f_smoke.json
```

To exercise the complete Route Pool → SP → reconstruction boundary with an
immediate SP trigger, use the dedicated functional smoke profile:

```bash
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/set_f_tw_sp_smoke.json
```

This profile is for integration coverage, not runtime or solution-quality
comparison. It preserves the published reference as `status: optimal`, while
`stop_at_reference: false` deliberately disables solver early termination so
that a normal run can reach the Route Pool → SP → reconstruction path. The
profile also requires an observed successful SP result before the run may pass.

The Set F smoke/MVP manifests use the 12 published, proven-optimal
`R101`--`R112_s1_3_10` values. Their calibrated distance policy is
`euclidean-floor`: both travel cost and travel time are
`floor(Euclidean distance)`. Larger representative instances retain the typed
reference `{"status": "unknown", "value": null}` until a source-specific
value has been verified.

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

For Farham CLRP-TW, omit the Set S depot-configuration file and select the
mode explicitly:

```bash
.venv/bin/python scripts/experiments/run_one.py \
  --binary build/release-gurobi/hgasp \
  --instance instances/set_F/R101_s1_3_10.txt \
  --problem-mode clrp-tw \
  --output-dir artifacts/experiments/manual/runs/R101_s1_3_10/seed_0 \
  --seed 0 \
  --time-limit 60 \
  --iteration-limit 1000 \
  --best-known 2687 \
  --is-optimal \
  --require-validation
```

## Matrix and resource behavior

`instances × seeds` defines the run matrix. Every matrix cell has its own
directory:

```text
artifacts/experiments/<name>/
├── manifest.original.json
├── manifest.resolved.json
├── summary.json
├── matrix.status.json
└── runs/
    └── <instance-id>/
        └── seed_<seed>/
            ├── config.resolved.json
            ├── run.json
            ├── solution.txt
            ├── solution.json
            └── logs/
                ├── attempt_001/
                │   ├── stdout.log
                │   ├── stderr.log
                │   ├── hgasp.log
                │   ├── events.jsonl
                │   ├── algorithm.status.json
                │   ├── process.status.json
                │   └── gurobi.log
                └── attempt_002/ ...
```

Each retry receives a new attempt directory, so earlier failure evidence is
preserved. The runner explicitly assigns unique HGA-SP and Gurobi log paths
for every matrix cell and attempt. `hgasp.log` and `events.jsonl` are bounded
rotating logs; status JSON files are atomically replaced snapshots rather than
append-only histories.

The solver receives `--problem-mode clrp` or `--problem-mode clrp-tw`
explicitly. Directory names are never used to infer the mathematical problem.

## Independent solution validation

`validation.required=true` is mandatory for `clrp-tw` manifests. In this mode,
a zero solver return code and a non-empty legacy `solution.txt` are not enough
for success. The solver must also write `solution.json` in the run directory,
and `scripts/validation/solution_validator.py` independently reconstructs:

- exact customer coverage;
- vehicle and depot capacity;
- depot, vehicle, and travel objective components;
- for CLRP-TW, waiting, service starts, hard due dates, and depot return time.

The validator reads the original instance and does not import solver
feasibility code. `run.json` records `validation_required`,
`validation_status`, the recomputed cost components, and all diagnostics. Set S
manifests default to non-required validation for compatibility; if a Set S run
writes `solution.json`, it is validated as well.

The validator can also be invoked directly:

```bash
.venv/bin/python scripts/validation/solution_validator.py \
  --problem-mode clrp-tw \
  --instance instances/set_F/R101_s1_3_10.txt \
  --solution artifacts/experiments/set_f_smoke/runs/R101_s1_3_10/seed_0/solution.json
```

`resources.workers` bounds concurrent processes and
`resources.gurobi_threads` is the per-process CPU budget. The runner rejects a
manifest when:

```text
workers × gurobi_threads > affinity-visible physical core count
```

By default, each solver thread is pinned to a distinct physical-core
representative. Set `resources.allow_smt=true` only for an explicit SMT
experiment; that mode uses every physical core once before assigning sibling
hardware threads. Requested and effective affinities, the physical-core
groups, and SMT topology are recorded in `run.json`. Common OpenMP and
numerical-library thread variables are pinned to the same per-run value.
`resources.gurobi_threads` is passed as `HGASP_SP_THREADS`; the standard
profiles keep it at one.

The required `sp` object makes every matheuristic setting part of the resolved
configuration and its hash. The runner exports it through this explicit
contract:

| Manifest value | Solver environment variable |
|---|---|
| `sp.trigger_frequency` | `HGASP_SP_TRIGGER_FREQUENCY` |
| `sp.trigger_offset` | `HGASP_SP_TRIGGER_OFFSET` |
| `sp.time_limit_seconds` | `HGASP_SP_TIME_LIMIT_SECONDS` |
| `sp.local_enabled` | `HGASP_SP_LOCAL_ENABLED` |
| `sp.local_trigger_frequency` | `HGASP_SP_LOCAL_TRIGGER_FREQUENCY` |
| `sp.local_time_limit_seconds` | `HGASP_SP_LOCAL_TIME_LIMIT_SECONDS` |
| `sp.adaptive_trigger_enabled` | `HGASP_SP_ADAPTIVE_TRIGGER_ENABLED` |
| `sp.adaptive_minimum_interval` | `HGASP_SP_ADAPTIVE_MINIMUM_INTERVAL` |
| `sp.adaptive_local_new_columns` | `HGASP_SP_ADAPTIVE_LOCAL_NEW_COLUMNS` |
| `sp.adaptive_local_stagnation` | `HGASP_SP_ADAPTIVE_LOCAL_STAGNATION` |
| `sp.adaptive_global_stagnation` | `HGASP_SP_ADAPTIVE_GLOBAL_STAGNATION` |
| `sp.adaptive_utility_ema_weight` | `HGASP_SP_ADAPTIVE_UTILITY_EMA_WEIGHT` |
| `sp.adaptive_high_utility_threshold` | `HGASP_SP_ADAPTIVE_HIGH_UTILITY_THRESHOLD` |
| `sp.lp_dual_feedback_enabled` | `HGASP_SP_LP_DUAL_FEEDBACK_ENABLED` |
| `sp.lp_time_limit_seconds` | `HGASP_SP_LP_TIME_LIMIT_SECONDS` |
| `sp.persistent_lp_enabled` | `HGASP_SP_PERSISTENT_LP_ENABLED` |
| `sp.solution_pool_size` | `HGASP_SP_SOLUTION_POOL_SIZE` |
| `sp.solution_pool_gap` | `HGASP_SP_SOLUTION_POOL_GAP` |
| `sp.solution_pool_search_mode` | `HGASP_SP_SOLUTION_POOL_SEARCH_MODE` |
| `sp.diverse_injection_count` | `HGASP_SP_DIVERSE_INJECTION_COUNT` |
| `sp.adaptive_search_allocation_enabled` | `HGASP_ADAPTIVE_SEARCH_ALLOCATION_ENABLED` |
| `sp.adaptive_population_exploration` | `HGASP_ADAPTIVE_POPULATION_EXPLORATION` |
| `sp.adaptive_population_diversity_weight` | `HGASP_ADAPTIVE_POPULATION_DIVERSITY_WEIGHT` |
| `sp.adaptive_population_column_weight` | `HGASP_ADAPTIVE_POPULATION_COLUMN_WEIGHT` |
| `sp.adaptive_search_ema_weight` | `HGASP_ADAPTIVE_SEARCH_EMA_WEIGHT` |
| `sp.adaptive_mutation_minimum_percent` | `HGASP_ADAPTIVE_MUTATION_MINIMUM_PERCENT` |
| `sp.adaptive_mutation_maximum_percent` | `HGASP_ADAPTIVE_MUTATION_MAXIMUM_PERCENT` |
| `sp.work_limit` | `HGASP_SP_WORK_LIMIT` |
| `sp.mip_gap` | `HGASP_SP_MIP_GAP` |
| `sp.seed` | `HGASP_SP_SEED` |
| `sp.max_pool_size` | `HGASP_SP_MAX_POOL_SIZE` |
| `sp.output_flag` | `HGASP_SP_OUTPUT_FLAG` |
| `sp.mip_focus` | `HGASP_SP_MIP_FOCUS` |
| `sp.heuristics` | `HGASP_SP_HEURISTICS` |
| `sp.wls_token_duration_minutes` | `HGASP_SP_WLS_TOKEN_DURATION_MINUTES` |
| `sp.wls_token_refresh` | `HGASP_SP_WLS_TOKEN_REFRESH` |
| `sp.client_log_level` | `HGASP_SP_CLIENT_LOG_LEVEL` |
| `sp.server_timeout_seconds` | `HGASP_SP_SERVER_TIMEOUT_SECONDS` |
| `sp.environment_max_attempts` | `HGASP_SP_ENVIRONMENT_MAX_ATTEMPTS` |
| `sp.environment_retry_base_milliseconds` | `HGASP_SP_ENVIRONMENT_RETRY_BASE_MILLISECONDS` |
| `sp.representation_route_limit` | `HGASP_SP_REPRESENTATION_ROUTE_LIMIT` |
| `resources.gurobi_threads` | `HGASP_SP_THREADS` |

Boolean settings are encoded as `1` or `0`. These effective values appear in
`run.json` under `environment_overrides`.

When `sp.adaptive_trigger_enabled=true`, `sp.local_trigger_frequency` is a
declared fallback value and is not the active local cadence. Local dispatch is
instead governed by the adaptive minimum interval, column-novelty threshold,
stagnation threshold, and utility feedback. The fixed local frequency becomes
active only when adaptive triggering is disabled. `sp.trigger_frequency` and
the resolved offset always define scheduled global checkpoints, although the
adaptive controller may skip the actual solve for an unchanged master or
insufficient remaining CPU.

For mdEAX ablations, set
`environment.HGASP_EAX_PARENT_MODE` to `paper-compatible` or `corrected`.
The solver default is `paper-compatible`; do not aggregate both modes as one
algorithm configuration.

Matrix runs default `sp.trigger_offset` to `"deterministic"`. The resolved
integer is derived from the run ID and limited so an iteration-bounded run
keeps the same number of scheduled SP calls. This spreads WLS environment
starts and SP solves without depending on process launch order. A direct
`run_one.py` invocation defaults to offset zero.

`work_limit=0` leaves Gurobi WorkLimit unset. Likewise,
`wls_token_duration_minutes=0` preserves Gurobi's automatic WLS token policy;
the runner does not lengthen short-lived license sessions by default.

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

For Set F, place a `reference_results.csv` in the directory passed through
`--benchmarks`. Its columns are:

```text
instance_id,status,value,lower_bound,source,distance_policy
```

`status` is `unknown`, `bks`, or `optimal`. Unknown is represented by an empty
value, never by numeric zero. Set F IDs are matched exactly, preventing
punctuation-normalization collisions. When a lower bound is present, the
comparison includes both gap-to-reference and gap-to-lower-bound columns.
