# Experiment protocol

This protocol defines the minimum reproducibility standard for HGA-SP
computational studies. The automation entry points are
`scripts/experiments/run_one.py` and `scripts/experiments/run_matrix.py`.

## 1. Freeze the study definition

Create a new JSON manifest under `experiments/manifests/` for each benchmark,
parameter study, or ablation. Give it a new stable `name`; never reuse a
published experiment name for a different configuration.

The manifest must fix:

- the explicit mathematical `problem_mode` (`clrp` or `clrp-tw`);
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
workers × gurobi_threads > affinity-visible physical core count
```

The runner leases non-overlapping physical-core representatives and makes each
solver process inherit its assigned affinity. SMT siblings are excluded unless
`resources.allow_smt=true` is declared; in that mode all physical cores are
used once before any sibling is allocated. Common OpenMP and BLAS thread
variables are pinned to `gurobi_threads` to reduce hidden oversubscription.
The runner sends this value through
`HGASP_SP_THREADS`, together with all values from the manifest's `sp` object
through named `HGASP_SP_*` variables. These exact effective overrides are
recorded in `run.json`, along with requested/effective CPU affinity and the
detected logical/physical/SMT topology. For comparable paper results, keep the
same CPU model, worker count, Gurobi thread count, solver version, and limits
across variants. Platform provenance includes an auditable `hardware_identity`
(machine, normalized CPU model, affinity-visible physical-core count, and
threads per core) plus its stable SHA-256 `hardware_id`.

The SP seed is independent from the HGA run seed. Declare both explicitly.

The solver accepts `HGASP_SP_TRIGGER_OFFSET` for deterministic SP staggering.
For trigger frequency `f`, the offset must satisfy `0 <= offset < f`; the first
scheduled global checkpoint occurs at iteration `f + offset`, followed by
checkpoints every `f` iterations. With adaptive triggering enabled, a
checkpoint may be skipped when the master is unchanged or the remaining CPU
budget is insufficient, and local/global SP may also be dispatched earlier by
novelty or stagnation. The direct-run offset default is `0`. Matrix manifests
default to `"deterministic"`: the runner derives a stable offset from the run
ID and caps it using the iteration-limit remainder, so staggering does not
reduce the scheduled checkpoint count. Every resolved integer is stored in
`config.resolved.json`.

Each process lazily creates one Gurobi environment and reuses it across its SP
calls. Retryable WLS/network acquisition failures rebuild that environment
with bounded exponential backoff; ordinary model, numerical, and permanent
license errors are not retried. Keep `sp.client_log_level=1` to retain WLS
error evidence, and leave `sp.wls_token_duration_minutes=0` unless the study
explicitly needs to override Gurobi's automatic token lifetime.

WLS also requires DNS plus outbound HTTPS access to
`token.gurobi.com:443`. A restricted execution sandbox must therefore provide
either a domain-level allow rule or a working `HTTPS_PROXY`; otherwise the
license handshake fails before optimization. Run the Gurobi-labelled CTest
gate in a network-enabled execution context. Such a failure is fatal: HGA-SP
does not silently continue as HGA-only.

`sp.time_limit_seconds` is accounted across environment acquisition, model
construction, and optimization; the model receives only the remaining budget,
and the client network timeout is capped to it. A blocking network call can
still return slightly after that budget, so `limits.process_timeout_seconds`
remains the outer safety guard. A positive `sp.work_limit` adds a deterministic
Gurobi work-unit limit; zero leaves it disabled.

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
and writes a non-empty `solution.txt`. For `clrp-tw`,
`validation.required=true` is mandatory: the run is successful only when the
sibling `solution.json` also passes independent coverage, vehicle/depot
capacity, objective, customer due-time, service-time, waiting, and
return-to-depot validation. Set S keeps validation optional for legacy
workflows, but publication runs should enable or separately perform the same
coverage/capacity/objective checks.

Dedicated SP integration tests may additionally set `sp.require_success=true`.
The runner then scans only that attempt's `stdout.log` and `hgasp.log` and
requires both an `[SP Result] status=` record and an `[SP Success]` record;
otherwise the run fails with `process_status=sp_assertion_failed`. This switch
is intended for functional coverage, not performance studies. When a published
optimum would stop such a test during initialization, keep the typed
`reference.status=optimal` metadata and set `stop_at_reference=false`; the
effective C++ early-stop flag is then zero without falsifying the reference.

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
  Gurobi provenance. On POSIX, the per-run supervisor reaps the exact solver
  with `wait4(pid)` and publishes its user/system CPU totals in
  `process.status.json`; the runner copies those solver totals into
  `cpu_user_seconds`, `cpu_system_seconds`, and `cpu_time_seconds`.
  `duration_seconds` is this operational solver CPU total and
  `wall_duration_seconds` is retained only for timeout and cleanup diagnosis.
  Supervisor CPU is never substituted for solver CPU, and neither operational
  field replaces the scientific search time;
- `solution.txt`: the solver result and its publication timing sample. New
  outputs explicitly declare `process_cpu` and the
  `search_after_instance_loading` scope. This clock starts after instance
  parsing and covers HGA-SP setup/search plus final validation up to the
  sample. It is the only run time used by benchmark performance aggregation;
- `solution.json`: machine-readable routes, original IDs, objective
  decomposition, and (for TW) service schedule;
- `logs/attempt_NNN/stdout.log` and `stderr.log`: supervisor/bootstrap streams.
  Managed runs set `HGASP_LOG_CONSOLE=0`, so normal solver output is not
  duplicated into `stdout.log`;
- `logs/attempt_NNN/hgasp.log`: the bounded human-readable solver log;
- `logs/attempt_NNN/events.jsonl`: bounded typed JSON events for lifecycle,
  HGA progress, SP dispatch/solve/injection, path-pool pruning, and defensive
  warnings;
- `logs/attempt_NNN/algorithm.status.json`: an atomically replaced snapshot of
  the current algorithm phase;
- `logs/attempt_NNN/process.status.json`: an atomically replaced supervisor
  heartbeat containing PID/PGID/start ticks, solver process CPU, RSS, process
  state, cleanup reason, and health;
- `logs/attempt_NNN/gurobi.log`: the isolated SP log when Gurobi is invoked.

`hgasp.log` and `events.jsonl` rotate at 8 MiB by default and retain two prior
segments. A message is limited to 4 KiB, structured field counts and status
payloads are bounded, and credentials in named fields are redacted.
`algorithm.status.json`, `process.status.json`, and experiment-level
`matrix.status.json` are overwritten atomically rather than appended. Log I/O
or disk-full failures disable only the affected sink and emit at most one
warning to the original stderr; they do not silently change the optimization
method. Every retry receives the next `attempt_NNN` directory, so previous
evidence is preserved while every individual attempt remains bounded.

At experiment level, preserve `manifest.original.json`,
`manifest.resolved.json`, and `summary.json`. Do not edit these artifacts by
hand. Archive the entire experiment directory before aggregation.

The runners translate `SIGINT`, `SIGTERM`, and `SIGHUP` into cooperative
cancellation. Every solver is owned by a small, single-threaded supervisor and
runs in its own process group. A private control pipe connects the matrix/run
process to that supervisor: normal cancellation closes the pipe, and an
uncatchable runner `SIGKILL` produces the same EOF automatically. The
supervisor then sends `SIGTERM` to the solver group, waits one shared bounded
grace interval, escalates remaining members with `SIGKILL`, and reaps adopted
descendants. Linux additionally uses a parent-death signal for the solver
leader. Experiment and run `flock` descriptors are inherited by supervisors,
so a second invocation cannot write the same artifact tree while cleanup is
still active.

Catchable interruption writes interrupted `run.json` files and the experiment
`summary.json`; a hard-killed runner cannot write its own final summary, but
the independent `process.status.json` records parent-control EOF and cleanup.
The same group-quiescence check runs after an ordinary solver exit, so a
background descendant cannot outlive a successful cell unnoticed. If cleanup
cannot be confirmed after TERM/KILL, the cell is marked `cleanup_failed` and
neither the cell nor matrix may report success. TERM/KILL deadlines are shared
across all workers, avoiding cleanup latency proportional to worker count.

SHA-256 values are recorded for the executable, instance, depot
configuration, resolved run config, `solution.txt`, and (when present)
`solution.json`. Resume accepts a successful cell only when the current output
hashes still match; required CLRP-TW solutions are independently revalidated
before being skipped. The full environment is deliberately not captured
because it may contain credentials; only declared and runner-controlled
overrides are stored.

## 6. Analyze without mutating raw runs

Analysis scripts should read run directories and write derived tables or plots
elsewhere. At minimum, report:

- number of planned, successful, skipped, failed, and timed-out runs;
- objective distribution by instance and algorithm variant;
- seed count and treatment of failures;
- search-scoped process CPU-time and iteration stopping rules. Whole-child
  `wait4` CPU and wall-clock are operational diagnostics only;
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
objectives, reference objective, best and average percentage gaps, mean
search-scoped process CPU, whole-child operational CPU, and wall-clock time.
It never substitutes the whole-child `wait4` measurement for the search clock.
Legacy solutions without explicit basis/scope retain their objectives but
their ambiguous timing is excluded.

For the calibrated Set F MVP subset, use the typed reference table:

```bash
.venv/bin/python scripts/analysis/compare_benchmarks.py \
  --experiments artifacts/experiments/set_f_mvp \
  --benchmarks "solutions/set F" \
  --output artifacts/experiments/set_f_mvp/benchmark_comparison.csv
```

Do not replace an unknown Set F reference with numeric zero. Use
`status=unknown` and an empty value until the exact instance/distance-policy
result is sourced. The current Set F policy is `euclidean-floor`.

`reference_results.csv` timing is descriptive unless the row declares
`time_basis`, `time_scope`, and `hardware_id`. A time gap or speedup is emitted
only when those declarations match the run basis, scope, and hardware exactly.
Published times from different or unspecified hardware therefore remain
visible as reference metadata but do not produce a pseudo-precise time gap.

Never silently discard failures or substitute extra seeds. Any excluded run
and rerun policy must be stated in the paper's computational protocol.
