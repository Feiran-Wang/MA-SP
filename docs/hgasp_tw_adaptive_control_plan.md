# HGA-SP for CLRP-TW: log findings and adaptive-control plan

> Historical status: this document records the interrupted pre-P0/P1/P2
> experiment and the design proposal derived from it. The event-tail fix,
> TW-aware mutation, P0--P2 route-pool/SP feedback, adaptive triggering,
> adaptive search allocation, and defensive logging described as proposals
> below have since been implemented. For current behavior, use
> `docs/architecture.md`, `docs/p0_refactor_report.md`,
> `docs/p1_refactor_report.md`, `docs/p2_refactor_report.md`, and
> `docs/global_static_audit_report.md`. Historical measurements and findings
> below are intentionally preserved as evidence, not as a description of the
> current executable.

This note is based on the frozen, interrupted logs under
`artifacts/experiments/set_f_b2t_table7_2runs_strict`. It does not treat the
logged incumbents as validated final solutions.

## 1. What the interrupted run establishes

The strict batch contains 14 runs (seven instances and two seeds). At
termination, the mean logged iteration was 127,929, with a range of
120,000--141,000 out of the requested 300,000 iterations.

| Instance | B2T | HGA-SP seed 0 | HGA-SP seed 1 | Best gap | Average gap |
|---|---:|---:|---:|---:|---:|
| C102_s3_5_50 | 7615.2 | 7589 | 7589 | -0.3440% | -0.3440% |
| C103_s3_5_50 | 7617.0 | 7583 | 7586 | -0.4464% | -0.4267% |
| C104_s3_5_50 | 7622.4 | 7580 | 7577 | -0.5956% | -0.5759% |
| C109_s3_5_50 | 7606.9 | 7603 | 7590 | -0.2222% | -0.1367% |
| C103_s4_5_50 | 7636.6 | 7590 | 7590 | -0.6102% | -0.6102% |
| C104_s4_5_50 | 7642.0 | 7584 | 7587 | -0.7590% | -0.7393% |
| C109_s4_5_50 | 7614.4 | 7600 | 7598 | -0.2154% | -0.2022% |

The macro best gap is -0.4561%, and the macro run-average gap is -0.4336%.
All 14 logged candidates are below the published B2T values. These values are
candidate snapshots only: the processes did not finish, no solution files were
written, and no independent route-level validation was performed.

## 2. SP contribution and cost

The frozen logs contain 21 completed SP calls; one additional call had emitted
`SP MONITOR` but was interrupted before returning a result, so it is excluded:

- 7 `OPTIMAL` and 14 `TIME_LIMIT`;
- every call returned a validated SP incumbent;
- no Gurobi, licence, or connection error was logged;
- the first SP improved its pre-SP checkpoint in 9/14 runs and tied it in 5/14;
- the first calls reduced the direct SP objective by 51 cost units in
  4,717.1 wall seconds;
- the seven second calls improved only once, by 2 cost units, and used
  2,396.8 wall seconds;
- six non-improving second calls consumed 2,036.8 wall seconds;
- direct objective gain per SP second was about 13 times higher on the first
  call than on the second call.

Model construction is not the bottleneck: all 21 builds used only 1.575 seconds
in total, while optimization used 7,111.917 seconds. A persistent incremental
model is therefore lower priority than better call selection, a smaller
effective route set, and an adaptive search budget.

The first SP can also create useful HGA trajectories. For example, one
C103_s3 seed moved from a logged HGA incumbent of 7642 to an SP incumbent of
7614; its next progress checkpoint (after injection/local search and fewer than
1,000 further HGA attempts) was 7595, and it later reached 7586. The log
granularity cannot separate those contributions exactly, but it supports a
post-SP HGA incubation phase. SP should remain a mandatory intensification
component; the desired change is not an HGA-only fallback.

## 3. Why the interrupted timing data is not publishable

In the frozen interrupted experiment, the HGA progress field `Time comsuming`
was process CPU time while SP phase fields were wall time. Subtracting those
historical values or calculating an SP share from them is invalid.

New runs use process CPU for scientific performance, with scope stated rather
than conflated. `solution.txt` supplies the benchmark time with scope
`search_after_instance_loading`; SP phase fields use process CPU for their own
phase scope. `run.json.cpu_time_seconds`/`duration_seconds` instead measure the
entire child lifecycle via `wait4` and are operational diagnostics, not a
replacement for search CPU. The runner also keeps `wall_duration_seconds`, and
SP keeps native Gurobi/runtime wall fields, solely for external timeout,
network, license, and cleanup diagnosis. Gurobi `TimeLimit` necessarily
remains a wall safety cap; deterministic SP effort should be compared through
`WorkLimit` and reported work units. Literature time gaps are left undefined
unless clock basis, scope, and hardware are explicitly compatible.

There was also an orchestration failure:

- an earlier non-strict matrix and the strict matrix were both left running;
- the external runner was terminated before its own timeout cleanup;
- both matrices left 14 orphan `hgasp` processes, for 28 simultaneous solver
  processes in total;
- this exceeded the intended global 20-worker cap and contaminated wall-time
  measurements;
- all 28 orphan processes were subsequently terminated explicitly.

The worker limit is currently per matrix, not global. A publishable timing run
requires one active matrix, a global worker lease, confirmed affinity, exact
per-child CPU accounting, and normal runner finalization.

## 4. Search-quality limitations in the current implementation

1. **Exact-iteration events can be lost.** SP, restart, and population
   reduction are checked only after a feasible child. An event due on a zero or
   rejected TW child is skipped permanently.
2. **Equal-cost structural diversity is rejected.** Population admission
   treats any individual with the same objective as a clone, even if its edges
   and routes differ. Integer-distance CLRP-TW instances can have many useful
   equal-cost alternatives.
3. **TW has no mutation branch.** The CLRP-TW refinement path rebuilds and runs
   local search but bypasses the mutation used by the CLRP path.
4. **Route admission is solution-level.** Routes are exported only from
   solutions better than a historical mean. A worse complete solution can
   still contain a new, valuable SP column.
5. **Pool pruning is not SP-aware.** Ranking by `load / travelCost` does not
   protect rare-customer coverage, incumbent routes, previous MIP-start routes,
   depot balance, temporal tightness, or route novelty.
6. **TW uses one free population.** Depot-configuration subpopulations are
   disabled for temporal safety. This avoids excluding feasible assignments,
   but it also removes a major diversity mechanism from the original CLRP HGA.
7. **SP feedback is incomplete.** The controller cannot currently observe
   whether SP reconstruction, post-SP local search, population admission, or
   newly exported routes produced the actual gain.
8. **The current HGA incumbent is not the SP MIP start.** The first SP calls all
   report `mip_start=false`; later calls reuse only the previous SP selection.
   Pinning and mapping the current HGA global solution would give every call a
   valid incumbent and a directly comparable improvement baseline.

## 5. Proposed adaptive HGA-SP controller

Use an auditable feedback controller before considering a learned policy. The
available interrupted sample is too small to train a reliable bandit or
regression policy.

### 5.1 State

At a common search-attempt boundary, collect:

- iterations and process CPU seconds since global improvement;
- accepted, rejected-TW, and zero-child rates over a rolling window;
- population broken-pairs diversity;
- route-pool version, effective new columns, duplicate rate, coverage by
  customer/depot, and protected-route retention;
- current incumbent;
- last SP objective, post-LS objective, bound, gap, work, CPU time, status,
  and accepted/injected outcome;
- SP CPU budget used, Gurobi work used, and remaining total CPU budget;
- last restart time and its subsequent reward.

Public BKS/OPT values must not be controller inputs. They are for stopping rules
or post-run evaluation only.

### 5.2 State machine

```text
HGA_EXPLORE
  | stagnation + useful new columns, or hard maximum interval
  v
SP_PROBE -> FULL_SP -> POST_SP_EXPLOIT
  | no useful restricted-master potential
  | or stale route pool
  v
HGA_DIVERSIFY / RESTART -> HGA_EXPLORE
```

- **HGA_EXPLORE:** accumulate diverse feasible columns.
- **SP_PROBE:** solve the LP relaxation of the curated restricted master. If
  its optimum is not below the current incumbent (within tolerance), the
  integer restricted master cannot improve it, so skip the expensive MIP.
- **FULL_SP:** solve the integer restricted master with a call-local budget.
- **POST_SP_EXPLOIT:** after an accepted SP solution, give EAX/local search a
  protected interval in which to exploit the new route combination.
- **DIVERSIFY/RESTART:** when HGA is stagnant and the pool is stale, diversify
  before asking SP to solve essentially the same restricted master again.

The SP request must be latched: once due, it executes at the next safe boundary
and cannot be lost because one child is rejected.

### 5.3 Trigger invariants

Run SP when all readiness conditions hold and either the adaptive or hard
condition fires:

```text
ready =
    warm-up complete
    and every customer has protected route coverage
    and the current incumbent routes are present

adaptive_due =
    HGA stagnation >= threshold
    and effective new-column ratio >= threshold
    and minimum SP cooldown elapsed

due = ready and (adaptive_due or maximum SP interval elapsed)
```

The hard maximum interval and at least one mandatory full SP call guarantee
that the algorithm never silently degenerates to HGA.

### 5.4 Budget feedback

Evaluate call `k` using the post-injection result:

\[
r_k =
\frac{\max(0,z_k^- - z_k^+)}
     {z_k^-\,t_k^{SP}},
\]

where \(z_k^-\) is the incumbent before SP and \(z_k^+\) is the best validated
incumbent after reconstruction and local search.

- Preserve a generous first-SP budget while initial evidence shows it is
  productive.
- After a zero-reward call, shorten the next budget, lengthen cooldown, and
  require more effective new columns.
- After a productive call, retain or increase the next budget and reset the
  stagnation state.
- Use Gurobi `WorkLimit` for reproducible effort and `TimeLimit` only as a hard
  wall cap.
- Stop proving the restricted master unnecessarily: this SP is an
  intensification heuristic, so use an improving objective target and/or a
  post-incumbent patience callback instead of always pursuing a 0.1% MIP gap.
- Enforce an SP CPU-budget share, initially tested around 10--15%, but let the
  hard maximum interval override it to preserve mandatory SP behavior.

## 6. Route-pool changes

Maintain a full HGA route archive and derive a smaller SP core pool.

Always protect:

- every current-incumbent route;
- routes selected by the previous SP MIP start;
- minimum coverage quotas for each customer and depot;
- recently admitted effective columns.

Apply a safe SP dominance rule first: for the same depot and exactly the same
customer set, a route with higher travel cost has identical master
coefficients and is dominated by the cheaper route. Then stratify the remaining
core by cost, load, route length, time-window slack/waiting, depot, recency, and
coverage scarcity. The full archive can remain available to HGA even when the
SP core is smaller.

## 7. Implementation phases

### Phase 0: reliability and instrumentation

- atomically checkpoint and independently validate every new global incumbent;
- use process CPU time for HGA, SP phases, stopping, and comparisons; retain
  monotonic wall time only for external safety timeouts and diagnostics;
- add outcome counters for EAX, TW reconstruction, LS, population admission,
  route admission, restart, and SP injection;
- handle `SIGINT`, `SIGTERM`, and `SIGHUP` in the matrix runner, terminate every
  owned process group, and finalize `run.json`;
- enforce a workspace-wide worker lease capped at 20;
- abort immediately on configured Gurobi/SP errors.

### Phase 1: correct event semantics

- move all per-attempt bookkeeping to one common loop tail;
- latch SP/restart/reduction events;
- add structured `OffspringOutcome`, `RouteAdmissionOutcome`,
  `SpInjectionOutcome`, and `RestartOutcome`;
- retain the current fixed policy as a selectable comparison mode.

### Phase 2: adaptive trigger and budget

- implement `AdaptiveSpController`;
- add call-local SP model overrides;
- pin and map the current HGA global solution as the SP MIP start;
- add minimum/maximum intervals, stagnation, effective novelty, LP probe,
  reward feedback, and an SP budget ledger;
- guarantee at least one full SP and a hard maximum interval;
- fail rather than continue as HGA after a fatal SP/Gurobi error.

### Phase 3: solve-quality improvements

- admit route-level novelty rather than filtering solely by whole-solution
  fitness;
- preserve equal-cost but structurally different individuals;
- add schedule-safe CLRP-TW mutation/diversification;
- implement protected, coverage-aware SP core-pool selection;
- only then investigate a temporal-aware depot-configuration population.

## 8. Required ablations

Every compared variant must retain SP:

1. current fixed trigger + fixed 360-second SP;
2. latched fixed trigger (event-semantics correction);
3. adaptive trigger + fixed SP budget;
4. adaptive trigger + adaptive budget;
5. full controller + protected core pool;
6. full controller + TW diversity changes.

Compare anytime quality at common process-CPU limits, validated best/average gaps,
time to B2T, primal integral, SP gain per work unit, SP budget share,
effective-column yield, and restart reward. The interrupted two-seed run is a
pilot only; final average-gap and timing claims require the planned three
completed, independently validated runs per instance.
