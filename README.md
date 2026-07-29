# HGA-SP

[中文](#中文) | [English](#english)

## 中文

### 项目简介

HGA-SP 是一个支持两种显式模式的研究型求解器：

- `clrp`：原 Set S 带容量设施选址—路径问题；
- `clrp-tw`：Farham Set F 的单层、硬时间窗 CLRP 扩展。

当前实现以 HGAMP 路线生成器为核心，维护高质量路径池，并通过“事件驱动的
local SP + 带固定最大间隔保障的 global SP”调用 Gurobi 集合划分
（Set Partitioning, SP）模型重组路线。TW 模式复用相同的数据结构、种群、
mdEAX、路径池和 SP，并在所有路线提交边界独立验证服务时刻。

目标函数包含仓库开启成本、车辆固定成本和路线行驶成本。项目通过统一协调器连接
Route Generator、Route Pool 与 SP，并使用结构化 SP 数据契约隔离 Gurobi 后端。

作者：Feiran Wang<br>
许可证：[MIT License](LICENSE)

### 项目结构

```text
HGA-SP/
├── CMakeLists.txt              # 主构建入口
├── CMakePresets.json           # dev、release-gurobi
├── cmake/                      # Gurobi 查找与 CMake 支持
├── code/
│   ├── core/                   # CLI、Route Generator、Coordinator、实例读取
│   ├── io/                     # Set S / Set F 格式解析
│   ├── problem/                # clrp / clrp-tw 模式
│   ├── evaluation/             # 路线成本、容量与时间窗评估
│   ├── models/                 # Individual、种群与 Route Pool
│   ├── operators/              # mdEAX、修复、局部搜索与变异
│   ├── solver/                 # SP 数据契约与 Gurobi 后端
│   └── utils/                  # 公共工具、命令行与运行日志
├── instances/                  # set_S、set_F 与仓库配置
├── scripts/
│   ├── experiments/            # 单次/矩阵实验、恢复与重试
│   └── analysis/               # 结果汇总工具
├── experiments/                # 实验 manifests、schemas 与说明
├── tests/                      # unit、integration、Python 测试
├── docs/                       # 架构、理论边界与实验协议
├── logs/                       # 直接 CLI 运行日志（生成文件不入库）
├── results/
│   └── set S/                  # Set S 历史原始结果（生成文件不入库）
├── solutions/
│   └── set F/reference_results.csv  # 已校准的 Farham 文献参考值
├── artifacts/                  # 新实验的隔离产物（生成）
├── build/                      # CMake 构建目录（生成）
└── .venv/                      # 项目 Python 虚拟环境（生成）
```

所有 C++ 源码均按职责位于 `code/core`、`code/io`、`code/problem`、
`code/evaluation`、`code/models`、`code/operators`、`code/solver` 和
`code/utils`。

### 构建与 Gurobi

项目面向 Linux，需要 CMake 3.20、支持 C++17 的编译器和 Gurobi。
查找顺序为：

1. `-DGUROBI_ROOT=/path/to/gurobi`；
2. `GUROBI_HOME`；
3. `/opt/gurobi-current`；
4. `/opt/gurobi*` 的版本化安装。

```bash
cmake --preset release-gurobi
cmake --build --preset release-gurobi
ctest --preset release-gurobi
```

可执行文件为 `build/release-gurobi/hgasp`。只运行 Gurobi
集成测试：

```bash
ctest --preset dev -L gurobi
```

兼容入口 `make -C code` 会委托给 CMake。

### 运行

使用外部仓库配置：

```bash
mkdir -p /tmp/set_f_run
build/release-gurobi/hgasp \
  <instance> <depot_config> <solution> \
  <best_value> <is_optimal> <time_limit> <iteration_limit> <seed>
```

不使用外部仓库配置：

```bash
build/release-gurobi/hgasp \
  <instance> <solution> \
  <best_value> <is_optimal> <time_limit> <iteration_limit> <seed>
```

运行 `build/release-gurobi/hgasp --help` 可查看参数说明。

旧命令默认 `clrp`，因此 Set S 调用保持兼容。Farham Set F 必须显式选择
`clrp-tw`：

```bash
build/release-gurobi/hgasp \
  instances/set_F/R101_s1_3_10.txt \
  /tmp/set_f_run/solution.txt \
  2687 1 60 1000 0 \
  --problem-mode clrp-tw
```

mdEAX 默认使用 `--eax-mode paper-compatible` 以保持参考实现兼容。需要启用
父/母边随机选择修正时，显式传入 `--eax-mode corrected`；也可设置
`HGASP_EAX_PARENT_MODE=paper-compatible|corrected`，命令行参数优先。

Set F 的旅行成本与旅行时间均按 `floor(Euclidean distance)` 计算；车辆数不
受限，每条路线支付车辆固定成本。求解器保留 `solution.txt`，并在同目录写出含
原始节点 ID、目标分解和时刻表的 `solution.json`。

### 搜索预算与 SP 调度

`iteration_limit` 是固定的 search-attempt 上限，不会由自适应控制器增减。
一个可行子代、一个被 TW/准入检查拒绝的子代，或一个没有生成子代的 EAX batch
都准确计为一次 attempt。正的 `time_limit` 使用
`search_after_instance_loading` 范围的进程 CPU 时间，并与迭代上限取先到者；
`time_limit=0` 只关闭 CPU 上限，不关闭迭代上限。自适应子种群分配和 mutation
只改变预算内“搜索哪里、如何变异”，不改变总 attempt 数。

SP 默认启用自适应调度：新列、停滞、历史单位 CPU 收益和末段强化可以触发
configuration-local/global SP；`spTriggerFrequency + spTriggerOffset` 定义
global SP 的固定检查点，用作最大间隔保障。检查点在 Master 未变化或 CPU
余量不足时可以跳过实际求解。关闭 adaptive trigger 后，系统才回退到固定
global cadence 和 `spLocalTriggerFrequency` 指定的固定 local cadence。

### 日志管理

直接运行时，程序默认创建：

```text
logs/<instance>/seed_<seed>/<UTC timestamp>_pid_<pid>/
├── hgasp.log
├── events.jsonl
├── algorithm.status.json
└── gurobi.log
```

`hgasp.log` 是供人工阅读的日志，`events.jsonl` 保存结构化事件，
`algorithm.status.json` 原子覆盖当前算法阶段；仅在 SP 实际调用 Gurobi 时
写入 `gurobi.log`。`hgasp.log` 与 `events.jsonl` 默认按 8 MiB 轮转并各保留
两个备份，避免长实验无限增长。相关环境变量：

- `HGASP_LOG_DIR`：指定本次运行的日志目录；
- `HGASP_LOG_CONSOLE=0|1`：关闭或开启终端镜像；
- `HGASP_LOG_LEVEL`：设置结构化事件的最低级别；
- `HGASP_LOG_SEGMENT_BYTES`、`HGASP_LOG_BACKUPS`：设置日志轮转；
- `HGASP_SP_LOG_FILE`：显式指定 Gurobi 日志文件。

实验运行器使用每次尝试独立的
`<run>/logs/attempt_NNN/`，因此重试不会覆盖旧日志，并发 worker
不会共享日志文件。托管运行还由独立 supervisor 原子更新
`process.status.json`，即使矩阵进程被终止也会清理求解器进程组。访问凭据不会
写入 provenance 或日志配置。

### 实验与结果

所有 Python 命令必须使用项目虚拟环境：

```bash
python3 -m venv .venv
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/smoke.json \
  --dry-run
```

新的可复现实验写入 `artifacts/experiments/`，每个实例/seed
均保存 resolved config、运行状态、输入/输出哈希、Git/Gurobi provenance、
solution 和 attempt 级日志。

CLRP-TW 的最小与校准子集分别使用：

```bash
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/set_f_smoke.json

.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/set_f_mvp.json

.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/set_f_tw_sp_smoke.json
```

TW manifest 强制独立验证 `solution.json`；12 个已证明最优的小实例参考值位于
`solutions/set F/reference_results.csv`。无需、也不建议为了验证实现而一次求解
全部 138 个实例。第三个 manifest 将 SP 频率降为 1，专门覆盖
Route Pool → Gurobi SP → 重建/注入链路；它保留已证明最优值作为参考元数据，
但用 `stop_at_reference=false` 关闭“匹配参考值即停止”，并要求日志中实际出现
成功的 SP 结果。它不是性能实验配置。

批量运行器会同时检测进程 affinity、逻辑 CPU、物理核和 SMT 拓扑。默认按
物理核分配并绑定 worker，只有显式设置 `resources.allow_smt=true` 才会使用
SMT sibling；标准配置保持每个 worker 的 Gurobi `Threads=1`。矩阵运行还会为
各实例/seed 生成确定性的 SP 触发偏移，从而错开首次 WLS 取证和后续 SP 求解。
发布实验应在 manifest 中显式固定 worker 数，并记录实际 affinity；是否预留
物理核属于实验协议的一部分，不应从主机总核数或某个历史 manifest 隐式推断。

每个 worker 惰性创建并复用一个 Gurobi 环境。可重试的 WLS/网络错误使用有限
指数退避并重建环境；默认只记录客户端错误，不延长 Gurobi 自动管理的 WLS token
生命周期。这里的 experiment worker 是独立本地进程，不等同于许可证中的
distributed worker；项目不会隐式启用 `DistributedMIPJobs` 或消费远程
`WorkerPool`。

本地历史 Set S 结果位于 `results/set S/`。汇总命令：

```bash
.venv/bin/python scripts/analysis/summarize_results.py
```

将一次实验与 `solutions/set S/` 中的参考解比较，并生成包含最优/平均
gap、平均算法时间和平均墙钟时间的 CSV：

```bash
.venv/bin/python scripts/analysis/compare_benchmarks.py \
  --experiments artifacts/experiments/small_scale_10_runs \
  --benchmarks "solutions/set S" \
  --output artifacts/experiments/small_scale_10_runs/benchmark_comparison.csv
```

详细说明见 [架构文档](docs/architecture.md)、
[理论边界](docs/theory-boundaries.md) 与
[实验协议](docs/experiment_protocol.md)。

---

## English

### Overview

HGA-SP is a research solver with two explicit modes:

- `clrp` for the original Set S capacitated location-routing problem;
- `clrp-tw` for the single-echelon Farham Set F extension with hard time
  windows.

It uses an HGAMP-based route generator, maintains a pool of high-quality
routes, and calls Gurobi set partitioning (SP) through event-driven local
solves plus globally scheduled maximum-interval checkpoints. The TW mode
reuses the same representation, population, mdEAX, route pool, and SP pipeline
while independently validating the schedule at every route admission boundary.

The objective combines depot opening costs, fixed vehicle costs, and route
travel costs. A coordinator connects the Route Generator, Route Pool, and SP
components, while structured SP contracts isolate the Gurobi backend.

Author: Feiran Wang<br>
License: [MIT License](LICENSE)

### Project layout

```text
HGA-SP/
├── CMakeLists.txt              # Main build entry point
├── CMakePresets.json           # dev, release-gurobi
├── cmake/                      # Gurobi discovery and CMake support
├── code/
│   ├── core/                   # CLI, Route Generator, coordinator, input
│   ├── io/                     # Set S / Set F parsers
│   ├── problem/                # clrp / clrp-tw mode
│   ├── evaluation/             # Route cost, capacity, and TW evaluation
│   ├── models/                 # Individual, populations, Route Pool
│   ├── operators/              # mdEAX, repair, local search, mutation
│   ├── solver/                 # SP contracts and Gurobi backend
│   └── utils/                  # Shared utilities, CLI, run logging
├── instances/                  # set_S, set_F, and depot configurations
├── scripts/
│   ├── experiments/            # Isolated single/matrix experiment runners
│   └── analysis/               # Result analysis tools
├── experiments/                # Manifests, schemas, and protocol material
├── tests/                      # Unit, integration, and Python tests
├── docs/                       # Architecture, theory, experiment protocol
├── logs/                       # Direct CLI logs (generated, ignored)
├── results/
│   └── set S/                  # Historical Set S raw results (generated, ignored)
├── solutions/
│   └── set F/reference_results.csv  # Calibrated Farham literature references
├── artifacts/                  # Isolated new experiment artifacts
├── build/                      # CMake build trees
└── .venv/                      # Project Python virtual environment
```

All C++ sources are organized by responsibility under `code/core`, `code/io`,
`code/problem`, `code/evaluation`, `code/models`, `code/operators`,
`code/solver`, and `code/utils`.

### Build and Gurobi

The supported platform is Linux. CMake 3.20, a C++17 compiler, and Gurobi
are required. Discovery order:

1. `-DGUROBI_ROOT=/path/to/gurobi`;
2. `GUROBI_HOME`;
3. `/opt/gurobi-current`;
4. versioned `/opt/gurobi*` installations.

```bash
cmake --preset release-gurobi
cmake --build --preset release-gurobi
ctest --preset release-gurobi
```

The executable is `build/release-gurobi/hgasp`. To run only the Gurobi
integration tests:

```bash
ctest --preset dev -L gurobi
```

The compatibility command `make -C code` delegates to CMake.

### Run

With an external depot configuration:

```bash
mkdir -p /tmp/set_f_run
build/release-gurobi/hgasp \
  <instance> <depot_config> <solution> \
  <best_value> <is_optimal> <time_limit> <iteration_limit> <seed>
```

Without an external depot configuration:

```bash
build/release-gurobi/hgasp \
  <instance> <solution> \
  <best_value> <is_optimal> <time_limit> <iteration_limit> <seed>
```

Run `build/release-gurobi/hgasp --help` for argument details.

The legacy command defaults to `clrp`. Farham Set F requires an explicit
mode:

```bash
build/release-gurobi/hgasp \
  instances/set_F/R101_s1_3_10.txt \
  /tmp/set_f_run/solution.txt \
  2687 1 60 1000 0 \
  --problem-mode clrp-tw
```

mdEAX defaults to `--eax-mode paper-compatible` for reference
compatibility. Use `--eax-mode corrected` to enable unbiased father/mother
edge selection. `HGASP_EAX_PARENT_MODE` provides the equivalent environment
setting, with an explicit command-line option taking precedence.

Set F uses `floor(Euclidean distance)` for both travel cost and travel time.
The fleet is unlimited and each used route pays the fixed vehicle cost. The
solver preserves `solution.txt` and writes a sibling `solution.json` with
original node IDs, objective components, and the service schedule.

### Search budget and SP scheduling

`iteration_limit` is a fixed search-attempt ceiling; adaptive control does not
increase or reduce it. A feasible child, a child rejected by TW/admission
checks, or a zero-child EAX batch each counts as exactly one attempt. A
positive `time_limit` uses process CPU in the
`search_after_instance_loading` scope and stops at whichever limit is reached
first. `time_limit=0` disables only the CPU limit, not the iteration limit.
Adaptive population allocation and mutation choose where and how to search
inside that budget without changing the total attempt count.

SP uses adaptive scheduling by default. New columns, stagnation, historical
improvement per CPU second, and final intensification can dispatch
configuration-local/global SP. `spTriggerFrequency + spTriggerOffset` defines
fixed global checkpoints as a maximum-interval safeguard; an unchanged master
or insufficient remaining CPU may still suppress the actual solve. Disabling
adaptive triggering restores fixed global cadence and the fixed local cadence
given by `spLocalTriggerFrequency`.

### Logging

A direct CLI run creates:

```text
logs/<instance>/seed_<seed>/<UTC timestamp>_pid_<pid>/
├── hgasp.log
├── events.jsonl
├── algorithm.status.json
└── gurobi.log
```

`hgasp.log` is the human-readable stream, `events.jsonl` stores typed events,
and `algorithm.status.json` atomically replaces the current phase snapshot.
`gurobi.log` is used only when SP invokes Gurobi. The human/event logs rotate
at 8 MiB by default and each retain two backups. Configuration:

- `HGASP_LOG_DIR`: explicit directory for this run;
- `HGASP_LOG_CONSOLE=0|1`: disable or enable console mirroring;
- `HGASP_LOG_LEVEL`: minimum structured-event level;
- `HGASP_LOG_SEGMENT_BYTES`, `HGASP_LOG_BACKUPS`: rotation policy;
- `HGASP_SP_LOG_FILE`: explicit Gurobi log path.

The experiment runner uses `<run>/logs/attempt_NNN/`, so retries retain prior
evidence and parallel workers never share log files. Managed runs also have an
independent supervisor atomically update `process.status.json` and clean the
solver process group if the matrix process exits. Access credentials are not
recorded in provenance or log configuration.

### Experiments and results

All Python commands must use the repository virtual environment:

```bash
python3 -m venv .venv
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/smoke.json \
  --dry-run
```

New reproducible runs are stored under `artifacts/experiments/`. Every
instance/seed run records its resolved configuration, state, input/output hashes,
Git/Gurobi provenance, solution, and attempt-specific logs.

Managed runs use an independent solver supervisor. It records a bounded
`process.status.json` heartbeat and removes the solver process group even when
the matrix runner is killed. Concurrent matrix/run locks prevent two processes
from writing the same result directory. The solver emits a rotating
human-readable `hgasp.log`, rotating typed `events.jsonl`, and an atomically
replaced `algorithm.status.json`; normal console output is disabled in managed
runs to avoid storing a second copy in `stdout.log`. Defaults retain at most
three 8 MiB segments for each rotating solver log.

Use `set_f_smoke.json` for one calibrated instance and `set_f_mvp.json` for
the 12 proven-optimal 10-customer instances. The dedicated
`set_f_tw_sp_smoke.json` lowers the trigger frequency to one to exercise the
Route Pool → Gurobi SP → reconstruction/injection path; it is not a performance
profile. It keeps the published optimum as reference metadata but sets
`stop_at_reference=false`, so matching that reference cannot stop the solver
before SP. The profile also fails unless a successful SP result is observed.
CLRP-TW manifests require independent `solution.json` validation;
their reference table is `solutions/set F/reference_results.csv`. Running all
138 files is not required for implementation validation.

The matrix runner detects process affinity, logical CPUs, physical cores, and
SMT topology. By default it pins workers to physical cores; SMT siblings are
used only when `resources.allow_smt=true` is explicit. Standard profiles keep
Gurobi at `Threads=1` per worker. A deterministic per-run SP trigger offset
also spreads initial WLS acquisition and later SP solves. On the current
Publication manifests must state the worker count explicitly and record the
effective affinity. Reserving physical cores is part of the declared
experimental protocol, not something inferred from the host size or a
historical manifest.

Each worker lazily creates and reuses one Gurobi environment. Retryable
WLS/network failures rebuild it with bounded exponential backoff; the defaults
log client errors only and preserve Gurobi's automatic WLS token lifetime.
An experiment worker is an independent local process, not a licensed
distributed worker. The project does not implicitly enable
`DistributedMIPJobs` or consume a remote `WorkerPool`.

Historical Set S results are stored under `results/set S/`. Summarize them with:

```bash
.venv/bin/python scripts/analysis/summarize_results.py
```

Compare an experiment with the reference solutions in `solutions/set S/` and
write best/average gaps plus mean solver and wall-clock times to CSV:

```bash
.venv/bin/python scripts/analysis/compare_benchmarks.py \
  --experiments artifacts/experiments/small_scale_10_runs \
  --benchmarks "solutions/set S" \
  --output artifacts/experiments/small_scale_10_runs/benchmark_comparison.csv
```

See the [architecture](docs/architecture.md),
[theory boundaries](docs/theory-boundaries.md), and
[experiment protocol](docs/experiment_protocol.md) for details.
