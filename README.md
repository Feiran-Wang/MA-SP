# HGA-SP

[中文](#中文) | [English](#english)

## 中文

### 项目简介

HGA-SP 是一个面向带容量设施选址—路径问题（Capacitated
Location-Routing Problem, CLRP）的研究型求解器。当前实现以 HGAMP
路线生成器为核心，维护高质量路径池，并周期性调用 Gurobi
集合划分（Set Partitioning, SP）模型重组路线。

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
│   ├── models/                 # Individual、种群与 Route Pool
│   ├── operators/              # mdEAX、修复、局部搜索与变异
│   ├── solver/                 # SP 数据契约与 Gurobi 后端
│   └── utils/                  # 公共工具、命令行与运行日志
├── instances/                  # CLRP 实例与仓库配置
├── scripts/
│   ├── experiments/            # 单次/矩阵实验、恢复与重试
│   └── analysis/               # 结果汇总工具
├── experiments/                # 实验 manifests、schemas 与说明
├── tests/                      # unit、integration、Python 测试
├── docs/                       # 架构、理论边界与实验协议
├── logs/                       # 直接 CLI 运行日志（生成文件不入库）
├── results/
│   └── set S/                  # Set S 历史原始结果（生成文件不入库）
├── artifacts/                  # 新实验的隔离产物（生成）
├── build/                      # CMake 构建目录（生成）
└── .venv/                      # 项目 Python 虚拟环境（生成）
```

所有 C++ 源码均按职责位于 `code/core`、`code/models`、`code/operators`、
`code/solver` 和 `code/utils`。

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

### 日志管理

直接运行时，程序默认创建：

```text
logs/<instance>/seed_<seed>/<UTC timestamp>_pid_<pid>/
├── hgasp.log
└── gurobi.log
```

`hgasp.log` 镜像程序的标准输出和错误输出；仅在 SP 实际调用 Gurobi
时写入 `gurobi.log`。相关环境变量：

- `HGASP_LOG_DIR`：指定本次运行的日志目录；
- `HGASP_LOG_CONSOLE=0|1`：关闭或开启终端镜像；
- `HGASP_SP_LOG_FILE`：显式指定 Gurobi 日志文件。

实验运行器使用每次尝试独立的
`<run>/logs/attempt_NNN/`，因此重试不会覆盖旧日志，并发 worker
不会共享日志文件。访问凭据不会写入 provenance 或日志配置。

### 实验与结果

所有 Python 命令必须使用项目虚拟环境：

```bash
python3 -m venv .venv
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/smoke.json \
  --dry-run
```

新的可复现实验写入 `artifacts/experiments/`，每个实例/seed
均保存 resolved config、运行状态、输入哈希、Git/Gurobi provenance、
solution 和 attempt 级日志。

批量运行器会同时检测进程 affinity、逻辑 CPU、物理核和 SMT 拓扑。默认按
物理核分配并绑定 worker，只有显式设置 `resources.allow_smt=true` 才会使用
SMT sibling；标准配置保持每个 worker 的 Gurobi `Threads=1`。矩阵运行还会为
各实例/seed 生成确定性的 SP 触发偏移，从而错开首次 WLS 取证和后续 SP 求解。
当前 48 逻辑 CPU / 24 物理核环境下，完整 `100-5-XX` manifest 使用 20 个
worker，并为系统与文件/网络服务保留 4 个物理核。

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

HGA-SP is a research solver for the Capacitated Location-Routing Problem
(CLRP). It uses an HGAMP-based route generator, maintains a pool of
high-quality routes, and periodically calls a Gurobi set-partitioning (SP)
model to recombine those routes.

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
│   ├── models/                 # Individual, populations, Route Pool
│   ├── operators/              # mdEAX, repair, local search, mutation
│   ├── solver/                 # SP contracts and Gurobi backend
│   └── utils/                  # Shared utilities, CLI, run logging
├── instances/                  # CLRP instances and depot configurations
├── scripts/
│   ├── experiments/            # Isolated single/matrix experiment runners
│   └── analysis/               # Result analysis tools
├── experiments/                # Manifests, schemas, and protocol material
├── tests/                      # Unit, integration, and Python tests
├── docs/                       # Architecture, theory, experiment protocol
├── logs/                       # Direct CLI logs (generated, ignored)
├── results/
│   └── set S/                  # Historical Set S raw results (generated, ignored)
├── artifacts/                  # Isolated new experiment artifacts
├── build/                      # CMake build trees
└── .venv/                      # Project Python virtual environment
```

All C++ sources are organized by responsibility under `code/core`,
`code/models`, `code/operators`, `code/solver`, and `code/utils`.

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

### Logging

A direct CLI run creates:

```text
logs/<instance>/seed_<seed>/<UTC timestamp>_pid_<pid>/
├── hgasp.log
└── gurobi.log
```

`hgasp.log` mirrors standard output and standard error. `gurobi.log` is used
only when SP invokes Gurobi. Configuration:

- `HGASP_LOG_DIR`: explicit directory for this run;
- `HGASP_LOG_CONSOLE=0|1`: disable or enable console mirroring;
- `HGASP_SP_LOG_FILE`: explicit Gurobi log path.

The experiment runner uses `<run>/logs/attempt_NNN/`, so retries retain prior
evidence and parallel workers never share log files. Access credentials are
not recorded in provenance or log configuration.

### Experiments and results

All Python commands must use the repository virtual environment:

```bash
python3 -m venv .venv
.venv/bin/python scripts/experiments/run_matrix.py \
  experiments/manifests/smoke.json \
  --dry-run
```

New reproducible runs are stored under `artifacts/experiments/`. Every
instance/seed run records its resolved configuration, state, input hashes,
Git/Gurobi provenance, solution, and attempt-specific logs.

The matrix runner detects process affinity, logical CPUs, physical cores, and
SMT topology. By default it pins workers to physical cores; SMT siblings are
used only when `resources.allow_smt=true` is explicit. Standard profiles keep
Gurobi at `Threads=1` per worker. A deterministic per-run SP trigger offset
also spreads initial WLS acquisition and later SP solves. On the current
48-logical-CPU/24-physical-core host, the full `100-5-XX` manifest uses 20
workers and leaves four physical cores for the OS and file/network services.

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
