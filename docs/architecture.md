# HGA-SP 架构

## 1. 源码布局

HGA-SP 面向 Linux，活动源码按职责划分为：

- `code/core/`：程序入口、实例读取、`AlgorithmOrchestrator` 和
  `RouteGenerator`；
- `code/models/`：`Individual`、种群管理和 `RoutePool`；
- `code/operators/`：初始解、mdEAX、修复、Shaw 变异和局部搜索；
- `code/solver/`：SP 数据契约与 Gurobi 实现；
- `code/utils/`：命令行、通用定义与运行日志。

根目录 `CMakeLists.txt` 显式列出全部编译单元，不使用源码通配符。

## 2. 运行数据流

```text
CLI / Experiment Runner
          |
          v
 AlgorithmOrchestrator
       /    |     \
      v     v      v
 Route Generator  Route Pool  SP Solver
      ^                          |
      +------ validated SP ------+
              injection
```

`AlgorithmOrchestrator` 持有并协调三个核心组件：

1. Route Generator 维护 HGAMP 子种群，执行 mdEAX、容量修复、15% Shaw
   变异和局部搜索；
2. Route Pool 接收可行个体中的路线，执行去重、准入和容量裁剪；
3. SP Solver 从问题数据和路线池快照建立集合划分模型；
4. SP incumbent 经过覆盖、容量、成本和表示限制验证后重建为
   `Individual`；
5. 重建解经过局部搜索并注入种群，其新路线重新进入路径池。

HGA 内部使用链表式 `Individual` 表示。公共 SP 接口只交换
`SpProblem`、`StandaloneRoute`、`SpConfig` 和 `SpResult` 等值对象，不暴露
Gurobi 类型。

SP 按迭代频率和可选错峰偏移触发。频率为 `f`、偏移为 `o` 时，第一次触发发生
在 `f + o`，之后每隔 `f` 次迭代触发一次。偏移必须满足 `0 <= o < f`；
默认偏移为 0，因此单次运行与原有触发序列完全兼容。Linux 批量调度器可以为
不同运行设置 `HGASP_SP_TRIGGER_OFFSET`，避免同批进程在相同迭代点同时进入
环境获取、模型构造和优化阶段。

## 3. SP 模型

对池中路线 `r` 使用二元变量 `x_r`，对仓库 `l` 使用二元变量 `y_l`。模型包含：

- 每个客户恰好被一条所选路线覆盖；
- 每个仓库的所选路线总需求不超过仓库容量；
- 每条路线通过强链接约束 `x_r <= y_l` 连接所属仓库，未开启仓库不能承载路线；
- 目标值由路线行驶成本、每条路线的车辆固定成本和仓库开启成本组成。

HGAMP 的 CLRP 定义采用无限车辆。`representationRouteLimit` 是
`Individual::maxVisit` 固定路线槽位带来的表示兼容约束，不是车队约束；其启用
状态必须进入实验 provenance。

成本矩阵允许非对称，因此路线序列保持方向，除非实例明确声明对称，否则反向路线
不会合并。

## 4. Gurobi 生命周期

每个 Linux 进程惰性创建一个 `GRBEnv`，由该进程中的 SP 后端复用；每次 SP
调用创建一个短生命周期 `GRBModel`。可重试的 WLS/网络错误会触发有限次数的
指数退避并重建环境，永久许可证错误、模型错误和数值错误不会被误重试。并行实验
采用多进程，每个 worker 拥有独立环境、CPU affinity、结果目录和日志。

时限、可选 WorkLimit、MIP gap、线程、seed、日志、输出开关和搜索参数均来自
显式配置。`ThreadLimit` 在环境启动前设置，批量配置默认每个 worker 使用一个
Gurobi 线程。阶段时限覆盖输入验证、环境等待、模型构造和优化，模型只获得剩余
预算。前一次有效 SP 选择在所有路线均可稳定映射并重新验证后，作为完整 MIP Start
提供给下一次同问题求解；不同实例之间不会复用。

SP 返回：

- 归一化状态和 Gurobi 原生状态；
- `SolCount` 与所选路线；
- 可用时的 objective、best bound 和 MIP gap；
- 输入验证、环境、重试等待、建模、优化和后处理时间；
- 模型变量/约束/非零元规模、Work、NodeCount、环境复用与 MIP Start 状态；
- 求解时间、总时间和 Gurobi 版本；
- Gurobi 错误码与诊断文本。

只有 `SolCount > 0` 且独立验证通过的 incumbent 才能注入 HGA。Gurobi
运行配置由 Linux 环境提供，访问凭据不得写入源码、配置、日志或实验结果。

## 5. 日志

直接 CLI 运行使用：

```text
logs/<instance>/seed_<seed>/<timestamp>_pid_<pid>/
├── hgasp.log
└── gurobi.log
```

实验运行器为每次尝试分配：

```text
<run>/logs/attempt_NNN/
├── stdout.log
├── stderr.log
├── hgasp.log
└── gurobi.log
```

每个 worker 和 retry 的日志路径唯一，旧 attempt 不会被覆盖。

## 6. 实验边界

实验自动化位于 `scripts/experiments/`，可提交的 manifest 与生成结果分离。每次
运行记录实例及哈希、resolved config、HGA/Gurobi seed、Git 状态、二进制哈希、
编译器与构建类型、Gurobi 版本、线程与时限、主机信息、起止时间、停止原因和最终
验证结果。

恢复运行时，只有 schema、输入哈希、配置和 solution 均匹配的成功结果才可复用。
