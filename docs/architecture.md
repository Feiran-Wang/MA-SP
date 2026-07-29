# HGA-SP 架构

## 1. 源码布局

HGA-SP 面向 Linux，活动源码按职责划分为：

- `code/core/`：程序入口、实例读取、`AlgorithmOrchestrator` 和
  `RouteGenerator`；
- `code/io/`：Set S 与 Farham Set F 的格式解析和统一输入 DTO；
- `code/problem/`：显式 `clrp` / `clrp-tw` 问题模式；
- `code/evaluation/`：共享路线成本、容量和时间可行性评估；
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

1. Route Generator 维护 HGAMP 种群；`clrp` 模式执行原 mdEAX、容量修复、
   15% Shaw 变异和原局部搜索；
2. `clrp-tw` 模式复用种群、mdEAX、路径池和 SP，但在初始构造及每次 mdEAX
   后通过共享路线评估器执行可行插入/重建；按同一 15% 概率执行时空相关
   Shaw-style removal/recreate mutation，再采用仅提交完整 TW 可行候选的
   relocate 局部搜索；
3. 每个完整可行候选在种群准入判断之前提交路线。Route Pool 分为保留有序
   路线遗传材料的 Search Archive，以及供 SP 使用的 Master Column Pool；
   后者按 `(仓库, 客户集合)` 合并列并精确保留成本最低的可行访问顺序。TW
   模式下每条路线在入池时独立重算时间表；
4. SP Solver 从问题数据和路线池快照建立集合划分模型；
5. SP incumbent 经过覆盖、容量、成本、时间窗和表示限制验证后重建为
   `Individual`；
6. 重建解经过相应模式的局部搜索并注入种群，其新路线重新进入路径池。

HGA 内部使用链表式 `Individual` 表示。公共 SP 接口只交换
`SpProblem`、`StandaloneRoute`、`SpConfig` 和 `SpResult` 等值对象，不暴露
Gurobi 类型。

SP 默认采用混合触发策略。频率 `f` 和偏移 `o` 定义的是 global SP 的固定
检查点：第一个检查点在 `f + o`，之后每隔 `f` 次迭代出现。检查点会被 latch
到统一事件尾部，但自适应控制器仍可在 Master 没有新列或剩余 CPU 不足时跳过
实际求解。除此之外，控制器可根据新列数量、停滞、历史单位 CPU 收益和末段强化
提前触发 configuration-local 或 global SP。只有关闭 adaptive 模式后，
`spLocalTriggerFrequency` 才恢复为固定 local SP 周期。偏移必须满足
`0 <= o < f`；批量调度器可以设置 `HGASP_SP_TRIGGER_OFFSET`，错开同批进程的
环境获取、模型构造和优化阶段。

每个搜索 attempt（可行子代、TW 拒绝子代或 zero-EAX）都进入同一个事件尾部：

```text
推进 iteration 并 latch 固定 global 检查点
  -> 停止条件预检查
  -> 自适应决策并执行可用的 local/global SP
  -> 统一结算 non-improvement
  -> 停止条件复检查
  -> restart 与子种群缩减检查
  -> 最终停止检查
```

因此子代结果不会再跳过预定 SP 或 restart。SP 请求在安全边界被显式消费一次，
不会因中间控制流分支丢失。

## 3. SP 模型

对池中路线 `r` 使用二元变量 `x_r`，对仓库 `l` 使用二元变量 `y_l`。模型包含：

- 每个客户恰好被一条所选路线覆盖；
- 每个仓库的所选路线总需求不超过仓库容量；
- 每条路线通过强链接约束 `x_r <= y_l` 连接所属仓库，未开启仓库不能承载路线；
- 目标值由路线行驶成本、每条路线的车辆固定成本和仓库开启成本组成。

HGAMP 的 CLRP 定义采用无限车辆。`Individual::maxVisit` 使用严格安全的有限
表示界：

```text
min(n, z + floor(Q_d / q_min))
```

其中 `z` 是零需求客户数，`Q_d` 是仓库容量，`q_min` 是最小正需求；全零需求时
安全回退到 `n`，存储层至少分配一个空槽。任意非空路线要么包含一个不同的零需求
客户，要么至少消耗 `q_min` 的仓库容量，因此该界对 CLRP 与 CLRP-TW 都成立，
不是车队约束。SP 中对应的冗余约束默认关闭；即使为兼容实验显式开启，也不会
收缩原问题可行域。

成本矩阵允许非对称，因此路线序列保持方向，除非实例明确声明对称，否则反向路线
不会在 Search Archive 中合并。若同仓库的两个有向路线覆盖相同客户集合，它们
在 SP 中的所有约束系数相同，Master Column Pool 只保留行驶成本更低者；这是
精确列支配，不依赖成本矩阵对称性。

mdEAX 的初始 AB-cycle 父/母边选择提供两个显式模式：

- `paper-compatible`（默认）保持参考实现的随机数消耗和父边选择行为；
- `corrected` 在父边和母边都可用时按随机位等概率选择。

可通过 `--eax-mode` 或 `HGASP_EAX_PARENT_MODE` 选择，模式必须进入实验
provenance，二者的结果不能在未标注时混合汇总。

### 3.1 P1 双层 RMP 协同

每次 global RMP 调用前，当前 HGA 全局最优解中的路线都会重新提交并按
`(仓库, 客户集合)` 映射到当前 Master Column Pool。这个完整解优先作为 MIP
Start；只有无法映射时才回退到上一次 SP incumbent。映射后仍执行精确覆盖、
仓库容量和目标值复验。

路径池裁剪分为四层：

1. Tier 0：初始可行 backbone、当前 HGA 全局最优、上一次 SP 解和各子种群
   elite；
2. Tier 1：优先保留覆盖替代较少客户的列，并尽量为每个客户保留两个候选；
3. Tier 2：为每个仍有候选列的仓库保留至少一条路线；
4. Tier 3：按装载/成本效率填满剩余预算。

Tier 0 可以暂时超过名义裁剪目标，以保证已知完整可行覆盖不会因容量裁剪丢失。

SP 分为两个独立求解器状态：

- configuration-local RMP 仅使用目标子种群已开启仓库的列，并把所有仓库变量
  固定到该配置；使用该配置 elite 作为 MIP Start；
- global RMP 使用完整 Master Column Pool，保留跨配置重组和新配置发现能力。

local RMP 使用较短独立时限。其结果不执行可能改变仓库配置的通用后处理局部搜索，
从而保证反馈解仍属于所求配置。

自适应控制器根据 Master Column Pool 版本增量、非改进迭代数、末段强化状态、
固定 global 事件和剩余 CPU 预算选择 `none/local/global`。控制器维护：

```text
EMA(objective improvement / SP CPU seconds)
```

若近期 SP 没有产生改进，则提高下一次 local RMP 所需的新列门槛；若单位 CPU
收益高，则降低门槛。固定 global cadence 仍作为最大间隔保证，但在主列完全未
变化时可以有理由地跳过。关闭 adaptive 模式后，恢复固定 global cadence 与固定
local frequency。

### 3.2 P2 双向反馈与资源分配

每次整数 SP 前可求解连续 LP-RMP。LP-RMP 返回客户精确覆盖对偶值、仓库容量
对偶值以及当前每条主列的 reduced cost。路线 reduced cost 直接进入路径池
分层排序；客户对偶值进入 mutation，使相关移除的起点优先从高对偶客户的受限
候选表中抽取。对偶值不加入完整解目标，因为每个完整解恰好覆盖全部客户一次，
该对偶和对所有完整解都是常数。

LP-RMP 在同一 global/local 求解器内持久化：新列增量加入，被裁剪列以 `UB=0`
停用，再次出现时重新启用。问题定义或 Gurobi environment generation 改变时
完整重建。整数 SP 仍按当前列池快照构造，防止历史停用列进入注入解。

整数 SP 使用 solution pool 返回多个已独立验证的近优解。注入器先选择目标值
最好的解，再按路线集合 Jaccard 距离执行 greedy max-min，最多注入配置给定的
`K` 个结构不同解。global SP 可系统搜索 top-K；高频 local SP 使用较轻量的
opportunistic pool。

HGA 子种群采用 UCB 风格预算：

```text
EMA(目标改进 / CPU)
+ exploration
+ diversity weight
+ EMA(新增主列 / CPU)
```

未尝试配置优先获得探索机会。mutation 概率在显式上下界内根据停滞、种群
broken-pairs 多样性、mutation 历史收益和近期 SP 改进自适应调整；局部搜索仍
保持必做，不会因预算控制退化为纯 HGA。

### 3.3 CLRP-TW 列语义

Farham Set F 的弧成本与旅行时间均为
`floor(Euclidean distance)`；服务时间在到达客户后单独加入，等待不进入目标。
`RouteEvaluator` 从仓库时刻 0 开始传播最早服务时刻，并验证客户 due time 与
返回仓库 close time。

SP 主问题不增加连续时间变量。原因是 Farham 模型中不同车辆之间没有时间资源
耦合：入池列已经固定仓库、客户顺序和一张完整可行时刻表，SP 对这些列的选择不会
改变任何列内时刻。SP 仍只负责精确覆盖、仓库容量、开启联动和三项成本；所有
TW 列在入池和 SP 导入两个信任边界分别复验。

两种模式的每个候选仓库均分配 `numV` 个路线槽。这个上界同时允许时间窗迫使
所有客户使用单客户路线的极端情形。

## 4. Gurobi 生命周期

每个 Linux 进程惰性创建一个 `GRBEnv`，由该进程中的 SP 后端复用；global 与
local 求解器各自保存一个增量 LP-RMP，整数 SP 每次使用短生命周期
`GRBModel`。可重试的 WLS/网络错误会触发有限次数的
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
- 输入验证、环境、重试等待、建模、优化和后处理的进程 CPU 时间；
- Gurobi 原生 runtime、重试休眠与 SP 总墙钟时间（仅安全诊断）；
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
├── events.jsonl
├── algorithm.status.json
├── process.status.json
└── gurobi.log
```

`hgasp.log` 和 `events.jsonl` 有界轮转；两个 status 文件采用原子覆盖，不随
迭代追加。每个 worker 和 retry 的日志路径唯一，旧 attempt 不会被覆盖。

## 6. 实验边界

实验自动化位于 `scripts/experiments/`，可提交的 manifest 与生成结果分离。每次
运行记录实例及哈希、resolved config、HGA/Gurobi seed、Git 状态、二进制哈希、
编译器与构建类型、Gurobi 版本、线程与时限、主机信息、起止时间、停止原因和最终
验证结果。

恢复运行时，只有 schema、输入哈希、配置和 solution 均匹配的成功结果才可复用。

算法时限、解文件、SP 阶段与 benchmark 表统一使用进程 CPU 时间。Linux runner
通过 `wait4(pid)` 为每个 solver 子进程独立记录 user/system CPU；墙钟只保留为
外部超时、网络/WLS 和清理诊断。runner 捕获 `SIGINT`、`SIGTERM`、`SIGHUP`
后协作取消所有 worker，对每个已登记的独立进程组执行 TERM/KILL 收敛并等待
`run.json` 与矩阵 `summary.json` 落盘。即使组长正常退出，runner 也会在注销前
确认同一 PGID 不再有活动后代；TERM/KILL 后仍无法确认时，该 run 必须记为
`cleanup_failed`，矩阵也不得报告活动进程数为零或成功退出。
