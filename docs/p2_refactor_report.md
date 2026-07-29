# HGA-SP P2 改造报告

日期：2026-07-28

## 1. 范围与结论

本轮完成四项 P2 改造：

1. LP-RMP 与路径级对偶反馈；
2. 持久化增量 LP 主问题；
3. Top-K solution pool 与多样化 SP 注入；
4. 自适应子种群与 mutation 预算分配。

四项功能均可由显式配置关闭，并分别具有独立测试。完整回归和一个 Farham Set F
端到端冒烟运行通过。现有证据证明模型同步、对偶映射、多解选择与资源控制按设计
运行；是否改善 Farham benchmark 的 best/avg gap 和 CPU 时间仍需配对消融。

## 2. P2-1：LP-RMP 与路径级对偶反馈

### 2.1 连续主问题

整数 SP 前求解相同受限主问题的连续松弛，记录：

- 客户精确覆盖约束的 dual；
- 仓库容量约束的 dual；
- 当前每条主列的 reduced cost；
- LP 目标值、同步 CPU 时间和优化 CPU 时间。

反馈向量严格按该次 `solve()` 的路线快照索引。尺寸或实例客户数不匹配时拒绝
导入，避免将旧问题对偶错误地用于新问题。

### 2.2 反馈使用

- Master Column Pool 按 `(depot, customer set)` 保存路线 reduced cost；
- 分层裁剪在稀缺性和普通层中优先保留 reduced cost 较小的列；
- mutation 保存客户 dual，Shaw/TW-aware mutation 的第一个移除客户从最高
  dual 四分位的受限候选表随机抽取；
- 对偶值不加入完整解适应度。

只有 global LP-RMP 的 dual 会更新全局 RoutePool 和 mutation guidance。
configuration-local LP 的 dual 属于固定仓库配置的不同受限主问题，仅保留在该次
求解结果/日志中，不再覆盖全局反馈。

最后一点是必要的：任何完整可行解都恰好覆盖每个客户一次，因此客户对偶和
`sum_i pi_i` 对所有完整解相同，直接加入完整解目标不会改变排序。

### 2.3 独立验证

`route_pool.dual_feedback` 验证：

- 客户 dual 完整保存；
- 路线反馈按仓库和客户集合映射，与访问顺序及旧路线成本无关；
- 错误尺寸的 dual 向量被拒绝。

`sp.known_optimum` 验证 LP 目标、客户 dual、仓库 dual 和所有路线 reduced cost
均在真实 Gurobi LP 求解后返回。

## 3. P2-2：持久化增量 LP-RMP

### 3.1 生命周期

global 与 local SPSolver 各维护一个连续主问题：

- 首次调用构造仓库变量与基础约束；
- 新客户集合列通过 `GRBColumn` 增量加入；
- 被路径池裁剪的历史列设置 `UB=0`；
- 再次出现的历史列重新启用；
- dominance 改进后的同身份列更新目标系数与保存顺序；
- 相同模型不主动 reset，使 Gurobi 可以继续利用已有 LP 状态；
- 当历史列数超过当前活跃快照的两倍时自动重建压缩，并记录压缩前/后列数；
- 问题定义或 Gurobi environment generation 改变时完整重建。

整数 SP 仍按当前路径池快照建立短生命周期模型。这样历史 LP 列只能提供连续
反馈，无法意外进入当前整数解。

### 3.2 Provenance

每次 SP 日志记录：

```text
lp_persistent_reused
lp_columns_added
lp_columns_reactivated
lp_columns_disabled
lp_columns_stored
lp_columns_active
lp_build
lp_optimize
```

### 3.3 独立验证

`sp.known_optimum` 连续执行：

1. 首次建立 6 列；
2. 相同快照再次求解，确认复用且没有增删；
3. 加入一个新列，确认只增量添加 1 列；
4. 恢复原快照，确认只停用 1 个历史列。

四次求解均保持已知整数最优值 11。

长序列同步测试还验证 inactive history 超过阈值后触发压缩，stored column 数重新
受当前活跃快照约束，避免持久模型随运行时间无界增长。

## 4. P2-3：Top-K 多样化 SP 注入

### 4.1 候选获取与验证

整数模型显式设置：

```text
PoolSolutions
PoolGap
PoolSearchMode
```

global SP 默认使用系统 top-K 搜索，local SP 使用 opportunistic solution pool。
每个候选解分别复验：

- 路线索引范围；
- 客户精确覆盖；
- 仓库容量；
- 固定仓库配置；
- 路线表示安全界；
- 重算目标值。

任何未经验证的候选都不会注入种群。

### 4.2 多样化选择

首先选择目标值最好的候选。之后按路线集合 Jaccard 距离执行 greedy max-min：

```text
arg max_s min_{q in selected} distance(s, q)
```

目标值和原 solution-pool 顺序用于确定性 tie-break。最多尝试注入
`sp.diverse_injection_count` 个解；每个解都经过正常路径提交和种群结构重复
判定。遥测分别报告 attempted、实际 admitted 和 rejected，不再把“已尝试”误记
为“已进入种群”。

### 4.3 独立验证

`sp.diverse_solution_selector` 验证：

- 最好解始终第一个；
- 第二个选择与当前解路线集合完全不相交的候选；
- 后续选择满足 max-min 距离；
- 空候选和非法解被拒绝。

`sp.known_optimum` 验证 solution-pool 主解与 Gurobi incumbent 一致。

## 5. P2-4：自适应搜索预算

### 5.1 子种群预算

每个仓库配置维护：

```text
EMA(objective improvement / process CPU)
EMA(new master columns / process CPU)
number of attempts
```

选择分数为：

```text
improvement EMA
+ UCB exploration
+ diversity_weight * broken-pairs diversity
+ column_weight * column EMA
```

未尝试子种群优先获得预算。跨子种群父代选择排除第一父代所在配置，只要仍有其他
非空配置。

仓库配置替换占用旧槽位或全局 restart 时，对应 arm 状态会重置；一次双父本
EAX 的改进与新列 credit 在两个父本之间各分配 50%，避免把全部收益错误归给
第一父本。

### 5.2 Mutation 预算

mutation 概率以 HGAMP 的 15% 为中心，在显式最小/最大值内根据以下信号调整：

- 全局停滞比例；
- 当前子种群 broken-pairs 多样性；
- mutation commit 与单位 CPU 改进 EMA；
- 最近一次 SP 是否改善 incumbent。

停滞或低多样性提高探索；SP 刚产生改进时短期降低 mutation 以强化新结构。
local search 保持必做，因此该控制器不会把 HGA-SP 退化成 HGA 或关闭强化步骤。

### 5.3 独立验证

`search.adaptive_allocator` 验证：

- 未尝试子种群依次获得探索；
- 高单位 CPU 收益配置随后获得 exploitation；
- 停滞和低多样性提高 mutation 概率；
- SP 改进后 mutation 概率下降；
- 概率始终满足配置上下界。

## 6. 配置

新增 manifest 字段：

```text
sp.lp_dual_feedback_enabled
sp.lp_time_limit_seconds
sp.persistent_lp_enabled
sp.solution_pool_size
sp.solution_pool_gap
sp.solution_pool_search_mode
sp.diverse_injection_count
sp.adaptive_search_allocation_enabled
sp.adaptive_population_exploration
sp.adaptive_population_diversity_weight
sp.adaptive_population_column_weight
sp.adaptive_search_ema_weight
sp.adaptive_mutation_minimum_percent
sp.adaptive_mutation_maximum_percent
```

实验 runner 对字段进行类型、范围及交叉约束验证，显式映射为环境变量，并进入
resolved manifest、配置哈希和 `run.json`。

## 7. 总回归与端到端验证

```text
CMake 完整构建：通过
C++ / CTest：29 / 29 通过
Python unittest：56 / 56 通过
git diff --check：通过
```

Farham `R101_s1_3_10` 的两迭代端到端冒烟运行得到：

```text
整数 SP 状态：OPTIMAL
整数目标：2687
LP-RMP 目标：1988.354
LP 主列：68
LP dual feedback：可用
Gurobi solution-pool 候选：5
多样化注入：3
最终解：2687
```

该运行验证完整链路，不构成 P2 优于 P1 的统计证据。

## 8. 推荐配对消融

固定实例、seed、process CPU 预算、线程与所有非目标参数，依次比较：

```text
P1
P1 + LP dual feedback（persistent LP off）
P1 + dual + persistent LP
P1 + dual + persistent LP + diverse injection
P1 + 全部 P2
```

除 best/avg gap 和平均 CPU 时间外，至少记录：

- LP build/optimize CPU 与复用率；
- 新增、停用、重新启用列数；
- reduced-cost 列的保留率；
- solution-pool 候选数与实际注入数；
- 注入解间最小 Jaccard 距离；
- 各子种群获得的 HGA 尝试数；
- 每个子种群的改进/CPU 与新增列/CPU；
- mutation 实际概率、commit 率和收益；
- local/global SP 调用次数与总 CPU 占比。

完成这些配对实验前，不应声称 P2 已统计显著地改善 Farham BKS。
