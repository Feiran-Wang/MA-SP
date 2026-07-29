# HGA-SP P1 改造报告

日期：2026-07-28

## 1. 范围与结果

本轮完成四项 P1 改造：

1. 当前 HGA 全局最优解作为 SP MIP Start；
2. protected backbone 与分层路径池裁剪；
3. configuration-local / global 两级 RMP；
4. 基于列新颖性、停滞和单位 CPU 收益的自适应 SP 触发。

全部改造均有独立测试，并通过项目总回归。当前证据证明实现不变量、模型边界和
触发策略按设计工作；真实 Farham 算例上的 gap 与 CPU 收益仍需正式配对消融。

## 2. P1-1：当前 HGA incumbent MIP Start

### 实现

global RMP 调用前：

1. 重新提交当前 HGA 全局最优解，防止其列被裁剪遗漏；
2. 导出完整路线集合；
3. 按 `(depot, customer set)` 映射到当前压缩主列；
4. 复验精确覆盖、仓库容量和目标；
5. 优先设置完整 MIP Start。

若当前 HGA 解无法映射，才使用同一问题上一次 SP 解。日志记录
`mip_start_submitted=true|false` 和
`mip_start_source=hga_global_incumbent|previous_sp|none`。这里的
`submitted` 表示完整 Start 属性已交给模型，并不声称 Gurobi 提供了独立的
presolve 接纳确认。

### 独立验证

`sp.known_optimum` 使用一个路线顺序已被 dominance 替换的 HGA Start。测试确认：

- HGA Start 仍按客户集合映射成功；
- 两条路线的 Start 被完整设置；
- Start 来源为 `hga_global_incumbent`；
- 求解仍得到已知最优值 11；
- 无显式 HGA Start 的下一次调用回退到 `previous_sp`。

## 3. P1-2：protected backbone 与分层裁剪

### 保护集合

Tier 0 永不主动裁掉：

- 初始化时的完整可行 backbone；
- 当前 HGA 全局最优解；
- 上一次 SP incumbent；
- 当前各非空子种群 elite。

保护以 Master key 表达，因此同客户集合的更便宜访问顺序仍可无损替换原顺序。

### 分层裁剪

目标大小仍为配置上限的 80%，按以下顺序选择：

1. Tier 0 protected；
2. Tier 1 客户覆盖稀缺性，预算允许时每个客户保留至多两个候选；
3. Tier 2 仓库多样性；
4. Tier 3 装载/行驶成本效率。

如果 protected 列数超过名义目标，允许保护集合临时突破目标，优先保证已知完整
覆盖和 incumbent 可重建性。

### 独立验证

`route_pool.tiered_pruning` 将 6 个主列裁剪到 3 个：

- 两条 backbone 路线全部保留；
- Tier 1 补入一条稀缺覆盖路线；
- 所有客户仍至少被一条保留列覆盖；
- 保护层和稀缺层计数分别为 2 和 1。

## 4. P1-3：configuration-local SP

### 模型

局部 RMP：

- 从 Master Column Pool 过滤出目标配置已开启仓库的路线；
- 固定每个仓库开启变量为该配置的 0/1 值；
- 对固定开启的仓库计入开启成本；
- 使用对应子种群 elite 作为 MIP Start；
- 使用独立 SPSolver 状态和更短时限；
- 注入前不执行可能改变仓库配置的通用 location local search。

global RMP 继续使用完整列池并允许跨配置重组。

### 独立验证

- `sp.configuration_local` 验证局部列过滤不保留关闭仓库路线；
- `sp.known_optimum` 中，全局 RMP 得到目标值 11；固定仓库 0 后，局部 RMP
  只能选择该仓库的两条路线并得到目标值 16。

因此局部求解并非缩小列池后再次允许全局选址，而是真正固定仓库配置。

## 5. P1-4：adaptive SP trigger

### 输入信号

控制器使用：

- Master Column Pool 版本增量；
- 当前非改进迭代数；
- 固定 global cadence；
- 是否进入最后 10% 搜索阶段；
- 上一次 SP CPU 时间；
- 剩余全局 CPU 预算；
- `EMA(objective improvement / SP CPU seconds)`。

### 决策规则

- 主列未变化：跳过；
- 剩余 CPU 不足以容纳近期一次 SP：跳过；
- 有新列的固定 global 事件：优先 global；
- 达到全局停滞或末段强化：global；
- 达到局部停滞或新列门槛：local；
- 其他情况继续 HGA。

local 与 global 分别维护已消费的 Master 版本、最近迭代、CPU 代价和 utility
EMA。local 求解不会让 global 错误地认为同一批跨配置列已经被消费。无改进的
global 调用采用指数增长的冷却间隔（有上界），末段强化至多发出一次；固定
scheduled global 事件在确有新列时仍作为最大间隔保证。

local SP 优先求解刚刚产生本轮子代/新列的活跃子种群配置；只有该槽位无效或为空
时才回退到当前最优子种群，避免长期只强化一个配置。

上一次 SP 单位 CPU 收益为零时，local 新列门槛加倍；达到高收益阈值时，门槛
减半。`adaptive_trigger_enabled=false` 时回退到固定 global cadence 和固定
local frequency。

### 独立验证

`sp.adaptive_controller` 验证：

- 主列未变化与最小 HGA 间隔均会跳过；
- 初始新列门槛触发 local；
- 零收益后门槛从 10 提高到 20；
- 高收益后门槛从 10 降至 5；
- 固定 global 事件在存在新列时优先；
- 全局停滞触发 global；
- local 已消费同一 Master 版本后，global 仍能基于自己的版本看到新列；
- 无收益 global 调用进入冷却，末段强化只发出一次；
- 剩余 CPU 不足时拒绝启动 SP。

## 6. 配置与 provenance

新增 manifest / 环境配置：

```text
sp.local_enabled
sp.local_trigger_frequency
sp.local_time_limit_seconds
sp.adaptive_trigger_enabled
sp.adaptive_minimum_interval
sp.adaptive_local_new_columns
sp.adaptive_local_stagnation
sp.adaptive_global_stagnation
sp.adaptive_utility_ema_weight
sp.adaptive_high_utility_threshold
```

这些字段由实验 runner 显式映射到 `HGASP_SP_*` 环境变量，并进入 resolved
manifest、配置哈希和 `run.json`。

## 7. 总回归

```text
CMake 完整构建：通过
C++ / CTest：26 / 26 通过
Python unittest：56 / 56 通过
git diff --check：通过
```

其中包括真实 SP 已知最优值、HGA MIP Start、固定仓库配置模型、路径池分层裁剪
和自适应控制测试。

## 8. 后续配对消融

建议固定实例、种子、CPU 预算和线程，依次比较：

```text
P0
P0 + HGA MIP Start
P0 + Start + protected/tiered pool
P0 + Start + pool + local/global RMP
P0 + 全部 P1
```

至少报告 best/avg gap、总 CPU 时间、SP CPU 占比、local/global 调用次数、跳过
原因、主列数、裁剪分层计数、MIP Start 接受率和单位 SP CPU 改进。没有完成这些
配对实验前，不应声称 P1 已统计显著地改善 Farham BKS。
