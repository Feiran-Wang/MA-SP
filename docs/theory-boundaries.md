# 理论依据与边界

## 1. 依据来源

算法基线来自 He、Hao 与 Wu 的论文 *A Hybrid Genetic Algorithm with
Multi-Population for Capacitated Location Routing*，INFORMS Journal on
Computing, 2025，DOI
[10.1287/ijoc.2023.0416](https://doi.org/10.1287/ijoc.2023.0416)。

文档和论文写作应明确区分三类内容：

1. **论文复现项**：论文明确给出的 CLRP/HGAMP 定义、算子和参数；
2. **实现兼容项**：由当前数据结构造成的限制；
3. **HGA-SP 扩展项**：Route Pool、SP、Gurobi、触发与注入等本项目设计。

未经实验验证，后两类不得表述为论文结论。

## 2. 论文能够支撑的内容

论文研究的是 CLRP。其问题定义包含：

- 从候选仓库中决定开启集合；
- 每个客户恰好访问一次；
- 每条路线从同一已开仓库出发并返回；
- 车辆容量和仓库容量均满足；
- 每个仓库拥有无限车辆，每辆已使用车辆承担固定成本；
- 目标为仓库开启成本、车辆固定使用成本和路线行驶成本之和；
- 成本矩阵可以对称，也可以非对称。

论文的 HGAMP 主流程是：

```text
初始仓库配置与多子种群
  -> 跨子种群父母选择
  -> mdEAX
  -> 容量修复
  -> mutation
  -> local search
  -> incumbent / population management
```

其中，前 `γ` 个子种群分别绑定一个仓库配置，另有一个容纳新配置的子种群；
mdEAX 同时重组仓库配置与路线边；不可行子代先修复车辆和仓库容量；局部搜索用于
强化；种群管理结合质量与多样性，并包含停滞重建和运行中期子种群缩减。

论文第 3.5.1 节的基线变异是 Shaw 思路的相关客户移除与重插入。表 2 给出的默认
变异概率为 `ζ = 0.15`，变异强度为 `ξ = 0.25`。因此活动实现应表达为：

```text
每个已修复且可行的子代，以 15% 概率执行 Shaw mutation；
未触发时不变异；触发后再进行局部搜索。
```

当前 `mutationProbabilityPercent = 15`、
`RandomInt(0, 99) < mutationProbabilityPercent` 和固定调用 `shawRemoval()` 与此
基线一致。`locationRemove()`、`distanceRemove()` 等留存函数只能作为显式消融
算子，不能混入“论文基线”。

论文还给出 `μ=30`、`λ=30`、`α=20`、`η=70000`，并在主实验中采用
`γ=15`、最多 300,000 个子代迭代和 20 次独立运行。若项目采用不同设置，必须在
配置与实验报告中明确标为工程配置或消融，而不是论文默认值。

## 3. 论文不能支撑的内容

该论文没有提出或评估以下组件：

- 路径池及其准入、去重、裁剪或容量上限；
- set-partitioning 主问题；
- Gurobi 或任何特定 MIP 后端；
- SP 触发频率、时限、gap、线程、seed、`MIPFocus`、`Heuristics`；
- SP 解的局部搜索、种群注入或子种群接管；
- `load / routeCost` 路线裁剪评分；
- `representationRouteLimit`。

因此，“HGA-SP 比 HGAMP 更优”“某个 pool/SP 参数由 HGAMP 论文支持”等说法在
完成对照实验前均不成立。HGA-SP 是以 HGAMP 为 Route Generator 的研究扩展；其
有效性必须通过多 seed 统计、受控对照和独立解验证建立。

## 4. SP 模型的合理推导

SP 可以从 CLRP 定义自然构造，但这是本项目模型设计，不是 HGAMP 论文中的模块。
对池中路线 `r` 定义二元变量 `x_r`，对仓库 `l` 定义二元变量 `y_l`。基线模型
应包含：

- 每个客户的精确覆盖约束 `sum(a_ir * x_r) = 1`；
- 仓库负载约束
  `sum(load_r * x_r) <= capacity_l * y_l`；
- 路线只能属于其标记的仓库，未开仓库不能选中所属路线；
- 目标
  `sum((travelCost_r + vehicleFixedCost) * x_r) + sum(openCost_l * y_l)`。

由于车队无限，模型不应施加真实的每仓库车辆数上限。当前可选
`representationRouteLimit` 仅保证所选路线可放入 `Individual::maxVisit` 固定
槽位。它会收缩可行域，启用时必须在结果中标记，不能用于声称求得原始 SP 的最优
解。

路线反向规范化也不能无条件采用：论文允许非对称成本。只有实例被显式确认是对称
成本时，`[a,b,c]` 与 `[c,b,a]` 才可按同仓库等价路线合并。

## 5. Gurobi 工程约束

以下是实现与实验的工程约束，不是 HGAMP 论文结论：

- 一个进程创建一个 Gurobi environment，多次精确阶段共享该 environment；
- 每次求解使用短生命周期 model，并把建模时间纳入阶段 deadline；
- 求解前验证路径池非空且覆盖全部客户；
- 根据 `Status` 与 `SolCount` 分别判断证明状态和 incumbent 可用性；
- `TIME_LIMIT` 下的可行 incumbent 可以独立验证后使用；
- seed、线程、版本、状态、时限和完整 run manifest 进入 provenance；
- 并行跑批按 worker 隔离 environment、日志和结果，限制总线程资源；
- 运行凭据由环境提供，不进入仓库或结果。

## 6. 论文声明规则

任何可能影响可行域、目标函数、随机轨迹或运行预算的修改，都必须：

1. 在配置中显式可见并进入 run manifest；
2. 更新数学模型或算法文档；
3. 增加相应单元/集成/回归验证；
4. 在结果中区分 baseline、compatibility 和 extension；
5. 若声称改进，至少报告相同实例、相同预算和多 seed 的受控对照。
