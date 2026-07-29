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

参考实现中，AB-cycle 起点同时存在父边与母边时，两个随机分支都选择父边。本项目
把该行为保留为默认 `paper-compatible` 模式，以维持复现实验；`corrected`
模式才按随机位选择父边或母边。后者是明确的工程修正，必须通过独立消融验证，
不能直接归为论文基线。

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

当前 HGA incumbent MIP Start、分层路径池保护、configuration-local/global
RMP 和自适应 SP 触发均属于 HGA-SP 工程与算法扩展，不是 HGAMP 或 Farham
论文中的默认组件。单元测试能够证明可行域、映射和调度不变量，但不能替代
Farham 算例上的配对消融。

LP-RMP 对偶反馈、持久化增量 LP 主问题、solution-pool 多样化注入以及自适应
子种群/mutation 预算同样属于 P2 研究扩展。LP 对偶只能指导路径级生成和保留；
由于完整解恰好覆盖所有客户，不能把客户对偶和直接加入完整解适应度并声称改变了
搜索排序。整数 SP 的 `OPTIMAL` 仍仅表示当前有限 Master Column Pool 上最优。

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

由于车队无限，模型不应施加真实的每仓库车辆数上限。当前
`Individual::maxVisit = numV`：精确覆盖使任何非空路线至少占用一个不同客户，
因此单仓库不可能需要超过 `numV` 条路线。可选
`representationRouteLimit` 现在只会加入这个严格安全、但冗余的界，默认关闭，
不再使用原容量倍数经验公式。

路线反向规范化也不能无条件采用：论文允许非对称成本。只有实例被显式确认是对称
成本时，`[a,b,c]` 与 `[c,b,a]` 才能在搜索档案中视为等价。不过在 SP 主问题
中，同仓库且客户集合相同的两条可行路线具有完全相同的约束列，成本更高者可被
成本更低者精确支配；这不要求路线对称。

## 5. Gurobi 工程约束

以下是实现与实验的工程约束，不是 HGAMP 论文结论：

- 一个进程创建一个 Gurobi environment，多次精确阶段共享该 environment；
- LP-RMP 可在相同问题与 environment generation 下持久化并增量同步；整数
  SP 仍使用当前快照的短生命周期 model，二者建模时间均纳入阶段记录；
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

## 7. Farham CLRP-TW 扩展

CLRP-TW 语义依据 Farham、Süral 与 İyigün 的 LRPTW 模型及
[Farham 博士论文](https://etd.lib.metu.edu.tr/upload/12625395/index.pdf)。
Set F 使用：

\[
c_{ij}=t_{ij}=
\left\lfloor
\sqrt{(x_i-x_j)^2+(y_i-y_j)^2}
\right\rfloor .
\]

等待和服务时间不进入目标；目标仍为仓库开启成本、每条路线的车辆固定成本与
弧行驶成本之和。车辆同质且数量充足。

### 7.1 给定路线的时间窗可行性充要判据

考虑固定路线
\(r=(d,v_1,\ldots,v_k,d)\)。令仓库开放时刻为 \(o_d\)，客户 \(v_j\)
的 ready、due 与服务时间分别为 \(a_{v_j},b_{v_j},s_{v_j}\)。定义最早服务
时刻：

\[
e_0=o_d,\qquad
e_j=\max\{a_{v_j},
e_{j-1}+s_{v_{j-1}}+t_{v_{j-1},v_j}\},
\]

其中 \(v_0=d,s_d=0\)。该路线可行，当且仅当

\[
e_j\le b_{v_j}\quad(j=1,\ldots,k)
\]

且

\[
e_k+s_{v_k}+t_{v_k,d}\le B_d .
\]

证明如下。对任何具有相同访问顺序的可行服务时刻表 \(u\)，有
\(e_0\le u_0\)。若 \(e_{j-1}\le u_{j-1}\)，则旅行、服务和 ready 约束给出

\[
e_j
\le
\max\{a_{v_j},
u_{j-1}+s_{v_{j-1}}+t_{v_{j-1},v_j}\}
\le u_j .
\]

归纳得到 \(e_j\le u_j\)。所以最早时刻表一旦违反某个 due 或仓库关闭时刻，
任何延迟出发/额外等待的时刻表都会同样违反；反之，如果上述不等式全部成立，
\(e\) 本身就是构造性的可行证书。

### 7.2 SP 选择保持时间可行

每个入池列固定一个仓库、一个客户序列，并已由上述判据证明可行。Farham 模型
没有跨车辆同步、共享时间容量或车队数量约束。因此 SP 只是在列集合中选择子集；
它不会修改任一列的访问顺序或服务时刻。只要所选列满足客户精确覆盖与仓库容量，
各列可行时刻表的并集就是整个 CLRP-TW 解的可行时刻表。这说明主问题无需重复加入
客户时刻变量，但路线池准入和 SP 导入必须拒绝未经独立 TW 验证的列。

### 7.3 启发式边界

当前 TW 构造器枚举每一步的全部仓库、路线与插入位置，并只提交完整可行方案；
这保证“返回成功的解一定可行”，但贪婪插入失败不等价于原实例不可行。当前
`set_f_mvp.json` 因而聚焦 12 个已校准的 10 客户实例，较大实例作为独立验证的
代表性实验，而不是宣称已覆盖全部 138 个实例。

### 7.4 TW-aware mutation 的依据与边界

TW mutation 采用 Shaw 的 related removal/recreate 思路：
[Shaw (1998)](https://doi.org/10.1007/3-540-49481-2_30) 给出了从当前路线中
随机偏置地移除相关客户、再重插入以探索大邻域的基本框架；
[Ropke 与 Pisinger (2006)](https://doi.org/10.1287/trsc.1050.0135) 说明该类
破坏/重建算子可在带时间窗问题中与多个启发式协同使用；
[Vidal 等 (2013)](https://doi.org/10.1016/j.cor.2012.07.018) 则为
HGS-VRPTW 中时间与容量感知的邻域评价提供依据。

本实现将客户 \(i,j\) 的 relatedness 定义为归一化加权和：

\[
R_{ij}=0.40\,d_{ij}+0.30\,|\tau_i-\tau_j|
+0.15\,|m_i-m_j|+0.10\,|q_i-q_j|+0.05\,I_{ij},
\]

其中 \(d_{ij}\) 是空间距离，\(\tau\) 是当前服务开始时刻，\(m\) 是时间窗中心，
\(q\) 是需求，\(I_{ij}\) 表示是否位于不同路线。移除规模沿用
\(\xi=0.25\)，重建从至多四个最低增量的可行插入中随机偏置选择。上述具体权重、
RCL 宽度和最多 12 次重建尝试是本项目的工程化扩展，并非三篇文献证明的唯一或
最优参数，必须通过消融实验验证。

实现只在 detached `StandaloneRoute` 方案上修改。每个插入候选检查车辆容量、
仓库容量、硬时间窗和路线表示上限；只有完整覆盖再次独立验证后才
`rebuildFromSP` 提交，异常时恢复原方案。因此可以证明：

> `mutationRun()` 在 TW 模式返回 `true` 时，提交解满足当前模型的全部硬约束；
> 返回 `false` 时，原 incumbent 的拓扑与目标值保持不变。

这是可行性与事务性保证，不是质量改进保证；mutation 可以接受更差但结构不同的
可行解，随后由 local search 和种群管理决定其搜索价值。
