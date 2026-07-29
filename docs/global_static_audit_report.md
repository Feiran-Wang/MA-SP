# HGA-SP 全局静态审计与防御性改造报告

日期：2026-07-28

## 1. 审计范围与结论

本轮从进程生命周期、HGA/SP 控制流、P0–P2 数据流、内存有界性、日志可诊断性
和两种问题模式六个层面复核当前项目。

结论如下：

1. 已复现并修复 runner 被杀后 solver 脱管、重复实验并发写入，以及批量清理耗时
   随 worker 数线性增长的问题。
2. 已确认此前长时间无输出的主要算法原因不是互斥锁死锁，而是 global stagnation
   达到阈值后，每隔最小间隔重复执行 360 秒 global SP。现在 local/global 状态
   分离，无收益 global 调用指数退避，final intensification 至多执行一次。
3. 已消除种群孵化、随机子种群选择、仓库配置构造、异常 EAX 端点和初始解修复中
   已确认的无界循环。
4. P0–P2 均已沿真实 CLRP 与 CLRP-TW 调用链复核。发现并修复了多处“单元功能存在，
   但全局状态相互污染或遥测失真”的集成问题。
5. 日志改为有界 human log、typed event log、算法状态快照、进程 heartbeat 和
   matrix 状态五层结构。正常运行不再把同一 stdout 保存两份。

本报告证明的是实现不变量、边界安全和端到端可运行性。它不把短时 smoke 当作
算法性能实验，也不声称 P0–P2 已统计显著地改善 Farham BKS。

## 2. 假死根因与修复

### 2.1 进程脱管

旧 runner 直接创建独立 solver session，只在内存中记录进程。若 runner 被
`SIGKILL`，其清理 handler 不可能执行，solver session 会继续运行；新 runner
又看不到旧进程，因而可能重复启动相同 instance/seed。

现在每个 solver 由 `process_supervisor.py` 管理：

- runner 与 supervisor 之间使用私有控制管道；
- runner 正常取消会关闭管道，runner `SIGKILL` 也会由内核产生 EOF；
- supervisor 对完整 solver PGID 执行 TERM、共享 grace、KILL 和 reap；
- Linux 启用 subreaper，并给 solver leader 配置 parent-death signal；
- `.matrix.lock` 与 `.run.lock` 的 FD 由 supervisor 持有到 solver 完全退出；
- `process.status.json` 原子记录 PID、PGID、start ticks、CPU、RSS 和 cleanup reason。

supervisor 使用 `wait4` 保存 solver 的 user/system CPU。runner 优先读取这些数值，
不会把 supervisor 自身的微小 CPU 时间误记为算法时间。

### 2.2 重复长 global SP

旧 adaptive controller 在达到 global stagnation 后，可按最小间隔不断重跑完整
global SP；local SP 又会覆盖 global 的版本和 utility 状态。长 time limit 下，
这会表现为持续数分钟无 HGA 进度。

修复后：

- local/global 分别维护 Master 版本、最近迭代、CPU 和 utility EMA；
- local 消费某一版本后，global 仍能看到相对于上一次 global 的新列；
- 连续零收益 global 调用使用有上界的指数冷却；
- final intensification 只发出一次；
- scheduled global 仍在确有新列时提供最大间隔保证；
- local SP 优先强化本轮活跃子种群，而不是始终选择全局最好配置。

### 2.3 其他确定性无界路径

本轮还修复：

- 新仓库配置种群的两个 `while(size < popMin)`；
- 非 adaptive 子种群随机 tournament；
- 总仓容恰等于需求时的仓库配置无限重试；
- 单仓已足够时绕过 `no_get` 终止条件；
- depot configuration 缺少 `e`/提前 EOF；
- EAX 两端均已占用时无状态 `while(1)`；
- dummy edge 和 AB-cycle 的随机/拓扑重试；
- 初始解 capacity repair 的无限 move 循环；
- spanning-tree 全客户已选完后访问 `demCus[-1]`。

repair guard 同时检查最大尝试、最大无进展次数和全局 process CPU。耗尽时只记录
一次 warning 并丢弃该不可行候选，不会退化为接受坏解，也不会中止整个 solver。

## 3. P0 全局复核

| 改造 | 全局复核结果 | 本轮补强 |
|---|---|---|
| 路径提交与种群准入解耦 | 可行子代在结构重复/种群替换前提交路线；CLRP/TW 共用该边界 | 保持；不可行候选在边界处拒绝 |
| 结构重复判定 | 等目标、不同路线结构可以共存；同结构克隆拒绝 | 保持；SP 注入也走相同判定 |
| 双层路径池与列支配 | 同 depot/customer-set 的已验证可行 TW 路线具有相同主问题系数，保留最低旅行成本列是精确支配 | Search Archive 与 best history 均增加硬上限和 discard telemetry |
| 路线表示上界 | 原经验上界会裁掉合法多路线解 | 改为 `min(n, z + floor(Qd/qmin))`，含零需求防御；SP 冗余约束默认关闭 |
| EAX 双模式 | `paper-compatible` 保持历史行为，`corrected` 修正父母方向 | 模式写入配置/解/日志；corrected 仍需显式启用，避免静默改变 baseline |

P0 的列支配对 CLRP-TW 仍然精确：进入池的两条路线都已单独通过 hard-TW
验证；客户集合、仓库、载重、车辆固定成本和所有 Master 系数相同，只有旅行成本
不同，因此贵列可被便宜列替换而不改变主问题可行性。

## 4. P1 全局复核

| 改造 | 全局复核结果 | 本轮补强 |
|---|---|---|
| HGA incumbent MIP Start | 当前全局解按压缩 column key 映射，失败才回退 previous SP | 日志改称 `mip_start_submitted`，不伪称 Gurobi 已确认接纳 |
| protected backbone / 分层裁剪 | backbone、global incumbent、last SP、population elite 受到保护 | Tier 1 明确优先 finite reduced cost；保护列允许暂时突破目标大小 |
| configuration-local SP | 固定 depot configuration，列过滤与开启成本一致 | local 对准 active population；local dual 不再覆盖 global guidance |
| adaptive trigger | 新列、停滞、剩余 CPU、历史收益均进入决策 | local/global 完全分离；global backoff；final-once |

## 5. P2 全局复核

| 改造 | 全局复核结果 | 本轮补强 |
|---|---|---|
| LP-RMP / dual feedback | dual 仅指导路径级选择，不加入完整解适应度 | 仅 global dual 更新 RoutePool/mutation；local dual 保持局部 |
| persistent LP | 新列增量加入，旧列 UB=0，再出现时 re-enable | stored > 2 × active 时重建压缩，并记录 before/after |
| diverse SP injection | 每个 pool candidate 独立复验覆盖、容量、配置、目标 | telemetry 分为 attempted/admitted/rejected |
| adaptive allocation | UCB、diversity、列生产率和 CPU 收益共同分配预算 | 双父本各获 50% credit；配置槽替换/restart 重置旧 arm |

## 6. 日志系统

### 6.1 文件职责

- `hgasp.log`：面向人工排查，按大小轮转；
- `events.jsonl`：typed JSON，包含 seq、UTC、monotonic、process CPU、
  component、event、level 和有界 fields；
- `algorithm.status.json`：原子覆盖的算法阶段快照；
- `process.status.json`：supervisor heartbeat 和进程组健康；
- `matrix.status.json`：约 30 秒覆盖一次的批量运行摘要。

### 6.2 防御性

- 默认 `hgasp.log` 与 `events.jsonl` 各为 8 MiB × 当前段及 2 个备份；
- 单消息 4 KiB，字段数、字段长度和 status 总大小均有硬界；
- NaN/Inf 写为 JSON `null`，字符串正确转义；
- token/secret/password/credential 类命名字段自动脱敏；
- 目录权限 0700、文件权限 0600；
- ENOSPC、权限或轮转失败只关闭对应 sink，向原 stderr 最多告警一次；
- managed runner 设置 `HGASP_LOG_CONSOLE=0`，避免 `stdout.log` 与
  `hgasp.log` 重复；
- 不记录逐 route、逐 insertion、逐 mutation 事件。

保留所有历史 attempt 是实验可审计性策略，因此整个 artifact 根目录仍会随人工
重跑次数线性增长；本轮保证的是单 attempt 严格有界，没有自动删除用户历史结果。

## 7. 验证证据

最终独立回归：

```text
Development CTest                 30 / 30 passed
Python unittest                   61 / 61 passed
ASAN + UBSAN（排除网络 SP）       28 / 28 passed
Release build                     passed
Python py_compile                 passed
git diff --check                  passed
```

故障注入覆盖：

- matrix runner `SIGKILL`；
- solver 与孙进程忽略 TERM；
- 同一 matrix/run 并发启动；
- heartbeat 多次覆盖与文件大小上限；
- solver CPU 与 supervisor CPU 归属；
- rotated canonical log 与 stdout 去重；
- cleanup 失败不能报告 success。

模式联调：

- Set S `100-5-1c`：1 秒非 SP smoke，独立验证 100/100 客户、容量及目标分解；
- Set F `R101_s1_3_10`：1 秒非 SP smoke，独立验证 10/10 客户、hard TW、
  仓库/车辆容量及目标值 2687。

一次真实 TW final-SP 联调遇到 `token.gurobi.com` DNS 解析失败，重试耗尽后按
非退化策略明确失败。结构化日志完整记录
`adaptive_decision → solve_started → solve_finished(GUROBI_ERROR) →
run.failed`。随后本地 `sp.known_optimum` 与 `gurobi.smoke` 均通过；因此该项记录
为外部网络波动证据，不把它解释成模型通过，也没有以 HGA-only 继续。

## 8. 剩余边界

以下不属于本轮已观察到的假死，但仍建议后续处理：

1. `Individual`、`RoutePool`、`LS` 和部分 crossover 仍有依赖双向链表拓扑正确
   的遍历。正常个体有限；若内存/拓扑损坏，尚无统一 hop limit。
2. 少数当前活动的父代/索引随机抽取仍属于“概率 1 终止”而非形式上的确定上界；
   已修复已知小种群和子种群选择路径。
3. `newRepair()` 等当前未调用的遗留代码仍有无界循环；重新启用前必须接入统一
   repair guard。
4. P0–P2 的 best/avg gap 与 CPU 收益仍需固定实例、种子和预算的正式配对消融。

