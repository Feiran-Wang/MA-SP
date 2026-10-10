# MA-SP v22

本仓库 `main` 的主要版本为 **v22 消融／Anytime 实验所用的 MA-SP 核心实现**，
支持 Set S 容量选址路径问题及 Set F 硬时间窗扩展。

31 个 C++ 源码／头文件原样来自冻结的 `masp-v22-20260928/solver-source/masp/`。
本次仅调整仓库构建入口和说明；源文件映射及身份见 [provenance.json](provenance.json)。
历史 v22 使用 **Gurobi**，不是后来的 COPT 版本。

## 构建

需要 Linux/POSIX、CMake 3.20+、支持 C++11 的编译器，以及带 C++ API 的 Gurobi
安装和有效许可。原实验使用 Gurobi 13.0.2。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGUROBI_ROOT=/opt/gurobi1302/linux64
cmake --build build --target masp --parallel 2
```

也可以通过 `GUROBI_HOME` 指定安装位置。Gurobi 库和许可需在本机提供。

## 主要 Set S 配置：v22 Full

v22 消融／Anytime 当时实际使用的参数为：

\[
(K,\mu,\alpha,\text{变异率},\rho,\text{路线池上限})
=(15,20,20,15\%,0.10,25000).
\]

SP 策略为 `F20001_b1`，每个原生搜索 episode 在 300,000 主计数内有 14 个计划机会。
启用以下环境配置才能使用当时的 selected Full 参数：

```bash
mkdir -p output
env HGAMP2_SETS_EXPERIMENT=1 \
  HGAMP2_R_CFG=15 HGAMP2_MU=20 HGAMP2_ALPHA=20 \
  HGAMP2_P_MUT=15 HGAMP2_RHO=0.10 HGAMP2_POOL=25000 \
  HGAMP2_SP_POLICY=F20001_b1 HGAMP2_SP_FREQUENCY=20001 HGAMP2_SP_BETA=1 \
  HGAMP2_ARM=Full \
  ./build/masp \
    examples/100-5-1c.json examples/depot_configuration_100-5-1c.txt \
    output/solution.txt 0 0 60000 300000 1
```

这只是单个原生 episode 的命令示例，没有在本次发布中执行。
不设置研究模式环境变量时，Set S 使用源码保留的普通默认值
`(15,30,20,15%,0.25,50000)` 和 50,001 的 SP 间隔。

`Full` 是主要配置；`GlobalOnly` 为历史准入消融对照：新配置安装另须严格改善接纳前的
全局 incumbent。两者保留在同一核心中。本实现使用全局路线池 SP。

## TW 参数：对应完整 36-run 实验

**TW 使用 `(3,30,10,15%,0.25,10000)`，对应已完成的 TW 完整 36-run 实验
（12 个算例 × 3 个 seeds）。**

| 参数 | 数值 |
|---|---:|
| K：每个候选配置的构造重复次数 | 3 |
| μ：最小种群规模 | 30 |
| α：邻域宽度 | 10 |
| 变异率 | 15% |
| ρ：Shaw 移除比例 | 0.25 |
| 路线池上限 | 10,000 |

这是 `set-f-tw-v1-p1` 普通 profile；最大常驻车场配置数另为 10。
10／25／40／50 客户算例的 SP 间隔均为 **50,001 个主计数**，单次 MIP 上限均为
**10 秒**。该时间上限不包含完整 SP 流程的建模和后处理耗时。

在没有 `HGAMP2_*` 研究覆盖变量的环境中，TW 命令格式为：

```text
./build/masp INSTANCE.txt NULL OUTPUT.txt 0 0 60000 300000 SEED
```

不启用 `HGAMP2_SETF_EXPERIMENT`：那个后续调参模式采用另一套默认参数，不能混称为
上述 36-run 配置。36-run 成绩属于原已封存批次，本次上传没有追加 TW 实验。

## 命令与时间口径

```text
masp INSTANCE [DEPOT_CONFIG] OUTPUT BEST_VALUE IS_OPTIMAL CPU_LIMIT MAIN_COUNTER_LIMIT SEED
```

Set S 使用独立车场配置文件；Set F 可以将该参数写为 `NULL`。
示例中的 `BEST_VALUE=0, IS_OPTIMAL=0` 不声明已知最优值。
研究模式要求 CPU 限额 60,000 秒、主计数限额 300,000；它们不是 3,600 秒墙钟预算。

原 Anytime 比较由外层程序执行 3,600 秒独立重复，保留跨 episode 的已验证最好解，
不向下一 episode 传递路线池或种群。本仓库提供原生核心，不自动执行该实验协议。
核心保留 `MASP_RUN_DEADLINE` 接口，其值为 `CLOCK_MONOTONIC` 的绝对秒数，不能直接
填入相对时长 `3600`；省略它表示不设置这个外层截止时间。

`StudyEvents.h`、`ExperimentClock.h` 是核心编译与运行所需的头文件。
默认研究事件输出为当前目录下的 `study_events.jsonl`，可用 `HGAMP2_EVENT_PATH` 指定。
独立验收器、实验调度脚本、HGAMP 对照实现、测试脚本和测试夹具均不随本次核心发布上传。

## 来源与边界

本次保留 v22 原有算法行为，没有合入后续路线池改进或修复历史种群槽位／排序问题。
有限路线池上的 SP 最优只表示池内最优，不提供原问题全局最优证明。
来源与历史证据的本机路径记录在 `provenance.json`；实验结果和历史版本留在原证据位置。

Author: Feiran Wang。原有源码作者注释保留。许可证见 [LICENSE](LICENSE)。
