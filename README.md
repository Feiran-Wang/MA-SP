# MA-SP

**Memetic Algorithm with Set Partitioning for Capacitated Location-Routing**

MA-SP combines population-based search, local improvement, and a global route-pool
set-partitioning model for the capacitated location-routing problem (CLRP) and its
hard time-window variant (CLRP-TW). This repository provides the v22 solver core.

## Build

Requirements: Linux/POSIX, C++11, CMake 3.20+, and Gurobi with its C++ API and a
valid license. The original implementation used Gurobi 13.0.2.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGUROBI_ROOT=/opt/gurobi1302/linux64
cmake --build build --target masp --parallel 2
```

## Usage

```text
masp INSTANCE [DEPOT_CONFIG] OUTPUT BEST_VALUE IS_OPTIMAL CPU_LIMIT MAIN_COUNTER_LIMIT SEED
```

Set S requires a depot-configuration file. For Set F (TW), use `NULL` in its place.
`CPU_LIMIT` is in seconds; `BEST_VALUE=0` and `IS_OPTIMAL=0` disable optimality claims.
The v22 Set S **Full** configuration can be run as follows:

```bash
mkdir -p output
env MASP_SETS_EXPERIMENT=1 MASP_ARM=Full \
  MASP_R_CFG=15 MASP_MU=20 MASP_ALPHA=20 MASP_P_MUT=15 \
  MASP_RHO=0.10 MASP_POOL=25000 MASP_SP_POLICY=F20001_b1 \
  ./build/masp examples/100-5-1c.json examples/depot_configuration_100-5-1c.txt \
    output/solution.txt 0 0 60000 300000 1
```

Runtime options use the `MASP_*` environment namespace; supported values are
listed in [SearchParameters.h](code/SearchParameters.h). Study mode requires the
60,000-second CPU limit and 300,000 main-counter limit shown above.

## Parameter profiles

| Profile | K | μ | α | Mutation | ρ | Route-pool cap |
|---|---:|---:|---:|---:|---:|---:|
| Set S: v22 Full | 15 | 20 | 20 | 15% | 0.10 | 25,000 |
| Set S: ordinary defaults | 15 | 30 | 20 | 15% | 0.25 | 50,000 |
| Set F (TW): ordinary defaults | 3 | 30 | 10 | 15% | 0.25 | 10,000 |

K denotes construction repetitions per candidate depot configuration; μ is the
minimum population size, α the neighborhood width, and ρ the Shaw removal fraction.
The TW tuple **(3, 30, 10, 15%, 0.25, 10000)** corresponds to the completed **36-run
experiment (12 instances × 3 seeds)**. Use the ordinary Set F profile for this
configuration. Its SP trigger interval is 50,001 main-counter increments and its
MIP time limit is 10 seconds for the 10/25/40/50-customer instances. Source lineage
and historical evidence pointers are recorded in [provenance.json](provenance.json).

## License

MIT License; see [LICENSE](LICENSE). Maintainer: Feiran Wang. Original source
author credits are retained.
