# MA-SP

**Memetic Algorithm with Set Partitioning for Capacitated Location-Routing**

MA-SP combines population search, local improvement, and route-pool set
partitioning for the capacitated location-routing problem (CLRP) and its hard
time-window variant (CLRP-TW). The executable uses the Gurobi C++ API.

## Build and install

Requirements: Linux/POSIX, a C++11 compiler, CMake 3.20+, and the Gurobi C/C++ SDK
with a valid license. The supplied implementation uses Gurobi 13.0.2.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGUROBI_ROOT=/opt/gurobi1302/linux64
cmake --build build --target masp --parallel 2
```

The executable is `build/masp`. `GUROBI_ROOT` accepts an installation prefix or
its platform directory; `GUROBI_HOME` is also supported. Build output and local
license files are excluded from Git. Optional installation:

```bash
cmake --install build --prefix "$HOME/.local"
```

## Run

```text
masp INSTANCE [DEPOT_CONFIG] OUTPUT BEST_VALUE IS_OPTIMAL CPU_LIMIT MAIN_COUNTER_LIMIT SEED
```

- `INSTANCE`: Set S JSON, supported CLRP DAT, or Farham Set F TXT.
- `DEPOT_CONFIG`: Set S depot configurations with zero-based depot indices.
  Omit it or use `NULL` for automatic configuration generation; Set F uses `NULL`.
- `OUTPUT`: solution file in an existing, writable directory.
- `BEST_VALUE`: a finite, nonnegative external target; `0` disables target stopping.
- `IS_OPTIMAL`: `0` or `1`. With `1`, matching the positive supplied target can
  stop the search.
- `CPU_LIMIT`: a positive integer number of process CPU seconds.
- `MAIN_COUNTER_LIMIT`: a nonnegative integer smaller than `INT_MAX`. The
  existing main-counter stop rule is `counter > limit`.
- `SEED`: a nonnegative integer seed up to `INT_MAX`.

The existing family profiles remain available without environment settings.
The v22 Set S Full profile uses the following explicit configuration:

```bash
mkdir -p output
env MASP_SETS_EXPERIMENT=1 MASP_ARM=Full \
  MASP_R_CFG=15 MASP_MU=20 MASP_ALPHA=20 MASP_P_MUT=15 \
  MASP_RHO=0.10 MASP_POOL=25000 MASP_SP_POLICY=F20001_b1 \
  ./build/masp examples/100-5-1c.json examples/depot_configuration_100-5-1c.txt \
    output/solution.txt 0 0 60000 300000 1
```

CPU and main-counter limits are independent; `100 300000` is also accepted.
Configuration settings and supported parameter values are defined in
[SearchParameters.h](code/SearchParameters.h). `MASP_SETS_EXPERIMENT=1` and
`MASP_SETF_EXPERIMENT=1` select the corresponding configurable family profile.
`MASP_ARM=Full` enables the complete algorithm; other arms retain completed
ablation configurations. Use a separate working directory for each concurrent
run because solver logs and optional diagnostic files are relative to it.

Solution costs are written with 17 significant digits. Set S and Farham Set F
use different distance conventions; their costs must be compared using the
matching input metric. Set S requires positive customer demands and vehicle
capacity. Depot bit signatures support at most 62 depots. Malformed or
unsupported input is rejected with a nonzero exit status.

The set-partitioning stage optimizes the retained route pool; `OPTIMAL` denotes
the pool model's optimum.

## Explicit regression checks

Checks are disabled in a normal build. They exercise input parsing, crossover
boundaries, route state, and population state without launching an optimization
study:

```bash
cmake -S . -B build-checks -DCMAKE_BUILD_TYPE=Release -DMASP_BUILD_CHECKS=ON \
  -DGUROBI_ROOT=/opt/gurobi1302/linux64
cmake --build build-checks --parallel 2
ctest --test-dir build-checks --output-on-failure
```

## License

MIT License; see [LICENSE](LICENSE). Maintainer: Feiran Wang. Original source
author credits are retained.
