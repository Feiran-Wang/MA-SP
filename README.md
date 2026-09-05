# HGA-SP

HGA-SP is an academic solver for capacitated location-routing problems. This
repository contains version `v4.1` of the solver implementation.

Author: Feiran Wang

## Requirements

- CMake 3.20 or newer
- A C++17 compiler
- A licensed Gurobi installation with the C++ API

Set `GUROBI_HOME` before configuring if Gurobi is not installed in a standard
location.

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target hgasp --parallel
```

## Submit a solve task

Create an output directory and run the included example:

```bash
mkdir -p output
./build/hgasp \
  examples/100-5-1c.json \
  examples/depot_configuration_100-5-1c.txt \
  output/solution.txt \
  0 0 60 1000 1
```

The general command format is:

```text
hgasp INSTANCE [DEPOT_CONFIG] OUTPUT BEST_VALUE IS_OPTIMAL \
  TIME_LIMIT ITERATION_LIMIT SEED
```

`DEPOT_CONFIG` is required only by instance formats that use a separate depot
configuration file. Use `./build/hgasp --help` for the argument contract. The
solution is written to `OUTPUT`; runtime logs are written below `logs/` by
default.

## License

See [`LICENSE`](LICENSE).
