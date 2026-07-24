# HGA-SP

HGA-SP is a hybrid genetic algorithm with a set-partitioning matheuristic for
the capacitated location-routing problem (CLRP).

**Author:** Feiran Wang

**Status:** Active research project; the associated manuscript is in preparation.

The project is distributed under the [MIT License](LICENSE).

## Running the programs

To generate the HGA-SP executable (`ilrp`) for the CLRP, run `make` under the
`code` folder on a Linux system.

### The CLRP

_Usage:_

./code/ilrp instance depot_config outputFile bestValue isOptimal timeLimit iterLimit randomSeed

- 'instance' is the input benchamrk instance
- 'depot_config' denotes the input depot configuration associated with the input instance, and this parameter can be removed
- 'outputFile' is the name of output file 
- 'bestValue' is the best result found from literature.
- 'isOptimal' is used to indicate whether the bestValue is the optimal result.
- 'timeLimit' is the time limit of each run
- 'iterLimit' is the maximum number of iterations
- 'randomSeed' is the random seed 

_See the scripts for details._

## Materials
This repository contains:

- HGA-SP source code under `code/`.
- Experiment and utility scripts under `scripts/`.
- Public benchmark instances and depot configurations under `instances/`.

Generated results, solver logs, compiled binaries, local environments, and
private credentials are intentionally excluded from version control.
