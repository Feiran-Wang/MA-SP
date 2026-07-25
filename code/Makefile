# Compatibility wrapper for users accustomed to running `make` in code/.
# CMake is the single source of build logic.

CMAKE ?= cmake
BUILD_TYPE ?= Release
BUILD_DIR ?= $(abspath $(CURDIR)/../build/release-gurobi)
SOURCE_DIR := $(abspath $(CURDIR)/..)
CMAKE_ARGS ?=

ifneq ($(strip $(GUROBI_ROOT)),)
CMAKE_ARGS += -DGUROBI_ROOT="$(GUROBI_ROOT)"
endif

.PHONY: all configure hgasp test clean

all: hgasp

configure:
	$(CMAKE) -S "$(SOURCE_DIR)" -B "$(BUILD_DIR)" \
		-DCMAKE_BUILD_TYPE="$(BUILD_TYPE)" \
		$(CMAKE_ARGS)

hgasp: configure
	$(CMAKE) --build "$(BUILD_DIR)" --target hgasp --parallel
	$(CMAKE) -E rm -f "$(CURDIR)/ilrp"
	$(CMAKE) -E create_symlink "$(BUILD_DIR)/hgasp" "$(CURDIR)/ilrp"

test: configure
	$(CMAKE) --build "$(BUILD_DIR)" --parallel
	ctest --test-dir "$(BUILD_DIR)" --output-on-failure

clean:
	@if test -f "$(BUILD_DIR)/CMakeCache.txt"; then \
		$(CMAKE) --build "$(BUILD_DIR)" --target clean; \
	fi
	$(CMAKE) -E rm -f "$(CURDIR)/ilrp"
