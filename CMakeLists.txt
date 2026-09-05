cmake_minimum_required(VERSION 3.20)

project(
  HGASP
  VERSION 4.1
  DESCRIPTION "Hybrid genetic algorithm with a set-partitioning matheuristic"
  LANGUAGES CXX
)

option(
  HGASP_ENABLE_HOT_PATH_COUNTERS
  "Compile low-overhead diagnostic counters for performance triage"
  OFF
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(HGASP_INCLUDE_DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}/code/core"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/evaluation"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/io"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/models"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/operators"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/problem"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/solver"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/utils"
)

function(hgasp_enable_warnings target_name)
  target_compile_options(
    "${target_name}"
    PRIVATE
      $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic>
  )
endfunction()

find_package(Threads REQUIRED)
find_package(Gurobi REQUIRED)
message(STATUS "HGA-SP Gurobi backend: ${Gurobi_VERSION}")

add_library(
  hgasp_sp
  STATIC
    code/solver/SPSolver.cpp
)
add_library(HGASP::sp ALIAS hgasp_sp)
target_include_directories(hgasp_sp PUBLIC ${HGASP_INCLUDE_DIRS})
target_compile_features(hgasp_sp PUBLIC cxx_std_17)
hgasp_enable_warnings(hgasp_sp)

target_link_libraries(
  hgasp_sp
  PUBLIC
    Gurobi::CXX
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

# HGA-SP algorithm engine.
add_library(
  hgasp_engine
  STATIC
    code/core/AlgorithmOrchestrator.cpp
    code/core/ConfigurationCandidateArchive.cpp
    code/core/DynamicChallengerPolicy.cpp
    code/core/GlobalSpPhasePolicy.cpp
    code/core/LocalSpFamilyBudget.cpp
    code/core/PopulationLocalSpController.cpp
    code/core/PopulationReductionPolicy.cpp
    code/core/RouteGenerator.cpp
    code/core/StaticConfigurationRegistry.cpp
    code/core/StaticHgampPopulationPolicy.cpp
    code/core/SolverContext.cpp
    code/evaluation/RouteEvaluator.cpp
    code/io/FarhamLrptwParser.cpp
    code/io/InstanceLoader.cpp
    code/io/SetSDatParser.cpp
    code/io/SetSJsonParser.cpp
    code/models/Individual.cpp
    code/models/RoutePool.cpp
    code/models/popManager.cpp
    code/operators/LS.cpp
    code/operators/PromisingArcCandidates.cpp
    code/operators/FeasibleInsertionFinder.cpp
    code/operators/RepairProcedure.cpp
    code/operators/CrhConfigurationRanker.cpp
    code/operators/SearchComponents.cpp
    code/operators/SpatioTemporalCrh.cpp
    code/operators/crossover.cpp
    code/operators/initialsol.cpp
    code/operators/mutation.cpp
    code/problem/ProblemContext.cpp
    code/problem/ProblemProfile.cpp
    code/utils/ComponentProfiler.cpp
)
add_library(HGASP::engine ALIAS hgasp_engine)
target_include_directories(hgasp_engine PUBLIC ${HGASP_INCLUDE_DIRS})
target_compile_features(hgasp_engine PUBLIC cxx_std_17)
target_link_libraries(hgasp_engine PUBLIC HGASP::sp)
if(HGASP_ENABLE_HOT_PATH_COUNTERS)
  target_compile_definitions(
    hgasp_engine
    PUBLIC HGASP_ENABLE_HOT_PATH_COUNTERS=1
  )
endif()
hgasp_enable_warnings(hgasp_engine)

add_executable(
  hgasp
  code/core/main.cpp
  code/utils/RunLogger.cpp
)
add_executable(HGASP::hgasp ALIAS hgasp)
target_link_libraries(hgasp PRIVATE HGASP::engine)
target_compile_features(hgasp PRIVATE cxx_std_17)
hgasp_enable_warnings(hgasp)

if(UNIX AND NOT APPLE)
  # Make build-tree binaries runnable without a caller-managed
  # LD_LIBRARY_PATH. The path comes from the discovered versioned C library.
  set_target_properties(
    hgasp
    hgasp_sp
    PROPERTIES BUILD_RPATH "${Gurobi_LIBRARY_DIR}"
  )
endif()
