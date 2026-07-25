cmake_minimum_required(VERSION 3.20)

project(
  HGASP
  VERSION 0.1.0
  DESCRIPTION "Hybrid genetic algorithm with a set-partitioning matheuristic"
  LANGUAGES CXX
)

option(
  HGASP_BUILD_GUROBI_SMOKE_TEST
  "Build a minimal Gurobi C++ API runtime test"
  ON
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(CTest)

set(HGASP_INCLUDE_DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}/code/core"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/models"
  "${CMAKE_CURRENT_SOURCE_DIR}/code/operators"
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

add_library(hgasp_sp STATIC code/solver/SPSolver.cpp)
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
    code/core/RouteGenerator.cpp
    code/core/read_data.cpp
    code/models/Individual.cpp
    code/models/RoutePool.cpp
    code/models/popManager.cpp
    code/operators/LS.cpp
    code/operators/RepairProcedure.cpp
    code/operators/crossover.cpp
    code/operators/initialsol.cpp
    code/operators/mutation.cpp
)
add_library(HGASP::engine ALIAS hgasp_engine)
target_include_directories(hgasp_engine PUBLIC ${HGASP_INCLUDE_DIRS})
target_compile_features(hgasp_engine PUBLIC cxx_std_17)
target_link_libraries(hgasp_engine PUBLIC HGASP::sp)
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

if(BUILD_TESTING)
  add_test(NAME cli.help COMMAND hgasp --help)
  add_test(
    NAME cli.reject_negative_seed
    COMMAND hgasp missing.instance unused.solution 0 0 1 1 -1
  )
  add_test(
    NAME cli.reject_trailing_numeric_text
    COMMAND hgasp missing.instance unused.solution 0 0 1x 1 1
  )
  add_test(
    NAME cli.reject_invalid_sp_threads
    COMMAND hgasp missing.instance unused.solution 0 0 1 1 1
  )
  set_tests_properties(
    cli.reject_negative_seed
    cli.reject_trailing_numeric_text
    PROPERTIES WILL_FAIL TRUE
  )
  set_tests_properties(
    cli.reject_invalid_sp_threads
    PROPERTIES
      ENVIRONMENT "HGASP_SP_THREADS=0"
      WILL_FAIL TRUE
  )

  add_executable(
    hgasp_route_cost_contract_test
    tests/unit/route_cost_contract_test.cpp
  )
  target_link_libraries(
    hgasp_route_cost_contract_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_route_cost_contract_test)
  add_test(
    NAME route.directed_cost_contract
    COMMAND hgasp_route_cost_contract_test
  )

  add_executable(
    hgasp_sp_input_validation_test
    tests/unit/sp_input_validation_test.cpp
  )
  target_link_libraries(hgasp_sp_input_validation_test PRIVATE HGASP::sp)
  hgasp_enable_warnings(hgasp_sp_input_validation_test)
  add_test(
    NAME sp.input_validation
    COMMAND hgasp_sp_input_validation_test
  )

  add_executable(
    hgasp_sp_solver_test
    tests/integration/sp_solver_test.cpp
  )
  target_link_libraries(hgasp_sp_solver_test PRIVATE HGASP::sp)
  hgasp_enable_warnings(hgasp_sp_solver_test)
  add_test(NAME sp.known_optimum COMMAND hgasp_sp_solver_test)
  set_tests_properties(
    sp.known_optimum
    PROPERTIES
      LABELS "gurobi;integration"
      TIMEOUT 60
  )

  if(HGASP_BUILD_GUROBI_SMOKE_TEST)
    add_executable(
      hgasp_gurobi_smoke
      tests/integration/gurobi_smoke.cpp
    )
    target_link_libraries(
      hgasp_gurobi_smoke
      PRIVATE
        Gurobi::CXX
        Threads::Threads
        ${CMAKE_DL_LIBS}
    )
    target_compile_features(hgasp_gurobi_smoke PRIVATE cxx_std_17)
    hgasp_enable_warnings(hgasp_gurobi_smoke)
    add_test(NAME gurobi.smoke COMMAND hgasp_gurobi_smoke)
    set_tests_properties(
      gurobi.smoke
      PROPERTIES
        LABELS "gurobi;integration"
        TIMEOUT 30
    )
  endif()
endif()
