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
    code/evaluation/RouteEvaluator.cpp
    code/io/FarhamLrptwParser.cpp
    code/io/InstanceLoader.cpp
    code/io/SetSDatParser.cpp
    code/io/SetSJsonParser.cpp
    code/models/Individual.cpp
    code/models/RoutePool.cpp
    code/models/popManager.cpp
    code/operators/LS.cpp
    code/operators/FeasibleInsertionFinder.cpp
    code/operators/RepairProcedure.cpp
    code/operators/crossover.cpp
    code/operators/initialsol.cpp
    code/operators/mutation.cpp
    code/problem/ProblemMode.cpp
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
  add_executable(
    hgasp_run_logger_test
    tests/unit/run_logger_test.cpp
    code/utils/RunLogger.cpp
  )
  target_include_directories(
    hgasp_run_logger_test
    PRIVATE ${HGASP_INCLUDE_DIRS}
  )
  target_compile_features(hgasp_run_logger_test PRIVATE cxx_std_17)
  target_link_libraries(
    hgasp_run_logger_test
    PRIVATE Threads::Threads
  )
  hgasp_enable_warnings(hgasp_run_logger_test)
  add_test(
    NAME logging.bounded_structured_logger
    COMMAND hgasp_run_logger_test
  )
  set_tests_properties(
    logging.bounded_structured_logger
    PROPERTIES
      LABELS "unit"
      TIMEOUT 30
  )

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
  add_test(
    NAME cli.reject_invalid_sp_trigger_offset
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
  set_tests_properties(
    cli.reject_invalid_sp_trigger_offset
    PROPERTIES
      ENVIRONMENT
        "HGASP_SP_TRIGGER_FREQUENCY=10;HGASP_SP_TRIGGER_OFFSET=10"
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
    hgasp_route_evaluator_test
    tests/unit/route_evaluator_test.cpp
  )
  target_link_libraries(
    hgasp_route_evaluator_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_route_evaluator_test)
  add_test(
    NAME route.mode_constraint_evaluator
    COMMAND hgasp_route_evaluator_test
  )

  add_executable(
    hgasp_tw_feasible_insertion_test
    tests/unit/feasible_insertion_finder_test.cpp
  )
  target_link_libraries(
    hgasp_tw_feasible_insertion_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_tw_feasible_insertion_test)
  add_test(
    NAME route.tw_feasible_insertion
    COMMAND hgasp_tw_feasible_insertion_test
  )

  add_executable(
    hgasp_tw_mutation_test
    tests/unit/tw_mutation_test.cpp
  )
  target_link_libraries(
    hgasp_tw_mutation_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_tw_mutation_test)
  add_test(
    NAME route.tw_mutation
    COMMAND hgasp_tw_mutation_test
  )

  add_executable(
    hgasp_tw_model_integrity_test
    tests/unit/tw_model_integrity_test.cpp
  )
  target_link_libraries(
    hgasp_tw_model_integrity_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_tw_model_integrity_test)
  add_test(
    NAME route.tw_model_integrity
    COMMAND hgasp_tw_model_integrity_test
  )

  add_executable(
    hgasp_farham_benchmark_semantics_test
    tests/unit/farham_benchmark_semantics_test.cpp
  )
  target_link_libraries(
    hgasp_farham_benchmark_semantics_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_farham_benchmark_semantics_test)
  add_test(
    NAME instance.farham_benchmark_semantics
    COMMAND
      hgasp_farham_benchmark_semantics_test
      "${CMAKE_SOURCE_DIR}"
  )

  add_executable(
    hgasp_instance_mode_parser_test
    tests/unit/instance_mode_parser_test.cpp
  )
  target_link_libraries(
    hgasp_instance_mode_parser_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_instance_mode_parser_test)
  add_test(
    NAME instance.mode_and_parser
    COMMAND hgasp_instance_mode_parser_test "${CMAKE_SOURCE_DIR}"
  )

  add_executable(
    hgasp_route_generator_memory_test
    tests/unit/route_generator_memory_test.cpp
  )
  target_link_libraries(
    hgasp_route_generator_memory_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_route_generator_memory_test)
  add_test(
    NAME route_generator.memory_layout
    COMMAND hgasp_route_generator_memory_test
  )

  add_executable(
    hgasp_route_pool_admission_decoupling_test
    tests/unit/route_pool_admission_decoupling_test.cpp
  )
  target_link_libraries(
    hgasp_route_pool_admission_decoupling_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_route_pool_admission_decoupling_test
  )
  add_test(
    NAME route_pool.admission_decoupling
    COMMAND hgasp_route_pool_admission_decoupling_test
  )

  add_executable(hgasp_population_structural_duplicate_test
    tests/unit/population_structural_duplicate_test.cpp
  )
  target_link_libraries(hgasp_population_structural_duplicate_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_population_structural_duplicate_test
  )
  add_test(
    NAME population.structural_duplicate
    COMMAND hgasp_population_structural_duplicate_test
  )

  add_executable(
    hgasp_route_pool_column_dominance_test
    tests/unit/route_pool_column_dominance_test.cpp
  )
  target_link_libraries(
    hgasp_route_pool_column_dominance_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_route_pool_column_dominance_test
  )
  add_test(
    NAME route_pool.column_dominance
    COMMAND hgasp_route_pool_column_dominance_test
  )

  add_executable(
    hgasp_route_pool_tiered_pruning_test
    tests/unit/route_pool_tiered_pruning_test.cpp
  )
  target_link_libraries(
    hgasp_route_pool_tiered_pruning_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_route_pool_tiered_pruning_test
  )
  add_test(
    NAME route_pool.tiered_pruning
    COMMAND hgasp_route_pool_tiered_pruning_test
  )

  add_executable(
    hgasp_representation_route_bound_test
    tests/unit/representation_route_bound_test.cpp
  )
  target_link_libraries(
    hgasp_representation_route_bound_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_representation_route_bound_test
  )
  add_test(
    NAME representation.safe_route_bound
    COMMAND hgasp_representation_route_bound_test
  )

  add_executable(
    hgasp_eax_parent_mode_test
    tests/unit/eax_parent_mode_test.cpp
  )
  target_link_libraries(
    hgasp_eax_parent_mode_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_eax_parent_mode_test)
  add_test(
    NAME crossover.eax_parent_mode
    COMMAND hgasp_eax_parent_mode_test
  )

  add_executable(
    hgasp_configuration_local_sp_test
    tests/unit/configuration_local_sp_test.cpp
  )
  target_link_libraries(
    hgasp_configuration_local_sp_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_configuration_local_sp_test
  )
  add_test(
    NAME sp.configuration_local
    COMMAND hgasp_configuration_local_sp_test
  )

  add_executable(
    hgasp_adaptive_sp_controller_test
    tests/unit/adaptive_sp_controller_test.cpp
  )
  target_link_libraries(
    hgasp_adaptive_sp_controller_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_adaptive_sp_controller_test
  )
  add_test(
    NAME sp.adaptive_controller
    COMMAND hgasp_adaptive_sp_controller_test
  )

  add_executable(
    hgasp_route_pool_dual_feedback_test
    tests/unit/route_pool_dual_feedback_test.cpp
  )
  target_link_libraries(
    hgasp_route_pool_dual_feedback_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_route_pool_dual_feedback_test
  )
  add_test(
    NAME route_pool.dual_feedback
    COMMAND hgasp_route_pool_dual_feedback_test
  )

  add_executable(
    hgasp_diverse_sp_solution_selector_test
    tests/unit/diverse_sp_solution_selector_test.cpp
  )
  target_link_libraries(
    hgasp_diverse_sp_solution_selector_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_diverse_sp_solution_selector_test
  )
  add_test(
    NAME sp.diverse_solution_selector
    COMMAND hgasp_diverse_sp_solution_selector_test
  )

  add_executable(
    hgasp_adaptive_search_allocator_test
    tests/unit/adaptive_search_allocator_test.cpp
  )
  target_link_libraries(
    hgasp_adaptive_search_allocator_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(
    hgasp_adaptive_search_allocator_test
  )
  add_test(
    NAME search.adaptive_allocator
    COMMAND hgasp_adaptive_search_allocator_test
  )

  add_executable(
    hgasp_search_control_safety_test
    tests/unit/search_control_safety_test.cpp
  )
  target_link_libraries(
    hgasp_search_control_safety_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_search_control_safety_test)
  add_test(
    NAME search.control_safety
    COMMAND hgasp_search_control_safety_test
  )
  set_tests_properties(
    search.control_safety
    PROPERTIES TIMEOUT 10
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
    hgasp_sp_trigger_policy_test
    tests/unit/sp_trigger_policy_test.cpp
  )
  target_link_libraries(
    hgasp_sp_trigger_policy_test
    PRIVATE HGASP::engine
  )
  hgasp_enable_warnings(hgasp_sp_trigger_policy_test)
  add_test(
    NAME sp.trigger_policy
    COMMAND hgasp_sp_trigger_policy_test
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
