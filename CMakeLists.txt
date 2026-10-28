cmake_minimum_required(VERSION 3.20)
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
project(MA_SP LANGUAGES CXX)

if(NOT UNIX)
  message(FATAL_ERROR "MA-SP requires a POSIX platform")
endif()
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Threads REQUIRED)
find_package(Gurobi REQUIRED)

add_library(masp_core STATIC
  code/FarhamParser.cpp
  code/Individual.cpp
  code/LS.cpp
  code/RepairProcedure.cpp
  code/RoutePool.cpp
  code/SPSolver.cpp
  code/TimeWindowOperators.cpp
  code/crossover.cpp
  code/initialsol.cpp
  code/mutation.cpp
  code/popManager.cpp
  code/read_data.cpp
  code/solution.cpp
)
add_executable(masp code/main.cpp)
target_link_libraries(masp PRIVATE masp_core)
set_target_properties(masp PROPERTIES
  BUILD_RPATH "${Gurobi_LIBRARY_DIR}"
  INSTALL_RPATH "${Gurobi_LIBRARY_DIR}"
)
target_include_directories(masp_core PUBLIC code)
target_compile_options(masp_core PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>
)
target_link_libraries(masp_core PUBLIC Gurobi::CXX Threads::Threads ${CMAKE_DL_LIBS})
if(UNIX)
  target_link_libraries(masp_core PUBLIC m)
endif()

install(TARGETS masp RUNTIME DESTINATION bin)

option(MASP_BUILD_CHECKS "Build the explicitly requested regression checks" OFF)
if(MASP_BUILD_CHECKS)
  enable_testing()
  foreach(check IN ITEMS commandline population_crossover route_state input_data)
    add_executable(${check}_check tests/${check}.cpp)
    target_link_libraries(${check}_check PRIVATE masp_core)
    set_target_properties(${check}_check PROPERTIES BUILD_RPATH "${Gurobi_LIBRARY_DIR}")
    add_test(NAME ${check} COMMAND ${check}_check)
    set_tests_properties(${check} PROPERTIES TIMEOUT 30)
  endforeach()
  set_tests_properties(route_state PROPERTIES TIMEOUT 15)
endif()
