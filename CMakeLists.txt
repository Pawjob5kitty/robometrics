cmake_minimum_required(VERSION 3.20)

project(
  robometrics
  VERSION 0.1.0
  DESCRIPTION "Kinematic metrics for offline evaluation of robot policies"
  LANGUAGES CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(ROBOMETRICS_IS_TOP_LEVEL ON)
else()
  set(ROBOMETRICS_IS_TOP_LEVEL OFF)
endif()

option(ROBOMETRICS_BUILD_TESTS "Build the robometrics test suite"
       ${ROBOMETRICS_IS_TOP_LEVEL})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
include(FetchContent)

# We do not want to build the dependencies' own tests/docs/benchmarks.
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_BLAS OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_LAPACK OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_CMAKE_PACKAGE OFF CACHE BOOL "" FORCE)
set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(DOCTEST_NO_INSTALL ON CACHE BOOL "" FORCE)
set(RC_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
set(RC_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PUGIXML_BUILD_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.1 # d71c30c47858effcbd39967097a2d99ee48db464
  GIT_SHALLOW TRUE)

FetchContent_Declare(
  doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.5.3 # 2d0a9359a60c51affe2a9bebb1be1dca47868151
  GIT_SHALLOW TRUE)

# rapidcheck cuts no releases and no tags, so the pin is a commit directly.
FetchContent_Declare(
  rapidcheck
  GIT_REPOSITORY https://github.com/emil-e/rapidcheck.git
  GIT_TAG b2d9ed2dddefc4b84318d664b4f221eb792d89c7)

FetchContent_Declare(
  pugixml
  GIT_REPOSITORY https://github.com/zeux/pugixml.git
  GIT_TAG v1.15 # ee86beb30e4973f5feffe3ce63bfa4fbadf72f38
  GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(eigen doctest rapidcheck pugixml)

# Dependency headers must be SYSTEM, otherwise -Wconversion -Wshadow -Werror
# fires on third-party code (Eigen/doctest/rapidcheck are template-heavy and the
# warnings are attributed to our translation unit). FetchContent gained the
# SYSTEM keyword only
# in CMake 3.25; this works on 3.20 too.
function(robometrics_mark_system)
  foreach(tgt IN LISTS ARGN)
    if(NOT TARGET ${tgt})
      message(FATAL_ERROR "robometrics: expected target '${tgt}' does not exist")
    endif()
    get_target_property(real ${tgt} ALIASED_TARGET)
    if(NOT real)
      set(real ${tgt})
    endif()
    get_target_property(dirs ${real} INTERFACE_INCLUDE_DIRECTORIES)
    if(dirs)
      set_target_properties(${real} PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
                                               "${dirs}")
    endif()
  endforeach()
endfunction()

robometrics_mark_system(Eigen3::Eigen doctest::doctest rapidcheck
                        pugixml::pugixml)

# ---------------------------------------------------------------------------
# Common compiler switches
# ---------------------------------------------------------------------------
# Note: warnings hang on an INTERFACE target linked PRIVATE only into our own
# targets. If they were global via add_compile_options they would take down
# -Werror kazdou zavislost.
add_library(robometrics_warnings INTERFACE)

if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
  # cl.exe or clang-cl: the -W* switches do not exist.
  target_compile_options(robometrics_warnings INTERFACE /W4 /WX /permissive-)
else()
  target_compile_options(
    robometrics_warnings INTERFACE -Wall -Wextra -Wconversion -Wshadow -Werror)
endif()

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(robometrics src/se3.cpp src/urdf.cpp src/kinematics.cpp src/jacobian.cpp
                        src/metrics.cpp src/rollout.cpp src/report.cpp src/cli.cpp)
add_library(robometrics::robometrics ALIAS robometrics)

target_include_directories(
  robometrics PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(robometrics PUBLIC cxx_std_20)

target_link_libraries(
  robometrics
  PUBLIC Eigen3::Eigen
  PRIVATE pugixml::pugixml robometrics_warnings)

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
# main() is deliberately trivial; all logic lives in runCli() so it can be
# tested without spawning a process. See include/robometrics/cli.hpp.
add_executable(robometrics_cli apps/main.cpp)
set_target_properties(robometrics_cli PROPERTIES OUTPUT_NAME robometrics)
target_link_libraries(robometrics_cli PRIVATE robometrics::robometrics
                                              robometrics_warnings)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(ROBOMETRICS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
