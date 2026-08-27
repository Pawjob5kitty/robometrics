cmake_minimum_required(VERSION 3.20)

project(
  robometrics
  VERSION 0.1.0
  DESCRIPTION "Kinematicke metriky pro offline hodnoceni robotickych politik"
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
# Zavislosti
# ---------------------------------------------------------------------------
include(FetchContent)

# Nechceme stavet testy/dokumentaci/benchmarky zavislosti.
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

# rapidcheck nedela release ani tagy, takze pin je primo na commit.
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

# Hlavicky zavislosti musi byt SYSTEM, jinak -Wconversion -Wshadow -Werror pada
# na cizim kodu (Eigen/doctest/rapidcheck jsou template-heavy a warningy se
# pripisuji nasi translacni jednotce). FetchContent umi klicove slovo SYSTEM az
# od CMake 3.25; tohle funguje i na 3.20.
function(robometrics_mark_system)
  foreach(tgt IN LISTS ARGN)
    if(NOT TARGET ${tgt})
      message(FATAL_ERROR "robometrics: ocekavany cil '${tgt}' neexistuje")
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
# Spolecne prepinace prekladace
# ---------------------------------------------------------------------------
# Poznamka: warningy visi na INTERFACE cili, ktery linkujeme PRIVATE jen do
# vlastnich cilu. Kdyby byly globalne pres add_compile_options, sestreli
# -Werror kazdou zavislost.
add_library(robometrics_warnings INTERFACE)

if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
  # cl.exe nebo clang-cl: -W* prepinace neexistuji.
  target_compile_options(robometrics_warnings INTERFACE /W4 /WX /permissive-)
else()
  target_compile_options(
    robometrics_warnings INTERFACE -Wall -Wextra -Wconversion -Wshadow -Werror)
endif()

# ---------------------------------------------------------------------------
# Knihovna
# ---------------------------------------------------------------------------
add_library(robometrics src/se3.cpp src/urdf.cpp src/kinematics.cpp src/jacobian.cpp)
add_library(robometrics::robometrics ALIAS robometrics)

target_include_directories(
  robometrics PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(robometrics PUBLIC cxx_std_20)

target_link_libraries(
  robometrics
  PUBLIC Eigen3::Eigen
  PRIVATE pugixml::pugixml robometrics_warnings)

# ---------------------------------------------------------------------------
# Testy
# ---------------------------------------------------------------------------
if(ROBOMETRICS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
