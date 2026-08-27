add_executable(robometrics_tests main.cpp test_se3.cpp test_urdf.cpp
                                test_kinematics.cpp test_jacobian.cpp test_metrics.cpp
                                test_rollout.cpp test_report.cpp test_cli.cpp)

target_link_libraries(
  robometrics_tests PRIVATE robometrics::robometrics doctest::doctest rapidcheck
                            robometrics_warnings)

# URDF fixtures are files on disk, not inline strings, so the load_file() path
# is exercised too. The directory is passed as a macro so the tests do not
# depend on where ctest runs them from.
target_compile_definitions(
  robometrics_tests PRIVATE
  ROBOMETRICS_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

add_test(NAME robometrics_tests COMMAND robometrics_tests)

# rapidcheck is driven by the RC_PARAMS environment variable, e.g. when
# reproducing a failure:
#   RC_PARAMS="seed=42 max_success=1000" ctest --preset debug-asan
set_tests_properties(robometrics_tests PROPERTIES TIMEOUT 300)
