add_executable(robometrics_tests main.cpp test_se3.cpp test_urdf.cpp
                                test_kinematics.cpp test_jacobian.cpp test_metrics.cpp
                                test_rollout.cpp test_report.cpp test_cli.cpp)

target_link_libraries(
  robometrics_tests PRIVATE robometrics::robometrics doctest::doctest rapidcheck
                            robometrics_warnings)

# URDF fixtury jsou soubory na disku, ne inline stringy — testujeme i cestu pres
# load_file(). Cesta se predava jako makro, aby test nezavisel na tom, odkud ho
# ctest spusti.
target_compile_definitions(
  robometrics_tests PRIVATE
  ROBOMETRICS_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")

add_test(NAME robometrics_tests COMMAND robometrics_tests)

# rapidcheck se ridi promennou prostredi RC_PARAMS, napr. pri ladeni padu:
#   RC_PARAMS="seed=42 max_success=1000" ctest --preset debug-asan
set_tests_properties(robometrics_tests PROPERTIES TIMEOUT 300)
