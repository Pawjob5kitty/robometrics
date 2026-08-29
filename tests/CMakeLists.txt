# nlohmann/json is a TEST-ONLY dependency: it parses the JSON the tool writes,
# so the round-trip and structure tests validate the output with a real parser
# rather than our own. The shipped library and CLI never link it -- the JSON
# writer is hand-rolled and depends on nothing.
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.3 # 9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(nlohmann_json)
robometrics_mark_system(nlohmann_json::nlohmann_json)

add_executable(robometrics_tests main.cpp test_se3.cpp test_urdf.cpp
                                test_kinematics.cpp test_jacobian.cpp test_metrics.cpp
                                test_rollout.cpp test_report.cpp test_report_json.cpp
                                test_cli.cpp)

target_link_libraries(
  robometrics_tests PRIVATE robometrics::robometrics doctest::doctest rapidcheck
                            nlohmann_json::nlohmann_json robometrics_warnings)

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
