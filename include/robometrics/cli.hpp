#pragma once

#include <iosfwd>
#include <string>
#include <vector>

/// The command line, as a library function.
///
/// The streams are parameters so the whole run is callable in process: a test
/// hands it real files on disk and two stringstreams and gets the exit code
/// back. Spawning the built binary instead would add subprocess plumbing whose
/// failures look exactly like failures of the code under test.
namespace robometrics {

/// Runs the CLI.
///
/// `args` excludes the program name. `out` receives the report CSV when no
/// --out file is given; `err` always receives the summary and warnings, even
/// when the CSV goes to a file, so a pipeline can consume the CSV while a
/// person still sees what happened.
///
/// Returns the exit code: 0 when at least one rollout was analysed, non-zero
/// otherwise. A single corrupt file among forty should not fail the run; forty
/// corrupt files should.
///
/// Does not throw. Every failure is reported through `err` and the exit code.
int runCli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

}  // namespace robometrics
