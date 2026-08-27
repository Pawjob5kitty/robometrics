#pragma once

#include <iosfwd>
#include <string>
#include <vector>

/// The command line, as a library function.
///
/// WHY THIS IS NOT JUST main(). The whole point of L5 is the vertical slice --
/// file in, CSV out -- and a slice is only trustworthy if it is tested end to
/// end. Testing it by spawning the built binary means the test has to find the
/// executable, deal with a shell, and parse its own subprocess plumbing; that
/// is a lot of machinery whose failures look exactly like failures of the code
/// under test.
///
/// Taking the streams as parameters instead makes the whole run callable in
/// process: a test hands it real files on disk and two stringstreams, and gets
/// the exit code back as a return value. main() is then five lines that cannot
/// contain a bug.
namespace robometrics {

/// Runs the CLI.
///
/// `args` is the argument list WITHOUT the program name, i.e. argv[1..argc-1].
/// `out` receives the report CSV when no --out file is given; `err` receives
/// the human-readable summary, warnings and error messages -- always, even
/// when the CSV goes to a file, so that a shell pipeline can consume the CSV
/// on stdout while a person still sees what happened.
///
/// Returns the process exit code: 0 when at least one rollout was analysed
/// successfully, non-zero otherwise. That rule is what makes the tool usable
/// in a batch: a single corrupt file among forty should not fail the run, but
/// forty corrupt files should.
///
/// Does not throw. Every failure is reported through `err` and the exit code.
int runCli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err);

}  // namespace robometrics
