#include <iostream>
#include <string>
#include <vector>

#include "robometrics/cli.hpp"

/// Five lines that cannot contain a bug. Everything testable lives in runCli;
/// see cli.hpp for why the streams are parameters.
int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  return robometrics::runCli(args, std::cout, std::cerr);
}
