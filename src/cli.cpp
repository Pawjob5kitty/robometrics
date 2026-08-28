#include "robometrics/cli.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <ostream>
#include <sstream>
#include <vector>

#include <Eigen/SVD>

#include "robometrics/jacobian.hpp"
#include "robometrics/metrics.hpp"
#include "robometrics/report.hpp"
#include "robometrics/rollout.hpp"
#include "robometrics/urdf.hpp"

namespace robometrics {
namespace {

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------

struct Options {
  std::string urdf;
  std::string tipLink;  // empty means "let the URDF loader auto-detect"
  std::string out;      // empty means "write the CSV to the out stream"
  std::string profileOut;
  double threshold = kDefaultDexterityThreshold;
  double charLength = 0.0;  // 0 means "compute it from the URDF"
  std::vector<std::string> inputs;
  bool help = false;
};

const char* kUsage =
    "usage: robometrics analyze --urdf FILE [options] ROLLOUT...\n"
    "\n"
    "  --urdf FILE          robot description (required)\n"
    "  --tip LINK           end-effector link; needed when the URDF has more\n"
    "                       than one leaf, which is any robot with a gripper\n"
    "  --out FILE           write the report CSV here (default: stdout)\n"
    "  --profile-out DIR    also write one per-step profile CSV per rollout\n"
    "  --threshold VALUE    low-dexterity threshold in m/rad (default 0.05)\n"
    "  --char-length M      characteristic length for normalising dexterity, in\n"
    "                       metres; default: summed link lengths from the URDF\n"
    "  -h, --help           this text\n"
    "\n"
    "The report CSV goes to stdout unless --out is given; the summary always\n"
    "goes to stderr, so a pipeline can consume the CSV while a person reads\n"
    "the summary.\n";

/// Parses one `--flag value` pair. A flag that takes a value must find one:
/// running off the end of argv is an error, not a silently empty string --
/// `--urdf` with nothing after it is a typo, not a request to load "".
bool takeValue(const std::vector<std::string>& args, std::size_t& i, const char* flag,
               std::string& target, std::string& error) {
  if (i + 1 >= args.size()) {
    error = std::string(flag) + " needs a value";
    return false;
  }
  target = args[++i];
  return true;
}

bool parseOptions(const std::vector<std::string>& args, Options& opts, std::string& error) {
  std::size_t i = 0;

  // One subcommand today; requiring it keeps the door open for others without
  // breaking the existing invocation.
  if (i < args.size() && (args[i] == "-h" || args[i] == "--help")) {
    opts.help = true;
    return true;
  }
  if (i >= args.size()) {
    error = "no subcommand given";
    return false;
  }
  if (args[i] != "analyze") {
    error = "unknown subcommand '" + args[i] + "'; expected 'analyze'";
    return false;
  }
  ++i;

  for (; i < args.size(); ++i) {
    const std::string& a = args[i];
    if (a == "-h" || a == "--help") {
      opts.help = true;
      return true;
    } else if (a == "--urdf") {
      if (!takeValue(args, i, "--urdf", opts.urdf, error)) return false;
    } else if (a == "--tip") {
      if (!takeValue(args, i, "--tip", opts.tipLink, error)) return false;
    } else if (a == "--out") {
      if (!takeValue(args, i, "--out", opts.out, error)) return false;
    } else if (a == "--profile-out") {
      if (!takeValue(args, i, "--profile-out", opts.profileOut, error)) return false;
    } else if (a == "--threshold") {
      std::string text;
      if (!takeValue(args, i, "--threshold", text, error)) return false;
      // Order matters here. Extracting the number sets eofbit when it
      // consumes the whole string, and a subsequent `>> std::ws` on a stream
      // that is already at EOF fails its sentry and sets FAILBIT -- so
      // checking fail() after the whitespace skip rejects every valid value
      // that reaches the end of the argument, which is all of them. Check the
      // extraction first, then look for leftovers.
      std::istringstream in(text);
      in >> opts.threshold;
      const bool extracted = !in.fail();
      std::string leftover;
      in.clear();
      in >> leftover;
      if (!extracted || !leftover.empty() || !std::isfinite(opts.threshold) ||
          opts.threshold < 0.0) {
        error = "--threshold must be a non-negative finite number, got '" + text + "'";
        return false;
      }
    } else if (a == "--char-length") {
      std::string text;
      if (!takeValue(args, i, "--char-length", text, error)) return false;
      // Same extract-then-check-for-leftovers dance as --threshold above; see
      // there for why the order matters. Strictly positive, because it divides.
      std::istringstream in(text);
      in >> opts.charLength;
      const bool extracted = !in.fail();
      std::string leftover;
      in.clear();
      in >> leftover;
      if (!extracted || !leftover.empty() || !std::isfinite(opts.charLength) ||
          opts.charLength <= 0.0) {
        error = "--char-length must be a positive finite number, got '" + text + "'";
        return false;
      }
    } else if (a.rfind("--", 0) == 0) {
      // Unknown flags are rejected rather than treated as filenames: a
      // mistyped --thresold would otherwise be reported as a missing file and
      // send the reader looking in the wrong place.
      error = "unknown option '" + a + "'";
      return false;
    } else {
      opts.inputs.push_back(a);
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Formatting
// ---------------------------------------------------------------------------

/// Six significant digits, so 0.184 stays "0.184" rather than "0.184000".
std::string num(double v) {
  std::ostringstream s;
  s << std::setprecision(6) << v;
  return s.str();
}

/// Below this a dexterity value is numerical noise, not a measurement: just off
/// a singularity sigma_min comes out around 1.3e-13 from cancellation, and
/// printing it claims a precision it does not have. Eleven orders below the
/// default threshold of 0.05, so nothing that matters is lost. Applied at
/// OUTPUT only -- analyze() and the metrics keep the raw value.
constexpr double kNoiseFloor = 1e-12;

/// Formats a metric value, flattening denormal-scale noise to a clean zero.
std::string metricNum(double v) { return num(std::fabs(v) < kNoiseFloor ? 0.0 : v); }

/// An absent optional becomes an EMPTY field, not 0. A rollout too short to
/// have a path efficiency is not one with efficiency zero, and 0 would make it
/// the worst row in the file. Empty is what a dataframe reads back as missing.
std::string optNum(const std::optional<double>& v) { return v.has_value() ? metricNum(*v) : ""; }

/// Minimal RFC4180 quoting, for paths containing a comma or a quote.
std::string csvField(const std::string& text) {
  if (text.find_first_of(",\"\n") == std::string::npos) {
    return text;
  }
  std::string quoted = "\"";
  for (const char c : text) {
    if (c == '"') {
      quoted += '"';  // a quote inside a quoted field is doubled
    }
    quoted += c;
  }
  quoted += '"';
  return quoted;
}

// ---------------------------------------------------------------------------
// Summary statistics
// ---------------------------------------------------------------------------

/// Nearest-rank percentile, no interpolation: every number the summary prints
/// is then a value some rollout actually produced, so "which file was that?"
/// has an answer. The median follows the same rule, reporting the lower of the
/// two middle values for an even count rather than their average.
double percentile(const std::vector<double>& sorted, double p) {
  if (sorted.empty()) {
    return 0.0;  // callers check emptiness first; this is not reachable
  }
  const double rank = std::ceil(p * static_cast<double>(sorted.size()));
  std::size_t index = static_cast<std::size_t>(std::max(1.0, rank)) - 1;
  index = std::min(index, sorted.size() - 1);
  return sorted[index];
}

void printStat(std::ostream& err, const char* label, std::vector<double> values) {
  if (values.empty()) {
    err << label << "  no values\n";
    return;
  }
  std::sort(values.begin(), values.end());
  err << label << "  median " << metricNum(percentile(values, 0.5)) << "   p05 "
      << metricNum(percentile(values, 0.05)) << "   min " << metricNum(values.front()) << "\n";
}

// ---------------------------------------------------------------------------
// Redundancy probe
// ---------------------------------------------------------------------------

/// Largest Jacobian rank the robot reaches over a sample of the rollout's own
/// configurations.
///
/// Rank is measured, not counted: it is bounded by the number of twist
/// components the mechanism can produce, well below 6 for a planar arm. A
/// planar 3R has three joints and rank 3, so it is NOT redundant.
///
/// The maximum over several configurations, not the rank at one: a singular
/// pose has lower rank than the mechanism generically does, so a single probe
/// could call an ordinary arm redundant just because the rollout started
/// stretched out.
Eigen::Index maxJacobianRank(const Robot& robot, const Rollout& rollout) {
  constexpr std::size_t kSamples = 16;
  const std::size_t stride = std::max<std::size_t>(1, rollout.size() / kSamples);

  Eigen::Index best = 0;
  for (std::size_t i = 0; i < rollout.size(); i += stride) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian(robot, rollout.q[i]));
    // A tolerance is unavoidable here -- rank is a discontinuous function and
    // floating point never returns an exact zero. 1e-9 sits far below any
    // singular value a real mechanism produces and far above rounding noise.
    svd.setThreshold(1e-9);
    best = std::max(best, svd.rank());
  }
  return best;
}

// ---------------------------------------------------------------------------
// Per-rollout profile output
// ---------------------------------------------------------------------------

/// One CSV per rollout: step index, timestamp, dexterity. The step index is its
/// own column because spans are reported in step indices, and a plot that
/// cannot be indexed the same way cannot be lined up with the report.
bool writeProfile(const std::string& dir, const std::string& inputPath, const Rollout& rollout,
                  const RolloutReport& report, std::ostream& err) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    err << "warning: could not create profile directory '" << dir << "': " << ec.message() << "\n";
    return false;
  }

  const std::filesystem::path target =
      std::filesystem::path(dir) /
      (std::filesystem::path(inputPath).stem().string() + "_profile.csv");

  std::ofstream file(target);
  if (!file) {
    err << "warning: could not write profile '" << target.string() << "'\n";
    return false;
  }
  file << "step,t,dexterity\n";
  for (std::size_t i = 0; i < report.dexterityProfile.size(); ++i) {
    file << i << ',' << num(rollout.t[i]) << ',' << metricNum(report.dexterityProfile[i]) << '\n';
  }
  return static_cast<bool>(file);
}

}  // namespace

// ---------------------------------------------------------------------------
// runCli
// ---------------------------------------------------------------------------

int runCli(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
  Options opts;
  std::string error;
  if (!parseOptions(args, opts, error)) {
    err << "robometrics: " << error << "\n\n" << kUsage;
    return 2;
  }
  if (opts.help) {
    err << kUsage;
    return 0;
  }
  if (opts.urdf.empty()) {
    err << "robometrics: --urdf is required\n\n" << kUsage;
    return 2;
  }
  if (opts.inputs.empty()) {
    err << "robometrics: no rollout files given\n\n" << kUsage;
    return 2;
  }

  // Loaded once; a failure here is fatal rather than per-file, because without
  // it there is nothing to analyse against.
  std::optional<Robot> robot;
  try {
    robot = opts.tipLink.empty() ? Robot::fromUrdfFile(opts.urdf)
                                 : Robot::fromUrdfFile(opts.urdf, opts.tipLink);
  } catch (const std::exception& e) {
    err << "robometrics: cannot load URDF: " << e.what() << "\n";
    return 2;
  }

  // Length scale for normalising dexterity, resolved once for the whole run. An
  // explicit --char-length wins; otherwise it is the summed link lengths from
  // the URDF. Printed either way so the number the metric is divided by is never
  // a mystery, and the automatic value can be sanity-checked or overridden.
  const bool charLengthGiven = opts.charLength > 0.0;
  const double charLength = charLengthGiven ? opts.charLength : characteristicLength(*robot);
  if (!(charLength > 0.0)) {
    err << "robometrics: characteristic length of '" << robot->name()
        << "' is zero; pass --char-length to set it\n";
    return 2;
  }
  err << "characteristic length: " << num(charLength) << " m ("
      << (charLengthGiven ? "given" : "computed from URDF") << ")\n";
  // The dimensionless threshold: the physical m/rad value divided by the same L
  // the dexterity is divided by, so the flagged region does not move with size.
  const double normalizedThreshold = opts.threshold / charLength;

  // The CSV goes to a file if asked, otherwise to the caller's out stream.
  std::ofstream outFile;
  if (!opts.out.empty()) {
    outFile.open(opts.out);
    if (!outFile) {
      err << "robometrics: cannot open '" << opts.out << "' for writing\n";
      return 2;
    }
  }
  std::ostream& csv = opts.out.empty() ? out : outFile;

  // No separate worst_dex column: it would be identical to dexterity_norm by
  // construction, since the margin IS the minimum of the profile. Two columns
  // that can never disagree are two chances to read the wrong one.
  //
  // dexterity_norm, not dexterity_margin: the value is now sigma_min divided by
  // the characteristic length (printed above), so it is dimensionless. The name
  // carries the units the number no longer does.
  csv << "file,dofs,steps,success,dexterity_norm,path_efficiency,low_dex_spans,worst_at\n";

  std::size_t ok = 0;
  std::size_t failed = 0;
  bool efficiencyChecked = false;
  EfficiencyStatus robotEfficiency = EfficiencyStatus::Available;
  std::size_t withSpans = 0;
  std::size_t efficiencyNA = 0;  // ok rollouts whose path_efficiency is N/A
  std::vector<double> margins;
  std::vector<double> efficiencies;

  for (const std::string& path : opts.inputs) {
    Rollout rollout;
    RolloutReport report;
    try {
      rollout = loadRollout(path);
      report = analyze(*robot, rollout, normalizedThreshold, charLength);
    } catch (const std::exception& e) {
      // One bad file must not end the run -- which is why the exit code is "at
      // least one succeeded" rather than "none failed".
      err << "skipping " << path << ": " << e.what() << "\n";
      ++failed;
      continue;
    }
    ++ok;

    // path_efficiency applies only to a redundant, uniform-joint-type robot;
    // otherwise the metric returns N/A rather than a misleading number (a
    // constant 1, or a mixed-unit ratio). This is a property of the robot, so it
    // is decided once and the reason is stated for the whole run.
    if (!efficiencyChecked) {
      efficiencyChecked = true;
      robotEfficiency = efficiencyStatus(*robot, rollout.q);
      switch (robotEfficiency) {
        case EfficiencyStatus::NotRedundant:
          err << "warning: robot '" << robot->name() << "' is not redundant (" << robot->numDofs()
              << " dofs, Jacobian rank " << maxJacobianRank(*robot, rollout)
              << "), so path_efficiency is N/A for every rollout -- the minimum-norm motion is "
                 "the actual one, so the ratio would be 1 and say nothing about the policy\n";
          break;
        case EfficiencyStatus::MixedJointTypes:
          err << "warning: robot '" << robot->name()
              << "' has both revolute and prismatic joints, so path_efficiency is N/A for "
                 "every rollout -- ||dq|| would add radians and metres into one sum\n";
          break;
        case EfficiencyStatus::Available:
          break;
      }
    }

    const auto successIt = rollout.meta.find("success");
    const std::string success = successIt == rollout.meta.end() ? "" : successIt->second;

    csv << csvField(path) << ',' << rollout.dofs << ',' << rollout.size() << ','
        << csvField(success) << ',' << optNum(report.dexterityMargin) << ','
        << optNum(report.pathEfficiency) << ',' << report.lowDexteritySpans.size() << ','
        << (report.worstIndex.has_value() ? std::to_string(*report.worstIndex) : "") << '\n';

    if (report.dexterityMargin.has_value()) {
      margins.push_back(*report.dexterityMargin);
    }
    if (report.pathEfficiency.has_value()) {
      efficiencies.push_back(*report.pathEfficiency);
    } else {
      ++efficiencyNA;
    }
    if (!report.lowDexteritySpans.empty()) {
      ++withSpans;
      // The localisation. A count in a CSV column says a rollout was bad; this
      // says which frames to open.
      err << path << ": " << report.lowDexteritySpans.size() << " low-dexterity span"
          << (report.lowDexteritySpans.size() == 1 ? "" : "s");
      for (const auto& span : report.lowDexteritySpans) {
        // INCLUSIVE for a human; the struct is half-open, so the last bad step
        // is end - 1. Getting this wrong names a step the robot was fine at.
        err << "  [" << span.begin << ".." << span.end - 1 << " worst " << metricNum(span.worst)
            << "]";
      }
      err << "\n";
    }

    if (!opts.profileOut.empty()) {
      writeProfile(opts.profileOut, path, rollout, report, err);
    }
  }

  if (!opts.out.empty()) {
    outFile.flush();
    if (!outFile) {
      err << "robometrics: writing '" << opts.out << "' failed\n";
      return 2;
    }
  }

  // --- Summary -----------------------------------------------------------
  const std::size_t total = ok + failed;
  err << total << " rollout" << (total == 1 ? "" : "s") << ", " << ok << " ok, " << failed
      << " failed to parse\n";
  printStat(err, "dexterity (norm): ", margins);
  // Skip the bare "no values" line when every efficiency is N/A: the reason line
  // below says the same thing and more. Still print it when some values exist.
  if (!efficiencies.empty() || efficiencyNA == 0) {
    printStat(err, "path efficiency:  ", efficiencies);
  }
  if (efficiencyNA > 0) {
    err << "path efficiency:   N/A for " << efficiencyNA << " of " << ok << " rollout"
        << (ok == 1 ? "" : "s") << " (";
    switch (robotEfficiency) {
      case EfficiencyStatus::NotRedundant:
        err << "robot not redundant";
        break;
      case EfficiencyStatus::MixedJointTypes:
        err << "mixed joint types";
        break;
      case EfficiencyStatus::Available:
        err << "trajectory too short or motionless";
        break;
    }
    err << ")\n";
  }
  if (ok > 0) {
    const double percent = 100.0 * static_cast<double>(withSpans) / static_cast<double>(ok);
    err << withSpans << " rollout" << (withSpans == 1 ? "" : "s") << " ("
        << num(std::round(percent)) << "%) have at least one low-dexterity span\n";
  }

  return ok > 0 ? 0 : 1;
}

}  // namespace robometrics
