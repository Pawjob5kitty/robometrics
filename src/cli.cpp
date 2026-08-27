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
    "  -h, --help           this text\n"
    "\n"
    "The report CSV goes to stdout unless --out is given; the summary always\n"
    "goes to stderr, so a pipeline can consume the CSV while a person reads\n"
    "the summary.\n";

/// Parses one `--flag value` pair. Returns false and fills `error` on trouble.
///
/// Hand-rolled rather than pulled from a library, because the argument surface
/// is five flags and a file list, and a dependency for that would cost more
/// than it saves. The one rule worth stating: a flag that takes a value must
/// find one, and running off the end of argv is an error rather than a silently
/// empty string -- `--urdf` with nothing after it is a typo, not a request to
/// load the file named "".
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

  // The subcommand. There is exactly one today; requiring it anyway keeps the
  // door open for `robometrics compare` or `robometrics plot` without ever
  // having to break the existing invocation.
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
    } else if (a.rfind("--", 0) == 0) {
      // Rejecting unknown flags rather than treating them as filenames: a
      // mistyped --thresold would otherwise be read as a rollout path and
      // reported as a missing file, which sends the reader looking in
      // completely the wrong place.
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

/// Six significant digits in the default (general) format, so 0.184 stays
/// "0.184" instead of becoming "0.184000". Enough precision to compare two
/// runs, few enough digits to read a column of them.
std::string num(double v) {
  std::ostringstream s;
  s << std::setprecision(6) << v;
  return s.str();
}

/// Below this, a dexterity value is numerical noise around zero rather than a
/// measurement. An exactly singular pose comes out of the SVD as something
/// like 8.7e-17 -- the accumulated rounding of the decomposition, not a
/// physical quantity -- and printing it suggests a precision the number does
/// not have, while making a column of results impossible to scan.
///
/// 1e-12 is eleven orders of magnitude below the default threshold of 0.05, so
/// nothing that could ever matter is lost by flattening it. The clamp is
/// applied at OUTPUT only: analyze() and the metrics keep the raw value, so
/// nothing downstream inherits a rounding decision made for a report.
constexpr double kNoiseFloor = 1e-12;

/// Formats a metric value, flattening denormal-scale noise to a clean zero.
std::string metricNum(double v) { return num(std::fabs(v) < kNoiseFloor ? 0.0 : v); }

/// An absent optional becomes an EMPTY field, not 0 and not "nan".
///
/// This matters more than it looks. A rollout too short to have a path
/// efficiency is not a rollout with efficiency zero, and writing 0 there would
/// make it the worst row in the file. Empty is what every spreadsheet and
/// dataframe library reads back as missing.
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

/// Nearest-rank percentile on an ascending-sorted vector.
///
/// No interpolation, deliberately: every number the summary prints is then a
/// value that actually occurred in some rollout, which is what you want when
/// the next step is to go and look at that rollout. An interpolated p05 is a
/// number no run produced, and "which file was that?" has no answer.
///
/// The same rule is used for the median, so for an even count it reports the
/// lower of the two middle values rather than their average -- consistent with
/// the above, and one less special case.
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
/// Rank has to be measured, not counted. It is bounded by the number of twist
/// components the mechanism can actually produce, which is well below 6 for a
/// planar arm -- a planar 3R has three joints and rank 3, and is therefore NOT
/// redundant despite "three joints for a two-dimensional task" sounding like
/// it should be.
///
/// The maximum over several configurations rather than the rank at one: a
/// singular pose has a lower rank than the mechanism generically has, so
/// probing a single configuration could report a perfectly ordinary arm as
/// redundant purely because the rollout happened to start stretched out.
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

/// One CSV per rollout: step index, timestamp, dexterity. For plotting, which
/// is why the step index is there as its own column -- a span is reported in
/// step indices, and a plot the reader cannot index the same way is a plot
/// they cannot line up with the report.
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

  // The robot is loaded once. A failure here is fatal rather than per-file,
  // because without it there is nothing to analyse against -- unlike a corrupt
  // rollout, which only costs one row.
  std::optional<Robot> robot;
  try {
    robot = opts.tipLink.empty() ? Robot::fromUrdfFile(opts.urdf)
                                 : Robot::fromUrdfFile(opts.urdf, opts.tipLink);
  } catch (const std::exception& e) {
    err << "robometrics: cannot load URDF: " << e.what() << "\n";
    return 2;
  }

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

  // No separate worst_dex column: it would be identical to dexterity_margin by
  // construction, since the margin IS the minimum of the profile. Two columns
  // that can never disagree are two chances to read the wrong one.
  csv << "file,dofs,steps,success,dexterity_margin,path_efficiency,low_dex_spans,worst_at\n";

  std::size_t ok = 0;
  std::size_t failed = 0;
  bool redundancyChecked = false;
  std::size_t withSpans = 0;
  std::vector<double> margins;
  std::vector<double> efficiencies;

  for (const std::string& path : opts.inputs) {
    Rollout rollout;
    RolloutReport report;
    try {
      rollout = loadRollout(path);
      report = analyze(*robot, rollout, opts.threshold);
    } catch (const std::exception& e) {
      // One bad file must not end the run. This is the whole reason the exit
      // code is "at least one succeeded" rather than "none failed": a batch of
      // forty rollouts with one corrupt file is a successful batch with a
      // warning, not a failed run.
      err << "skipping " << path << ": " << e.what() << "\n";
      ++failed;
      continue;
    }
    ++ok;

    // Said once, on the first rollout that loads, and only because a reader
    // who does not know this will draw exactly the wrong conclusion from the
    // column. path_efficiency is the ratio of the minimum-norm joint motion to
    // the actual one; without a null space those are the same vector, so the
    // ratio is 1 for every rollout however clumsy the policy was. A column of
    // 1.000 then looks like a flawless policy and is in fact a statement about
    // the mechanism.
    if (!redundancyChecked) {
      redundancyChecked = true;
      const Eigen::Index rank = maxJacobianRank(*robot, rollout);
      if (static_cast<Eigen::Index>(robot->numDofs()) <= rank) {
        err << "warning: robot '" << robot->name() << "' is not redundant (" << robot->numDofs()
            << " dofs, Jacobian rank " << rank
            << "), so path_efficiency is 1 for every rollout and says nothing about the "
               "policy\n";
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
    }
    if (!report.lowDexteritySpans.empty()) {
      ++withSpans;
      // The localisation, in the one place a person actually reads. A count in
      // a CSV column says a rollout was bad; this says which frames to open.
      err << path << ": " << report.lowDexteritySpans.size() << " low-dexterity span"
          << (report.lowDexteritySpans.size() == 1 ? "" : "s");
      for (const auto& span : report.lowDexteritySpans) {
        // Printed INCLUSIVE for a human -- the struct is half-open, so the
        // last bad step is end - 1. Getting this wrong would report a step the
        // robot was fine at.
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
  printStat(err, "dexterity margin: ", margins);
  printStat(err, "path efficiency:  ", efficiencies);
  if (ok > 0) {
    const double percent = 100.0 * static_cast<double>(withSpans) / static_cast<double>(ok);
    err << withSpans << " rollout" << (withSpans == 1 ? "" : "s") << " ("
        << num(std::round(percent)) << "%) have at least one low-dexterity span\n";
  }

  return ok > 0 ? 0 : 1;
}

}  // namespace robometrics
