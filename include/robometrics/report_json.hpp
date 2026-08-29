#pragma once

#include <cstddef>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "robometrics/metrics.hpp"
#include "robometrics/report.hpp"

/// The JSON report is a PUBLIC output format, not an internal channel to any one
/// display. It is versioned (`format_version`), its field names and units are
/// documented in examples/FORMAT.md, and any consumer -- another tool, a script,
/// a UI -- can read it without reading this code. The CSV stays the format for
/// pipelines; JSON is the format for structured data.
///
/// This header is the in-memory model the CLI fills in and the serialiser reads.
/// It is deliberately a plain data model with no behaviour, so a test can build
/// one by hand and check the bytes that come out.
namespace robometrics {

/// How the per-step dexterity profiles are written into the report.
enum class ProfileMode {
  Embedded,  ///< every rollout carries its full `profile` array (default)
  None,      ///< `profile` is null everywhere; use --profile-out for per-step CSVs
};

/// One rollout's entry. The scalar fields mirror a CSV row; `profile` is the
/// extra the JSON carries and the CSV does not.
struct JsonRollout {
  std::string file;             ///< input file stem (no directory, no extension)
  std::string task;             ///< `file` with a trailing `_demo_<N>` removed
  std::size_t steps = 0;        ///< number of recorded steps
  std::optional<bool> success;  ///< from the `success` metadata; null when absent or non-boolean
  RolloutReport report;         ///< dexterity margin/profile/spans and efficiency
  EfficiencyStatus status = EfficiencyStatus::Available;  ///< robot-level efficiency applicability
};

/// The run-wide aggregates, exactly the numbers the stderr summary prints. The
/// CLI computes them once and both outputs read them, so they cannot disagree.
/// The optionals are null when there were no rollouts to aggregate.
struct JsonSummary {
  std::size_t numRollouts = 0;  ///< rollouts successfully analysed
  std::size_t numFailed = 0;    ///< files that failed to parse and were skipped
  std::optional<double> dexterityMedian;
  std::optional<double> dexterityP05;
  std::optional<double> dexterityMin;
  std::optional<double> efficiencyMedian;
  std::size_t efficiencyAvailable = 0;  ///< rollouts that produced an efficiency value
  std::size_t rolloutsWithSpan = 0;     ///< rollouts with at least one low-dexterity span
};

/// The whole report, ready to serialise.
struct JsonReport {
  std::string urdf;                   ///< URDF path as given on the command line
  int numDofs = 0;                    ///< robot degrees of freedom
  double characteristicLength = 0.0;  ///< length dexterity is normalised by, in metres
  double thresholdNormalized = 0.0;   ///< dimensionless low-dexterity threshold actually applied
  std::optional<std::string> tip;     ///< end-effector link; null when auto-detected
  EfficiencyStatus robotStatus = EfficiencyStatus::Available;  ///< why efficiency is N/A, run-wide
  JsonSummary summary;
  std::vector<JsonRollout> rollouts;
  ProfileMode profiles = ProfileMode::Embedded;
};

/// The stable string a consumer reads for an EfficiencyStatus. These exact
/// tokens are part of the format (see FORMAT.md): `available`, `not_redundant`,
/// `mixed_joint_types`.
const char* efficiencyStatusName(EfficiencyStatus status);

/// Serialises `report` as JSON (format_version 1) to `out`. Pretty-printed with
/// two-space indentation; profile arrays are kept on one line to stay compact.
void writeJsonReport(std::ostream& out, const JsonReport& report);

}  // namespace robometrics
