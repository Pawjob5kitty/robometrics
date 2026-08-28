#include "robometrics/report.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>

#include "robometrics/metrics.hpp"

namespace robometrics {
namespace {

// Scans the profile for contiguous runs below the threshold.
//
// An explicit state machine rather than find_if/adjacent_find: the two places
// this goes wrong -- a run starting at index 0, and one still open when the
// profile ends -- are both boundaries, and an explicit "close whatever is open"
// step after the loop makes them visible.
std::vector<RolloutReport::Span> findLowSpans(const std::vector<double>& profile,
                                              double threshold) {
  std::vector<RolloutReport::Span> spans;

  bool inSpan = false;
  std::size_t begin = 0;
  double worst = 0.0;

  for (std::size_t i = 0; i < profile.size(); ++i) {
    // Strictly below. A value exactly at the threshold is the last acceptable
    // one, not the first unacceptable one -- see the header.
    const bool bad = profile[i] < threshold;

    if (bad && !inSpan) {
      inSpan = true;
      begin = i;
      worst = profile[i];
    } else if (bad) {
      worst = std::min(worst, profile[i]);
    } else if (inSpan) {
      // i is the first GOOD step, so it is one past the end -- which is
      // exactly the half-open `end` the header promises.
      spans.push_back({begin, i, worst});
      inSpan = false;
    }
  }

  // A span still open when the profile runs out ends at the profile's end.
  // Forgetting this loses every rollout that finished while still in trouble,
  // which is not a rare case -- it is what a rollout that failed at the last
  // moment looks like.
  if (inSpan) {
    spans.push_back({begin, profile.size(), worst});
  }
  return spans;
}

}  // namespace

RolloutReport analyze(const Robot& robot, const Rollout& rollout, double threshold,
                      std::optional<double> charLength) {
  if (rollout.dofs != robot.numDofs()) {
    // Pairing the wrong URDF with a rollout would otherwise produce a full set
    // of plausible numbers describing a robot that never ran.
    std::ostringstream msg;
    msg << "rollout has dofs=" << rollout.dofs << " but robot '" << robot.name() << "' has "
        << robot.numDofs() << " degrees of freedom; the URDF and the rollout do not match";
    throw std::invalid_argument(msg.str());
  }

  RolloutReport report;

  // Length scale, resolved once for the whole rollout: the caller's override or
  // the robot's own characteristic length. Never per frame -- the normalisation
  // must divide every step by the SAME number for the metric to stay a property
  // of the mechanism rather than of the pose.
  const double length = charLength.value_or(characteristicLength(robot));

  // Computed once and everything else read off it. Calling dexterityMargin()
  // as well would recompute the profile internally and leave two code paths
  // that could disagree about the minimum.
  report.dexterityProfile = sigmaMinProfile(robot, rollout.q);

  // Normalise in place: divide by the length scale so the profile is
  // dimensionless. The threshold is already in these units, so findLowSpans
  // below compares like with like -- dividing the threshold here as well would
  // divide by L twice and move every boundary.
  for (double& value : report.dexterityProfile) {
    value /= length;
  }

  if (!report.dexterityProfile.empty()) {
    const auto worstIt =
        std::min_element(report.dexterityProfile.begin(), report.dexterityProfile.end());
    report.dexterityMargin = *worstIt;
    report.worstIndex =
        static_cast<std::size_t>(std::distance(report.dexterityProfile.begin(), worstIt));
  }

  report.pathEfficiency = pathEfficiency(robot, rollout.q);
  report.lowDexteritySpans = findLowSpans(report.dexterityProfile, threshold);
  return report;
}

}  // namespace robometrics
