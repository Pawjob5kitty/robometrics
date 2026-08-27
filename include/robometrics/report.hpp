#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "robometrics/rollout.hpp"
#include "robometrics/urdf.hpp"

/// Turning one recorded rollout into something a person can act on.
///
/// metrics.hpp answers "how much"; this answers "WHERE". `dexterity margin
/// 0.008` says the rollout was bad. `steps 340..380 were below 0.05, worst
/// 0.008` says which forty frames to open.
namespace robometrics {

/// Default threshold for "the arm was struggling here", in metres per radian.
///
/// Where 0.05 comes from. sigma_min is tip speed per unit of joint speed in
/// the worst direction, so the question is what a normal task speed costs:
///
///     joint speed needed = tip speed / sigma_min
///
/// At an ordinary manipulation speed of 0.1 m/s, sigma_min = 0.05 m/rad demands
/// 2 rad/s from the joints, against a Franka Panda's per-joint limit of about
/// 2.6 rad/s. So 0.05 is roughly where tracking an ordinary motion in the worst
/// direction starts to saturate the hardware.
///
/// IT SCALES WITH THE ROBOT, which is the bigger caveat. sigma_min is in metres
/// per radian, so it is proportional to reach: the same pose on a tabletop arm
/// and on a Panda give values an order of magnitude apart while being equally
/// close to singular. 0.05 is calibrated to Panda scale (~0.85 m reach); on a
/// small arm it flags almost everything, on a large one almost nothing. The
/// principled fix is to normalise by a characteristic length from the URDF and
/// compare against a dimensionless threshold -- a change to what the metric
/// MEANS, not a tuning tweak, so it is not done here.
inline constexpr double kDefaultDexterityThreshold = 0.05;

/// Everything computed about one rollout.
struct RolloutReport {
  /// Worst dexterity over the whole rollout. nullopt for an empty rollout.
  std::optional<double> dexterityMargin;

  /// Fraction of joint motion that moved the tip. nullopt for a rollout with
  /// fewer than two steps, or one where the robot never moved. Also always 1
  /// for a non-redundant arm -- see metrics.hpp before reading anything into
  /// it.
  std::optional<double> pathEfficiency;

  /// sigma_min at every step. Same length as the rollout.
  std::vector<double> dexterityProfile;

  /// Index of the worst step, the argmin of dexterityProfile. nullopt for an
  /// empty rollout. Stored rather than recomputed by each caller: an argmin
  /// computed in two places is two places that can disagree.
  std::optional<std::size_t> worstIndex;

  /// A contiguous run of steps whose dexterity fell below the threshold.
  struct Span {
    /// HALF-OPEN: the span covers steps [begin, end), so `end` is one past the
    /// last bad step. Stated loudly because it is where an off-by-one lives:
    /// a report printed for a human says `begin .. end - 1`, and the two
    /// conventions must not meet unguarded.
    std::size_t begin;
    std::size_t end;

    /// The smallest dexterity inside this span. Always < threshold.
    double worst;

    /// Number of steps in the span.
    std::size_t length() const { return end - begin; }
  };

  /// Every low-dexterity run, in order, non-overlapping and non-adjacent -- two
  /// spans are never separated by zero good steps, because that would have been
  /// one span.
  std::vector<Span> lowDexteritySpans;
};

/// Computes every metric for one rollout.
///
/// A step is low-dexterity when its sigma_min is STRICTLY below the threshold:
/// the threshold is the last acceptable value, not the first unacceptable one.
///
/// Pre:  rollout.dofs == robot.numDofs(), else std::invalid_argument. A
///       mismatch means the wrong URDF was paired with the rollout, and
///       silently producing numbers for a robot that never ran is exactly the
///       failure this guards against.
/// Post: dexterityProfile.size() == rollout.size();
///       spans are ordered, disjoint, and every one is non-empty.
RolloutReport analyze(const Robot& robot, const Rollout& rollout,
                      double threshold = kDefaultDexterityThreshold);

}  // namespace robometrics
