#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "robometrics/rollout.hpp"
#include "robometrics/urdf.hpp"

/// Turning one recorded rollout into something a person can act on.
///
/// The metrics in metrics.hpp answer "how much". This header answers "WHERE",
/// and that is the part that changes what somebody does next. A report saying
/// `dexterity margin 0.008` tells you the rollout was bad. A report saying
/// `steps 340..380 were below 0.05, worst 0.008` tells you which 40 frames to
/// look at, and that is a different kind of information: the first is a score,
/// the second is a lead.
namespace robometrics {

/// Default threshold for "the arm was struggling here", in metres per radian.
///
/// Where 0.05 comes from, since a magic number in a metric is worth deriving.
/// sigma_min is the tip speed obtained per unit of joint speed in the worst
/// direction, so the question is what joint speed a normal task speed costs:
///
///     joint speed needed = tip speed / sigma_min
///
/// At a fairly ordinary manipulation speed of 0.1 m/s, sigma_min = 0.05 m/rad
/// demands 2 rad/s from the joints. A Franka Panda's per-joint velocity limit
/// is about 2.6 rad/s, so 0.05 is roughly the point where tracking an ordinary
/// motion in the worst direction starts to saturate the hardware. Above it the
/// arm has headroom; below it, the direction is effectively unavailable at
/// task speed regardless of what the controller wants.
///
/// It is a default, not a law. A slower task tolerates a smaller value, and a
/// robot with different velocity limits shifts the whole scale -- which is why
/// analyze() takes it as an argument rather than hard-coding it.
///
/// IT ALSO SCALES WITH THE ROBOT, which is the bigger caveat. sigma_min is in
/// metres per radian, so it is proportional to the arm's reach: rotating a
/// joint by one radian moves a tip that is 1 m away roughly ten times as far
/// as a tip 0.1 m away. The same pose on a tabletop arm and on a Panda
/// therefore produce sigma_min values an order of magnitude apart while being
/// equally close to singular.
///
/// 0.05 is calibrated to Franka Panda scale (about 0.85 m of reach). On a
/// small arm it will flag almost everything; on a large one, almost nothing.
/// The principled fix is to normalise by a characteristic length L taken from
/// the URDF -- the distance from the base to the tip in the zero
/// configuration, or the maximum over the workspace -- and compare
/// sigma_min / L against a dimensionless threshold. That is a change to what
/// the metric MEANS, not a tuning tweak, so it is deliberately not done here;
/// until it is, treat this default as valid for one robot size and re-derive
/// it from the formula above for anything else.
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

  /// Index of the worst step, i.e. the argmin of dexterityProfile. nullopt for
  /// an empty rollout.
  ///
  /// Not in the original sketch of this struct, and added because both the CLI
  /// and any plotting code want it. Computing an argmin in two places is how
  /// two places end up disagreeing about which frame was worst.
  std::optional<std::size_t> worstIndex;

  /// A contiguous run of steps whose dexterity fell below the threshold.
  struct Span {
    /// HALF-OPEN: the span covers steps [begin, end), so `end` is one past the
    /// last bad step and `end - begin` is the length. This is stated loudly
    /// because it is exactly where an off-by-one lives, and because a report
    /// printed for a human should say `begin .. end - 1` -- the two
    /// conventions must not be confused at the boundary between them.
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
/// A step counts as low-dexterity when its sigma_min is STRICTLY below the
/// threshold. A value exactly equal to the threshold is fine, on the ordinary
/// reading of "below": the threshold is the last acceptable value, not the
/// first unacceptable one. The distinction is invisible on real data and
/// glaring in a test, which is why there is a test for it.
///
/// Pre:  rollout.dofs == robot.numDofs(), else std::invalid_argument. A
///       mismatch here is a configuration error -- the wrong URDF paired with
///       the rollout -- and silently producing numbers for the wrong robot is
///       the failure this guards against.
/// Post: dexterityProfile.size() == rollout.size();
///       spans are ordered, disjoint, and every one is non-empty.
RolloutReport analyze(const Robot& robot, const Rollout& rollout,
                      double threshold = kDefaultDexterityThreshold);

}  // namespace robometrics
