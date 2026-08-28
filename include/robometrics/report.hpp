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

/// Default low-dexterity threshold, in metres per radian -- the PHYSICAL scale.
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
/// IT SCALES WITH THE ROBOT, which is why it is a PHYSICAL value that must be
/// normalised before use. sigma_min is proportional to reach, so the same pose
/// on a tabletop arm and on a Panda give values an order of magnitude apart
/// while being equally close to singular. 0.05 is calibrated to Panda scale
/// (~0.85 m reach); raw, it would flag almost everything on a small arm and
/// almost nothing on a large one.
///
/// The fix: divide sigma_min by the robot's characteristic length L (see
/// characteristicLength in metrics.hpp) to get a dimensionless dexterity, and
/// divide THIS threshold by the SAME L. Dividing both sides of the comparison
/// by one L leaves the physical operating point exactly where it was -- a step
/// flagged before normalisation is flagged after it, whatever the robot's size
/// -- so the change makes the metric comparable across robots without moving
/// any boundary on a given one. analyze() works in the normalised space; the
/// CLI recomputes 0.05 / L with the actual robot's L.
inline constexpr double kDefaultDexterityThreshold = 0.05;

/// Nominal characteristic length, in metres, for the calibration above: a Panda
/// reaches about 0.85 m. Only used to express the physical threshold as a
/// dimensionless default for analyze() when no robot-specific L is supplied;
/// the CLI always uses the real L instead.
inline constexpr double kNominalCharLength = 0.85;

/// The default threshold as a DIMENSIONLESS value -- the physical 0.05 m/rad at
/// Panda scale. This is what analyze() compares directly against the normalised
/// dexterity; supply a robot's real L (or let analyze compute it) to keep the
/// physical operating point exact on any size of arm.
inline constexpr double kDefaultNormalizedThreshold =
    kDefaultDexterityThreshold / kNominalCharLength;

/// Everything computed about one rollout.
struct RolloutReport {
  /// Worst dexterity over the whole rollout, NORMALISED by the characteristic
  /// length (dimensionless). nullopt for an empty rollout.
  std::optional<double> dexterityMargin;

  /// Fraction of joint motion that moved the tip. nullopt for a rollout with
  /// fewer than two steps, or one where the robot never moved. Also always 1
  /// for a non-redundant arm -- see metrics.hpp before reading anything into
  /// it.
  std::optional<double> pathEfficiency;

  /// Normalised dexterity (sigma_min / characteristic length) at every step.
  /// Same length as the rollout.
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
/// The reported dexterityProfile and dexterityMargin are NORMALISED: each
/// sigma_min is divided by `charLength`, so they are dimensionless and
/// comparable across robots. `threshold` is likewise dimensionless and is
/// compared DIRECTLY against the normalised profile -- it is not divided again.
/// A step is low-dexterity when its normalised dexterity is STRICTLY below the
/// threshold: the threshold is the last acceptable value, not the first
/// unacceptable one.
///
/// `charLength` defaults to characteristicLength(robot); pass an explicit value
/// to override the automatic length scale. It must be > 0.
///
/// To reproduce the pre-normalisation physical threshold of 0.05 m/rad, pass
/// `0.05 / charLength` -- dividing both dexterity and threshold by the same L
/// leaves every span boundary exactly where it was (that is what the CLI does).
///
/// Pre:  rollout.dofs == robot.numDofs(), else std::invalid_argument. A
///       mismatch means the wrong URDF was paired with the rollout, and
///       silently producing numbers for a robot that never ran is exactly the
///       failure this guards against.
/// Post: dexterityProfile.size() == rollout.size();
///       spans are ordered, disjoint, and every one is non-empty.
RolloutReport analyze(const Robot& robot, const Rollout& rollout,
                      double threshold = kDefaultNormalizedThreshold,
                      std::optional<double> charLength = std::nullopt);

}  // namespace robometrics
