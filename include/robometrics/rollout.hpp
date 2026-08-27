#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

/// Reading and writing recorded rollouts.
///
/// A home-grown text format rather than HDF5 or a JSON library, so the C++ core
/// builds anywhere that has Eigen and pugixml and nothing else. The formats the
/// data actually arrives in are converted by an optional Python script in
/// python/, outside the build -- that is also the fastest-moving part, and it
/// can change there without recompiling anything.
///
/// THE FORMAT, by example:
///
///     # robometrics rollout v1
///     # robot: panda_arm_hand.urdf
///     # dofs: 7
///     # success: 1
///     t,q0,q1,q2,q3,q4,q5,q6
///     0.000,0.0,-0.785,0.0,-2.356,0.0,1.571,0.785
///     0.050,0.012,-0.781,0.0,-2.351,0.0,1.570,0.785
///
/// Rules, in the order the parser applies them:
///
///   - Leading `#` lines carry metadata as `key: value`. A `#` line without a
///     colon is a plain comment and is ignored, which is what makes the
///     version banner on the first line legal.
///   - `dofs` is REQUIRED. Everything else is optional; unknown keys are kept
///     verbatim in `meta`, so a converter can record provenance without this
///     parser learning about it.
///   - The first non-comment line is the column header and must be exactly
///     `t,q0,...,q{dofs-1}`. A file whose header and metadata disagree is a
///     converter bug, and catching it here beats catching it as a wrong metric
///     three steps later.
///   - Every data row has dofs + 1 numeric fields, and every one must be
///     FINITE. "nan" and "inf" are valid floating-point text and would parse;
///     they are rejected with the row and column named, because a NaN that gets
///     through reappears as a NaN metric far away with nothing pointing back at
///     the row that produced it.
///   - `t` must not DECREASE from one row to the next. A backwards timestamp
///     means the rows are out of order, which would silently reverse a
///     segment of the trajectory. Equal consecutive timestamps are ALLOWED,
///     because no metric here differentiates by time. That tolerance expires
///     the moment a time-based metric (velocity, jerk, anything dividing by
///     dt) is added: duplicate instants become a division by zero and the
///     rule has to tighten to strictly increasing. See the implementation.
///   - `t` is in seconds, `q` in SI units -- radians for revolute joints,
///     metres for prismatic ones. The file carries no units field because a
///     units field that can be wrong is worse than a convention that cannot.
///
/// Text, not binary: a rollout that misbehaves can be opened, diffed and
/// hand-edited, and at hundreds of rows the space cost is irrelevant.
namespace robometrics {

/// One recorded trajectory.
struct Rollout {
  /// Timestamps in seconds, one per step. Not required to be uniform -- the
  /// metrics in this library are geometric and never differentiate by time.
  std::vector<double> t;

  /// Joint configurations, one per step, each of length `dofs`.
  std::vector<Eigen::VectorXd> q;

  /// Everything from the `#` header, unparsed. `dofs` appears here as well as
  /// below, so a writer can reproduce the file it read.
  std::map<std::string, std::string> meta;

  /// Degrees of freedom, from the `dofs` metadata key. Stored explicitly
  /// rather than derived from q.front().size(), because a rollout with zero
  /// steps is a legal file -- a recording cut short -- and still knows how wide
  /// it is. Deriving would make that case silently zero-dimensional.
  int dofs = 0;

  /// Number of steps. Equal to t.size() and to q.size(); the parser will not
  /// produce a Rollout where those differ.
  std::size_t size() const { return t.size(); }
};

/// Thrown for every malformed rollout file.
///
/// Carries the line number separately, so no message can be written without
/// one. On a 400-row file, "parse error" is not a diagnostic.
///
/// Message format:  "demo_001.csv:12: expected 8 fields, found 7"
class RolloutError : public std::runtime_error {
public:
  /// `line` is 1-based. Zero means the problem is with the file as a whole and
  /// no single line is to blame -- a missing `dofs`, or an empty file.
  RolloutError(std::string source, std::size_t line, std::string detail);

  /// File name or other origin label, for messages.
  const std::string& source() const;

  /// 1-based line number, or 0 when the fault is not on one particular line.
  std::size_t line() const;

  /// The explanation alone, without the location prefix.
  const std::string& detail() const;

private:
  std::string source_;
  std::size_t line_;
  std::string detail_;
};

/// Reads a rollout from disk.
///
/// Pre:  path names a readable file in the format above.
/// Post: result.t.size() == result.q.size(); every q has length result.dofs;
///       every value is finite; result.t is non-decreasing.
/// Throws RolloutError, never a bare runtime_error.
Rollout loadRollout(const std::string& path);

/// Parses a rollout already in memory.
///
/// `source` only labels error messages. Separate from loadRollout so tests can
/// state a malformed file inline and still take the path a real file takes.
Rollout parseRollout(const std::string& text, const std::string& source = "<string>");

/// Serialises a rollout back to the text format.
///
/// Post: parseRollout(formatRollout(r)) reproduces r EXACTLY, floating-point
///       values included. Numbers use the shortest decimal form that
///       round-trips, so files stay readable (0.05 stays "0.05") losslessly.
std::string formatRollout(const Rollout& rollout);

/// formatRollout, straight to a file.
/// Throws RolloutError if the file cannot be written.
void saveRollout(const std::string& path, const Rollout& rollout);

}  // namespace robometrics
