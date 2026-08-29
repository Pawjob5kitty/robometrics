#include "robometrics/report_json.hpp"

#include <cmath>
#include <cstdio>
#include <ostream>
#include <string>

#include "format.hpp"

namespace robometrics {
namespace {

/// A JSON string literal, with the escapes the grammar requires. Raw UTF-8
/// bytes (>= 0x20) pass through unchanged; only the control range is \u-escaped.
/// File names, task names and paths all go through here, so the format stays
/// valid whatever a URDF path happens to contain.
std::string jsonString(const std::string& s) {
  std::string out = "\"";
  for (const unsigned char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  out += '"';
  return out;
}

/// A finite double, formatted to the tool's six significant figures. Non-finite
/// values become `null`: JSON has no NaN or Infinity, and emitting one would
/// make the whole document unparseable. Valid metric data never hits that path.
std::string jsonNumber(double v, bool metric) {
  if (!std::isfinite(v)) {
    return "null";
  }
  return metric ? detail::formatMetric(v) : detail::formatValue(v);
}

std::string jsonOptMetric(const std::optional<double>& v) {
  return v.has_value() ? jsonNumber(*v, true) : "null";
}

/// One rollout's `profile` value: the whole array on a single line to keep the
/// file compact, `null` when profiles were not embedded, `[]` for a rollout with
/// no steps.
std::string jsonProfile(const RolloutReport& report, ProfileMode mode) {
  if (mode == ProfileMode::None) {
    return "null";
  }
  std::string out = "[";
  for (std::size_t i = 0; i < report.dexterityProfile.size(); ++i) {
    if (i != 0) {
      out += ", ";
    }
    out += jsonNumber(report.dexterityProfile[i], true);
  }
  out += ']';
  return out;
}

}  // namespace

const char* efficiencyStatusName(EfficiencyStatus status) {
  switch (status) {
    case EfficiencyStatus::Available:
      return "available";
    case EfficiencyStatus::NotRedundant:
      return "not_redundant";
    case EfficiencyStatus::MixedJointTypes:
      return "mixed_joint_types";
  }
  return "available";  // unreachable: the switch is exhaustive over the enum
}

void writeJsonReport(std::ostream& out, const JsonReport& report) {
  const JsonSummary& s = report.summary;
  const std::string naReason = report.robotStatus == EfficiencyStatus::Available
                                   ? "null"
                                   : jsonString(efficiencyStatusName(report.robotStatus));

  out << "{\n";
  out << "  \"format_version\": 1,\n";
  out << "  \"tool\": \"robometrics\",\n";

  out << "  \"robot\": {\n";
  out << "    \"urdf\": " << jsonString(report.urdf) << ",\n";
  out << "    \"num_dofs\": " << report.numDofs << ",\n";
  out << "    \"characteristic_length\": " << jsonNumber(report.characteristicLength, false)
      << "\n";
  out << "  },\n";

  out << "  \"params\": {\n";
  out << "    \"threshold_normalized\": " << jsonNumber(report.thresholdNormalized, false) << ",\n";
  out << "    \"tip\": " << (report.tip.has_value() ? jsonString(*report.tip) : "null") << "\n";
  out << "  },\n";

  out << "  \"summary\": {\n";
  out << "    \"num_rollouts\": " << s.numRollouts << ",\n";
  out << "    \"num_failed\": " << s.numFailed << ",\n";
  out << "    \"dexterity_median\": " << jsonOptMetric(s.dexterityMedian) << ",\n";
  out << "    \"dexterity_p05\": " << jsonOptMetric(s.dexterityP05) << ",\n";
  out << "    \"dexterity_min\": " << jsonOptMetric(s.dexterityMin) << ",\n";
  out << "    \"efficiency_median\": " << jsonOptMetric(s.efficiencyMedian) << ",\n";
  out << "    \"efficiency_available\": " << s.efficiencyAvailable << ",\n";
  out << "    \"efficiency_na_reason\": " << naReason << ",\n";
  out << "    \"rollouts_with_span\": " << s.rolloutsWithSpan << "\n";
  out << "  },\n";

  out << "  \"rollouts\": [";
  for (std::size_t r = 0; r < report.rollouts.size(); ++r) {
    const JsonRollout& roll = report.rollouts[r];
    const RolloutReport& rep = roll.report;
    out << (r == 0 ? "\n" : ",\n");
    out << "    {\n";
    out << "      \"file\": " << jsonString(roll.file) << ",\n";
    out << "      \"task\": " << jsonString(roll.task) << ",\n";
    out << "      \"steps\": " << roll.steps << ",\n";
    out << "      \"success\": "
        << (roll.success.has_value() ? (*roll.success ? "true" : "false") : "null") << ",\n";
    out << "      \"dexterity_min\": " << jsonOptMetric(rep.dexterityMargin) << ",\n";
    out << "      \"dexterity_worst_at\": "
        << (rep.worstIndex.has_value() ? std::to_string(*rep.worstIndex) : "null") << ",\n";
    out << "      \"efficiency\": " << jsonOptMetric(rep.pathEfficiency) << ",\n";
    out << "      \"efficiency_status\": " << jsonString(efficiencyStatusName(roll.status))
        << ",\n";

    // Spans: half-open [begin, end) step indices, matching Span in report.hpp,
    // with the span's worst normalised dexterity.
    out << "      \"spans\": [";
    for (std::size_t i = 0; i < rep.lowDexteritySpans.size(); ++i) {
      const RolloutReport::Span& span = rep.lowDexteritySpans[i];
      out << (i == 0 ? "\n" : ",\n");
      out << "        {\"begin\": " << span.begin << ", \"end\": " << span.end
          << ", \"worst\": " << jsonNumber(span.worst, true) << "}";
    }
    out << (rep.lowDexteritySpans.empty() ? "]" : "\n      ]") << ",\n";

    out << "      \"profile\": " << jsonProfile(rep, report.profiles) << "\n";
    out << "    }";
  }
  out << (report.rollouts.empty() ? "]\n" : "\n  ]\n");
  out << "}\n";
}

}  // namespace robometrics
