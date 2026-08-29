#pragma once

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

/// Number formatting shared by every output the tool produces. It lives in one
/// place on purpose: the CSV row, the stderr summary, and the JSON report all
/// pass through here, so a value printed in report.json is the same six
/// significant figures as the same value in report.csv, character for
/// character. A round-trip test relies on exactly that.
///
/// A private header (src/, not include/): it is an implementation detail of the
/// output layer, not part of the library's API.
namespace robometrics::detail {

/// Six significant figures -- the precision the whole tool commits to. 0.184
/// stays "0.184" rather than "0.184000", and 1.31926 keeps its digits.
inline std::string formatValue(double v) {
  std::ostringstream s;
  s << std::setprecision(6) << v;
  return s.str();
}

/// Below this a dexterity value is numerical noise, not a measurement: just off
/// a singularity sigma_min comes out around 1.3e-13 from cancellation, and
/// printing it claims a precision it does not have. Eleven orders below the
/// default threshold of 0.05, so nothing that matters is lost. Applied at
/// OUTPUT only -- analyze() and the metrics keep the raw value.
inline constexpr double kNoiseFloor = 1e-12;

/// Formats a metric value, flattening denormal-scale noise to a clean zero.
inline std::string formatMetric(double v) {
  return formatValue(std::fabs(v) < kNoiseFloor ? 0.0 : v);
}

}  // namespace robometrics::detail
