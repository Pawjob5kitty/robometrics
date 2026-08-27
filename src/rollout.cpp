#include "robometrics/rollout.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <sstream>
#include <system_error>
#include <utility>

namespace robometrics {
namespace {

// ---------------------------------------------------------------------------
// Small text helpers
// ---------------------------------------------------------------------------

[[noreturn]] void fail(const std::string& source, std::size_t line, std::string detail) {
  throw RolloutError(source, line, std::move(detail));
}

// Trims ASCII whitespace from both ends. Fields are trimmed because real files
// come out of spreadsheets and hand edits with stray spaces around the commas,
// and rejecting those would be pedantry about something that cannot be
// ambiguous.
std::string_view trim(std::string_view s) {
  const auto isSpace = [](char c) { return c == ' ' || c == '\t' || c == '\r'; };
  while (!s.empty() && isSpace(s.front())) {
    s.remove_prefix(1);
  }
  while (!s.empty() && isSpace(s.back())) {
    s.remove_suffix(1);
  }
  return s;
}

// Splits on commas without trimming; the caller trims what it needs. Returns
// exactly one field for an empty line, which is what makes an empty data row
// report "expected N fields, found 1" rather than being silently skipped.
std::vector<std::string_view> splitFields(std::string_view line) {
  std::vector<std::string_view> out;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = line.find(',', start);
    if (comma == std::string_view::npos) {
      out.push_back(line.substr(start));
      return out;
    }
    out.push_back(line.substr(start, comma - start));
    start = comma + 1;
  }
}

// std::from_chars rather than stod or istringstream: it is locale-independent.
// That matters more than it looks -- a machine with a Czech or German locale
// reads "0.05" as 0 with strtod-family functions, and the rollout would parse
// without error into a trajectory that never moves. from_chars has no locale
// to get wrong.
//
// It also rejects trailing junk, so "0.05abc" fails instead of silently
// becoming 0.05.
bool parseDouble(std::string_view text, double& out) {
  if (text.empty()) {
    return false;
  }
  const char* first = text.data();
  const char* last = text.data() + text.size();
  const std::from_chars_result r = std::from_chars(first, last, out);
  return r.ec == std::errc() && r.ptr == last;
}

bool parseInt(std::string_view text, int& out) {
  if (text.empty()) {
    return false;
  }
  const char* first = text.data();
  const char* last = text.data() + text.size();
  const std::from_chars_result r = std::from_chars(first, last, out);
  return r.ec == std::errc() && r.ptr == last;
}

// Shortest decimal form that round-trips back to the same double. This is what
// keeps written files readable -- 0.05 stays "0.05" instead of becoming
// 0.050000000000000003 -- while still guaranteeing that a load-save-load cycle
// is bit-exact. setprecision(17) would guarantee the round trip too, but at
// the cost of making every file unreadable.
std::string formatDouble(double value) {
  char buffer[64];
  const std::to_chars_result r = std::to_chars(buffer, buffer + sizeof(buffer), value);
  if (r.ec != std::errc()) {
    return "nan";  // unreachable for finite doubles with a 64-byte buffer
  }
  return std::string(buffer, r.ptr);
}

}  // namespace

// ---------------------------------------------------------------------------
// RolloutError
// ---------------------------------------------------------------------------

namespace {

std::string composeMessage(const std::string& source, std::size_t line, const std::string& detail) {
  std::ostringstream msg;
  msg << source;
  if (line > 0) {
    msg << ':' << line;
  }
  msg << ": " << detail;
  return msg.str();
}

}  // namespace

RolloutError::RolloutError(std::string source, std::size_t line, std::string detail)
    : std::runtime_error(composeMessage(source, line, detail)),
      source_(std::move(source)),
      line_(line),
      detail_(std::move(detail)) {}

const std::string& RolloutError::source() const { return source_; }

std::size_t RolloutError::line() const { return line_; }

const std::string& RolloutError::detail() const { return detail_; }

// ---------------------------------------------------------------------------
// Parsing
// ---------------------------------------------------------------------------

Rollout parseRollout(const std::string& text, const std::string& source) {
  Rollout rollout;

  std::istringstream in(text);
  std::string rawLine;
  std::size_t lineNo = 0;

  // --- Header block: `#` lines, then the column header -------------------
  bool haveColumnHeader = false;
  std::size_t columnHeaderLine = 0;

  while (std::getline(in, rawLine)) {
    ++lineNo;
    // getline leaves the '\r' of a CRLF file attached to every line. Stripping
    // it here rather than at each use is the difference between this parser
    // working on a file authored on Windows and failing on every numeric field
    // of it.
    const std::string_view line = trim(rawLine);

    if (line.empty()) {
      continue;  // blank lines are allowed anywhere and mean nothing
    }

    if (line.front() == '#') {
      const std::string_view body = trim(line.substr(1));
      const std::size_t colon = body.find(':');
      if (colon == std::string_view::npos) {
        // A comment with no key: value, such as the version banner. Ignored on
        // purpose -- see the header for why the banner is not mandatory.
        continue;
      }
      const std::string key(trim(body.substr(0, colon)));
      const std::string value(trim(body.substr(colon + 1)));
      if (key.empty()) {
        fail(source, lineNo, "metadata line has an empty key before the colon");
      }
      rollout.meta[key] = value;
      continue;
    }

    // First non-comment, non-blank line is the column header.
    haveColumnHeader = true;
    columnHeaderLine = lineNo;
    break;
  }

  if (!haveColumnHeader) {
    // Covers both a completely empty file and one that is nothing but header
    // comments. Either way there is no table, so there is no rollout.
    fail(source, 0, "file contains no column header line; it is empty or all comments");
  }

  // --- dofs, which everything below is checked against -------------------
  const auto dofsIt = rollout.meta.find("dofs");
  if (dofsIt == rollout.meta.end()) {
    fail(source, 0, "required metadata key 'dofs' is missing; expected a line like '# dofs: 7'");
  }
  if (!parseInt(dofsIt->second, rollout.dofs) || rollout.dofs <= 0) {
    fail(source, 0, "metadata 'dofs' is not a positive integer: '" + dofsIt->second + "'");
  }
  const std::size_t expectedFields = static_cast<std::size_t>(rollout.dofs) + 1;

  // --- Column header must agree with dofs --------------------------------
  // The check exists because the alternative is worse than a parse error: a
  // file whose header says q0..q5 while dofs says 7 is a converter bug, and
  // without this it would surface much later as a wrong metric.
  {
    const std::vector<std::string_view> columns = splitFields(trim(rawLine));
    if (columns.size() != expectedFields) {
      std::ostringstream msg;
      msg << "column header has " << columns.size()
          << " columns but metadata says dofs=" << rollout.dofs << ", which needs "
          << expectedFields << " (t plus q0.." << rollout.dofs - 1 << ")";
      fail(source, columnHeaderLine, msg.str());
    }
    if (trim(columns[0]) != "t") {
      fail(source, columnHeaderLine,
           "first column must be named 't', found '" + std::string(trim(columns[0])) + "'");
    }
    for (std::size_t i = 1; i < columns.size(); ++i) {
      const std::string want = "q" + std::to_string(i - 1);
      if (trim(columns[i]) != want) {
        fail(source, columnHeaderLine,
             "column " + std::to_string(i) + " must be named '" + want + "', found '" +
                 std::string(trim(columns[i])) + "'");
      }
    }
  }

  // --- Data rows ---------------------------------------------------------
  while (std::getline(in, rawLine)) {
    ++lineNo;
    const std::string_view line = trim(rawLine);
    if (line.empty()) {
      continue;
    }
    if (line.front() == '#') {
      // Metadata is a header-only construct. A `#` line down among the data is
      // more likely a half-deleted row than an intentional comment, so it is
      // rejected rather than skipped.
      fail(source, lineNo, "'#' lines are only allowed in the header, before the column line");
    }

    const std::vector<std::string_view> fields = splitFields(line);
    if (fields.size() != expectedFields) {
      std::ostringstream msg;
      msg << "expected " << expectedFields << " fields (t plus " << rollout.dofs
          << " joint values), found " << fields.size();
      fail(source, lineNo, msg.str());
    }

    double time = 0.0;
    if (!parseDouble(trim(fields[0]), time)) {
      fail(source, lineNo, "column 1 (t) is not a number: '" + std::string(trim(fields[0])) + "'");
    }
    // from_chars accepts "nan", "inf" and "-infinity" as perfectly valid
    // floating-point text, so a finiteness check is a SEPARATE step from
    // parsing, not a consequence of it. Without it a NaN would sail through
    // here and reappear as a NaN metric several layers away, where nothing
    // left points back at the row that produced it.
    if (!std::isfinite(time)) {
      fail(source, lineNo,
           "column 1 (t) is not a finite number: '" + std::string(trim(fields[0])) + "'");
    }
    // Monotonicity. A timestamp that goes BACKWARDS means the rows are out of
    // order, which is unambiguously a bug in whatever wrote the file, and it
    // would silently reverse a segment of the trajectory.
    //
    // Equal consecutive timestamps are ALLOWED, and that is a deliberate line
    // rather than an oversight. A duplicate instant is ambiguous: it can be a
    // duplicated row, but it is just as often a clock too coarse to separate
    // two samples, which real converters do produce. Since none of the metrics
    // in this library differentiate by time -- they are all geometric -- an
    // equal timestamp costs nothing, while rejecting it would throw away an
    // otherwise sound rollout. If a caller ever needs strict increase, this is
    // the one comparison to tighten.
    //
    // KNOWN CONSEQUENCE, worth writing down before it bites: this tolerance is
    // only free while every metric is geometric. The moment a TIME-BASED
    // metric arrives -- joint velocity, jerk, time-to-completion, anything
    // dividing by dt -- a pair of equal timestamps is a division by zero, and
    // the guard will have to move from "must not decrease" to "must strictly
    // increase". At that point this line becomes the place to change, and
    // existing rollouts with duplicate instants become invalid input rather
    // than merely odd.
    if (!rollout.t.empty() && time < rollout.t.back()) {
      std::ostringstream msg;
      msg << "t must not decrease: " << formatDouble(time) << " follows "
          << formatDouble(rollout.t.back()) << ", so the rows are out of order";
      fail(source, lineNo, msg.str());
    }

    Eigen::VectorXd q(rollout.dofs);
    for (std::size_t i = 1; i < fields.size(); ++i) {
      double value = 0.0;
      if (!parseDouble(trim(fields[i]), value)) {
        fail(source, lineNo,
             "column " + std::to_string(i + 1) + " (q" + std::to_string(i - 1) +
                 ") is not a number: '" + std::string(trim(fields[i])) + "'");
      }
      if (!std::isfinite(value)) {
        fail(source, lineNo,
             "column " + std::to_string(i + 1) + " (q" + std::to_string(i - 1) +
                 ") is not a finite number: '" + std::string(trim(fields[i])) + "'");
      }
      q(static_cast<Eigen::Index>(i - 1)) = value;
    }

    rollout.t.push_back(time);
    rollout.q.push_back(std::move(q));
  }

  // A file with a valid header and no data rows is a rollout of zero steps,
  // not an error. Recordings do get cut short, and the caller is better placed
  // to decide whether that is worth complaining about -- the metrics already
  // return nullopt for it.
  return rollout;
}

Rollout loadRollout(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    fail(path, 0, "file could not be opened for reading");
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return parseRollout(buffer.str(), path);
}

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------

std::string formatRollout(const Rollout& rollout) {
  std::ostringstream out;
  out << "# robometrics rollout v1\n";

  // std::map iterates in sorted key order, so the output is deterministic and
  // two runs produce byte-identical files. That is what makes these files
  // diffable, which was half the reason for choosing a text format.
  //
  // `dofs` is written from the struct field rather than from meta, so that a
  // caller who built a Rollout by hand and forgot to set meta["dofs"] still
  // gets a valid file instead of one that cannot be read back.
  out << "# dofs: " << rollout.dofs << "\n";
  for (const auto& [key, value] : rollout.meta) {
    if (key == "dofs") {
      continue;  // already written, and the struct field is authoritative
    }
    out << "# " << key << ": " << value << "\n";
  }

  out << "t";
  for (int i = 0; i < rollout.dofs; ++i) {
    out << ",q" << i;
  }
  out << "\n";

  for (std::size_t row = 0; row < rollout.t.size(); ++row) {
    out << formatDouble(rollout.t[row]);
    const Eigen::VectorXd& q = rollout.q[row];
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      out << ',' << formatDouble(q(i));
    }
    out << "\n";
  }
  return out.str();
}

void saveRollout(const std::string& path, const Rollout& rollout) {
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    fail(path, 0, "file could not be opened for writing");
  }
  file << formatRollout(rollout);
  if (!file) {
    fail(path, 0, "writing the file failed partway through");
  }
}

}  // namespace robometrics
