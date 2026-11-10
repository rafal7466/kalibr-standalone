#include "common/Common.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>

namespace kalibr {

LogLevel& logLevel() {
  static LogLevel level = LogLevel::Info;
  return level;
}

Progress::Progress(size_t total, const std::string& label)
    : total_(total), label_(label), start_(std::chrono::steady_clock::now()) {}

void Progress::sample(size_t steps) {
  current_ += steps;
  if (total_ == 0) return;
  int percent = static_cast<int>(100.0 * current_ / total_);
  if (percent == lastPercent_ && current_ < total_) return;
  lastPercent_ = percent;
  double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
  double eta = current_ > 0 ? elapsed / current_ * (total_ - current_) : 0.0;
  std::cout << "\r  " << label_ << " Progress " << current_ << " / " << total_ << "  (" << percent
            << "%)  time left: " << std::fixed << std::setprecision(1) << eta << "s     " << std::flush;
  std::cout.unsetf(std::ios::floatfield);
  if (current_ >= total_) std::cout << std::endl;
}

void Progress::done() {
  if (current_ < total_) std::cout << std::endl;
}

std::string fmtVec(const Eigen::VectorXd& v, int precision) {
  std::ostringstream os;
  os << std::setprecision(precision) << "[";
  for (int i = 0; i < v.size(); ++i) {
    os << (i ? " " : "") << v[i];
  }
  os << "]";
  return os.str();
}

std::string fmtMat(const Eigen::MatrixXd& m, int precision, const std::string& indent) {
  std::ostringstream os;
  os << std::setprecision(precision) << std::fixed;
  for (int r = 0; r < m.rows(); ++r) {
    os << indent << (r == 0 ? "[[" : " [");
    for (int c = 0; c < m.cols(); ++c) {
      os << std::setw(precision + 4) << m(r, c) << (c + 1 < m.cols() ? " " : "");
    }
    os << (r + 1 < m.rows() ? "]\n" : "]]");
  }
  return os.str();
}

double median(std::vector<double> v) {
  if (v.empty()) return std::nan("");
  std::sort(v.begin(), v.end());
  size_t n = v.size();
  return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

double mean(const std::vector<double>& v) {
  if (v.empty()) return std::nan("");
  return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

double stddev(const std::vector<double>& v) {
  if (v.empty()) return std::nan("");
  double m = mean(v), s = 0.0;
  for (double x : v) s += (x - m) * (x - m);
  return std::sqrt(s / v.size());
}

std::string joinPath(const std::string& a, const std::string& b) {
  if (a.empty()) return b;
  char last = a.back();
  if (last == '/' || last == '\\') return a + b;
  return a + "/" + b;
}

bool fileExists(const std::string& p) {
  std::ifstream f(p);
  return f.good();
}

std::string stripExtension(const std::string& p) {
  size_t slash = p.find_last_of("/\\");
  size_t dot = p.find_last_of('.');
  if (dot == std::string::npos || (slash != std::string::npos && dot < slash)) return p;
  return p.substr(0, dot);
}

}  // namespace kalibr
