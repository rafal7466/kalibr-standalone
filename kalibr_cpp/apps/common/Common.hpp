// kalibr_cpp - ROS-free C++ front-end for Kalibr (ethz-asl/kalibr)
// Common helpers shared by the command line tools.
#pragma once

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sm/kinematics/Transformation.hpp>

namespace kalibr {

/// Number of worker threads (= cores - 1, like the python version)
inline int defaultNumThreads() {
  unsigned n = std::thread::hardware_concurrency();
  return n > 1 ? static_cast<int>(n) - 1 : 1;
}

enum class LogLevel { Debug = 0, Info = 1, Warn = 2, Error = 3 };
LogLevel& logLevel();

#define KLOG_DEBUG(msg) do { if (kalibr::logLevel() <= kalibr::LogLevel::Debug) { std::cout << "[DEBUG] " << msg << std::endl; } } while (0)
#define KLOG_INFO(msg)  do { if (kalibr::logLevel() <= kalibr::LogLevel::Info)  { std::cout << "[ INFO] " << msg << std::endl; } } while (0)
#define KLOG_WARN(msg)  do { if (kalibr::logLevel() <= kalibr::LogLevel::Warn)  { std::cout << "[ WARN] " << msg << std::endl; } } while (0)
#define KLOG_ERROR(msg) do { std::cerr << "[ERROR] " << msg << std::endl; } while (0)

/// Simple console progress bar (replacement of sm.Progress2)
class Progress {
 public:
  explicit Progress(size_t total, const std::string& label = "");
  void sample(size_t steps = 1);
  void done();
 private:
  size_t total_, current_ = 0;
  std::string label_;
  std::chrono::steady_clock::time_point start_;
  int lastPercent_ = -1;
};

/// numpy-like vector formatting
std::string fmtVec(const Eigen::VectorXd& v, int precision = 8);
std::string fmtMat(const Eigen::MatrixXd& m, int precision = 8, const std::string& indent = "");

/// median (numpy semantics: average of the two central values for even n)
double median(std::vector<double> v);
double mean(const std::vector<double>& v);
double stddev(const std::vector<double>& v);  // population std (numpy default)

/// filesystem helpers
std::string joinPath(const std::string& a, const std::string& b);
bool fileExists(const std::string& p);
std::string stripExtension(const std::string& p);

/// Thrown when the incremental optimization does not converge (python: OptimizationDiverged)
struct OptimizationDiverged : public std::runtime_error {
  OptimizationDiverged() : std::runtime_error("Optimization diverged") {}
};

}  // namespace kalibr
