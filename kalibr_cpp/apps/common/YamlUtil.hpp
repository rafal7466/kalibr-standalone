// Small yaml-cpp helpers. Accepts OpenCV style files that start with "%YAML:1.0"
// (EuRoC sensor.yaml files) which are not valid YAML for yaml-cpp.
#pragma once

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace kalibr {

inline YAML::Node loadYamlFile(const std::string& file) {
  std::ifstream f(file);
  if (!f.good()) throw std::runtime_error("Could not open yaml file " + file);
  std::stringstream out;
  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("%YAML", 0) == 0) continue;  // OpenCV header
    out << line << "\n";
  }
  return YAML::Load(out.str());
}

inline Eigen::VectorXd yamlToVector(const YAML::Node& n) {
  Eigen::VectorXd v(n.size());
  for (size_t i = 0; i < n.size(); ++i) v[i] = n[i].as<double>();
  return v;
}

/// reads either a list of lists (kalibr) or an OpenCV matrix {rows, cols, data}
inline Eigen::Matrix4d yamlToMatrix4d(const YAML::Node& n) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  if (n.IsMap() && n["data"]) {
    const YAML::Node& d = n["data"];
    if (d.size() != 16) throw std::runtime_error("expected 4x4 matrix");
    for (int i = 0; i < 16; ++i) T(i / 4, i % 4) = d[i].as<double>();
  } else if (n.IsSequence() && n.size() == 4) {
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c) T(r, c) = n[r][c].as<double>();
  } else {
    throw std::runtime_error("cannot parse 4x4 matrix from yaml");
  }
  return T;
}

inline void emitMatrix(YAML::Emitter& e, const Eigen::MatrixXd& M) {
  e << YAML::BeginSeq;
  for (int r = 0; r < M.rows(); ++r) {
    e << YAML::Flow << YAML::BeginSeq;
    for (int c = 0; c < M.cols(); ++c) e << M(r, c);
    e << YAML::EndSeq;
  }
  e << YAML::EndSeq;
}

inline void emitVector(YAML::Emitter& e, const Eigen::VectorXd& v) {
  e << YAML::Flow << YAML::BeginSeq;
  for (int i = 0; i < v.size(); ++i) e << v[i];
  e << YAML::EndSeq;
}

}  // namespace kalibr
