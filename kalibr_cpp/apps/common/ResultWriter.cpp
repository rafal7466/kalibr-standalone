#include "common/ResultWriter.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

#include "common/CameraCalibrator.hpp"
#include "common/Common.hpp"
#include "common/YamlUtil.hpp"

namespace kalibr {

using sm::kinematics::Transformation;

// ---------------------------------------------------------------------------
CameraChain CameraChain::fromYaml(const std::string& file) {
  YAML::Node n = loadYamlFile(file);
  CameraChain chain;
  for (int i = 0;; ++i) {
    std::string key = "cam" + std::to_string(i);
    if (!n[key]) break;
    const YAML::Node c = n[key];
    CameraChainEntry e;
    e.cameraModel = c["camera_model"].as<std::string>();
    e.intrinsics = yamlToVector(c["intrinsics"]);
    e.distortionModel = c["distortion_model"].as<std::string>();
    e.distortionCoeffs = yamlToVector(c["distortion_coeffs"]);
    e.resolution << c["resolution"][0].as<int>(), c["resolution"][1].as<int>();
    if (c["rostopic"]) e.rostopic = c["rostopic"].as<std::string>();
    if (c["cam_overlaps"])
      for (auto o : c["cam_overlaps"]) e.camOverlaps.push_back(o.as<int>());
    if (c["T_cn_cnm1"]) {
      e.hasT_cn_cnm1 = true;
      e.T_cn_cnm1 = yamlToMatrix4d(c["T_cn_cnm1"]);
    }
    if (c["T_cam_imu"]) {
      e.hasT_cam_imu = true;
      e.T_cam_imu = yamlToMatrix4d(c["T_cam_imu"]);
    }
    if (c["timeshift_cam_imu"]) {
      e.hasTimeshift = true;
      e.timeshift_cam_imu = c["timeshift_cam_imu"].as<double>();
    }
    chain.cams.push_back(e);
  }
  if (chain.cams.empty()) throw std::runtime_error("no cameras (cam0, cam1, ...) in " + file);
  return chain;
}

void CameraChain::writeYaml(const std::string& file) const {
  YAML::Emitter e;
  e.SetDoublePrecision(16);
  e << YAML::BeginMap;
  for (size_t i = 0; i < cams.size(); ++i) {
    const auto& c = cams[i];
    e << YAML::Key << ("cam" + std::to_string(i)) << YAML::Value << YAML::BeginMap;
    if (c.hasT_cam_imu) {
      e << YAML::Key << "T_cam_imu" << YAML::Value;
      emitMatrix(e, c.T_cam_imu);
    }
    if (c.hasT_cn_cnm1 && i > 0) {
      e << YAML::Key << "T_cn_cnm1" << YAML::Value;
      emitMatrix(e, c.T_cn_cnm1);
    }
    e << YAML::Key << "cam_overlaps" << YAML::Value << YAML::Flow << c.camOverlaps;
    e << YAML::Key << "camera_model" << YAML::Value << c.cameraModel;
    e << YAML::Key << "distortion_coeffs" << YAML::Value;
    emitVector(e, c.distortionCoeffs);
    e << YAML::Key << "distortion_model" << YAML::Value << c.distortionModel;
    e << YAML::Key << "intrinsics" << YAML::Value;
    emitVector(e, c.intrinsics);
    e << YAML::Key << "resolution" << YAML::Value << YAML::Flow << YAML::BeginSeq << c.resolution[0]
      << c.resolution[1] << YAML::EndSeq;
    e << YAML::Key << "rostopic" << YAML::Value << c.rostopic;
    if (c.hasTimeshift) e << YAML::Key << "timeshift_cam_imu" << YAML::Value << c.timeshift_cam_imu;
    e << YAML::EndMap;
  }
  e << YAML::EndMap;
  std::ofstream f(file);
  if (!f.good()) throw std::runtime_error("Could not write " + file);
  f << e.c_str() << "\n";
}

void CameraChain::printDetails(std::ostream& os) const {
  for (size_t i = 0; i < cams.size(); ++i) {
    const auto& c = cams[i];
    os << "Camera chain - cam" << i << ":\n";
    os << "  Camera model: " << c.cameraModel << "\n";
    os << "  Intrinsics: " << fmtVec(c.intrinsics) << "\n";
    os << "  Distortion model: " << c.distortionModel << "\n";
    os << "  Distortion coefficients: " << fmtVec(c.distortionCoeffs) << "\n";
    os << "  Resolution: " << c.resolution.transpose() << "\n";
    if (c.hasT_cn_cnm1 && i > 0) os << "  baseline (T_cn_cnm1):\n" << fmtMat(c.T_cn_cnm1, 8, "    ") << "\n";
  }
}

// ---------------------------------------------------------------------------
ImuConfig ImuConfig::fromYaml(const std::string& file) {
  YAML::Node n = loadYamlFile(file);
  ImuConfig c;
  if (n["rostopic"]) c.rostopic = n["rostopic"].as<std::string>();
  if (n["update_rate"])
    c.updateRate = n["update_rate"].as<double>();
  else if (n["rate_hz"])
    c.updateRate = n["rate_hz"].as<double>();
  else
    throw std::runtime_error("imu yaml " + file + " needs update_rate (or rate_hz)");
  c.accelNoiseDensity = n["accelerometer_noise_density"].as<double>();
  c.accelRandomWalk = n["accelerometer_random_walk"].as<double>();
  c.gyroNoiseDensity = n["gyroscope_noise_density"].as<double>();
  c.gyroRandomWalk = n["gyroscope_random_walk"].as<double>();
  if (c.accelNoiseDensity <= 0 || c.accelRandomWalk <= 0 || c.gyroNoiseDensity <= 0 || c.gyroRandomWalk <= 0)
    throw std::runtime_error("invalid IMU noise statistics (must be > 0)");
  return c;
}

double ImuConfig::accelUncertaintyDiscrete() const { return accelNoiseDensity / std::sqrt(1.0 / updateRate); }
double ImuConfig::gyroUncertaintyDiscrete() const { return gyroNoiseDensity / std::sqrt(1.0 / updateRate); }

void ImuConfig::writeYaml(const std::string& file) const {
  YAML::Emitter e;
  e.SetDoublePrecision(16);
  e << YAML::BeginMap << YAML::Key << "imu0" << YAML::Value << YAML::BeginMap;
  e << YAML::Key << "T_i_b" << YAML::Value;
  emitMatrix(e, T_i_b);
  e << YAML::Key << "accelerometer_noise_density" << YAML::Value << accelNoiseDensity;
  e << YAML::Key << "accelerometer_random_walk" << YAML::Value << accelRandomWalk;
  e << YAML::Key << "gyroscope_noise_density" << YAML::Value << gyroNoiseDensity;
  e << YAML::Key << "gyroscope_random_walk" << YAML::Value << gyroRandomWalk;
  e << YAML::Key << "model" << YAML::Value << model;
  e << YAML::Key << "rostopic" << YAML::Value << rostopic;
  e << YAML::Key << "time_offset" << YAML::Value << timeOffset;
  e << YAML::Key << "update_rate" << YAML::Value << updateRate;
  e << YAML::EndMap << YAML::EndMap;
  std::ofstream f(file);
  if (!f.good()) throw std::runtime_error("Could not write " + file);
  f << e.c_str() << "\n";
}

void ImuConfig::printDetails(std::ostream& os) const {
  os << "  Model: " << model << "\n";
  os << "  Update rate: " << updateRate << "\n";
  os << "  Accelerometer:\n";
  os << "    Noise density: " << accelNoiseDensity << " \n";
  os << "    Noise density (discrete): " << accelUncertaintyDiscrete() << " \n";
  os << "    Random walk: " << accelRandomWalk << "\n";
  os << "  Gyroscope:\n";
  os << "    Noise density: " << gyroNoiseDensity << "\n";
  os << "    Noise density (discrete): " << gyroUncertaintyDiscrete() << " \n";
  os << "    Random walk: " << gyroRandomWalk << "\n";
  os << "  T_ib (imu0 to imu0)\n" << fmtMat(T_i_b, 8, "    ") << "\n";
  os << "  time offset with respect to IMU0: " << timeOffset << " [s]\n";
}

// ---------------------------------------------------------------------------
void saveChainParametersYaml(CameraCalibration& calib, const std::string& file, const MulticamCalibrationGraph& graph) {
  CameraChain chain;
  for (size_t c = 0; c < calib.cameras.size(); ++c) {
    auto& cam = calib.cameras[c];
    CameraChainEntry e;
    e.cameraModel = cam->model->cameraModelName();
    e.distortionModel = cam->model->distortionModelName();
    e.intrinsics = cam->model->yamlIntrinsics();
    e.distortionCoeffs = cam->model->distortionParameters();
    e.resolution = cam->model->resolution();
    e.rostopic = "/" + cam->topic + "/image_raw";
    e.camOverlaps = graph.getCamOverlaps(static_cast<int>(c));
    if (c > 0) {
      e.hasT_cn_cnm1 = true;
      e.T_cn_cnm1 = calib.baselines[c - 1]->T();
    }
    chain.cams.push_back(e);
  }
  chain.writeYaml(file);
}

void saveResultTxt(CameraCalibration& calib, const std::string& file) {
  std::ofstream f(file);
  printParameters(calib, f);
}

void exportPoses(CameraCalibration& calib, const std::string& file) {
  std::vector<std::pair<double, Transformation>> poses;
  for (auto& v : calib.views) poses.emplace_back(v->timestamp, Transformation(v->dv_T_target_camera->T()));
  std::sort(poses.begin(), poses.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  std::ofstream f(file);
  f << "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []\n";
  f << std::setprecision(17);
  for (auto& p : poses) {
    Eigen::Vector3d t = p.second.t();
    Eigen::Vector4d q = p.second.q();  // JPL x y z w
    f << static_cast<long long>(std::llround(p.first * 1e9)) << "," << t[0] << "," << t[1] << "," << t[2] << ","
      << q[3] << "," << q[0] << "," << q[1] << "," << q[2] << "\n";
  }
}

}  // namespace kalibr
