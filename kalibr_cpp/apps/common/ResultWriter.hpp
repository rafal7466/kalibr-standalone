// camchain / imu yaml I/O (port of kalibr_common.ConfigReader parts) and result writers
#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace kalibr {

class CameraCalibration;
class MulticamCalibrationGraph;

struct CameraChainEntry {
  std::string cameraModel = "pinhole", distortionModel = "radtan", rostopic;
  Eigen::VectorXd intrinsics, distortionCoeffs;
  Eigen::Vector2i resolution = Eigen::Vector2i::Zero();
  std::vector<int> camOverlaps;
  bool hasT_cn_cnm1 = false;
  Eigen::Matrix4d T_cn_cnm1 = Eigen::Matrix4d::Identity();
  bool hasT_cam_imu = false;
  Eigen::Matrix4d T_cam_imu = Eigen::Matrix4d::Identity();
  bool hasTimeshift = false;
  double timeshift_cam_imu = 0.0;
};

struct CameraChain {
  std::vector<CameraChainEntry> cams;
  static CameraChain fromYaml(const std::string& file);
  void writeYaml(const std::string& file) const;
  void printDetails(std::ostream& os) const;
};

struct ImuConfig {
  std::string rostopic = "/imu0";
  double updateRate = 200.0;
  double accelNoiseDensity = 0.0, accelRandomWalk = 0.0;
  double gyroNoiseDensity = 0.0, gyroRandomWalk = 0.0;
  // outputs
  std::string model = "calibrated";
  Eigen::Matrix4d T_i_b = Eigen::Matrix4d::Identity();
  double timeOffset = 0.0;

  /// accepts kalibr imu.yaml (update_rate) and EuRoC imu0/sensor.yaml (rate_hz)
  static ImuConfig fromYaml(const std::string& file);
  void writeYaml(const std::string& file) const;  // kalibr "imu set" format (imu0: ...)
  void printDetails(std::ostream& os) const;
  double accelUncertaintyDiscrete() const;
  double gyroUncertaintyDiscrete() const;
};

/// kalibr_camera_calibration.saveChainParametersYaml
void saveChainParametersYaml(CameraCalibration& calib, const std::string& file, const MulticamCalibrationGraph& graph);
/// kalibr_camera_calibration.saveResultTxt
void saveResultTxt(CameraCalibration& calib, const std::string& file);
/// exportPoses: time_ns, position, quaternion of the target-to-cam0 poses
void exportPoses(CameraCalibration& calib, const std::string& file);

}  // namespace kalibr
