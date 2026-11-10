// kalibr_compare_euroc
//
// Compares kalibr_cpp results (camchain.yaml / camchain-imucam.yaml) with the
// reference calibration shipped with EuRoC (sensor.yaml files: T_BS + intrinsics).
//
//   kalibr_compare_euroc --ref cam0/sensor.yaml cam1/sensor.yaml
//                        [--camchain out-camchain.yaml] [--imucam out-camchain-imucam.yaml]
//                        [--report report.md]
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/Transformation.hpp>

#include "common/ArgParser.hpp"
#include "common/Common.hpp"
#include "common/ResultWriter.hpp"
#include "common/YamlUtil.hpp"

using namespace kalibr;
using sm::kinematics::Transformation;

struct RefCam {
  Eigen::Matrix4d T_BS = Eigen::Matrix4d::Identity();  // body(imu) <- sensor(cam)
  Eigen::VectorXd intrinsics, distortion;
  bool hasIntrinsics = false;
};

static RefCam loadRef(const std::string& file) {
  YAML::Node n = loadYamlFile(file);
  RefCam r;
  r.T_BS = yamlToMatrix4d(n["T_BS"]);
  if (n["intrinsics"]) {
    r.intrinsics = yamlToVector(n["intrinsics"]);
    r.hasIntrinsics = true;
  }
  if (n["distortion_coefficients"]) r.distortion = yamlToVector(n["distortion_coefficients"]);
  if (n["distortion_coeffs"]) r.distortion = yamlToVector(n["distortion_coeffs"]);
  return r;
}

static double rotErrDeg(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
  Eigen::Matrix3d D = A.transpose() * B;
  double c = std::max(-1.0, std::min(1.0, (D.trace() - 1.0) / 2.0));
  return std::acos(c) * 180.0 / M_PI;
}

static std::string num(double v, int prec = 4) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(prec) << v;
  return os.str();
}

int main(int argc, char** argv) {
  ArgParser p("Compare kalibr_cpp results with the EuRoC reference calibration (sensor.yaml).");
  p.add("--ref", -1, "Reference sensor.yaml per camera (cam0 cam1 ...), must contain T_BS and optionally intrinsics",
        "", true);
  p.add("--camchain", 1, "camchain.yaml produced by kalibr_calibrate_cameras");
  p.add("--imucam", 1, "camchain-imucam.yaml produced by kalibr_calibrate_imu_camera");
  p.add("--report", 1, "Write a markdown report to this file");
  try {
    if (!p.parse(argc, argv)) {
      p.printHelp();
      return 2;
    }
  } catch (const std::exception& e) {
    p.printHelp();
    KLOG_ERROR(e.what());
    return 2;
  }
  std::ostringstream md;
  try {
    std::vector<RefCam> refs;
    for (auto& f : p.list("--ref")) refs.push_back(loadRef(f));

    if (p.has("--camchain")) {
      CameraChain chain = CameraChain::fromYaml(p.str("--camchain"));
      md << "## Camera intrinsics (kalibr_calibrate_cameras vs EuRoC)\n\n";
      md << "| cam | param | kalibr_cpp | EuRoC | diff | rel. diff [%] |\n|---|---|---|---|---|---|\n";
      const char* pn[] = {"fu", "fv", "cu", "cv"};
      const char* dn[] = {"k1", "k2", "p1", "p2"};
      for (size_t c = 0; c < chain.cams.size() && c < refs.size(); ++c) {
        if (!refs[c].hasIntrinsics) continue;
        for (int i = 0; i < std::min<int>(4, chain.cams[c].intrinsics.size()); ++i) {
          double a = chain.cams[c].intrinsics[i], b = refs[c].intrinsics[i];
          md << "| cam" << c << " | " << pn[i] << " | " << num(a, 3) << " | " << num(b, 3) << " | " << num(a - b, 3)
             << " | " << num(100.0 * (a - b) / b, 3) << " |\n";
        }
        for (int i = 0; i < std::min<int>(4, std::min<int>(chain.cams[c].distortionCoeffs.size(), refs[c].distortion.size())); ++i) {
          double a = chain.cams[c].distortionCoeffs[i], b = refs[c].distortion[i];
          md << "| cam" << c << " | " << dn[i] << " | " << num(a, 6) << " | " << num(b, 6) << " | " << num(a - b, 6)
             << " | - |\n";
        }
      }
      md << "\n## Stereo baseline T_cn_cnm1 (vs EuRoC T_BS)\n\n";
      md << "| pair | rotation error [deg] | translation error [mm] | baseline kalibr_cpp [mm] | baseline EuRoC [mm] |\n"
            "|---|---|---|---|---|\n";
      for (size_t c = 1; c < chain.cams.size() && c < refs.size(); ++c) {
        Transformation T_est(chain.cams[c].T_cn_cnm1);
        Transformation T_ref = Transformation(refs[c].T_BS).inverse() * Transformation(refs[c - 1].T_BS);
        md << "| cam" << c - 1 << "->cam" << c << " | " << num(rotErrDeg(T_est.C(), T_ref.C()), 4) << " | "
           << num(1000.0 * (T_est.t() - T_ref.t()).norm(), 3) << " | " << num(1000.0 * T_est.t().norm(), 3) << " | "
           << num(1000.0 * T_ref.t().norm(), 3) << " |\n";
        md << "\nEstimated T_cam" << c << "_cam" << c - 1 << ":\n```\n" << fmtMat(T_est.T()) << "\n```\n";
        md << "EuRoC T_cam" << c << "_cam" << c - 1 << ":\n```\n" << fmtMat(T_ref.T()) << "\n```\n";
      }
    }

    if (p.has("--imucam")) {
      CameraChain chain = CameraChain::fromYaml(p.str("--imucam"));
      md << "\n## Camera-IMU extrinsics T_cam_imu (kalibr_calibrate_imu_camera vs EuRoC)\n\n";
      md << "| cam | rotation error [deg] | translation error [mm] | t kalibr_cpp [m] | t EuRoC [m] | time shift [ms] |\n"
            "|---|---|---|---|---|---|\n";
      for (size_t c = 0; c < chain.cams.size() && c < refs.size(); ++c) {
        if (!chain.cams[c].hasT_cam_imu) continue;
        Transformation T_est(chain.cams[c].T_cam_imu);
        Transformation T_ref = Transformation(refs[c].T_BS).inverse();
        md << "| cam" << c << " | " << num(rotErrDeg(T_est.C(), T_ref.C()), 4) << " | "
           << num(1000.0 * (T_est.t() - T_ref.t()).norm(), 3) << " | " << fmtVec(T_est.t(), 5) << " | "
           << fmtVec(T_ref.t(), 5) << " | "
           << (chain.cams[c].hasTimeshift ? num(1000.0 * chain.cams[c].timeshift_cam_imu, 4) : std::string("-"))
           << " |\n";
      }
      for (size_t c = 0; c < chain.cams.size() && c < refs.size(); ++c) {
        if (!chain.cams[c].hasT_cam_imu) continue;
        md << "\nEstimated T_cam" << c << "_imu:\n```\n" << fmtMat(chain.cams[c].T_cam_imu) << "\n```\n";
        md << "EuRoC inv(T_BS) cam" << c << ":\n```\n" << fmtMat(Transformation(refs[c].T_BS).inverse().T()) << "\n```\n";
      }
    }
  } catch (const std::exception& e) {
    KLOG_ERROR("Exception: " << e.what());
    return 1;
  }
  std::cout << md.str() << std::endl;
  if (p.has("--report")) {
    std::ofstream f(p.str("--report"));
    f << "# kalibr_cpp vs. EuRoC reference calibration\n\n" << md.str();
  }
  return 0;
}
