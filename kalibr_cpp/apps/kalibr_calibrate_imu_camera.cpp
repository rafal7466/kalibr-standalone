// kalibr_calibrate_imu_camera (C++ / no ROS)
//
// Port of kalibr/python/kalibr_calibrate_imu_camera: calibrates the spatial and
// temporal parameters of an IMU w.r.t. a (pre-calibrated) camera chain.
// Reads the EuRoC/ASL folder format instead of a rosbag.
//
// Example (EuRoC imu_april):
//   kalibr_calibrate_imu_camera --dataset imu_april/mav0 --cams camchain.yaml
//        --imu imu_april/mav0/imu0/sensor.yaml --target april_6x6.yaml --output out/imu_april
#include <fstream>
#include <iostream>

#include <boost/filesystem.hpp>
#include <sm/logging.hpp>

#include "common/ArgParser.hpp"
#include "common/IccCalibrator.hpp"

using namespace kalibr;

int main(int argc, char** argv) {
  ArgParser p("Calibrate the spatial and temporal parameters of an IMU to a camera chain.\n"
              "(C++ port of kalibr_calibrate_imu_camera, reads EuRoC/ASL folders instead of rosbags)");
  p.add("--dataset", 1, "Dataset root folder in ASL format (e.g. .../imu_april/mav0)", "", true);
  p.add("--bag-from-to", 2, "Use the data starting from up to this time [s]");
  p.add("--bag-freq", 1, "Frequency to extract features at [hz]");
  p.add("--cams", 1, "Camera system configuration as yaml file (camchain.yaml from kalibr_calibrate_cameras)", "", true);
  p.add("--cam-folders", -1, "Camera folders (default: derived from the rostopic of every camera, e.g. cam0)");
  p.add("--recompute-camera-chain-extrinsics", 0, "Recompute the camera chain extrinsics (debugging only)");
  p.add("--reprojection-sigma", 1, "Standard deviation of the distribution of reprojected corner points [px]", "1.0");
  p.add("--imu", 1, "Yaml file with IMU noise parameters (kalibr imu.yaml or EuRoC imu0/sensor.yaml)", "", true);
  p.add("--imu-folder", 1, "IMU folder inside --dataset", "imu0");
  p.add("--target", 1, "Calibration target configuration as yaml file", "", true);
  p.add("--no-time-calibration", 0, "Disable the temporal calibration");
  p.add("--max-iter", 1, "Max. iterations", "30");
  p.add("--recover-covariance", 0, "Recover the covariance of the design variables.");
  p.add("--timeoffset-padding", 1, "Maximum range in which the timeoffset may change during estimation [s]", "0.03");
  p.add("--threads", 1, "Number of threads", std::to_string(defaultNumThreads()));
  p.add("--verbose", 0, "Verbose output");
  p.add("--export-poses", 0, "Export the optimized poses into a CSV");
  p.add("--output", 1, "Output prefix for the result files (default: <dataset>-...)");
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
  const bool verbose = p.flag("--verbose");
  logLevel() = verbose ? LogLevel::Debug : LogLevel::Info;
  sm::logging::setLevel(verbose ? sm::logging::levels::Debug : sm::logging::levels::Warn);
  const std::string datasetRoot = p.str("--dataset");
  const std::vector<double> fromTo = p.dblList("--bag-from-to");
  const double freq = p.has("--bag-freq") ? p.dbl("--bag-freq") : -1.0;
  const int nThreads = std::max(1, p.integer("--threads"));

  std::string outPrefix = p.str("--output");
  if (outPrefix.empty()) {
    boost::filesystem::path d(datasetRoot);
    if (d.filename() == "mav0") d = d.parent_path();
    outPrefix = d.string();
  }
  boost::filesystem::path outDir = boost::filesystem::path(outPrefix).parent_path();
  if (!outDir.empty()) boost::filesystem::create_directories(outDir);

  try {
    ImuConfig imuConfig = ImuConfig::fromYaml(p.str("--imu"));
    std::cout << "IMU configuration\n";
    imuConfig.printDetails(std::cout);
    std::string imuFolder = p.str("--imu-folder");
    if (!boost::filesystem::exists(imuFolder)) imuFolder = joinPath(datasetRoot, imuFolder);
    auto imu = std::make_shared<IccImu>(imuConfig, imuFolder, fromTo);

    TargetConfig targetConfig = TargetConfig::fromYaml(p.str("--target"));
    std::cout << "Calibration target configuration\n";
    targetConfig.printDetails(std::cout);

    CameraChain chain = CameraChain::fromYaml(p.str("--cams"));
    chain.printDetails(std::cout);
    auto camChain = std::make_shared<IccCameraChain>(chain, targetConfig, datasetRoot, p.list("--cam-folders"), fromTo,
                                                     freq, p.dbl("--reprojection-sigma"), nThreads);

    IccCalibrator iCal;
    iCal.registerCamChain(camChain);
    iCal.registerImu(imu);

    std::cout << "\nBuilding the problem" << std::endl;
    IccCalibrator::BuildOptions o;
    o.splineOrder = 6;
    o.poseKnotsPerSecond = 100;
    o.biasKnotsPerSecond = 50;
    o.doBiasMotionError = true;
    o.blakeZisserCam = -1;
    o.huberAccel = -1;
    o.huberGyro = -1;
    o.noTimeCalibration = p.flag("--no-time-calibration");
    o.noChainExtrinsics = !p.flag("--recompute-camera-chain-extrinsics");
    o.timeOffsetPadding = p.dbl("--timeoffset-padding");
    o.verbose = verbose;
    iCal.buildProblem(o);

    std::cout << "\nBefore Optimization\n===================" << std::endl;
    iCal.printErrorStatistics(std::cout);
    std::cout << "\nOptimizing..." << std::endl;
    iCal.optimize(p.integer("--max-iter"), p.flag("--recover-covariance"), nThreads);
    std::cout << "\nAfter Optimization (Results)\n==================" << std::endl;
    iCal.printErrorStatistics(std::cout);
    iCal.printResults(std::cout, p.flag("--recover-covariance"));

    const std::string yamlCam = outPrefix + "-camchain-imucam.yaml";
    iCal.saveCamChainParametersYaml(yamlCam);
    std::cout << "\nResults written to file: " << yamlCam << std::endl;
    const std::string yamlImu = outPrefix + "-imu.yaml";
    iCal.saveImuSetParametersYaml(yamlImu);
    std::cout << "  Saving imu calibration to file: " << yamlImu << std::endl;
    const std::string txt = outPrefix + "-results-imucam.txt";
    {
      std::ofstream f(txt);
      f << "Calibration results\n===================\n";
      iCal.printErrorStatistics(f);
      iCal.printResults(f, p.flag("--recover-covariance"));
      f << "\n\nTarget configuration\n====================\n\n";
      targetConfig.printDetails(f);
    }
    std::cout << "  Detailed results written to file: " << txt << std::endl;
    if (p.flag("--export-poses")) {
      const std::string poses = outPrefix + "-poses-imucam-imu0.csv";
      iCal.exportPoses(poses);
      std::cout << "  Poses written to " << poses << std::endl;
    }
  } catch (const std::exception& e) {
    KLOG_ERROR("Exception: " << e.what());
    return 1;
  }
  return 0;
}
