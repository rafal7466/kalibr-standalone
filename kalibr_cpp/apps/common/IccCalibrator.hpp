// C++ port of kalibr_imu_camera_calibration (IccCalibrator.py, IccSensors.py, IccUtil.py)
// Supports the default "calibrated" IMU model and a single IMU.
#pragma once

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/calibration/core/OptimizationProblem.h>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <bsplines/BSpline.hpp>
#include <bsplines/BSplinePose.hpp>

#include "common/CameraCalibrator.hpp"
#include "common/ResultWriter.hpp"

namespace kalibr {

enum : size_t { ICC_CALIBRATION_GROUP_ID = 0, ICC_HELPER_GROUP_ID = 1 };

struct ImuMeasurement {
  double t;  // [s]
  Eigen::Vector3d omega, alpha;
  Eigen::Matrix3d omegaInvR, alphaInvR;
};

class IccImu {
 public:
  IccImu(const ImuConfig& config, const std::string& imuFolder, const std::vector<double>& fromTo);

  void initBiasSplines(const bsplines::BSplinePose& poseSpline, int splineOrder, int biasKnotsPerSecond);
  void addDesignVariables(aslam::calibration::OptimizationProblem& problem);
  void addAccelerometerErrorTerms(aslam::calibration::OptimizationProblem& problem,
                                  aslam::splines::BSplinePoseDesignVariable& poseDv,
                                  const aslam::backend::EuclideanExpression& g_w, double mSigma,
                                  double accelNoiseScale);
  void addGyroscopeErrorTerms(aslam::calibration::OptimizationProblem& problem,
                              aslam::splines::BSplinePoseDesignVariable& poseDv, double mSigma, double gyroNoiseScale);
  void addBiasMotionTerms(aslam::calibration::OptimizationProblem& problem);

  ImuConfig config;
  std::string name;
  std::vector<ImuMeasurement> imuData;
  double accelUncertaintyDiscrete, accelRandomWalk, gyroUncertaintyDiscrete, gyroRandomWalk;
  Eigen::Vector3d gyroBiasPrior = Eigen::Vector3d::Zero();
  int gyroBiasPriorCount = 0;
  double timeOffset = 0.0;

  std::unique_ptr<bsplines::BSpline> gyroBias, accelBias;
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> gyroBiasDv, accelBiasDv;
  boost::shared_ptr<aslam::backend::RotationQuaternion> q_i_b_Dv;
  boost::shared_ptr<aslam::backend::EuclideanPoint> r_b_Dv;
  std::vector<boost::shared_ptr<aslam::backend::ErrorTerm>> accelErrors, gyroErrors, biasMotionErrors;
};

class IccCamera {
 public:
  IccCamera(const CameraChainEntry& camConfig, const TargetConfig& targetConfig, const std::string& camFolder,
            const std::vector<double>& fromTo, double freq, double reprojectionSigma, int numThreads);

  void findOrientationPriorCameraToImu(IccImu& imu);
  void findTimeshiftCameraImuPrior(IccImu& imu, bool verbose);
  std::unique_ptr<bsplines::BSplinePose> initPoseSplineFromCamera(int splineOrder = 6, int poseKnotsPerSecond = 100,
                                                                 double timeOffsetPadding = 0.02) const;
  void addDesignVariables(aslam::calibration::OptimizationProblem& problem, bool noExtrinsics, bool noTimeCalibration,
                          size_t baselineGroupId);
  void addCameraErrorTerms(aslam::calibration::OptimizationProblem& problem,
                           aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
                           const aslam::backend::TransformationExpression& T_cN_b, double blakeZissermanDf,
                           double timeOffsetPadding);

  CameraChainEntry camConfig;
  std::string name;
  CameraModel::Ptr camera;
  std::vector<ObservationPtr> targetObservations;
  sm::kinematics::Transformation T_extrinsic;
  double timeshiftCamToImuPrior = 0.0;
  Eigen::Vector3d gravity_w = Eigen::Vector3d(9.80655, 0.0, 0.0);
  double cornerUncertainty;

  TransformationDvPtr T_c_b_Dv;
  boost::shared_ptr<aslam::backend::Scalar> cameraTimeToImuTimeDv;
  std::vector<std::vector<boost::shared_ptr<aslam::backend::ErrorTerm>>> allReprojectionErrors;
};

class IccCameraChain {
 public:
  IccCameraChain(const CameraChain& chain, const TargetConfig& targetConfig, const std::string& datasetRoot,
                 const std::vector<std::string>& camFolders, const std::vector<double>& fromTo, double freq,
                 double reprojectionSigma, int numThreads);
  sm::kinematics::Transformation getResultTrafoImuToCam(int camNr) const;
  sm::kinematics::Transformation getResultBaseline(int fromCam, int toCam, double& baseline) const;
  double getResultTimeShift(int camNr) const;
  void addDesignVariables(aslam::calibration::OptimizationProblem& problem, bool noTimeCalibration,
                          bool noChainExtrinsics);
  void addCameraChainErrorTerms(aslam::calibration::OptimizationProblem& problem,
                                aslam::splines::BSplinePoseDesignVariable& poseSplineDv, double blakeZissermanDf,
                                double timeOffsetPadding);

  CameraChain chainConfig;
  std::vector<std::shared_ptr<IccCamera>> camList;
};

class IccCalibrator {
 public:
  struct BuildOptions {
    int splineOrder = 6;
    int poseKnotsPerSecond = 100;
    int biasKnotsPerSecond = 50;
    bool doBiasMotionError = true;
    double blakeZisserCam = -1;
    double huberAccel = -1;
    double huberGyro = -1;
    bool noTimeCalibration = false;
    bool noChainExtrinsics = true;
    double timeOffsetPadding = 0.03;
    double gyroNoiseScale = 1.0, accelNoiseScale = 1.0;
    bool verbose = false;
  };

  void registerCamChain(std::shared_ptr<IccCameraChain> c) { cameraChain = c; }
  void registerImu(std::shared_ptr<IccImu> imu) { imuList.push_back(imu); }
  void buildProblem(const BuildOptions& o);
  void optimize(int maxIterations, bool recoverCov, int numThreads);
  void recoverCovariance();

  void printErrorStatistics(std::ostream& os);
  void printResults(std::ostream& os, bool withCov);
  void saveCamChainParametersYaml(const std::string& file);
  void saveImuSetParametersYaml(const std::string& file);
  void exportPoses(const std::string& file);

  std::shared_ptr<IccCameraChain> cameraChain;
  std::vector<std::shared_ptr<IccImu>> imuList;
  std::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseDv;
  boost::shared_ptr<aslam::backend::EuclideanDirection> gravityDv;
  std::shared_ptr<aslam::backend::EuclideanExpression> gravityExpression;
  boost::shared_ptr<aslam::calibration::OptimizationProblem> problem;
  bool noTimeCalibration = false;
  bool hasCovariance = false;
  Eigen::VectorXd std_trafo_ic, std_times;
};

}  // namespace kalibr
