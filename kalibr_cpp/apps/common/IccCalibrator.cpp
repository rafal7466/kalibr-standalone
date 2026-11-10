#include "common/IccCalibrator.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <unordered_set>

#include <boost/filesystem.hpp>

#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <kalibr_errorterms/EuclideanError.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>
#include <sm/boost/null_deleter.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/transformations.hpp>

namespace kalibr {

namespace ab = aslam::backend;
using sm::kinematics::Transformation;

template <class T>
static boost::shared_ptr<ab::DesignVariable> nonOwning(T* dv) {
  return boost::shared_ptr<ab::DesignVariable>(dv, sm::null_deleter());
}

/// python int(round(x)) (round half to even)
static int pyRound(double x) { return static_cast<int>(std::nearbyint(x)); }

// ===========================================================================
// IccImu
// ===========================================================================
IccImu::IccImu(const ImuConfig& cfg, const std::string& imuFolder, const std::vector<double>& fromTo) : config(cfg) {
  std::cout << "Initializing imu dataset reader:\n\tDataset:          " << imuFolder << std::endl;
  ImuCsvReader reader(imuFolder, fromTo);
  name = reader.name();
  accelUncertaintyDiscrete = config.accelUncertaintyDiscrete();
  accelRandomWalk = config.accelRandomWalk;
  gyroUncertaintyDiscrete = config.gyroUncertaintyDiscrete();
  gyroRandomWalk = config.gyroRandomWalk;
  const Eigen::Matrix3d Rgyro = Eigen::Matrix3d::Identity() * gyroUncertaintyDiscrete * gyroUncertaintyDiscrete;
  const Eigen::Matrix3d Raccel = Eigen::Matrix3d::Identity() * accelUncertaintyDiscrete * accelUncertaintyDiscrete;
  for (auto& m : reader.data()) {
    ImuMeasurement im;
    im.t = aslam::Time(static_cast<uint32_t>(m.stamp_ns / 1000000000LL), static_cast<uint32_t>(m.stamp_ns % 1000000000LL)).toSec();
    im.omega = m.omega;
    im.alpha = m.alpha;
    im.omegaInvR = Rgyro.inverse();
    im.alphaInvR = Raccel.inverse();
    imuData.push_back(im);
  }
  if (imuData.size() < 2) throw std::runtime_error("Could not find any IMU messages. Please check the dataset.");
  std::cout << "  Read " << imuData.size() << " imu readings over " << std::fixed << std::setprecision(1)
            << imuData.back().t - imuData.front().t << " seconds" << std::endl;
  std::cout.unsetf(std::ios::floatfield);
}

void IccImu::initBiasSplines(const bsplines::BSplinePose& poseSpline, int splineOrder, int biasKnotsPerSecond) {
  double start = poseSpline.t_min(), end = poseSpline.t_max();
  int knots = pyRound((end - start) * biasKnotsPerSecond);
  std::cout << "\nInitializing the bias splines with " << knots << " knots" << std::endl;
  gyroBias.reset(new bsplines::BSpline(splineOrder));
  gyroBias->initConstantSpline(start, end, knots, gyroBiasPrior);
  accelBias.reset(new bsplines::BSpline(splineOrder));
  accelBias->initConstantSpline(start, end, knots, Eigen::Vector3d::Zero());
}

void IccImu::addDesignVariables(aslam::calibration::OptimizationProblem& problem) {
  gyroBiasDv = std::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(*gyroBias);
  accelBiasDv = std::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(*accelBias);
  for (size_t i = 0; i < gyroBiasDv->numDesignVariables(); ++i) {
    gyroBiasDv->designVariable(i)->setActive(true);
    problem.addDesignVariable(nonOwning(gyroBiasDv->designVariable(i)), ICC_HELPER_GROUP_ID);
  }
  for (size_t i = 0; i < accelBiasDv->numDesignVariables(); ++i) {
    accelBiasDv->designVariable(i)->setActive(true);
    problem.addDesignVariable(nonOwning(accelBiasDv->designVariable(i)), ICC_HELPER_GROUP_ID);
  }
  q_i_b_Dv.reset(new ab::RotationQuaternion(Eigen::Vector4d(0, 0, 0, 1)));
  problem.addDesignVariable(q_i_b_Dv, ICC_HELPER_GROUP_ID);
  q_i_b_Dv->setActive(false);
  r_b_Dv.reset(new ab::EuclideanPoint(Eigen::Vector3d::Zero()));
  problem.addDesignVariable(r_b_Dv, ICC_HELPER_GROUP_ID);
  r_b_Dv->setActive(false);
}

void IccImu::addAccelerometerErrorTerms(aslam::calibration::OptimizationProblem& problem,
                                        aslam::splines::BSplinePoseDesignVariable& poseDv,
                                        const ab::EuclideanExpression& g_w, double mSigma, double accelNoiseScale) {
  std::cout << "\nAdding accelerometer error terms (" << name << ")" << std::endl;
  const double weight = 1.0 / accelNoiseScale;
  boost::shared_ptr<ab::MEstimator> mest;
  if (mSigma > 0.0)
    mest.reset(new ab::HuberMEstimator(mSigma));
  else
    mest.reset(new ab::NoMEstimator());
  size_t skipped = 0;
  const double tmin = poseDv.spline().t_min(), tmax = poseDv.spline().t_max();
  accelErrors.clear();
  for (auto& im : imuData) {
    double tk = im.t + timeOffset;
    if (tk > tmin && tk < tmax) {
      ab::RotationExpression C_b_w = poseDv.orientation(tk).inverse();
      ab::EuclideanExpression a_w = poseDv.linearAcceleration(tk);
      ab::EuclideanExpression b_i = accelBiasDv->toEuclideanExpression(tk, 0);
      ab::EuclideanExpression w_b = poseDv.angularVelocityBodyFrame(tk);
      ab::EuclideanExpression w_dot_b = poseDv.angularAccelerationBodyFrame(tk);
      ab::RotationExpression C_i_b = q_i_b_Dv->toExpression();
      ab::EuclideanExpression r_b = r_b_Dv->toExpression();
      ab::EuclideanExpression a = C_i_b * (C_b_w * (a_w - g_w) + w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b)));
      boost::shared_ptr<kalibr_errorterms::EuclideanError> aerr(
          new kalibr_errorterms::EuclideanError(im.alpha, im.alphaInvR * weight, a + b_i));
      aerr->setMEstimatorPolicy(mest);
      accelErrors.push_back(aerr);
      problem.addErrorTerm(aerr);
    } else {
      ++skipped;
    }
  }
  std::cout << "  Added " << imuData.size() - skipped << " of " << imuData.size()
            << " accelerometer error terms (skipped " << skipped << " out-of-bounds measurements)" << std::endl;
}

void IccImu::addGyroscopeErrorTerms(aslam::calibration::OptimizationProblem& problem,
                                    aslam::splines::BSplinePoseDesignVariable& poseDv, double mSigma,
                                    double gyroNoiseScale) {
  std::cout << "\nAdding gyroscope error terms (" << name << ")" << std::endl;
  const double weight = 1.0 / gyroNoiseScale;
  boost::shared_ptr<ab::MEstimator> mest;
  if (mSigma > 0.0)
    mest.reset(new ab::HuberMEstimator(mSigma));
  else
    mest.reset(new ab::NoMEstimator());
  size_t skipped = 0;
  const double tmin = poseDv.spline().t_min(), tmax = poseDv.spline().t_max();
  gyroErrors.clear();
  for (auto& im : imuData) {
    double tk = im.t + timeOffset;
    if (tk > tmin && tk < tmax) {
      ab::EuclideanExpression w_b = poseDv.angularVelocityBodyFrame(tk);
      ab::EuclideanExpression b_i = gyroBiasDv->toEuclideanExpression(tk, 0);
      ab::RotationExpression C_i_b = q_i_b_Dv->toExpression();
      ab::EuclideanExpression w = C_i_b * w_b;
      boost::shared_ptr<kalibr_errorterms::EuclideanError> gerr(
          new kalibr_errorterms::EuclideanError(im.omega, im.omegaInvR * weight, w + b_i));
      gerr->setMEstimatorPolicy(mest);
      gyroErrors.push_back(gerr);
      problem.addErrorTerm(gerr);
    } else {
      ++skipped;
    }
  }
  std::cout << "  Added " << imuData.size() - skipped << " of " << imuData.size()
            << " gyroscope error terms (skipped " << skipped << " out-of-bounds measurements)" << std::endl;
}

void IccImu::addBiasMotionTerms(aslam::calibration::OptimizationProblem& problem) {
  Eigen::MatrixXd Wgyro = Eigen::Matrix3d::Identity() / (gyroRandomWalk * gyroRandomWalk);
  Eigen::MatrixXd Waccel = Eigen::Matrix3d::Identity() / (accelRandomWalk * accelRandomWalk);
  typedef ab::BSplineMotionError<aslam::splines::EuclideanBSplineDesignVariable> err_t;
  biasMotionErrors.clear();
  biasMotionErrors.emplace_back(new err_t(gyroBiasDv.get(), Wgyro, 1));
  biasMotionErrors.emplace_back(new err_t(accelBiasDv.get(), Waccel, 1));
  for (auto& e : biasMotionErrors) problem.addErrorTerm(e);
}

// ===========================================================================
// IccCamera
// ===========================================================================
IccCamera::IccCamera(const CameraChainEntry& cfg, const TargetConfig& targetConfig, const std::string& camFolder,
                     const std::vector<double>& fromTo, double freq, double reprojectionSigma, int numThreads)
    : camConfig(cfg), cornerUncertainty(reprojectionSigma) {
  std::cout << "Initializing camera dataset reader:\n\tDataset:          " << camFolder << std::endl;
  ImageFolderReader dataset(camFolder, fromTo, freq);
  name = dataset.name();
  std::cout << "\tNumber of images: " << dataset.numImages() << std::endl;
  camera = CameraModel::fromParameters(cfg.cameraModel, cfg.distortionModel, cfg.intrinsics, cfg.distortionCoeffs,
                                       cfg.resolution);
  targetObservations = extractCornersFromDataset(dataset, *camera, targetConfig, /*noTransformation=*/false,
                                                 /*filterCornerOutliers=*/true, numThreads, /*clearImages=*/true);
  if (targetObservations.empty()) throw std::runtime_error("No calibration target observations for " + name);
}

void IccCamera::findOrientationPriorCameraToImu(IccImu& imu) {
  std::cout << "\nEstimating imu-camera rotation prior" << std::endl;
  boost::shared_ptr<ab::OptimizationProblem> problem(new ab::OptimizationProblem());
  boost::shared_ptr<ab::RotationQuaternion> q_i_c_Dv(new ab::RotationQuaternion(T_extrinsic.q()));
  q_i_c_Dv->setActive(true);
  problem->addDesignVariable(q_i_c_Dv);
  boost::shared_ptr<ab::EuclideanPoint> gyroBiasDv(new ab::EuclideanPoint(Eigen::Vector3d::Zero()));
  gyroBiasDv->setActive(true);
  problem->addDesignVariable(gyroBiasDv);

  auto poseSpline = initPoseSplineFromCamera(6, 100, 0.0);
  for (auto& im : imu.imuData) {
    double tk = im.t;
    if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
      ab::RotationExpression R_i_c = q_i_c_Dv->toExpression();
      ab::EuclideanExpression bias = gyroBiasDv->toExpression();
      ab::EuclideanExpression omega_predicted =
          R_i_c * ab::EuclideanExpression(Eigen::Vector3d(poseSpline->angularVelocityBodyFrame(tk)));
      boost::shared_ptr<kalibr_errorterms::GyroscopeError> gerr(
          new kalibr_errorterms::GyroscopeError(im.omega, im.omegaInvR, omega_predicted, bias));
      problem->addErrorTerm(gerr);
    }
  }
  if (problem->numErrorTerms() == 0)
    throw std::runtime_error("Failed to obtain orientation prior. Please make sure that your sensors are "
                             "synchronized correctly.");
  ab::Optimizer2Options options;
  options.verbose = false;
  options.linearSystemSolver.reset(new ab::BlockCholeskyLinearSystemSolver());
  options.nThreads = 2;
  options.convergenceDeltaX = 1e-4;
  options.convergenceDeltaJ = 1;
  options.maxIterations = 50;
  ab::Optimizer2 optimizer(options);
  optimizer.setProblem(problem);
  try {
    optimizer.optimize();
  } catch (const std::exception& e) {
    throw std::runtime_error(std::string("Failed to obtain orientation prior! ") + e.what());
  }

  Eigen::Matrix3d R_i_c = q_i_c_Dv->toExpression().toRotationMatrix().transpose();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.topLeftCorner<3, 3>() = R_i_c;
  T.topRightCorner<3, 1>() = T_extrinsic.t();
  T_extrinsic = Transformation(T);

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  size_t n = 0;
  for (auto& im : imu.imuData) {
    double tk = im.t;
    if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
      sum += poseSpline->orientation(tk) * (R_i_c * (-im.alpha));
      ++n;
    }
  }
  Eigen::Vector3d mean_a_w = sum / static_cast<double>(n);
  gravity_w = mean_a_w / mean_a_w.norm() * 9.80655;
  std::cout << "Gravity was intialized to " << fmtVec(gravity_w) << " [m/s^2]" << std::endl;

  Eigen::Vector3d b_gyro = gyroBiasDv->toExpression().toEuclidean();
  imu.gyroBiasPriorCount += 1;
  double c = imu.gyroBiasPriorCount;
  imu.gyroBiasPrior = (c - 1.0) / c * imu.gyroBiasPrior + 1.0 / c * b_gyro;
  std::cout << "  Orientation prior camera-imu found as: (T_i_c)\n" << fmtMat(R_i_c) << std::endl;
  std::cout << "  Gyro bias prior found as: (b_gyro)\n" << fmtVec(b_gyro) << std::endl;
}

void IccCamera::findTimeshiftCameraImuPrior(IccImu& imu, bool verbose) {
  std::cout << "Estimating time shift camera to imu:" << std::endl;
  auto poseSpline = initPoseSplineFromCamera(6, 100, 0.0);
  std::vector<double> measNorm, predNorm;
  for (auto& im : imu.imuData) {
    double tk = im.t;
    if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
      measNorm.push_back(im.omega.norm());
      predNorm.push_back(poseSpline->angularVelocityBodyFrame(tk).norm());
    }
  }
  if (measNorm.empty())
    throw std::runtime_error("The time ranges of the camera and IMU do not overlap. Please make sure that your "
                             "sensors are synchronized correctly.");
  // numpy.correlate(pred, meas, "full"): c[k] = sum_n pred[n+k]*meas[n], k = -(N-1) .. N-1
  const long N = static_cast<long>(measNorm.size());
  const long Na = static_cast<long>(predNorm.size());
  double best = -std::numeric_limits<double>::infinity();
  long bestK = 0;
  for (long k = -(N - 1); k <= Na - 1; ++k) {
    double s = 0.0;
    long n0 = std::max(0L, -k), n1 = std::min(N, Na - k);
    for (long nn = n0; nn < n1; ++nn) s += predNorm[nn + k] * measNorm[nn];
    if (s > best) {
      best = s;
      bestK = k;
    }
  }
  long discrete_shift = bestK;
  double dT = (imu.imuData.back().t - imu.imuData.front().t) / static_cast<double>(imu.imuData.size() - 1);
  double shift = -discrete_shift * dT;
  if (verbose) {
    KLOG_DEBUG("discrete time shift: " << discrete_shift);
    KLOG_DEBUG("cont. time shift: " << shift);
    KLOG_DEBUG("dT: " << dT);
  }
  timeshiftCamToImuPrior = shift;
  std::cout << "  Time shift camera to imu (t_imu = t_cam + shift):\n" << timeshiftCamToImuPrior << std::endl;
}

std::unique_ptr<bsplines::BSplinePose> IccCamera::initPoseSplineFromCamera(int splineOrder, int poseKnotsPerSecond,
                                                                          double timeOffsetPadding) const {
  const Eigen::Matrix4d T_c_b = T_extrinsic.T();
  std::unique_ptr<bsplines::BSplinePose> pose(new bsplines::BSplinePose(
      splineOrder, sm::kinematics::RotationalKinematics::Ptr(new sm::kinematics::RotationVector())));
  const size_t M = targetObservations.size();
  Eigen::VectorXd times(M + 2);
  Eigen::Matrix<double, 6, Eigen::Dynamic> curve(6, M + 2);
  for (size_t i = 0; i < M; ++i) {
    times[i + 1] = targetObservations[i]->time().toSec() + timeshiftCamToImuPrior;
    curve.col(i + 1) = pose->transformationToCurveValue(targetObservations[i]->T_t_c().T() * T_c_b);
  }
  if (!curve.middleCols(1, M).allFinite()) throw std::runtime_error("Nans in curve values");
  times[0] = times[1] - timeOffsetPadding * 2.0;
  times[M + 1] = times[M] + timeOffsetPadding * 2.0;
  curve.col(0) = curve.col(1);
  curve.col(M + 1) = curve.col(M);
  // make sure the rotation vector doesn't flip
  for (int i = 1; i < curve.cols(); ++i) {
    Eigen::Vector3d prev = curve.block<3, 1>(3, i - 1);
    Eigen::Vector3d r = curve.block<3, 1>(3, i);
    double angle = r.norm();
    Eigen::Vector3d axis = r / angle;
    Eigen::Vector3d best_r = r;
    double best_dist = (best_r - prev).norm();
    for (int s = -3; s <= 3; ++s) {
      Eigen::Vector3d aa = axis * (angle + M_PI * 2.0 * s);
      double dist = (aa - prev).norm();
      if (dist < best_dist) {
        best_r = aa;
        best_dist = dist;
      }
    }
    curve.block<3, 1>(3, i) = best_r;
  }
  double seconds = times[M + 1] - times[0];
  int knots = pyRound(seconds * poseKnotsPerSecond);
  std::cout << "\nInitializing a pose spline with " << knots << " knots (" << poseKnotsPerSecond
            << " knots per second over " << seconds << " seconds)" << std::endl;
  pose->initPoseSplineSparse(times, curve, knots, 1e-4);
  return pose;
}

void IccCamera::addDesignVariables(aslam::calibration::OptimizationProblem& problem, bool noExtrinsics,
                                   bool noTimeCalibration, size_t baselineGroupId) {
  bool active = !noExtrinsics;
  T_c_b_Dv = std::make_shared<TransformationDv>(T_extrinsic, active, active);
  for (size_t i = 0; i < T_c_b_Dv->numDesignVariables(); ++i)
    problem.addDesignVariable(T_c_b_Dv->getDesignVariable(i), baselineGroupId);
  cameraTimeToImuTimeDv.reset(new ab::Scalar(0.0));
  cameraTimeToImuTimeDv->setActive(!noTimeCalibration);
  problem.addDesignVariable(cameraTimeToImuTimeDv, ICC_CALIBRATION_GROUP_ID);
}

void IccCamera::addCameraErrorTerms(aslam::calibration::OptimizationProblem& problem,
                                    aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
                                    const ab::TransformationExpression& T_cN_b, double blakeZissermanDf,
                                    double timeOffsetPadding) {
  std::cout << "\nAdding camera error terms (" << name << ")" << std::endl;
  allReprojectionErrors.clear();
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
  const Eigen::Matrix2d invR = R.inverse();
  const double tmin = poseSplineDv.spline().t_min(), tmax = poseSplineDv.spline().t_max();
  ab::TransformationExpression T_cN_b_ = T_cN_b;
  for (auto& obs : targetObservations) {
    ab::ScalarExpression frameTime = (cameraTimeToImuTimeDv->toExpression() + obs->time().toSec()) + timeshiftCamToImuPrior;
    double frameTimeScalar = frameTime.toScalar();
    if (frameTimeScalar <= tmin || frameTimeScalar >= tmax) continue;
    ab::TransformationExpression T_w_b = poseSplineDv.transformationAtTime(frameTime, timeOffsetPadding, timeOffsetPadding);
    ab::TransformationExpression T_b_w = T_w_b.inverse();
    ab::TransformationExpression T_c_w = T_cN_b_ * T_b_w;
    std::vector<boost::shared_ptr<ab::ErrorTerm>> reprojectionErrors;
    auto target = obs->target();
    for (size_t i = 0; i < target->size(); ++i) {
      Eigen::Vector2d y;
      if (!obs->imagePoint(i, y)) continue;
      ab::HomogeneousExpression p = T_c_w * ab::HomogeneousExpression(sm::kinematics::toHomogeneous(target->point(i)));
      auto rerr = camera->createSimpleReprojectionError(y, invR, p);
      if (blakeZissermanDf > 0.0)
        rerr->setMEstimatorPolicy(boost::shared_ptr<ab::MEstimator>(new ab::BlakeZissermanMEstimator(blakeZissermanDf)));
      problem.addErrorTerm(rerr);
      reprojectionErrors.push_back(rerr);
    }
    allReprojectionErrors.push_back(reprojectionErrors);
  }
  std::cout << "  Added " << allReprojectionErrors.size() << " camera error terms (views of "
            << targetObservations.size() << ")" << std::endl;
}

// ===========================================================================
// IccCameraChain
// ===========================================================================
static std::string folderFromTopic(const std::string& topic, int camNr) {
  // "/cam0/image_raw" -> "cam0"
  std::string t = topic;
  while (!t.empty() && t[0] == '/') t.erase(0, 1);
  size_t slash = t.find('/');
  if (slash != std::string::npos) t = t.substr(0, slash);
  if (t.empty()) t = "cam" + std::to_string(camNr);
  return t;
}

IccCameraChain::IccCameraChain(const CameraChain& chain, const TargetConfig& targetConfig,
                               const std::string& datasetRoot, const std::vector<std::string>& camFolders,
                               const std::vector<double>& fromTo, double freq, double reprojectionSigma, int numThreads)
    : chainConfig(chain) {
  for (size_t c = 0; c < chain.cams.size(); ++c) {
    std::string folder;
    if (c < camFolders.size())
      folder = camFolders[c];
    else
      folder = folderFromTopic(chain.cams[c].rostopic, static_cast<int>(c));
    if (!boost::filesystem::exists(folder)) folder = joinPath(datasetRoot, folder);
    camList.push_back(std::make_shared<IccCamera>(chain.cams[c], targetConfig, folder, fromTo, freq,
                                                  reprojectionSigma, numThreads));
  }
  for (size_t c = 1; c < camList.size(); ++c) {
    camList[c]->T_extrinsic = Transformation(chain.cams[c].T_cn_cnm1);
    std::cout << "Baseline between cam" << c - 1 << " and cam" << c << " set to:\nT= \n"
              << fmtMat(camList[c]->T_extrinsic.T()) << "\nBaseline: " << camList[c]->T_extrinsic.t().norm()
              << " [m]" << std::endl;
  }
}

Transformation IccCameraChain::getResultTrafoImuToCam(int camNr) const {
  Transformation T_cN_imu(camList[0]->T_c_b_Dv->T());
  for (int c = 1; c <= camNr; ++c) T_cN_imu = Transformation(camList[c]->T_c_b_Dv->T()) * T_cN_imu;
  return T_cN_imu;
}

Transformation IccCameraChain::getResultBaseline(int fromCam, int toCam, double& baseline) const {
  int idx = std::max(fromCam, toCam);
  Transformation T_cB_cA(camList[idx]->T_c_b_Dv->T());
  if (fromCam > toCam) T_cB_cA = T_cB_cA.inverse();
  baseline = T_cB_cA.t().norm();
  return T_cB_cA;
}

double IccCameraChain::getResultTimeShift(int camNr) const {
  return camList[camNr]->cameraTimeToImuTimeDv->toExpression().toScalar() + camList[camNr]->timeshiftCamToImuPrior;
}

void IccCameraChain::addDesignVariables(aslam::calibration::OptimizationProblem& problem, bool noTimeCalibration,
                                        bool noChainExtrinsics) {
  for (size_t c = 0; c < camList.size(); ++c) {
    bool noExtrinsics = (c == 0) ? false : noChainExtrinsics;
    size_t group = (c == 0) ? ICC_CALIBRATION_GROUP_ID : ICC_HELPER_GROUP_ID;
    camList[c]->addDesignVariables(problem, noExtrinsics, noTimeCalibration, group);
  }
}

void IccCameraChain::addCameraChainErrorTerms(aslam::calibration::OptimizationProblem& problem,
                                              aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
                                              double blakeZissermanDf, double timeOffsetPadding) {
  ab::TransformationExpression T_chain;
  for (size_t c = 0; c < camList.size(); ++c) {
    if (c == 0)
      T_chain = camList[c]->T_c_b_Dv->toExpression();
    else
      T_chain = camList[c]->T_c_b_Dv->toExpression() * T_chain;
    camList[c]->addCameraErrorTerms(problem, poseSplineDv, T_chain, blakeZissermanDf, timeOffsetPadding);
  }
}

// ===========================================================================
// IccCalibrator
// ===========================================================================
void IccCalibrator::buildProblem(const BuildOptions& o) {
  std::cout << "\tSpline order: " << o.splineOrder << "\n\tPose knots per second: " << o.poseKnotsPerSecond
            << "\n\tDo pose motion regularization: False\n\tBias knots per second: " << o.biasKnotsPerSecond
            << "\n\tDo bias motion regularization: " << (o.doBiasMotionError ? "True" : "False")
            << "\n\tBlake-Zisserman on reprojection errors " << o.blakeZisserCam
            << "\n\tAcceleration Huber width (sigma): " << o.huberAccel
            << "\n\tGyroscope Huber width (sigma): " << o.huberGyro
            << "\n\tDo time calibration: " << (!o.noTimeCalibration ? "True" : "False")
            << "\n\tTime offset padding: " << o.timeOffsetPadding << std::endl;
  noTimeCalibration = o.noTimeCalibration;
  IccImu& imu0 = *imuList[0];
  if (!o.noTimeCalibration)
    for (auto& cam : cameraChain->camList) cam->findTimeshiftCameraImuPrior(imu0, o.verbose);
  cameraChain->camList[0]->findOrientationPriorCameraToImu(imu0);
  Eigen::Vector3d estimatedGravity = cameraChain->camList[0]->gravity_w;
  auto poseSpline =
      cameraChain->camList[0]->initPoseSplineFromCamera(o.splineOrder, o.poseKnotsPerSecond, o.timeOffsetPadding);
  for (auto& imu : imuList) imu->initBiasSplines(*poseSpline, o.splineOrder, o.biasKnotsPerSecond);

  problem.reset(new aslam::calibration::OptimizationProblem());
  // design variables
  poseDv = std::make_shared<aslam::splines::BSplinePoseDesignVariable>(*poseSpline);
  for (size_t i = 0; i < poseDv->numDesignVariables(); ++i) {
    poseDv->designVariable(i)->setActive(true);
    problem->addDesignVariable(nonOwning(poseDv->designVariable(i)), ICC_HELPER_GROUP_ID);
  }
  gravityDv.reset(new ab::EuclideanDirection(estimatedGravity));
  gravityExpression = std::make_shared<ab::EuclideanExpression>(gravityDv->toExpression());
  gravityDv->setActive(true);
  problem->addDesignVariable(gravityDv, ICC_HELPER_GROUP_ID);
  for (auto& imu : imuList) imu->addDesignVariables(*problem);
  cameraChain->addDesignVariables(*problem, o.noTimeCalibration, o.noChainExtrinsics);

  // error terms
  cameraChain->addCameraChainErrorTerms(*problem, *poseDv, o.blakeZisserCam, o.timeOffsetPadding);
  for (auto& imu : imuList) {
    imu->addAccelerometerErrorTerms(*problem, *poseDv, *gravityExpression, o.huberAccel, o.accelNoiseScale);
    imu->addGyroscopeErrorTerms(*problem, *poseDv, o.huberGyro, o.gyroNoiseScale);
    if (o.doBiasMotionError) imu->addBiasMotionTerms(*problem);
  }
}

void IccCalibrator::optimize(int maxIterations, bool recoverCov, int numThreads) {
  ab::Optimizer2Options options;
  options.verbose = true;
  options.nThreads = std::max(1, numThreads);
  options.convergenceDeltaX = 1e-5;
  options.convergenceDeltaJ = 1e-2;
  options.maxIterations = maxIterations;
  options.trustRegionPolicy.reset(new ab::LevenbergMarquardtTrustRegionPolicy(10.0));
  options.linearSystemSolver.reset(new ab::BlockCholeskyLinearSystemSolver());
  bool failed = false;
  {
    ab::Optimizer2 optimizer(options);
    optimizer.setProblem(problem);
    try {
      ab::SolutionReturnValue rv = optimizer.optimize();
      if (rv.linearSolverFailure) failed = true;
    } catch (const std::exception& e) {
      KLOG_ERROR(e.what());
      failed = true;
    }
  }
  if (failed) throw std::runtime_error("Optimization failed!");
  if (recoverCov) recoverCovariance();
}

void IccCalibrator::recoverCovariance() {
  std::cout << "Recovering covariance..." << std::endl;
  // Kalibr's BSplineMotionError (bias random-walk prior) only implements buildHessian(), not the
  // evaluateJacobians() needed by the IncrementalEstimator (upstream kalibr aborts here with
  // "This is currently unsupported"). The covariance is therefore computed from a copy of the
  // problem without these prior terms, which gives slightly conservative standard deviations.
  std::unordered_set<const ab::ErrorTerm*> skip;
  for (auto& imu : imuList)
    for (auto& e : imu->biasMotionErrors) skip.insert(e.get());
  boost::shared_ptr<aslam::calibration::OptimizationProblem> covProblem(new aslam::calibration::OptimizationProblem());
  for (auto& g : problem->getDesignVariablesGroups())
    for (auto& dv : g.second) covProblem->addDesignVariable(dv, g.first);
  for (auto& e : problem->getErrorTerms())
    if (!skip.count(e.get())) covProblem->addErrorTerm(e);
  if (!skip.empty()) KLOG_WARN("covariance recovery: " << skip.size() << " bias random-walk prior terms are not used");
  // addBatch() re-optimizes the problem: keep the Optimizer2 solution untouched
  covProblem->saveDesignVariables();
  aslam::calibration::IncrementalEstimator estimator(ICC_CALIBRATION_GROUP_ID);
  try {
    estimator.addBatch(covProblem, true);
  } catch (const std::exception& e) {
    covProblem->restoreDesignVariables();
    KLOG_WARN("covariance recovery failed inside aslam IncrementalEstimator (" << e.what()
              << "); results are reported without standard deviations");
    return;
  }
  covProblem->restoreDesignVariables();
  Eigen::VectorXd est_stds = estimator.getSigma2Theta().diagonal().cwiseSqrt();
  std_trafo_ic = est_stds.head(std::min<int>(6, est_stds.size()));
  std_times = est_stds.size() > 6 ? Eigen::VectorXd(est_stds.tail(est_stds.size() - 6)) : Eigen::VectorXd();
  hasCovariance = true;
}

static void stats(const std::vector<double>& e, std::ostream& os) {
  os << "mean " << mean(e) << ", median " << median(e) << ", std: " << stddev(e);
}

void IccCalibrator::printErrorStatistics(std::ostream& os) {
  os << "Normalized Residuals\n----------------------------\n";
  for (size_t c = 0; c < cameraChain->camList.size(); ++c) {
    std::vector<double> e;
    for (auto& v : cameraChain->camList[c]->allReprojectionErrors)
      for (auto& r : v) e.push_back(std::sqrt(r->evaluateError()));
    os << "Reprojection error (cam" << c << "):     ";
    if (e.empty()) os << "no corners"; else stats(e, os);
    os << "\n";
  }
  for (size_t i = 0; i < imuList.size(); ++i) {
    std::vector<double> g, a;
    for (auto& r : imuList[i]->gyroErrors) g.push_back(std::sqrt(r->evaluateError()));
    for (auto& r : imuList[i]->accelErrors) a.push_back(std::sqrt(r->evaluateError()));
    os << "Gyroscope error (imu" << i << "):        "; stats(g, os); os << "\n";
    os << "Accelerometer error (imu" << i << "):    "; stats(a, os); os << "\n";
  }
  os << "\nResiduals\n----------------------------\n";
  for (size_t c = 0; c < cameraChain->camList.size(); ++c) {
    std::vector<double> e;
    for (auto& v : cameraChain->camList[c]->allReprojectionErrors)
      for (auto& r : v) { r->evaluateError(); e.push_back(r->vsError().norm()); }
    os << "Reprojection error (cam" << c << ") [px]:     ";
    if (e.empty()) os << "no corners"; else stats(e, os);
    os << "\n";
  }
  for (size_t i = 0; i < imuList.size(); ++i) {
    std::vector<double> g, a;
    for (auto& r : imuList[i]->gyroErrors) { r->evaluateError(); g.push_back(r->vsError().norm()); }
    for (auto& r : imuList[i]->accelErrors) { r->evaluateError(); a.push_back(r->vsError().norm()); }
    os << "Gyroscope error (imu" << i << ") [rad/s]:     "; stats(g, os); os << "\n";
    os << "Accelerometer error (imu" << i << ") [m/s^2]: "; stats(a, os); os << "\n";
  }
  os << std::endl;
}

void IccCalibrator::printResults(std::ostream& os, bool withCov) {
  for (size_t c = 0; c < cameraChain->camList.size(); ++c) {
    Transformation T = cameraChain->getResultTrafoImuToCam(static_cast<int>(c));
    os << "\nTransformation T_cam" << c << "_imu0 (imu0 to cam" << c << ", T_ci): \n";
    if (withCov && hasCovariance && c == 0) {
      os << "    quaternion: " << fmtVec(T.q()) << " +- " << fmtVec(std_trafo_ic.head(3)) << "\n";
      os << "    translation: " << fmtVec(T.t()) << " +- " << fmtVec(std_trafo_ic.tail(3)) << "\n";
    }
    os << fmtMat(T.T()) << "\n";
    if (!noTimeCalibration) {
      os << "\ncam" << c << " to imu0 time: [s] (t_imu = t_cam + shift)\n"
         << std::setprecision(10) << cameraChain->getResultTimeShift(static_cast<int>(c));
      if (withCov && hasCovariance && static_cast<int>(c) < std_times.size()) os << " +- " << std_times[c];
      os << "\n";
    }
  }
  os << "\n";
  for (size_t i = 0; i < imuList.size(); ++i) {
    os << "IMU" << i << ":\n ----------------------------\n";
    imuList[i]->config.T_i_b = Eigen::Matrix4d::Identity();
    imuList[i]->config.timeOffset = imuList[i]->timeOffset;
    imuList[i]->config.printDetails(os);
  }
  os << "\nGravity vector in target coords: [m/s^2]\n" << fmtVec(gravityExpression->toEuclidean()) << "\n";
}

void IccCalibrator::saveCamChainParametersYaml(const std::string& file) {
  CameraChain chain = cameraChain->chainConfig;
  for (size_t c = 0; c < cameraChain->camList.size(); ++c) {
    if (c > 0) {
      double b;
      chain.cams[c].hasT_cn_cnm1 = true;
      chain.cams[c].T_cn_cnm1 = cameraChain->getResultBaseline(static_cast<int>(c) - 1, static_cast<int>(c), b).T();
    }
    chain.cams[c].hasT_cam_imu = true;
    chain.cams[c].T_cam_imu = cameraChain->getResultTrafoImuToCam(static_cast<int>(c)).T();
    if (!noTimeCalibration) {
      chain.cams[c].hasTimeshift = true;
      chain.cams[c].timeshift_cam_imu = cameraChain->getResultTimeShift(static_cast<int>(c));
    }
    if (chain.cams[c].rostopic.empty()) chain.cams[c].rostopic = "/" + cameraChain->camList[c]->name + "/image_raw";
  }
  chain.writeYaml(file);
}

void IccCalibrator::saveImuSetParametersYaml(const std::string& file) {
  ImuConfig c = imuList[0]->config;
  c.T_i_b = Eigen::Matrix4d::Identity();
  c.timeOffset = imuList[0]->timeOffset;
  c.writeYaml(file);
}

void IccCalibrator::exportPoses(const std::string& file) {
  const auto& spline = poseDv->spline();
  auto& imu = *imuList[0];
  std::ofstream f(file);
  f << "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z []\n";
  f << std::setprecision(17);
  for (auto& im : imu.imuData) {
    double t = im.t + imu.timeOffset;
    if (t <= spline.t_min() || t >= spline.t_max()) continue;
    Eigen::Vector3d p = spline.position(t);
    Eigen::Vector4d q = sm::kinematics::r2quat(spline.orientation(t));  // JPL (x,y,z,w)
    f << static_cast<long long>(std::llround(im.t * 1e9)) << "," << p[0] << "," << p[1] << "," << p[2] << ","
      << q[3] << "," << q[0] << "," << q[1] << "," << q[2] << "\n";
  }
}

}  // namespace kalibr
