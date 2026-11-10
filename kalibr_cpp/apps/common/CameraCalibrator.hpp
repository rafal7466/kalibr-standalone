// C++ port of kalibr_camera_calibration (CameraCalibrator.py, CameraIntializers.py,
// MulticamGraph.py, ObsDb.py and the numeric parts of CameraUtils.py).
#pragma once

#include <map>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationBasic.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/OptimizationProblem.h>

#include "common/CameraModels.hpp"
#include "common/Common.hpp"
#include "common/DatasetReader.hpp"
#include "common/TargetConfig.hpp"

namespace kalibr {

enum : size_t { CALIBRATION_GROUP_ID = 0, TRANSFORMATION_GROUP_ID = 1, LANDMARK_GROUP_ID = 2 };

/// equivalent of aslam_backend.TransformationDv (python helper class)
struct TransformationDv {
  boost::shared_ptr<aslam::backend::RotationQuaternion> q;
  boost::shared_ptr<aslam::backend::EuclideanPoint> t;
  boost::shared_ptr<aslam::backend::TransformationBasic> basic;
  aslam::backend::TransformationExpression expression;

  explicit TransformationDv(const sm::kinematics::Transformation& T, bool rotationActive = true,
                            bool translationActive = true);
  aslam::backend::TransformationExpression toExpression() const { return expression; }
  size_t numDesignVariables() const { return 2; }
  boost::shared_ptr<aslam::backend::DesignVariable> getDesignVariable(size_t i) const;
  Eigen::Matrix4d T() const { return expression.toTransformationMatrix(); }
};
typedef std::shared_ptr<TransformationDv> TransformationDvPtr;

/// python: kalibr_camera_calibration.CameraGeometry
struct CameraGeometry {
  CameraModel::Ptr model;
  std::string topic;  // stream name (e.g. cam0)
  TargetConfig targetConfig;
  aslam::cameras::GridCalibrationTargetBase::Ptr target;
  bool isGeometryInitialized = false;

  CameraGeometry(CameraModel::Ptr m, const TargetConfig& tc, const std::string& topicName);
  void setDvActiveStatus(bool p, bool d, bool s) { model->setDvActiveStatus(p, d, s); }
  bool initGeometryFromObservations(const std::vector<ObservationPtr>& observations);
};
typedef std::shared_ptr<CameraGeometry> CameraGeometryPtr;

/// python: kalibr_common.extractCornersFromDataset (multithreaded)
std::vector<ObservationPtr> extractCornersFromDataset(const ImageFolderReader& dataset, const CameraModel& camera,
                                                      const TargetConfig& targetConfig, bool noTransformation,
                                                      bool filterCornerOutliers, int numThreads,
                                                      bool clearImages = true);

// ---------------------------------------------------------------------------
/// python: ObsDb.ObservationDatabase
class ObservationDatabase {
 public:
  explicit ObservationDatabase(double maxDeltaApproxSync = 0.0) : maxDelta_(maxDeltaApproxSync) {}
  void addObservation(int camId, ObservationPtr obs);

  struct Entry {
    size_t obsId;
    std::set<unsigned> observedCorners;
  };
  std::vector<std::pair<int, ObservationPtr>> getAllObsAtTimestamp(double t) const;
  int numCameras() const { return static_cast<int>(observations_.size()); }
  const std::vector<double>& getAllViewTimestamps() const { return timestamps_; }
  std::vector<int> getCamIdsAtTimestamp(double t) const;
  ObservationPtr getObservationAtTime(double t, int camId) const;  // nullptr if none
  const std::set<unsigned>& getCornerIdsAtTime(double t, int camId) const;
  size_t getObsIdForCamAtTime(double t, int camId) const;
  std::vector<std::pair<ObservationPtr, ObservationPtr>> getAllObsTwoCams(int camA, int camB) const;
  std::vector<ObservationPtr> getAllObsCam(int camId) const;
  void printTable(std::ostream& os) const;

 private:
  double maxDelta_;
  std::map<int, std::vector<ObservationPtr>> observations_;
  std::vector<double> timestamps_;                       // insertion order (python OrderedDict)
  std::map<double, std::map<int, Entry>> targetViews_;
};

// ---------------------------------------------------------------------------
/// python: MulticamGraph.MulticamCalibrationGraph (igraph replaced by small graph code)
class MulticamCalibrationGraph {
 public:
  explicit MulticamCalibrationGraph(const ObservationDatabase& db);
  bool isGraphConnected() const;
  std::vector<int> getCamOverlaps(int camId) const;
  std::vector<sm::kinematics::Transformation> getInitialGuesses(std::vector<CameraGeometryPtr>& cameras);
  sm::kinematics::Transformation getTargetPoseGuess(double timestamp, const std::vector<CameraGeometryPtr>& cameras,
                                                    const std::vector<sm::kinematics::Transformation>& baselines_HL) const;
  const ObservationDatabase& obsDb() const { return db_; }
  void print(std::ostream& os) const;

 private:
  struct Edge {
    int a, b;  // a < b
    int weight = 0;
    sm::kinematics::Transformation baseline_HL;  // T_b_a
  };
  int edgeIndex(int a, int b) const;
  std::vector<int> shortestEdgePathsFrom0() const;
  std::vector<int> pathInTree(const std::set<int>& treeEdges, int from, int to) const;

  const ObservationDatabase& db_;
  int numCams_;
  std::vector<Edge> edges_;
  std::set<int> optimalBaselineEdges_;
};

// ---------------------------------------------------------------------------
// initializers (CameraIntializers.py)
bool calibrateIntrinsics(CameraGeometry& cam, const std::vector<ObservationPtr>& obslist, bool distortionActive = true,
                         bool intrinsicsActive = true);
bool stereoCalibrate(CameraGeometry& camL, CameraGeometry& camH,
                     const std::vector<std::pair<ObservationPtr, ObservationPtr>>& obslist, bool distortionActive,
                     sm::kinematics::Transformation& baseline_HL);
bool solveFullBatch(std::vector<CameraGeometryPtr>& cameras,
                    const std::vector<sm::kinematics::Transformation>& baselineGuesses,
                    const MulticamCalibrationGraph& graph, std::vector<sm::kinematics::Transformation>& baselines);

// ---------------------------------------------------------------------------
struct CalibrationTarget {
  aslam::cameras::GridCalibrationTargetBase::Ptr target;
  std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> P_t_dv;
  std::vector<aslam::backend::HomogeneousExpression> P_t_ex;
  CalibrationTarget(aslam::cameras::GridCalibrationTargetBase::Ptr t, bool estimateLandmarks = false);
};

typedef std::vector<std::pair<int, ObservationPtr>> RigObservations;

class CalibrationTargetOptimizationProblem : public aslam::calibration::OptimizationProblem {
 public:
  typedef boost::shared_ptr<CalibrationTargetOptimizationProblem> Ptr;
  static Ptr fromTargetViewObservations(const std::vector<CameraGeometryPtr>& cameras,
                                        const std::shared_ptr<CalibrationTarget>& target,
                                        const std::vector<TransformationDvPtr>& baselines, double timestamp,
                                        const sm::kinematics::Transformation& T_tc_guess,
                                        const RigObservations& rig_observations, bool useBlakeZissermanMest);

  std::vector<CameraGeometryPtr> cameras;
  std::shared_ptr<CalibrationTarget> target;
  std::vector<TransformationDvPtr> baselines;
  double timestamp = 0.0;
  sm::kinematics::Transformation T_tc_guess;
  RigObservations rig_observations;
  TransformationDvPtr dv_T_target_camera;
  std::map<int, std::vector<ReprojectionErrorHandlePtr>> rerrs;  // nullptr for unobserved corners
};

/// python: CameraCalibrator.CameraCalibration
class CameraCalibration {
 public:
  CameraCalibration(const std::vector<CameraGeometryPtr>& cameras,
                    const std::vector<sm::kinematics::Transformation>& baselineGuesses, bool estimateLandmarks,
                    bool verbose, bool useBlakeZissermanMest);
  bool addTargetView(double timestamp, const RigObservations& rig_observations,
                     const sm::kinematics::Transformation& T_tc_guess, bool force = false);

  std::vector<CameraGeometryPtr> cameras;
  bool useBlakeZissermanMest;
  std::unique_ptr<aslam::calibration::IncrementalEstimator> estimator;
  std::shared_ptr<CalibrationTarget> target;
  std::vector<TransformationDvPtr> baselines;
  std::vector<CalibrationTargetOptimizationProblem::Ptr> views;
  aslam::calibration::IncrementalEstimator::ReturnValue lastReturnValue;
};

CalibrationTargetOptimizationProblem::Ptr removeCornersFromBatch(
    const CalibrationTargetOptimizationProblem::Ptr& batch,
    const std::vector<std::pair<int, std::vector<int>>>& camId_cornerIdList, bool useBlakeZissermanMest);

// ---------------------------------------------------------------------------
// CameraUtils.py (numeric parts)
/// per view: camInView=false if camera not in view, else per corner (valid, error = measured - predicted)
struct ViewReprojectionErrors {
  bool camInView = false;
  std::vector<bool> valid;
  std::vector<Eigen::Vector2d> corners, reprojections, errors;
};
std::vector<ViewReprojectionErrors> getReprojectionErrors(CameraCalibration& calib, int camId);
void getReprojectionErrorStatistics(const std::vector<ViewReprojectionErrors>& all, Eigen::Vector2d& mean,
                                    Eigen::Vector2d& std);
void recoverCovariance(CameraCalibration& calib, std::vector<Eigen::VectorXd>& std_baselines,
                       std::vector<Eigen::VectorXd>& std_cameras);
void printParameters(CameraCalibration& calib, std::ostream& os);

}  // namespace kalibr
