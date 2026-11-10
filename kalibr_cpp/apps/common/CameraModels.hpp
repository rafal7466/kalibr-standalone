// Runtime-polymorphic wrapper around the templated aslam camera geometries.
// Replaces the python "cameraModel" objects of kalibr (acvb.DistortedPinhole, ...),
// i.e. geometry + CameraDesignVariable + ReprojectionError factory.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <sm/kinematics/Transformation.hpp>

namespace kalibr {

typedef aslam::cameras::GridCalibrationTargetObservation Observation;
typedef boost::shared_ptr<Observation> ObservationPtr;

/// handle to a (templated) aslam::backend::ReprojectionError
class ReprojectionErrorHandle {
 public:
  virtual ~ReprojectionErrorHandle() {}
  virtual boost::shared_ptr<aslam::backend::ErrorTerm> term() const = 0;
  virtual Eigen::Vector2d measurement() const = 0;
  virtual Eigen::Vector2d predicted() = 0;
  int idx = -1;  // corner index in the target
};
typedef std::shared_ptr<ReprojectionErrorHandle> ReprojectionErrorHandlePtr;

class CameraModel {
 public:
  typedef std::shared_ptr<CameraModel> Ptr;
  virtual ~CameraModel() {}

  /// "pinhole-radtan", "pinhole-equi", "pinhole-fov", "omni-none", "omni-radtan",
  /// "eucm-none", "ds-none" (as in kalibr_calibrate_cameras --models)
  static Ptr create(const std::string& modelName);
  /// construct from camchain yaml parameters (kalibr_common.AslamCamera.fromParameters)
  static Ptr fromParameters(const std::string& cameraModel, const std::string& distortionModel,
                            const Eigen::VectorXd& intrinsics, const Eigen::VectorXd& distCoeffs,
                            const Eigen::Vector2i& resolution);
  static std::vector<std::string> availableModels();

  virtual std::string modelName() const = 0;        // e.g. pinhole-radtan
  virtual std::string cameraModelName() const = 0;  // e.g. pinhole
  virtual std::string distortionModelName() const = 0;  // e.g. radtan
  virtual std::string geometryTypeName() const = 0;

  virtual boost::shared_ptr<aslam::cameras::CameraGeometryBase> geometry() const = 0;

  // ------------------------------------------------ calibration support
  virtual bool initializeIntrinsics(const std::vector<Observation>& observations) = 0;
  virtual bool estimateTransformation(const Observation& obs, sm::kinematics::Transformation& T_t_c) const = 0;
  virtual void setDvActiveStatus(bool projectionActive, bool distortionActive, bool shutterActive) = 0;
  virtual boost::shared_ptr<aslam::backend::DesignVariable> distortionDesignVariable() = 0;
  virtual boost::shared_ptr<aslam::backend::DesignVariable> projectionDesignVariable() = 0;
  virtual boost::shared_ptr<aslam::backend::DesignVariable> shutterDesignVariable() = 0;
  /// ReprojectionError<geometry>(y, invR, p_c, cameraDesignVariable)  (intrinsics are estimated)
  virtual ReprojectionErrorHandlePtr createReprojectionError(const Eigen::Vector2d& y, const Eigen::Matrix2d& invR,
                                                             const aslam::backend::HomogeneousExpression& p_c) = 0;
  /// SimpleReprojectionError<frame>(y, invR, p_c, geometry)  (intrinsics fixed, camera-imu calibration)
  virtual boost::shared_ptr<aslam::backend::ErrorTerm> createSimpleReprojectionError(
      const Eigen::Vector2d& y, const Eigen::Matrix2d& invR, const aslam::backend::HomogeneousExpression& p_c) = 0;

  // ------------------------------------------------ parameters
  virtual Eigen::VectorXd projectionParameters() const = 0;  // getParameters() of the projection
  virtual Eigen::VectorXd distortionParameters() const = 0;
  /// intrinsics as written in the camchain yaml (pinhole: fu fv cu cv, omni: xi fu fv cu cv, ...)
  Eigen::VectorXd yamlIntrinsics() const { return projectionParameters(); }
  virtual Eigen::Vector2i resolution() const = 0;
  virtual int minimalDimensionsProjection() const = 0;
  virtual int minimalDimensionsDistortion() const = 0;
  virtual int minimalDimensionsShutter() const = 0;

  /// grid detector bound to this geometry
  boost::shared_ptr<aslam::cameras::GridDetector> createDetector(
      aslam::cameras::GridCalibrationTargetBase::Ptr target, bool filterCornerOutliers) const;
};

}  // namespace kalibr
