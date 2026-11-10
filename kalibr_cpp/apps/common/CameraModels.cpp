#include "common/CameraModels.hpp"

#include <stdexcept>

#include <aslam/Frame.hpp>
#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
#include <aslam/cameras.hpp>

namespace kalibr {

using namespace aslam::cameras;

// ---------------------------------------------------------------------------
// projection / distortion construction from yaml parameters
// ---------------------------------------------------------------------------
template <class D>
static D makeDistortion(const Eigen::VectorXd& coeffs) {
  D d;
  if (coeffs.size() > 0) {
    Eigen::MatrixXd P = coeffs;
    d.setParameters(P);
  }
  return d;
}

template <class P>
struct ProjectionMaker;

template <class D>
struct ProjectionMaker<PinholeProjection<D>> {
  static constexpr int N = 4;
  static PinholeProjection<D> make(const Eigen::VectorXd& k, int ru, int rv, const D& d) {
    return PinholeProjection<D>(k[0], k[1], k[2], k[3], ru, rv, d);
  }
};
template <class D>
struct ProjectionMaker<OmniProjection<D>> {
  static constexpr int N = 5;
  static OmniProjection<D> make(const Eigen::VectorXd& k, int ru, int rv, const D& d) {
    return OmniProjection<D>(k[0], k[1], k[2], k[3], k[4], ru, rv, d);
  }
};
template <class D>
struct ProjectionMaker<ExtendedUnifiedProjection<D>> {
  static constexpr int N = 6;
  static ExtendedUnifiedProjection<D> make(const Eigen::VectorXd& k, int ru, int rv, const D& d) {
    return ExtendedUnifiedProjection<D>(k[0], k[1], k[2], k[3], k[4], k[5], ru, rv, d);
  }
};
template <class D>
struct ProjectionMaker<DoubleSphereProjection<D>> {
  static constexpr int N = 6;
  static DoubleSphereProjection<D> make(const Eigen::VectorXd& k, int ru, int rv, const D& d) {
    return DoubleSphereProjection<D>(k[0], k[1], k[2], k[3], k[4], k[5], ru, rv, d);
  }
};

// ---------------------------------------------------------------------------
template <class C>
class ReprojectionErrorHandleT : public ReprojectionErrorHandle {
 public:
  typedef aslam::backend::ReprojectionError<C> rerr_t;
  explicit ReprojectionErrorHandleT(boost::shared_ptr<rerr_t> e) : e_(e) {}
  boost::shared_ptr<aslam::backend::ErrorTerm> term() const override { return e_; }
  Eigen::Vector2d measurement() const override { return e_->getMeasurement(); }
  Eigen::Vector2d predicted() override { return e_->getPredictedMeasurement(); }
 private:
  boost::shared_ptr<rerr_t> e_;
};

template <class C>
class CameraModelT : public CameraModel {
 public:
  typedef C geometry_t;
  typedef typename C::projection_t projection_t;
  typedef typename projection_t::distortion_t distortion_t;
  typedef aslam::backend::CameraDesignVariable<C> dv_t;

  CameraModelT(const std::string& name, const std::string& cam, const std::string& dist,
               const std::string& typeName, boost::shared_ptr<C> geometry)
      : name_(name), cam_(cam), dist_(dist), typeName_(typeName), geometry_(geometry) {
    dv_.reset(new dv_t(geometry_));
    setDvActiveStatus(true, true, false);
  }

  std::string modelName() const override { return name_; }
  std::string cameraModelName() const override { return cam_; }
  std::string distortionModelName() const override { return dist_; }
  std::string geometryTypeName() const override { return typeName_; }
  boost::shared_ptr<CameraGeometryBase> geometry() const override { return geometry_; }

  bool initializeIntrinsics(const std::vector<Observation>& observations) override {
    return geometry_->initializeIntrinsics(observations);
  }
  bool estimateTransformation(const Observation& obs, sm::kinematics::Transformation& T) const override {
    return geometry_->estimateTransformation(obs, T);
  }
  void setDvActiveStatus(bool p, bool d, bool s) override {
    dv_->projectionDesignVariable()->setActive(p);
    dv_->distortionDesignVariable()->setActive(d);
    dv_->shutterDesignVariable()->setActive(s);
  }
  boost::shared_ptr<aslam::backend::DesignVariable> distortionDesignVariable() override {
    return dv_->distortionDesignVariable();
  }
  boost::shared_ptr<aslam::backend::DesignVariable> projectionDesignVariable() override {
    return dv_->projectionDesignVariable();
  }
  boost::shared_ptr<aslam::backend::DesignVariable> shutterDesignVariable() override {
    return dv_->shutterDesignVariable();
  }

  ReprojectionErrorHandlePtr createReprojectionError(const Eigen::Vector2d& y, const Eigen::Matrix2d& invR,
                                                     const aslam::backend::HomogeneousExpression& p) override {
    boost::shared_ptr<aslam::backend::ReprojectionError<C>> e(
        new aslam::backend::ReprojectionError<C>(y, invR, p, *dv_));
    return std::make_shared<ReprojectionErrorHandleT<C>>(e);
  }

  boost::shared_ptr<aslam::backend::ErrorTerm> createSimpleReprojectionError(
      const Eigen::Vector2d& y, const Eigen::Matrix2d& invR, const aslam::backend::HomogeneousExpression& p) override {
    typedef aslam::Frame<C> frame_t;
    return boost::shared_ptr<aslam::backend::ErrorTerm>(
        new aslam::backend::SimpleReprojectionError<frame_t>(y, invR, p, *geometry_));
  }

  Eigen::VectorXd projectionParameters() const override {
    Eigen::MatrixXd P;
    geometry_->projection().getParameters(P);
    return Eigen::Map<Eigen::VectorXd>(P.data(), P.size());
  }
  Eigen::VectorXd distortionParameters() const override {
    Eigen::MatrixXd P;
    geometry_->projection().distortion().getParameters(P);
    return Eigen::Map<Eigen::VectorXd>(P.data(), P.size());
  }
  Eigen::Vector2i resolution() const override {
    return Eigen::Vector2i(geometry_->projection().ru(), geometry_->projection().rv());
  }
  int minimalDimensionsProjection() const override { return geometry_->minimalDimensionsProjection(); }
  int minimalDimensionsDistortion() const override { return geometry_->minimalDimensionsDistortion(); }
  int minimalDimensionsShutter() const override { return geometry_->minimalDimensionsShutter(); }

  static Ptr createDefault(const std::string& name, const std::string& cam, const std::string& dist,
                           const std::string& typeName) {
    return Ptr(new CameraModelT<C>(name, cam, dist, typeName, boost::shared_ptr<C>(new C())));
  }
  static Ptr createFromParams(const std::string& name, const std::string& cam, const std::string& dist,
                              const std::string& typeName, const Eigen::VectorXd& intr,
                              const Eigen::VectorXd& distCoeffs, const Eigen::Vector2i& res) {
    if (intr.size() != ProjectionMaker<projection_t>::N)
      throw std::runtime_error("camera model " + cam + " needs " +
                               std::to_string(ProjectionMaker<projection_t>::N) + " intrinsics");
    distortion_t d = makeDistortion<distortion_t>(distCoeffs);
    projection_t p = ProjectionMaker<projection_t>::make(intr, res[0], res[1], d);
    return Ptr(new CameraModelT<C>(name, cam, dist, typeName, boost::shared_ptr<C>(new C(p))));
  }

 private:
  std::string name_, cam_, dist_, typeName_;
  boost::shared_ptr<C> geometry_;
  boost::shared_ptr<dv_t> dv_;
};

// ---------------------------------------------------------------------------
std::vector<std::string> CameraModel::availableModels() {
  return {"pinhole-radtan", "pinhole-equi", "pinhole-fov", "pinhole-none",
          "omni-none",      "omni-radtan",  "eucm-none",   "ds-none"};
}

#define KALIBR_MODEL_TABLE(X)                                                                     \
  X("pinhole-radtan", "pinhole", "radtan", DistortedPinholeCameraGeometry)                        \
  X("pinhole-equi", "pinhole", "equidistant", EquidistantDistortedPinholeCameraGeometry)          \
  X("pinhole-fov", "pinhole", "fov", FovDistortedPinholeCameraGeometry)                           \
  X("pinhole-none", "pinhole", "none", PinholeCameraGeometry)                                     \
  X("omni-none", "omni", "none", OmniCameraGeometry)                                              \
  X("omni-radtan", "omni", "radtan", DistortedOmniCameraGeometry)                                 \
  X("eucm-none", "eucm", "none", ExtendedUnifiedCameraGeometry)                                   \
  X("ds-none", "ds", "none", DoubleSphereCameraGeometry)

CameraModel::Ptr CameraModel::create(const std::string& modelName) {
#define X(NAME, CAM, DIST, TYPE) \
  if (modelName == NAME) return CameraModelT<TYPE>::createDefault(NAME, CAM, DIST, #TYPE);
  KALIBR_MODEL_TABLE(X)
#undef X
  std::string all;
  for (auto& m : availableModels()) all += m + " ";
  throw std::runtime_error("Unknown camera model: " + modelName + ". Try one of: " + all);
}

CameraModel::Ptr CameraModel::fromParameters(const std::string& cameraModel, const std::string& distModelIn,
                                             const Eigen::VectorXd& intr, const Eigen::VectorXd& distCoeffs,
                                             const Eigen::Vector2i& res) {
  std::string distModel = distModelIn;
  if (distModel == "radial-tangential") distModel = "radtan";  // EuRoC naming
  if (distModel == "equi") distModel = "equidistant";
  std::string name = cameraModel + "-" + (distModel == "equidistant" ? std::string("equi") : distModel);
#define X(NAME, CAM, DIST, TYPE)                                                                  \
  if (cameraModel == CAM && distModel == DIST)                                                    \
    return CameraModelT<TYPE>::createFromParams(NAME, CAM, DIST, #TYPE, intr, distCoeffs, res);
  KALIBR_MODEL_TABLE(X)
#undef X
  throw std::runtime_error("Unsupported camera/distortion model combination: " + cameraModel + "/" + distModelIn +
                           " (" + name + ")");
}

boost::shared_ptr<GridDetector> CameraModel::createDetector(GridCalibrationTargetBase::Ptr target,
                                                            bool filterCornerOutliers) const {
  GridDetector::GridDetectorOptions o;
  o.imageStepping = false;
  o.plotCornerReprojection = false;
  o.filterCornerOutliers = filterCornerOutliers;
  return boost::shared_ptr<GridDetector>(new GridDetector(geometry(), target, o));
}

}  // namespace kalibr
