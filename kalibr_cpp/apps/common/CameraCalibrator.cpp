#include "common/CameraCalibrator.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>

#include <aslam/backend/BlockCholeskyLinearSystemSolver.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/kinematics/transformations.hpp>

namespace kalibr {

using sm::kinematics::Transformation;
namespace ab = aslam::backend;

// ===========================================================================
// TransformationDv
// ===========================================================================
TransformationDv::TransformationDv(const Transformation& T, bool rotationActive, bool translationActive) {
  q.reset(new ab::RotationQuaternion(T.q()));
  q->setActive(rotationActive);
  t.reset(new ab::EuclideanPoint(T.t()));
  t->setActive(translationActive);
  basic.reset(new ab::TransformationBasic(q->toExpression(), t->toExpression()));
  expression = basic->toExpression();
}

boost::shared_ptr<ab::DesignVariable> TransformationDv::getDesignVariable(size_t i) const {
  if (i == 0) return q;
  if (i == 1) return t;
  throw std::out_of_range("TransformationDv: index out of bounds");
}

// ===========================================================================
// CameraGeometry
// ===========================================================================
CameraGeometry::CameraGeometry(CameraModel::Ptr m, const TargetConfig& tc, const std::string& topicName)
    : model(m), topic(topicName), targetConfig(tc) {
  target = tc.createTarget(false);
  model->setDvActiveStatus(true, true, false);
}

bool CameraGeometry::initGeometryFromObservations(const std::vector<ObservationPtr>& observations) {
  std::vector<Observation> obs;
  obs.reserve(observations.size());
  for (auto& o : observations) obs.push_back(*o);
  bool success = model->initializeIntrinsics(obs);
  if (!success) KLOG_ERROR("initialization of focal length for cam with topic " << topic << " failed");
  // in case of an omni model, first optimize over intrinsics only
  if (model->modelName() == "omni-radtan") {
    success = calibrateIntrinsics(*this, observations, /*distortionActive=*/false);
    if (!success) KLOG_ERROR("initialization of intrinsics for cam with topic " << topic << " failed");
  }
  success = calibrateIntrinsics(*this, observations);
  if (!success) KLOG_ERROR("initialization of intrinsics for cam with topic " << topic << " failed");
  isGeometryInitialized = success;
  return success;
}

// ===========================================================================
// corner extraction
// ===========================================================================
std::vector<ObservationPtr> extractCornersFromDataset(const ImageFolderReader& dataset, const CameraModel& camera,
                                                      const TargetConfig& targetConfig, bool noTransformation,
                                                      bool filterCornerOutliers, int numThreads, bool clearImages) {
  std::cout << "Extracting calibration target corners" << std::endl;
  const size_t N = dataset.numImages();
  std::vector<ObservationPtr> results(N);
  std::atomic<size_t> next(0), done(0);
  std::mutex progressMutex;
  Progress progress(N);
  numThreads = std::max(1, numThreads);

  auto worker = [&]() {
    // every thread has its own target + detector (AprilTags detector is not thread-safe)
    auto target = targetConfig.createTarget(false);
    auto detector = camera.createDetector(target, filterCornerOutliers);
    while (true) {
      size_t i = next.fetch_add(1);
      if (i >= N) break;
      cv::Mat img;
      try {
        img = dataset.readImage(i);
      } catch (const std::exception& e) {
        KLOG_WARN(e.what());
        ++done;
        continue;
      }
      ObservationPtr obs(new Observation(target));
      bool success = false;
      try {
        if (noTransformation)
          success = detector->findTargetNoTransformation(img, dataset.entries()[i].time(), *obs);
        else
          success = detector->findTarget(img, dataset.entries()[i].time(), *obs);
      } catch (const std::exception& e) {
        KLOG_DEBUG("target extraction failed on image " << i << ": " << e.what());
        success = false;
      }
      if (clearImages) obs->clearImage();
      if (success) results[i] = obs;
      ++done;
    }
  };
  // progress printing thread
  std::atomic<bool> finished(false);
  std::thread printer([&]() {
    size_t last = 0;
    while (!finished) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      size_t d = done.load();
      if (d > last) {
        std::lock_guard<std::mutex> lock(progressMutex);
        progress.sample(d - last);
        last = d;
      }
    }
    size_t d = done.load();
    if (d > last) progress.sample(d - last);
  });
  std::vector<std::thread> threads;
  for (int t = 0; t < numThreads; ++t) threads.emplace_back(worker);
  for (auto& t : threads) t.join();
  finished = true;
  printer.join();

  std::vector<ObservationPtr> out;
  for (auto& r : results)
    if (r) out.push_back(r);
  std::cout << "  Extracted corners for " << out.size() << " images (of " << N << " images)" << std::endl;
  return out;
}

// ===========================================================================
// ObservationDatabase
// ===========================================================================
void ObservationDatabase::addObservation(int camId, ObservationPtr obs) {
  auto& list = observations_[camId];
  list.push_back(obs);
  size_t obsIdx = list.size() - 1;
  double tObs = obs->time().toSec();

  double nearest;
  if (timestamps_.empty()) {
    nearest = tObs + 5 * (maxDelta_ + 1);
  } else {
    nearest = timestamps_[0];
    double best = std::fabs(nearest - tObs);
    for (double t : timestamps_) {
      double d = std::fabs(t - tObs);
      if (d < best) {
        best = d;
        nearest = t;
      }
    }
  }
  double ts;
  if (std::fabs(nearest - tObs) <= maxDelta_) {
    ts = nearest;
  } else {
    ts = tObs;
    timestamps_.push_back(ts);
    targetViews_[ts];
  }
  auto& camMap = targetViews_[ts];
  if (camMap.count(camId) == 0) {
    Entry e;
    e.obsId = obsIdx;
    std::vector<unsigned> idx;
    obs->getCornersIdx(idx);
    e.observedCorners.insert(idx.begin(), idx.end());
    camMap[camId] = e;
  } else {
    KLOG_ERROR("[TargetViewTable]: Tried to add second view to a given cameraId & timestamp. "
               "Maybe try to reduce the approximate syncing tolerance..");
  }
}

std::vector<std::pair<int, ObservationPtr>> ObservationDatabase::getAllObsAtTimestamp(double t) const {
  std::vector<std::pair<int, ObservationPtr>> out;
  for (auto& kv : targetViews_.at(t)) out.emplace_back(kv.first, observations_.at(kv.first)[kv.second.obsId]);
  return out;
}

std::vector<int> ObservationDatabase::getCamIdsAtTimestamp(double t) const {
  std::vector<int> ids;
  for (auto& kv : targetViews_.at(t)) ids.push_back(kv.first);
  return ids;
}

ObservationPtr ObservationDatabase::getObservationAtTime(double t, int camId) const {
  auto it = targetViews_.find(t);
  if (it == targetViews_.end()) return ObservationPtr();
  auto jt = it->second.find(camId);
  if (jt == it->second.end()) return ObservationPtr();
  return observations_.at(camId)[jt->second.obsId];
}

const std::set<unsigned>& ObservationDatabase::getCornerIdsAtTime(double t, int camId) const {
  return targetViews_.at(t).at(camId).observedCorners;
}

size_t ObservationDatabase::getObsIdForCamAtTime(double t, int camId) const {
  return targetViews_.at(t).at(camId).obsId;
}

std::vector<std::pair<ObservationPtr, ObservationPtr>> ObservationDatabase::getAllObsTwoCams(int a, int b) const {
  std::vector<std::pair<ObservationPtr, ObservationPtr>> out;
  for (double t : timestamps_) {
    ObservationPtr oa = getObservationAtTime(t, a), ob = getObservationAtTime(t, b);
    if (oa || ob) out.emplace_back(oa, ob);
  }
  return out;
}

std::vector<ObservationPtr> ObservationDatabase::getAllObsCam(int camId) const {
  std::vector<ObservationPtr> out;
  for (double t : timestamps_) {
    ObservationPtr o = getObservationAtTime(t, camId);
    if (o) out.push_back(o);
  }
  return out;
}

void ObservationDatabase::printTable(std::ostream& os) const {
  os << "timestamp \t";
  for (int c = 0; c < numCameras(); ++c) os << "cam" << c << " \t";
  os << "\n";
  for (auto& kv : targetViews_) {
    os << std::setprecision(15) << kv.first;
    for (int c = 0; c < numCameras(); ++c) {
      auto it = kv.second.find(c);
      os << "\t " << (it == kv.second.end() ? std::string("-") : std::to_string(it->second.observedCorners.size()));
    }
    os << "\n";
  }
}

// ===========================================================================
// MulticamCalibrationGraph
// ===========================================================================
MulticamCalibrationGraph::MulticamCalibrationGraph(const ObservationDatabase& db)
    : db_(db), numCams_(db.numCameras()) {
  for (double t : db_.getAllViewTimestamps()) {
    std::vector<int> ids = db_.getCamIdsAtTimestamp(t);
    std::sort(ids.begin(), ids.end());
    for (size_t i = 0; i < ids.size(); ++i) {
      for (size_t j = i + 1; j < ids.size(); ++j) {
        const auto& cA = db_.getCornerIdsAtTime(t, ids[i]);
        const auto& cB = db_.getCornerIdsAtTime(t, ids[j]);
        int common = 0;
        for (unsigned c : cA)
          if (cB.count(c)) ++common;
        if (common > 0) {
          int e = edgeIndex(ids[i], ids[j]);
          if (e < 0) {
            Edge ed;
            ed.a = ids[i];
            ed.b = ids[j];
            edges_.push_back(ed);
            e = static_cast<int>(edges_.size()) - 1;
          }
          edges_[e].weight += common;
        }
      }
    }
  }
}

int MulticamCalibrationGraph::edgeIndex(int a, int b) const {
  if (a > b) std::swap(a, b);
  for (size_t i = 0; i < edges_.size(); ++i)
    if (edges_[i].a == a && edges_[i].b == b) return static_cast<int>(i);
  return -1;
}

bool MulticamCalibrationGraph::isGraphConnected() const {
  if (numCams_ == 1) return true;
  std::vector<bool> seen(numCams_, false);
  std::queue<int> q;
  q.push(0);
  seen[0] = true;
  while (!q.empty()) {
    int v = q.front();
    q.pop();
    for (auto& e : edges_) {
      int w = (e.a == v) ? e.b : (e.b == v ? e.a : -1);
      if (w >= 0 && !seen[w]) {
        seen[w] = true;
        q.push(w);
      }
    }
  }
  return std::all_of(seen.begin(), seen.end(), [](bool s) { return s; });
}

std::vector<int> MulticamCalibrationGraph::getCamOverlaps(int camId) const {
  std::vector<int> o;
  for (auto& e : edges_) {
    if (e.a == camId) o.push_back(e.b);
    if (e.b == camId) o.push_back(e.a);
  }
  std::sort(o.begin(), o.end());
  return o;
}

std::vector<int> MulticamCalibrationGraph::shortestEdgePathsFrom0() const {
  // Dijkstra with weights 1/commonPoints, returns the union of the edges of all shortest paths from cam0
  std::vector<double> dist(numCams_, std::numeric_limits<double>::infinity());
  std::vector<int> predEdge(numCams_, -1);
  std::vector<bool> doneV(numCams_, false);
  dist[0] = 0.0;
  for (int it = 0; it < numCams_; ++it) {
    int u = -1;
    for (int v = 0; v < numCams_; ++v)
      if (!doneV[v] && (u < 0 || dist[v] < dist[u])) u = v;
    if (u < 0 || std::isinf(dist[u])) break;
    doneV[u] = true;
    for (size_t e = 0; e < edges_.size(); ++e) {
      int w = (edges_[e].a == u) ? edges_[e].b : (edges_[e].b == u ? edges_[e].a : -1);
      if (w < 0) continue;
      double nd = dist[u] + 1.0 / edges_[e].weight;
      if (nd < dist[w]) {
        dist[w] = nd;
        predEdge[w] = static_cast<int>(e);
      }
    }
  }
  std::set<int> s;
  for (int v = 1; v < numCams_; ++v) {
    int cur = v;
    while (cur != 0 && predEdge[cur] >= 0) {
      int e = predEdge[cur];
      s.insert(e);
      cur = (edges_[e].a == cur) ? edges_[e].b : edges_[e].a;
    }
  }
  return std::vector<int>(s.begin(), s.end());
}

std::vector<int> MulticamCalibrationGraph::pathInTree(const std::set<int>& treeEdges, int from, int to) const {
  // BFS over the tree edges, returns vertex path
  std::vector<int> prev(numCams_, -2);
  std::queue<int> q;
  q.push(from);
  prev[from] = -1;
  while (!q.empty()) {
    int v = q.front();
    q.pop();
    if (v == to) break;
    for (int e : treeEdges) {
      int w = (edges_[e].a == v) ? edges_[e].b : (edges_[e].b == v ? edges_[e].a : -1);
      if (w >= 0 && prev[w] == -2) {
        prev[w] = v;
        q.push(w);
      }
    }
  }
  std::vector<int> path;
  if (prev[to] == -2) return path;
  for (int v = to; v != -1; v = prev[v]) path.push_back(v);
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<Transformation> MulticamCalibrationGraph::getInitialGuesses(std::vector<CameraGeometryPtr>& cameras) {
  if (!isGraphConnected())
    throw std::runtime_error("The cameras are not connected through mutual target observations! "
                             "Please provide another dataset...");
  std::vector<int> optimal = shortestEdgePathsFrom0();
  optimalBaselineEdges_ = std::set<int>(optimal.begin(), optimal.end());

  for (int eid : optimalBaselineEdges_) {
    int camL = edges_[eid].a, camH = edges_[eid].b;
    std::cout << "\t initializing camera pair (" << camL << "," << camH << ")...  " << std::endl;
    auto obsList = db_.getAllObsTwoCams(camL, camH);
    Transformation baseline_HL;
    bool success = stereoCalibrate(*cameras[camL], *cameras[camH], obsList, /*distortionActive=*/false, baseline_HL);
    if (success) {
      KLOG_DEBUG("baseline_" << camL << "_" << camH << "=\n" << baseline_HL.T());
    } else {
      KLOG_ERROR("initialization of camera pair (" << camL << "," << camH << ") failed");
      KLOG_ERROR("estimated baseline_" << camL << "_" << camH << "=\n" << baseline_HL.T());
    }
    edges_[eid].baseline_HL = baseline_HL;
  }

  std::vector<Transformation> baselines;
  for (int b = 0; b < numCams_ - 1; ++b) {
    std::vector<int> path = pathInTree(optimalBaselineEdges_, b, b + 1);
    Transformation baseline_HL;
    for (size_t p = 0; p + 1 < path.size(); ++p) {
      int s = path[p], t = path[p + 1];
      Transformation T_edge = edges_[edgeIndex(s, t)].baseline_HL;
      if (s > t) T_edge = T_edge.inverse();
      baseline_HL = T_edge * baseline_HL;
    }
    baselines.push_back(baseline_HL);
  }

  std::vector<Transformation> refined;
  bool success = solveFullBatch(cameras, baselines, *this, refined);
  if (!success) KLOG_WARN("Full batch refinement failed!");
  return refined;
}

Transformation MulticamCalibrationGraph::getTargetPoseGuess(double timestamp,
                                                            const std::vector<CameraGeometryPtr>& cameras,
                                                            const std::vector<Transformation>& baselines_HL) const {
  int camMax = -1;
  size_t maxCorners = 0;
  for (int camId : db_.getCamIdsAtTimestamp(timestamp)) {
    size_t n = db_.getCornerIdsAtTime(timestamp, camId).size();
    if (camMax < 0 || n > maxCorners) {
      camMax = camId;
      maxCorners = n;
    }
  }
  Transformation T_t_cN;
  bool success = cameras[camMax]->model->estimateTransformation(*db_.getObservationAtTime(timestamp, camMax), T_t_cN);
  if (!success) KLOG_WARN("getTargetPoseGuess: solvePnP failed with solution: \n" << T_t_cN.T());
  Transformation T_cN_c0;
  for (int i = 0; i < camMax && i < static_cast<int>(baselines_HL.size()); ++i) T_cN_c0 = baselines_HL[i] * T_cN_c0;
  return T_t_cN * T_cN_c0;
}

void MulticamCalibrationGraph::print(std::ostream& os) const {
  for (auto& e : edges_) os << "  cam" << e.a << " <-> cam" << e.b << " : " << e.weight << " common corners\n";
}

// ===========================================================================
// Initializers
// ===========================================================================
static ab::Optimizer2Options initializerOptions(int maxIterations) {
  ab::Optimizer2Options options;
  options.verbose = (logLevel() == LogLevel::Debug);
  options.nThreads = 4;
  options.convergenceDeltaX = 1e-3;
  options.convergenceDeltaJ = 1;
  options.maxIterations = maxIterations;
  options.trustRegionPolicy.reset(new ab::LevenbergMarquardtTrustRegionPolicy(10));
  return options;
}

static TransformationDvPtr addPoseDesignVariable(ab::OptimizationProblem& problem,
                                                 const Transformation& T0 = Transformation()) {
  TransformationDvPtr dv = std::make_shared<TransformationDv>(T0, true, true);
  problem.addDesignVariable(dv->q);
  problem.addDesignVariable(dv->t);
  return dv;
}

static void addCameraDvs(ab::OptimizationProblem& problem, CameraGeometry& cam) {
  problem.addDesignVariable(cam.model->distortionDesignVariable());
  problem.addDesignVariable(cam.model->projectionDesignVariable());
  problem.addDesignVariable(cam.model->shutterDesignVariable());
}

static bool runOptimizer(const boost::shared_ptr<ab::OptimizationProblem>& problem, int maxIterations,
                         const char* who) {
  ab::Optimizer2 optimizer(initializerOptions(maxIterations));
  optimizer.setProblem(problem);
  try {
    ab::SolutionReturnValue rv = optimizer.optimize();
    if (rv.linearSolverFailure) KLOG_ERROR(who << ": Optimization failed!");
    return !rv.linearSolverFailure;
  } catch (const std::exception& e) {
    KLOG_ERROR(who << ": Optimization failed! (" << e.what() << ")");
    return false;
  }
}

bool calibrateIntrinsics(CameraGeometry& cam, const std::vector<ObservationPtr>& obslist, bool distortionActive,
                         bool intrinsicsActive) {
  KLOG_DEBUG("calibrateIntrinsics: intrinsics guess: " << fmtVec(cam.model->projectionParameters()));
  KLOG_DEBUG("calibrateIntrinsics: distortion guess: " << fmtVec(cam.model->distortionParameters()));
  boost::shared_ptr<ab::OptimizationProblem> problem(new ab::OptimizationProblem());
  cam.setDvActiveStatus(intrinsicsActive, distortionActive, false);
  addCameraDvs(*problem, cam);

  const Eigen::Matrix2d invR = Eigen::Matrix2d::Identity();  // cornerUncertainty = 1
  auto target = cam.target;
  std::vector<TransformationDvPtr> poseDvs;
  std::vector<ReprojectionErrorHandlePtr> rerrs;
  for (auto& obs : obslist) {
    Transformation T_t_c;
    cam.model->estimateTransformation(*obs, T_t_c);
    TransformationDvPtr poseDv = addPoseDesignVariable(*problem, T_t_c);
    poseDvs.push_back(poseDv);
    ab::TransformationExpression T_cam_w = poseDv->toExpression().inverse();
    for (size_t i = 0; i < target->size(); ++i) {
      ab::HomogeneousExpression p_target(sm::kinematics::toHomogeneous(target->point(i)));
      Eigen::Vector2d y;
      if (obs->imagePoint(i, y)) {
        auto rerr = cam.model->createReprojectionError(y, invR, T_cam_w * p_target);
        problem->addErrorTerm(rerr->term());
        rerrs.push_back(rerr);
      }
    }
  }
  KLOG_DEBUG("calibrateIntrinsics: added " << rerrs.size() << " camera error terms");
  bool success = runOptimizer(problem, 200, "calibrateIntrinsics");
  KLOG_DEBUG("calibrateIntrinsics: guess for intrinsics cam: " << fmtVec(cam.model->projectionParameters()));
  KLOG_DEBUG("calibrateIntrinsics: guess for distortion cam: " << fmtVec(cam.model->distortionParameters()));
  return success;
}

bool stereoCalibrate(CameraGeometry& camL, CameraGeometry& camH,
                     const std::vector<std::pair<ObservationPtr, ObservationPtr>>& obslist, bool distortionActive,
                     Transformation& baseline_HL) {
  // median of the pairwise baselines as initial guess
  std::vector<std::vector<double>> r(3), t(3);
  sm::kinematics::RotationVector rv;
  for (auto& p : obslist) {
    if (p.first && p.second) {
      Transformation T_L, T_H;
      camL.model->estimateTransformation(*p.first, T_L);
      camH.model->estimateTransformation(*p.second, T_H);
      Transformation baseline = T_H.inverse() * T_L;
      Eigen::Vector3d rr = rv.rotationMatrixToParameters(baseline.C());
      for (int k = 0; k < 3; ++k) {
        r[k].push_back(rr[k]);
        t[k].push_back(baseline.t()[k]);
      }
    }
  }
  if (r[0].empty()) {
    KLOG_ERROR("stereoCalibrate: no mutual observations");
    return false;
  }
  Eigen::Vector3d r_med(median(r[0]), median(r[1]), median(r[2]));
  Eigen::Vector3d t_med(median(t[0]), median(t[1]), median(t[2]));
  Eigen::Matrix4d Tm = Eigen::Matrix4d::Identity();
  Tm.topLeftCorner<3, 3>() = rv.parametersToRotationMatrix(r_med);
  Tm.topRightCorner<3, 1>() = t_med;
  baseline_HL = Transformation(Tm);
  KLOG_DEBUG("initial guess for stereo calib: \n" << baseline_HL.T());

  boost::shared_ptr<ab::OptimizationProblem> problem(new ab::OptimizationProblem());
  TransformationDvPtr baselineDv = addPoseDesignVariable(*problem, baseline_HL);
  std::vector<TransformationDvPtr> targetPoseDvs;
  for (auto& p : obslist) {
    Transformation T_t_cL;
    if (p.first) {
      camL.model->estimateTransformation(*p.first, T_t_cL);
    } else {
      Transformation T_t_cH;
      camH.model->estimateTransformation(*p.second, T_t_cH);
      T_t_cL = T_t_cH * baseline_HL;
    }
    targetPoseDvs.push_back(addPoseDesignVariable(*problem, T_t_cL));
  }
  camL.setDvActiveStatus(true, distortionActive, false);
  camH.setDvActiveStatus(true, distortionActive, false);
  addCameraDvs(*problem, camL);
  addCameraDvs(*problem, camH);

  const Eigen::Matrix2d invR = Eigen::Matrix2d::Identity();
  std::vector<ReprojectionErrorHandlePtr> rerrs;
  CameraGeometry* cams[2] = {&camL, &camH};
  for (int cidx = 0; cidx < 2; ++cidx) {
    CameraGeometry& cam = *cams[cidx];
    auto target = cam.target;
    for (size_t v = 0; v < obslist.size(); ++v) {
      ObservationPtr obs = cidx == 0 ? obslist[v].first : obslist[v].second;
      if (!obs) continue;
      ab::TransformationExpression T_cam_w = targetPoseDvs[v]->toExpression().inverse();
      if (cidx != 0) T_cam_w = baselineDv->toExpression() * T_cam_w;
      for (size_t i = 0; i < target->size(); ++i) {
        ab::HomogeneousExpression p_target(sm::kinematics::toHomogeneous(target->point(i)));
        Eigen::Vector2d y;
        if (obs->imagePoint(i, y)) {
          auto rerr = cam.model->createReprojectionError(y, invR, T_cam_w * p_target);
          problem->addErrorTerm(rerr->term());
          rerrs.push_back(rerr);
        }
      }
    }
  }
  KLOG_DEBUG("stereoCalibrate: added " << rerrs.size() << " camera error terms");
  bool success = runOptimizer(problem, 200, "stereoCalibrate");
  if (success) baseline_HL = Transformation(baselineDv->T());
  return success;
}

bool solveFullBatch(std::vector<CameraGeometryPtr>& cameras, const std::vector<Transformation>& baselineGuesses,
                    const MulticamCalibrationGraph& graph, std::vector<Transformation>& baselines) {
  boost::shared_ptr<ab::OptimizationProblem> problem(new ab::OptimizationProblem());
  for (auto& cam : cameras) {
    cam->setDvActiveStatus(true, true, false);
    addCameraDvs(*problem, *cam);
  }
  std::vector<TransformationDvPtr> baselineDvs;
  for (size_t b = 0; b + 1 < cameras.size(); ++b) {
    TransformationDvPtr dv = std::make_shared<TransformationDv>(baselineGuesses[b]);
    for (size_t i = 0; i < dv->numDesignVariables(); ++i) problem->addDesignVariable(dv->getDesignVariable(i));
    baselineDvs.push_back(dv);
  }
  const Eigen::Matrix2d invR = Eigen::Matrix2d::Identity();
  auto target = cameras[0]->target;
  std::vector<TransformationDvPtr> targetPoseDvs;
  std::vector<ReprojectionErrorHandlePtr> rerrs;
  for (double ts : graph.obsDb().getAllViewTimestamps()) {
    auto obsTuple = graph.obsDb().getAllObsAtTimestamp(ts);
    Transformation T0 = graph.getTargetPoseGuess(ts, cameras, baselineGuesses);
    TransformationDvPtr poseDv = addPoseDesignVariable(*problem, T0);
    targetPoseDvs.push_back(poseDv);
    for (auto& co : obsTuple) {
      int cidx = co.first;
      ab::TransformationExpression T_camN_calib = poseDv->toExpression().inverse();
      for (int idx = 0; idx < cidx; ++idx) T_camN_calib = baselineDvs[idx]->toExpression() * T_camN_calib;
      for (size_t i = 0; i < target->size(); ++i) {
        ab::HomogeneousExpression p_target(sm::kinematics::toHomogeneous(target->point(i)));
        Eigen::Vector2d y;
        if (co.second->imagePoint(i, y)) {
          auto rerr = cameras[cidx]->model->createReprojectionError(y, invR, T_camN_calib * p_target);
          problem->addErrorTerm(rerr->term());
          rerrs.push_back(rerr);
        }
      }
    }
  }
  KLOG_DEBUG("solveFullBatch: added " << rerrs.size() << " camera error terms");
  bool success = runOptimizer(problem, 250, "solveFullBatch");
  baselines.clear();
  for (auto& dv : baselineDvs) baselines.push_back(Transformation(dv->T()));
  return success;
}

// ===========================================================================
// CalibrationTarget / batch problem
// ===========================================================================
CalibrationTarget::CalibrationTarget(aslam::cameras::GridCalibrationTargetBase::Ptr t, bool estimateLandmarks)
    : target(t) {
  for (size_t i = 0; i < target->size(); ++i) {
    boost::shared_ptr<ab::HomogeneousPoint> p(
        new ab::HomogeneousPoint(sm::kinematics::toHomogeneous(target->point(i))));
    p->setActive(estimateLandmarks);
    P_t_dv.push_back(p);
    P_t_ex.push_back(p->toExpression());
  }
}

CalibrationTargetOptimizationProblem::Ptr CalibrationTargetOptimizationProblem::fromTargetViewObservations(
    const std::vector<CameraGeometryPtr>& cameras, const std::shared_ptr<CalibrationTarget>& target,
    const std::vector<TransformationDvPtr>& baselines, double timestamp, const Transformation& T_tc_guess,
    const RigObservations& rig_observations, bool useBlakeZissermanMest) {
  Ptr rval(new CalibrationTargetOptimizationProblem());
  rval->cameras = cameras;
  rval->target = target;
  rval->baselines = baselines;
  rval->timestamp = timestamp;
  rval->T_tc_guess = T_tc_guess;
  rval->rig_observations = rig_observations;

  // 1. design variable for the target pose
  rval->dv_T_target_camera = std::make_shared<TransformationDv>(T_tc_guess);
  for (size_t i = 0; i < rval->dv_T_target_camera->numDesignVariables(); ++i)
    rval->addDesignVariable(rval->dv_T_target_camera->getDesignVariable(i), TRANSFORMATION_GROUP_ID);
  // 2. baselines
  for (auto& b : baselines)
    for (size_t i = 0; i < b->numDesignVariables(); ++i)
      rval->addDesignVariable(b->getDesignVariable(i), CALIBRATION_GROUP_ID);
  // 3. landmarks
  for (auto& p : target->P_t_dv) rval->addDesignVariable(p, LANDMARK_GROUP_ID);
  // 4. cameras
  for (auto& cam : cameras) {
    if (!cam->isGeometryInitialized)
      throw std::runtime_error("The camera geometry is not initialized. Please initialize with initGeometry()");
    cam->setDvActiveStatus(true, true, false);
    rval->addDesignVariable(cam->model->distortionDesignVariable(), CALIBRATION_GROUP_ID);
    rval->addDesignVariable(cam->model->projectionDesignVariable(), CALIBRATION_GROUP_ID);
    rval->addDesignVariable(cam->model->shutterDesignVariable(), CALIBRATION_GROUP_ID);
  }
  // 5. observations
  const Eigen::Matrix2d invR = Eigen::Matrix2d::Identity();
  int rerrCnt = 0;
  for (auto& co : rig_observations) {
    int camId = co.first;
    auto& camera = cameras[camId];
    ab::TransformationExpression T_camN_calib = rval->dv_T_target_camera->toExpression().inverse();
    for (int idx = 0; idx < camId; ++idx) T_camN_calib = baselines[idx]->toExpression() * T_camN_calib;
    auto& list = rval->rerrs[camId];
    list.clear();
    for (size_t i = 0; i < target->P_t_ex.size(); ++i) {
      Eigen::Vector2d y;
      if (co.second->imagePoint(i, y)) {
        ++rerrCnt;
        auto rerr = camera->model->createReprojectionError(y, invR, T_camN_calib * target->P_t_ex[i]);
        rerr->idx = static_cast<int>(i);
        if (useBlakeZissermanMest)
          rerr->term()->setMEstimatorPolicy(boost::shared_ptr<ab::MEstimator>(new ab::BlakeZissermanMEstimator(2.0)));
        rval->addErrorTerm(rerr->term());
        list.push_back(rerr);
      } else {
        list.push_back(ReprojectionErrorHandlePtr());
      }
    }
  }
  KLOG_DEBUG("Adding a view with " << rig_observations.size() << " cameras and " << rerrCnt << " error terms");
  return rval;
}

CalibrationTargetOptimizationProblem::Ptr removeCornersFromBatch(
    const CalibrationTargetOptimizationProblem::Ptr& batch,
    const std::vector<std::pair<int, std::vector<int>>>& camId_cornerIdList, bool useBlakeZissermanMest) {
  bool hasCornerRemoved = false;
  for (auto& cl : camId_cornerIdList) {
    for (auto& co : batch->rig_observations) {
      if (co.first != cl.first) continue;
      for (int cornerId : cl.second) {
        co.second->removeImagePoint(cornerId);
        hasCornerRemoved = true;
      }
    }
  }
  if (!hasCornerRemoved) throw std::runtime_error("need to remove at least one corner...");
  return CalibrationTargetOptimizationProblem::fromTargetViewObservations(
      batch->cameras, batch->target, batch->baselines, batch->timestamp, batch->T_tc_guess, batch->rig_observations,
      useBlakeZissermanMest);
}

// ===========================================================================
// CameraCalibration
// ===========================================================================
CameraCalibration::CameraCalibration(const std::vector<CameraGeometryPtr>& cams,
                                     const std::vector<Transformation>& baselineGuesses, bool estimateLandmarks,
                                     bool /*verbose*/, bool useBZ)
    : cameras(cams), useBlakeZissermanMest(useBZ) {
  estimator.reset(new aslam::calibration::IncrementalEstimator(CALIBRATION_GROUP_ID));
  target = std::make_shared<CalibrationTarget>(cameras[0]->target, estimateLandmarks);
  for (size_t b = 0; b + 1 < cameras.size(); ++b) baselines.push_back(std::make_shared<TransformationDv>(baselineGuesses[b]));
}

bool CameraCalibration::addTargetView(double timestamp, const RigObservations& rig_observations,
                                      const Transformation& T_tc_guess, bool force) {
  auto batch = CalibrationTargetOptimizationProblem::fromTargetViewObservations(
      cameras, target, baselines, timestamp, T_tc_guess, rig_observations, useBlakeZissermanMest);
  lastReturnValue = estimator->addBatch(batch, force);
  if (static_cast<int>(lastReturnValue.numIterations) >= estimator->getOptimizerOptions().maxIterations) {
    KLOG_ERROR("Did not converge in maxIterations... restarting...");
    throw OptimizationDiverged();
  }
  bool success = lastReturnValue.batchAccepted;
  if (success) {
    KLOG_DEBUG("The estimator accepted this batch");
    views.push_back(batch);
  } else {
    KLOG_DEBUG("The estimator did not accept this batch");
  }
  return success;
}

// ===========================================================================
// statistics / output
// ===========================================================================
std::vector<ViewReprojectionErrors> getReprojectionErrors(CameraCalibration& calib, int camId) {
  std::vector<ViewReprojectionErrors> all;
  for (auto& view : calib.views) {
    ViewReprojectionErrors v;
    auto it = view->rerrs.find(camId);
    if (it != view->rerrs.end()) {
      v.camInView = true;
      for (auto& rerr : it->second) {
        if (rerr) {
          Eigen::Vector2d c = rerr->measurement();
          Eigen::Vector2d r = rerr->predicted();
          v.valid.push_back(true);
          v.corners.push_back(c);
          v.reprojections.push_back(r);
          v.errors.push_back(c - r);
        } else {
          v.valid.push_back(false);
          v.corners.push_back(Eigen::Vector2d::Constant(std::nan("")));
          v.reprojections.push_back(Eigen::Vector2d::Constant(std::nan("")));
          v.errors.push_back(Eigen::Vector2d::Constant(std::nan("")));
        }
      }
    }
    all.push_back(v);
  }
  return all;
}

void getReprojectionErrorStatistics(const std::vector<ViewReprojectionErrors>& all, Eigen::Vector2d& mean,
                                    Eigen::Vector2d& std) {
  std::vector<double> ex, ey;
  for (auto& v : all) {
    if (!v.camInView) continue;
    for (size_t i = 0; i < v.valid.size(); ++i) {
      if (!v.valid[i]) continue;
      ex.push_back(v.errors[i][0]);
      ey.push_back(v.errors[i][1]);
    }
  }
  if (ex.empty()) throw std::runtime_error("rerrs has invalid dimension");
  mean << kalibr::mean(ex), kalibr::mean(ey);
  std << stddev(ex), stddev(ey);
}

void recoverCovariance(CameraCalibration& calib, std::vector<Eigen::VectorXd>& std_baselines,
                       std::vector<Eigen::VectorXd>& std_cameras) {
  const size_t numCams = calib.cameras.size();
  Eigen::VectorXd est_stds = calib.estimator->getSigma2Theta().diagonal().cwiseSqrt();
  std_baselines.clear();
  std_cameras.clear();
  for (size_t b = 0; b + 1 < numCams; ++b) {
    if (static_cast<int>(6 * b + 6) <= est_stds.size())
      std_baselines.push_back(est_stds.segment(6 * b, 6));
    else
      std_baselines.push_back(Eigen::VectorXd::Constant(6, std::nan("")));
  }
  int offset = static_cast<int>(6 * (numCams - 1));
  for (auto& cam : calib.cameras) {
    int nt = cam->model->minimalDimensionsDistortion() + cam->model->minimalDimensionsProjection() +
             cam->model->minimalDimensionsShutter();
    if (offset + nt <= est_stds.size())
      std_cameras.push_back(est_stds.segment(offset, nt));
    else
      std_cameras.push_back(Eigen::VectorXd::Constant(nt, std::nan("")));
    offset += nt;
  }
}

void printParameters(CameraCalibration& calib, std::ostream& os) {
  os << "Calibration results \n";
  os << "====================\n";
  std::vector<Eigen::VectorXd> std_baselines, std_cameras;
  recoverCovariance(calib, std_baselines, std_cameras);
  os << "Camera-system parameters:\n";
  for (size_t c = 0; c < calib.cameras.size(); ++c) {
    auto& cam = calib.cameras[c];
    Eigen::VectorXd d = cam->model->distortionParameters();
    Eigen::VectorXd p = cam->model->projectionParameters();
    Eigen::VectorXd dd = std_cameras[c].head(std::min<int>(d.size(), std_cameras[c].size()));
    Eigen::VectorXd dp = std_cameras[c].tail(std::max<int>(0, std_cameras[c].size() - d.size()));
    os << "cam" << c << " (" << cam->topic << "):\n";
    os << "    type: " << cam->model->geometryTypeName() << " (" << cam->model->modelName() << ")\n";
    os << "    distortion: " << fmtVec(d) << " +- " << fmtVec(dd) << "\n";
    os << "    projection: " << fmtVec(p) << " +- " << fmtVec(dp) << "\n";
    auto rerrs = getReprojectionErrors(calib, static_cast<int>(c));
    try {
      Eigen::Vector2d me, se;
      getReprojectionErrorStatistics(rerrs, me, se);
      os << std::fixed << std::setprecision(6) << "    reprojection error: [" << me[0] << ", " << me[1] << "] +- ["
         << se[0] << ", " << se[1] << "]\n";
      os.unsetf(std::ios::floatfield);
    } catch (...) {
      os << "    failed printing the reprojection error!\n";
    }
    os << "\n";
  }
  for (size_t b = 0; b < calib.baselines.size(); ++b) {
    Transformation T(calib.baselines[b]->T());
    os << "baseline T_" << b + 1 << "_" << b << ":\n";
    os << "    q: " << fmtVec(T.q()) << " +- " << fmtVec(std_baselines[b].head(3)) << "\n";
    os << "    t: " << fmtVec(T.t()) << " +- " << fmtVec(std_baselines[b].tail(3)) << "\n\n";
  }
  os << "\n\nTarget configuration\n====================\n\n";
  calib.cameras[0]->targetConfig.printDetails(os);
}

}  // namespace kalibr
