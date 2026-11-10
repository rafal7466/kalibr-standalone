// kalibr_calibrate_cameras (C++ / no ROS)
//
// Port of kalibr/python/kalibr_calibrate_cameras. Calibrates the intrinsics and
// extrinsics of a multi-camera system from images stored in the EuRoC / ASL
// folder format instead of a rosbag.
//
// Example (EuRoC cam_april):
//   kalibr_calibrate_cameras --target april_6x6.yaml --dataset cam_april/mav0
//        --topics cam0 cam1 --models pinhole-radtan pinhole-radtan --output out/cam_april
#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>

#include <boost/filesystem.hpp>
#include <sm/logging.hpp>

#include "common/ArgParser.hpp"
#include "common/CameraCalibrator.hpp"
#include "common/ResultWriter.hpp"

using namespace kalibr;
using sm::kinematics::Transformation;

int main(int argc, char** argv) {
  ArgParser p("Calibrate the intrinsics and extrinsics of a camera system with non-shared overlapping field of view.\n"
              "(C++ port of kalibr_calibrate_cameras, reads EuRoC/ASL folders instead of rosbags)");
  p.add("--models", -1, "The camera model to estimate for every camera {pinhole-radtan, pinhole-equi, pinhole-fov, "
                        "omni-none, omni-radtan, eucm-none, ds-none}", "", true);
  p.add("--dataset", 1, "Dataset root folder in ASL format (e.g. .../cam_april/mav0)");
  p.add("--topics", -1, "Camera sub-folders inside --dataset (e.g. cam0 cam1) or absolute camera folders", "", true);
  p.add("--bag-from-to", 2, "Use the data starting from up to this time [s]");
  p.add("--bag-freq", 1, "Frequency to extract features at [hz]");
  p.add("--target", 1, "Calibration target configuration as yaml file", "", true);
  p.add("--approx-sync", 1, "Time tolerance for approximate image synchronization [s]", "0.02");
  p.add("--qr-tol", 1, "The tolerance on the factors of the QR decomposition", "0.02");
  p.add("--mi-tol", 1, "The tolerance on the mutual information for adding an image. Higher means fewer images will "
                       "be added. Use -1 to force all images.", "0.2");
  p.add("--no-shuffle", 0, "Do not shuffle the dataset processing order");
  p.add("--seed", 1, "Random seed used for shuffling the views", "0");
  p.add("--no-outliers-removal", 0, "Disable corner outlier filtering");
  p.add("--no-final-filtering", 0, "Disable filtering after all views have been processed.");
  p.add("--min-views-outlier", 1, "Number of raw views to initialize statistics", "20");
  p.add("--use-blakezisserman", 0, "Enable the Blake-Zisserman m-estimator");
  p.add("--threads", 1, "Number of threads (corner extraction / optimizer)", std::to_string(defaultNumThreads()));
  p.add("--verbose", 0, "Enable (really) verbose output");
  p.add("--export-poses", 0, "Export the optimized poses into a CSV (time_ns, position, quaternion)");
  p.add("--output", 1, "Output prefix for the result files (default: <dataset>-...)");
  p.epilog =
      "example aprilgrid.yaml:\n    target_type: 'aprilgrid'\n    tagCols: 6\n    tagRows: 6\n"
      "    tagSize: 0.088  #m\n    tagSpacing: 0.3 #percent of tagSize";
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
  const auto models = p.list("--models");
  const auto topics = p.list("--topics");
  if (models.size() != topics.size()) {
    KLOG_ERROR("Please specify exactly one camera model (--models) for each topic (--topics).");
    return 2;
  }
  const int minViewOutlier = p.integer("--min-views-outlier");
  if (minViewOutlier < 1) {
    KLOG_ERROR("Please specify a positive integer (--min-views-outlier).");
    return 2;
  }
  const bool removeOutliers = !p.flag("--no-outliers-removal");
  const bool allowEndFiltering = !p.flag("--no-final-filtering");
  const bool doBlakeZisserman = p.flag("--use-blakezisserman");
  const int nThreads = std::max(1, p.integer("--threads"));
  const std::vector<double> fromTo = p.dblList("--bag-from-to");
  const double freq = p.has("--bag-freq") ? p.dbl("--bag-freq") : -1.0;
  const std::string datasetRoot = p.str("--dataset");

  std::string outPrefix = p.str("--output");
  if (outPrefix.empty()) {
    boost::filesystem::path d(datasetRoot.empty() ? topics[0] : datasetRoot);
    if (d.filename() == "mav0") d = d.parent_path();
    outPrefix = d.string();
  }
  boost::filesystem::path outDir = boost::filesystem::path(outPrefix).parent_path();
  if (!outDir.empty()) boost::filesystem::create_directories(outDir);

  try {
    TargetConfig targetConfig = TargetConfig::fromYaml(p.str("--target"));

    // ------------------------------------------------ cameras, corner extraction, intrinsics init
    std::vector<CameraGeometryPtr> cameraList;
    const int numCams = static_cast<int>(topics.size());
    ObservationDatabase obsdb(p.dbl("--approx-sync"));
    std::vector<std::shared_ptr<ImageFolderReader>> datasets;
    for (int camId = 0; camId < numCams; ++camId) {
      std::string folder = topics[camId];
      if (!datasetRoot.empty() && !boost::filesystem::exists(folder)) folder = joinPath(datasetRoot, topics[camId]);
      std::cout << "Initializing cam" << camId << ":" << std::endl;
      std::cout << "\tCamera model:\t  " << models[camId] << std::endl;
      auto dataset = std::make_shared<ImageFolderReader>(folder, fromTo, freq);
      datasets.push_back(dataset);
      std::cout << "\tDataset:          " << folder << std::endl;
      std::cout << "\tNumber of images: " << dataset->numImages() << std::endl;

      CameraModel::Ptr model = CameraModel::create(models[camId]);
      auto cam = std::make_shared<CameraGeometry>(model, targetConfig, dataset->name());
      auto observations = extractCornersFromDataset(*dataset, *model, targetConfig, /*noTransformation=*/true,
                                                    /*filterCornerOutliers=*/false, nThreads);
      for (auto& obs : observations) obsdb.addObservation(camId, obs);
      if (observations.empty() || !cam->initGeometryFromObservations(observations))
        throw std::runtime_error("Could not initialize the intrinsics for camera " + folder +
                                 ". Try to use --verbose and check whether the calibration target extraction is "
                                 "successful.");
      std::cout << "\tProjection initialized to: " << fmtVec(model->projectionParameters()) << std::endl;
      std::cout << "\tDistortion initialized to: " << fmtVec(model->distortionParameters()) << std::endl;
      cameraList.push_back(cam);
    }
    if (verbose) obsdb.printTable(std::cout);

    // ------------------------------------------------ calibration graph
    MulticamCalibrationGraph graph(obsdb);
    if (!graph.isGraphConnected()) {
      obsdb.printTable(std::cout);
      KLOG_ERROR("Cameras are not connected through mutual observations, please check the dataset. Maybe adjust the "
                 "approx. sync. tolerance.");
      return -1;
    }

    std::mt19937 rng(static_cast<unsigned>(p.integer("--seed")));
    int restartAttempts = 3;
    bool initOutlierRejection = true;
    std::vector<std::pair<int, Eigen::Vector2d>> removedOutlierCorners;

    while (true) {
      try {
        std::cout << "initializing initial guesses" << std::endl;
        std::vector<Transformation> baselineGuesses;
        if (cameraList.size() > 1) baselineGuesses = graph.getInitialGuesses(cameraList);
        for (size_t b = 0; b < baselineGuesses.size(); ++b)
          std::cout << "initialized baseline between cam" << b << " and cam" << b + 1 << " to:\n"
                    << fmtMat(baselineGuesses[b].T()) << std::endl;
        for (size_t c = 0; c < cameraList.size(); ++c) {
          std::cout << "initialized cam" << c << " to:\n";
          std::cout << "\t projection cam" << c << ": " << fmtVec(cameraList[c]->model->projectionParameters()) << "\n";
          std::cout << "\t distortion cam" << c << ": " << fmtVec(cameraList[c]->model->distortionParameters())
                    << std::endl;
        }

        std::cout << "initializing calibrator" << std::endl;
        CameraCalibration calibrator(cameraList, baselineGuesses, false, verbose, doBlakeZisserman);
        auto& options = calibrator.estimator->getOptions();
        options.infoGainDelta = p.dbl("--mi-tol");
        options.checkValidity = true;
        options.verbose = verbose;
        auto& lsOptions = calibrator.estimator->getLinearSolverOptions();
        lsOptions.columnScaling = true;
        lsOptions.verbose = verbose;
        lsOptions.epsSVD = 1e-6;
        auto& optOptions = calibrator.estimator->getOptimizerOptions();
        optOptions.maxIterations = 50;
        optOptions.nThreads = nThreads;
        optOptions.verbose = verbose;

        std::vector<double> timestamps = obsdb.getAllViewTimestamps();
        if (!p.flag("--no-shuffle")) std::shuffle(timestamps.begin(), timestamps.end(), rng);

        std::cout << "starting calibration..." << std::endl;
        const size_t numViews = timestamps.size();
        Progress progress(numViews);
        for (size_t viewId = 0; viewId < numViews; ++viewId) {
          double timestamp = timestamps[viewId];
          RigObservations obsTuple = obsdb.getAllObsAtTimestamp(timestamp);
          std::vector<Transformation> estBaselines;
          for (auto& b : calibrator.baselines) estBaselines.push_back(Transformation(b->T()));
          Transformation T_tc_guess = graph.getTargetPoseGuess(timestamp, cameraList, estBaselines);
          bool success = calibrator.addTargetView(timestamp, obsTuple, T_tc_guess);

          if ((verbose || (viewId % 25) == 0) && calibrator.estimator->getNumBatches() > 0 && viewId > 1) {
            std::cout << "\n------------------------------------------------------------------\n\n";
            std::cout << "Processed " << viewId + 1 << " of " << numViews << " views with "
                      << calibrator.estimator->getNumBatches() << " views used\n\n";
            printParameters(calibrator, std::cout);
            std::cout << "\n------------------------------------------------------------------\n";
          }
          progress.sample();

          // ------------------------------------------ outlier filtering
          const bool runEndFiltering = (viewId == numViews - 1) && allowEndFiltering;
          const size_t numActiveBatches = calibrator.estimator->getNumBatches();
          const size_t minBatches = static_cast<size_t>(minViewOutlier * numCams);
          if (((success && numActiveBatches > minBatches) || (runEndFiltering && numActiveBatches > minBatches)) &&
              removeOutliers) {
            std::vector<size_t> batchesToCheck;
            if (initOutlierRejection) {
              for (size_t b = 0; b < calibrator.estimator->getNumBatches(); ++b) batchesToCheck.push_back(b);
              std::cout << "\n\nFiltering outliers in all batches..." << std::endl;
              initOutlierRejection = false;
            } else if (runEndFiltering) {
              std::cout << "\n\nAll views have been processed.\n\nStarting final outlier filtering..." << std::endl;
              for (size_t b = 0; b < calibrator.estimator->getNumBatches(); ++b) batchesToCheck.push_back(b);
            } else {
              batchesToCheck.push_back(calibrator.estimator->getNumBatches() - 1);
            }
            std::sort(batchesToCheck.rbegin(), batchesToCheck.rend());
            std::unique_ptr<Progress> filterProgress;
            if (batchesToCheck.size() > 1) filterProgress.reset(new Progress(batchesToCheck.size()));

            for (size_t batchId : batchesToCheck) {
              if (batchId >= calibrator.views.size()) continue;
              std::vector<std::pair<int, std::vector<int>>> cornerRemovalAllCams;
              std::vector<int> camerasInBatch;
              for (auto& kv : calibrator.views[batchId]->rerrs) camerasInBatch.push_back(kv.first);
              for (int cidx : camerasInBatch) {
                auto rerrs = getReprojectionErrors(calibrator, cidx);
                Eigen::Vector2d me, se;
                getReprojectionErrorStatistics(rerrs, me, se);
                Eigen::Vector2d seThreshold = 4.0 * se;
                std::vector<int> cornerRemovalList;
                const auto& vr = rerrs[batchId];
                for (size_t pidx = 0; pidx < vr.valid.size(); ++pidx) {
                  if (!vr.valid[pidx]) continue;
                  const Eigen::Vector2d& r = vr.errors[pidx];
                  if (std::fabs(r[0]) > seThreshold[0] || std::fabs(r[1]) > seThreshold[1]) {
                    cornerRemovalList.push_back(static_cast<int>(pidx));
                    if (verbose)
                      KLOG_INFO("Outlier detected on view " << viewId << " with idx " << pidx << " (rerr=(" << r[0]
                                                            << ", " << r[1] << ") > (" << seThreshold[0] << ","
                                                            << seThreshold[1] << ") )");
                    removedOutlierCorners.emplace_back(cidx, vr.corners[pidx]);
                  }
                }
                cornerRemovalAllCams.emplace_back(cidx, cornerRemovalList);
              }
              size_t removeCount = 0;
              for (auto& c : cornerRemovalAllCams) removeCount += c.second.size();
              if (removeCount > 0) {
                auto original = calibrator.views[batchId];
                auto newBatch = removeCornersFromBatch(original, cornerRemovalAllCams, doBlakeZisserman);
                calibrator.estimator->removeBatch(original);
                calibrator.views[batchId] = newBatch;
                auto rval = calibrator.estimator->addBatch(newBatch, false);
                if (!rval.batchAccepted) {
                  KLOG_DEBUG("corrected view rejected! removing from optimization...");
                  calibrator.views.erase(calibrator.views.begin() + batchId);
                }
                KLOG_DEBUG("Removed " << removeCount << " outlier corners on batch " << batchId);
              }
              if (filterProgress) filterProgress->sample();
            }
          }
        }

        // ------------------------------------------------ final output
        std::cout << "\n\n..................................................................\n\n";
        std::cout << "Calibration complete.\n\n";
        if (removeOutliers) KLOG_WARN("Removed " << removedOutlierCorners.size() << " outlier corners.");
        std::cout << "\nProcessed " << numViews << " images with " << calibrator.estimator->getNumBatches()
                  << " images used" << std::endl;
        printParameters(calibrator, std::cout);
        std::cout << std::endl;

        const std::string resultFile = outPrefix + "-camchain.yaml";
        saveChainParametersYaml(calibrator, resultFile, graph);
        std::cout << "Results written to:\n";
        std::cout << "  Saving camera chain calibration to file: " << resultFile << "\n";
        const std::string resultFileTxt = outPrefix + "-results-cam.txt";
        saveResultTxt(calibrator, resultFileTxt);
        std::cout << "  Detailed results written to file: " << resultFileTxt << "\n";
        if (p.flag("--export-poses")) {
          const std::string posesFile = outPrefix + "-poses-cam0.csv";
          exportPoses(calibrator, posesFile);
          std::cout << "  Poses written to " << posesFile << "\n";
        }
        // reprojection errors for external analysis
        {
          const std::string rerrFile = outPrefix + "-reprojection-errors.csv";
          std::ofstream f(rerrFile);
          f << "#cam,view_timestamp,corner_id,u_meas,v_meas,u_pred,v_pred\n" << std::setprecision(12);
          for (size_t c = 0; c < cameraList.size(); ++c) {
            auto rerrs = getReprojectionErrors(calibrator, static_cast<int>(c));
            for (size_t v = 0; v < rerrs.size(); ++v) {
              if (!rerrs[v].camInView) continue;
              for (size_t i = 0; i < rerrs[v].valid.size(); ++i) {
                if (!rerrs[v].valid[i]) continue;
                f << c << "," << calibrator.views[v]->timestamp << "," << i << "," << rerrs[v].corners[i][0] << ","
                  << rerrs[v].corners[i][1] << "," << rerrs[v].reprojections[i][0] << ","
                  << rerrs[v].reprojections[i][1] << "\n";
              }
            }
          }
          std::cout << "  Reprojection errors written to " << rerrFile << "\n";
        }
        break;  // normal exit
      } catch (const OptimizationDiverged&) {
        --restartAttempts;
        KLOG_WARN("Optimization diverged possibly due to a bad initialization. (Do the models fit the lenses well?)");
        if (restartAttempts == 0) {
          KLOG_ERROR("Max. attemps reached... Giving up...");
          return 1;
        }
        KLOG_WARN("Restarting for a new attempt...");
        for (size_t c = 0; c < cameraList.size(); ++c) {
          std::cout << "Reinitialize the intrinsics for camera " << c << std::endl;
          auto observations = obsdb.getAllObsCam(static_cast<int>(c));
          if (!cameraList[c]->initGeometryFromObservations(observations))
            throw std::runtime_error("Could not re-initialize the intrinsics for camera " + cameraList[c]->topic);
        }
      }
    }
  } catch (const std::exception& e) {
    KLOG_ERROR("Exception: " << e.what());
    return 1;
  }
  return 0;
}
