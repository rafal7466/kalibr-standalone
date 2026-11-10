# ---------------------------------------------------------------------------
#  Kalibr (ethz-asl/kalibr) - plain CMake build, NO ROS / catkin / Python.
#
#  Builds the C++ core libraries of Kalibr (Schweizer-Messer, aslam_cv,
#  aslam_optimizer, aslam_nonparametric_estimation, incremental_calibration,
#  ethz_apriltag2) as static libraries and the C++ command line tools:
#
#    kalibr_calibrate_cameras      (port of kalibr_calibrate_cameras.py)
#    kalibr_calibrate_imu_camera   (port of kalibr_calibrate_imu_camera.py)
#    kalibr_compare_euroc          (evaluation against EuRoC reference)
#
#  The tools read the EuRoC / ASL folder format (mav0/cam0/data.csv, ...)
#  instead of ROS bags.
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(kalibr_cpp CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(KALIBR_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../kalibr-master" CACHE PATH
    "Path to the unmodified kalibr source tree (github.com/ethz-asl/kalibr)")
get_filename_component(KALIBR_SRC_DIR "${KALIBR_SRC_DIR}" ABSOLUTE)
if(NOT EXISTS "${KALIBR_SRC_DIR}/aslam_cv")
  message(FATAL_ERROR "KALIBR_SRC_DIR=${KALIBR_SRC_DIR} does not contain the kalibr sources")
endif()
set(K "${KALIBR_SRC_DIR}")
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/KalibrPatches.cmake)

# ------------------------------------------------------------------ deps ---
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread regex)
# OpenCV 4 (calib3d) or OpenCV 5 (3d + calib): link all available modules
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV ${OpenCV_VERSION}")
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(SuiteSparseSimple REQUIRED)

if(MSVC)
  add_compile_options(/bigobj /EHsc /permissive- /W1)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX _CRT_SECURE_NO_WARNINGS)
else()
  add_compile_options(-Wno-deprecated-copy -Wno-deprecated-declarations
                      -Wno-ignored-attributes -Wno-int-in-bool-context
                      -Wno-unused-parameter -Wno-misleading-indentation)
  if(MINGW)
    add_compile_options(-Wa,-mbig-obj)
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX WIN32_LEAN_AND_MEAN)
  endif()
endif()
# Boost >= 1.73 deprecation noise for bind placeholders
add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS BOOST_ALLOW_DEPRECATED_HEADERS)
# SuiteSparse >= 7 removed CHOLMOD_INTLONG (value was 1) which sparse_helper.h references
add_compile_definitions(CHOLMOD_INTLONG=1)

# All include folders of the kalibr packages (header-only + libs)
# NOTE: aslam_splines must come before aslam_cv_error_terms: both ship
# aslam/backend/BSplineMotionError.hpp and only the aslam_splines one is current.
set(KALIBR_PACKAGES
  aslam_nonparametric_estimation/aslam_splines
  Schweizer-Messer/sm_common Schweizer-Messer/sm_boost Schweizer-Messer/sm_eigen
  Schweizer-Messer/sm_kinematics Schweizer-Messer/sm_logging Schweizer-Messer/sm_opencv
  Schweizer-Messer/sm_property_tree Schweizer-Messer/sm_random Schweizer-Messer/sm_timing
  Schweizer-Messer/sm_matrix_archive
  aslam_cv/aslam_time aslam_cv/aslam_cameras aslam_cv/aslam_cameras_april
  aslam_cv/aslam_cv_backend aslam_cv/aslam_cv_error_terms
  aslam_optimizer/aslam_backend aslam_optimizer/aslam_backend_expressions
  aslam_optimizer/sparse_block_matrix
  aslam_nonparametric_estimation/bsplines
  aslam_incremental_calibration/incremental_calibration
  aslam_offline_calibration/ethz_apriltag2
  aslam_offline_calibration/kalibr)
set(KALIBR_INCLUDE_DIRS "")
foreach(p ${KALIBR_PACKAGES})
  list(APPEND KALIBR_INCLUDE_DIRS "${K}/${p}/include")
endforeach()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/compat
  ${KALIBR_INCLUDE_DIRS}
  ${SUITESPARSE_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})
link_libraries(Eigen3::Eigen)

# helper: kalibr_lib(name pkg_dir src1 src2 ...)
function(kalibr_lib name pkgdir)
  set(srcs "")
  foreach(s ${ARGN})
    list(APPEND srcs "${K}/${pkgdir}/${s}")
  endforeach()
  add_library(${name} STATIC ${srcs})
endfunction()

# ------------------------------------------------------- Schweizer-Messer --
kalibr_lib(sm_common Schweizer-Messer/sm_common src/progress_info.cpp)
target_link_libraries(sm_common PUBLIC Boost::system)

kalibr_lib(sm_random Schweizer-Messer/sm_random src/random.cpp)

kalibr_lib(sm_eigen Schweizer-Messer/sm_eigen src/random.cpp)
target_link_libraries(sm_eigen PUBLIC sm_common sm_random Boost::serialization)

kalibr_lib(sm_boost Schweizer-Messer/sm_boost src/JobQueue.cpp)
target_link_libraries(sm_boost PUBLIC sm_common Boost::serialization Boost::thread Boost::system)

kalibr_lib(sm_kinematics Schweizer-Messer/sm_kinematics
  src/quaternion_algebra.cpp src/rotations.cpp src/transformations.cpp
  src/RotationalKinematics.cpp src/RotationVector.cpp src/EulerAnglesZYX.cpp
  src/EulerAnglesYawPitchRoll.cpp src/EulerRodriguez.cpp src/Transformation.cpp
  src/homogeneous_coordinates.cpp src/HomogeneousPoint.cpp
  src/UncertainTransformation.cpp src/UncertainHomogeneousPoint.cpp
  src/three_point_methods.cpp src/EulerAnglesZXY.cpp)
target_link_libraries(sm_kinematics PUBLIC sm_eigen sm_boost Boost::filesystem)

kalibr_lib(sm_logging Schweizer-Messer/sm_logging
  src/Logger.cpp src/StdOutLogger.cpp src/LoggingEvent.cpp src/LoggingGlobals.cpp
  src/Formatter.cpp src/Tokens.cpp)
target_link_libraries(sm_logging PUBLIC Boost::thread Boost::regex Boost::system Threads::Threads)

kalibr_lib(sm_property_tree Schweizer-Messer/sm_property_tree
  src/PropertyTree.cpp src/BoostPropertyTreeImplementation.cpp src/BoostPropertyTree.cpp
  src/PropertyTreeImplementation.cpp src/BoostPropertyTreeSupport.cpp)
target_link_libraries(sm_property_tree PUBLIC sm_common Boost::filesystem)

kalibr_lib(sm_timing Schweizer-Messer/sm_timing src/Timer.cpp src/NsecTimeUtilities.cpp)
target_link_libraries(sm_timing PUBLIC sm_common sm_random)

kalibr_lib(sm_matrix_archive Schweizer-Messer/sm_matrix_archive src/MatrixArchive.cpp)
target_link_libraries(sm_matrix_archive PUBLIC sm_common Boost::filesystem)

# ---------------------------------------------------------------- aslam_cv --
kalibr_lib(aslam_time aslam_cv/aslam_time src/time.cpp src/duration.cpp)
target_link_libraries(aslam_time PUBLIC Boost::system)

kalibr_lib(aslam_cameras aslam_cv/aslam_cameras
  src/CameraGeometryBase.cpp src/GlobalShutter.cpp src/RollingShutter.cpp src/NoMask.cpp
  src/NoDistortion.cpp src/RadialTangentialDistortion.cpp src/EquidistantDistortion.cpp
  src/FovDistortion.cpp src/ImageMask.cpp src/GridCalibrationTargetObservation.cpp
  src/GridCalibrationTargetBase.cpp src/GridCalibrationTargetCheckerboard.cpp
  src/GridCalibrationTargetCirclegrid.cpp src/GridDetector.cpp src/Triangulation.cpp
  src/FrameBase.cpp src/KeypointBase.cpp src/DescriptorBase.cpp src/BackProjection.cpp
  src/Landmark.cpp src/Image.cpp)
target_link_libraries(aslam_cameras PUBLIC sm_kinematics sm_logging sm_property_tree
  aslam_time ${OpenCV_LIBS} Boost::serialization Boost::filesystem)

file(GLOB APRILTAG_SRCS "${K}/aslam_offline_calibration/ethz_apriltag2/src/*.cc")
add_library(ethz_apriltag2 STATIC ${APRILTAG_SRCS})
target_link_libraries(ethz_apriltag2 PUBLIC ${OpenCV_LIBS})

kalibr_lib(aslam_cameras_april aslam_cv/aslam_cameras_april src/GridCalibrationTargetAprilgrid.cpp)
target_link_libraries(aslam_cameras_april PUBLIC aslam_cameras ethz_apriltag2)

# --------------------------------------------------------- aslam_optimizer --
kalibr_lib(sparse_block_matrix aslam_optimizer/sparse_block_matrix
  src/matrix_structure.cpp src/sparse_helper.cpp src/marginal_covariance_cholesky.cpp)
target_link_libraries(sparse_block_matrix PUBLIC sm_eigen ${SUITESPARSE_LIBRARIES})

kalibr_lib(aslam_backend aslam_optimizer/aslam_backend
  src/MEstimatorPolicies.cpp src/JacobianContainer.cpp src/DesignVariable.cpp
  src/ErrorTerm.cpp src/OptimizationProblemBase.cpp src/Optimizer.cpp
  src/sparse_matrix_functions.cpp src/OptimizationProblem.cpp src/SimpleOptimizationProblem.cpp
  src/JacobianBuilder.cpp src/LinearSystemSolver.cpp src/BlockCholeskyLinearSystemSolver.cpp
  src/SparseCholeskyLinearSystemSolver.cpp src/SparseQrLinearSystemSolver.cpp src/Matrix.cpp
  src/DenseMatrix.cpp src/SparseBlockMatrixWrapper.cpp src/DenseQrLinearSystemSolver.cpp
  src/BlockCholeskyLinearSolverOptions.cpp src/SparseCholeskyLinearSolverOptions.cpp
  src/SparseQRLinearSolverOptions.cpp src/DenseQRLinearSolverOptions.cpp
  src/TrustRegionPolicy.cpp src/ErrorTermDs.cpp src/GaussNewtonTrustRegionPolicy.cpp
  src/LevenbergMarquardtTrustRegionPolicy.cpp src/Marginalizer.cpp
  src/MarginalizationPriorErrorTerm.cpp src/DogLegTrustRegionPolicy.cpp src/Optimizer2.cpp)
target_link_libraries(aslam_backend PUBLIC sparse_block_matrix sm_boost sm_random sm_timing
  sm_logging sm_property_tree Boost::thread ${SUITESPARSE_LIBRARIES})

kalibr_lib(aslam_backend_expressions aslam_optimizer/aslam_backend_expressions
  src/RotationExpression.cpp src/RotationExpressionNode.cpp src/RotationQuaternion.cpp
  src/MappedRotationQuaternion.cpp src/Vector2RotationQuaternionExpressionAdapter.cpp
  src/MatrixExpression.cpp src/MatrixExpressionNode.cpp src/MatrixTransformation.cpp
  src/TransformationExpression.cpp src/TransformationExpressionNode.cpp
  src/TransformationBasic.cpp src/EuclideanExpression.cpp src/EuclideanExpressionNode.cpp
  src/EuclideanPoint.cpp src/MappedEuclideanPoint.cpp src/HomogeneousExpression.cpp
  src/HomogeneousExpressionNode.cpp src/HomogeneousPoint.cpp src/MappedHomogeneousPoint.cpp
  src/ScalarExpression.cpp src/ScalarExpressionNode.cpp src/Scalar.cpp
  src/EuclideanDirection.cpp src/ErrorTermTransformation.cpp src/ErrorTermEuclidean.cpp
  src/MapTransformation.cpp src/MatrixBasic.cpp)
target_link_libraries(aslam_backend_expressions PUBLIC aslam_backend sm_kinematics)

kalibr_lib(aslam_cv_backend aslam_cv/aslam_cv_backend src/GridCalibrationTargetDesignVariableContainer.cpp)
target_link_libraries(aslam_cv_backend PUBLIC aslam_backend_expressions aslam_cameras)

# ------------------------------------------ aslam_nonparametric_estimation --
kalibr_lib(bsplines aslam_nonparametric_estimation/bsplines src/BSpline.cpp src/BSplinePose.cpp)
target_link_libraries(bsplines PUBLIC sm_kinematics sm_timing sparse_block_matrix)

kalibr_lib(aslam_splines aslam_nonparametric_estimation/aslam_splines
  src/BSplinePoseDesignVariable.cpp src/BSplineExpressions.cpp src/EuclideanBSplineDesignVariable.cpp)
target_link_libraries(aslam_splines PUBLIC bsplines aslam_backend_expressions)

# -------------------------------------------------- incremental_calibration --
kalibr_lib(incremental_calibration aslam_incremental_calibration/incremental_calibration
  src/base/Serializable.cpp src/base/Timestamp.cpp src/base/Timer.cpp
  src/exceptions/Exception.cpp src/exceptions/InvalidOperationException.cpp
  src/exceptions/NullPointerException.cpp src/exceptions/SystemException.cpp
  src/statistics/NormalDistribution1v.cpp src/statistics/ChiSquareDistribution.cpp
  src/statistics/EstimatorMLNormal1v.cpp src/functions/IncompleteGammaPFunction.cpp
  src/functions/IncompleteGammaQFunction.cpp src/functions/LogFactorialFunction.cpp
  src/functions/LogGammaFunction.cpp src/core/IncrementalEstimator.cpp
  src/core/OptimizationProblem.cpp src/core/IncrementalOptimizationProblem.cpp
  src/core/LinearSolver.cpp src/core/LinearSolverOptions.cpp
  src/algorithms/marginalize.cpp src/algorithms/linalg.cpp)
target_link_libraries(incremental_calibration PUBLIC aslam_backend sm_eigen ${SUITESPARSE_LIBRARIES})

# ------------------------------------------- kalibr IMU error terms (C++) --
kalibr_lib(kalibr_errorterms aslam_offline_calibration/kalibr
  src/AccelerometerError.cpp src/GyroscopeError.cpp src/EuclideanError.cpp)
target_link_libraries(kalibr_errorterms PUBLIC aslam_backend_expressions)

# ================================================ C++ applications (new) ===
add_library(kalibr_app_common STATIC
  apps/common/Common.cpp
  apps/common/DatasetReader.cpp
  apps/common/TargetConfig.cpp
  apps/common/CameraModels.cpp
  apps/common/CameraCalibrator.cpp
  apps/common/ResultWriter.cpp)
target_include_directories(kalibr_app_common PUBLIC apps)
target_link_libraries(kalibr_app_common PUBLIC
  aslam_cameras_april aslam_cv_backend incremental_calibration aslam_splines
  kalibr_errorterms yaml-cpp::yaml-cpp ${OpenCV_LIBS} Threads::Threads)

add_executable(kalibr_calibrate_cameras apps/kalibr_calibrate_cameras.cpp)
target_link_libraries(kalibr_calibrate_cameras kalibr_app_common)

add_executable(kalibr_calibrate_imu_camera
  apps/kalibr_calibrate_imu_camera.cpp apps/common/IccCalibrator.cpp)
target_link_libraries(kalibr_calibrate_imu_camera kalibr_app_common)

add_executable(kalibr_compare_euroc apps/kalibr_compare_euroc.cpp)
target_link_libraries(kalibr_compare_euroc kalibr_app_common)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/PlatformCompat.cmake)

install(TARGETS kalibr_calibrate_cameras kalibr_calibrate_imu_camera kalibr_compare_euroc
        RUNTIME DESTINATION bin)
