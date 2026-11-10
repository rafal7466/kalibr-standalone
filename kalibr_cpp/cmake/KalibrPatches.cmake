# Idempotent source patches applied to the (otherwise unmodified) kalibr tree at
# configure time. They only fix incompatibilities with recent dependencies
# (SuiteSparse >= 7, Boost >= 1.73). No algorithmic change.
function(kalibr_patch file old new)
  set(f "${KALIBR_SRC_DIR}/${file}")
  file(READ "${f}" content)
  string(FIND "${content}" "${new}" already)
  if(already GREATER -1)
    return()
  endif()
  string(FIND "${content}" "${old}" found)
  if(found EQUAL -1)
    message(WARNING "kalibr_patch: pattern not found in ${file}")
    return()
  endif()
  string(REPLACE "${old}" "${new}" content "${content}")
  file(WRITE "${f}" "${content}")
  message(STATUS "patched ${file}")
endfunction()

# 1) SuiteSparseQR >= 4 (SuiteSparse 7) templated on the index type -> explicit Int
kalibr_patch(aslam_optimizer/aslam_backend/include/aslam/backend/implementation/Cholmod.hpp
"      SuiteSparseQR<double>(SPQR_ORDERING_FIXED, SPQR_NO_TOL, qrJ->ncol, 0,"
"#if defined(SPQR_MAIN_VERSION) && SPQR_MAIN_VERSION >= 4 /*KALIBR_CPP_PATCH*/
      SuiteSparseQR<double, SuiteSparse_long>(SPQR_ORDERING_FIXED, SPQR_NO_TOL, (SuiteSparse_long)qrJ->ncol, 0,
#else
      SuiteSparseQR<double>(SPQR_ORDERING_FIXED, SPQR_NO_TOL, qrJ->ncol, 0,
#endif")

# 2) SuiteSparse 7: SuiteSparseQR_factorization has 2 template params -> use the
#    real declaration instead of a 1-parameter forward declaration.
foreach(hdr
    aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/core/LinearSolver.h
    aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/algorithms/linalg.h)
  kalibr_patch(${hdr}
    "template <typename Entry> struct SuiteSparseQR_factorization;"
    "#include <SuiteSparseQR.hpp> /*KALIBR_CPP_PATCH*/")
endforeach()

# 3) spqr.hpp is an internal header no longer installed; replace spqr_maxcolnorm
kalibr_patch(aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp
  "#include <spqr.hpp>\n"
  "/*KALIBR_CPP_PATCH: spqr.hpp removed*/\n")
kalibr_patch(aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp
  "        spqr_maxcolnorm<double>(A, cholmod);"
  "        [&]() { double m = 0.0; /*KALIBR_CPP_PATCH: spqr_maxcolnorm*/
          for (std::ptrdiff_t j = 0; j < static_cast<std::ptrdiff_t>(A->ncol); ++j)
            m = std::max(m, colNorm(A, j));
          return m; }();")

# 4) Windows: timeval::tv_sec is 'long' (32 bit) but localtime() needs time_t (64 bit)
kalibr_patch(aslam_incremental_calibration/incremental_calibration/src/base/Timestamp.cpp
  "      ptm = localtime(&time.tv_sec);"
  "      time_t tt_ = time.tv_sec; /*KALIBR_CPP_PATCH*/\n      ptm = localtime(&tt_);")

# 5) OpenCV 5: findChessboardCorners/findCirclesGrid may return a 1-D Mat, so centers.row(i)
#    asserts. Copy the corners into a std::vector<cv::Point2f> instead (valid for OpenCV 4 too).
foreach(src aslam_cv/aslam_cameras/src/GridCalibrationTargetCheckerboard.cpp
            aslam_cv/aslam_cameras/src/GridCalibrationTargetCirclegrid.cpp)
  kalibr_patch(${src}
    "  //convert to eigen for output\n"
    "  //convert to eigen for output\n  std::vector<cv::Point2f> centerPts_; centers.copyTo(centerPts_); /*KALIBR_CPP_PATCH*/\n")
  kalibr_patch(${src}
    "centers.row(i).at<float>(0), centers.row(i).at<float>(1));"
    "centerPts_[i].x, centerPts_[i].y); /*KALIBR_CPP_PATCH*/")
endforeach()
