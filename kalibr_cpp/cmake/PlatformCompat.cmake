# Platform / dependency-version compatibility settings (included after the targets).
if(MINGW)
  # POSIX random()/srandom() for incremental_calibration
  target_compile_options(incremental_calibration PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/compat/kalibr_platform_compat.h)
endif()
if(OpenCV_VERSION VERSION_GREATER_EQUAL 5.0)
  # apriltag's Homography33.cc only includes opencv2/opencv.hpp but needs cv::findHomography
  target_compile_options(ethz_apriltag2 PRIVATE -include opencv2/calib3d/calib3d.hpp)
endif()
if(WIN32)
  # sm_logging/Tokens.hpp defines no LIGHT colors in its WIN32 branch
  target_compile_definitions(sm_logging PRIVATE "COLOR_LIGHT_RED=\"\"" "COLOR_LIGHT_GREEN=\"\"")
endif()
