// Compatibility shim: OpenCV 5 removed the legacy header opencv2/calib3d/calib3d.hpp
// and moved solvePnP/Rodrigues/findHomography to the "geometry" module, the camera
// calibration to "calib" and findChessboardCorners/findCirclesGrid/CALIB_CB_* to "objdetect".
#pragma once
#include <opencv2/core/version.hpp>
#include <opencv2/calib3d.hpp>
#if CV_VERSION_MAJOR >= 5
#include <opencv2/geometry.hpp>
#include <opencv2/calib.hpp>
#include <opencv2/objdetect.hpp>
#endif
