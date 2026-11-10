// Calibration target configuration (port of kalibr_common.CalibrationTargetParameters)
#pragma once

#include <ostream>
#include <string>

#include <aslam/cameras/GridCalibrationTargetBase.hpp>

namespace kalibr {

struct TargetConfig {
  std::string targetType;  // aprilgrid | checkerboard | circlegrid
  // aprilgrid
  int tagRows = 0, tagCols = 0;
  double tagSize = 0.0, tagSpacing = 0.0;
  // checkerboard / circlegrid
  int targetRows = 0, targetCols = 0;
  double rowSpacingMeters = 0.0, colSpacingMeters = 0.0, spacingMeters = 0.0;
  bool asymmetricGrid = false;

  static TargetConfig fromYaml(const std::string& file);
  void printDetails(std::ostream& os) const;

  /// creates a new target object (one per detector thread)
  aslam::cameras::GridCalibrationTargetBase::Ptr createTarget(bool showExtraction = false) const;
};

}  // namespace kalibr
