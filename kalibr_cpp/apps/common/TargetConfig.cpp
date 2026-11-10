#include "common/TargetConfig.hpp"
#include "common/YamlUtil.hpp"

#include <algorithm>
#include <stdexcept>

#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>

namespace kalibr {

TargetConfig TargetConfig::fromYaml(const std::string& file) {
  YAML::Node n = loadYamlFile(file);
  TargetConfig c;
  if (!n["target_type"]) throw std::runtime_error("target yaml " + file + " misses target_type");
  c.targetType = n["target_type"].as<std::string>();
  if (c.targetType == "aprilgrid") {
    c.tagRows = n["tagRows"].as<int>();
    c.tagCols = n["tagCols"].as<int>();
    c.tagSize = n["tagSize"].as<double>();
    c.tagSpacing = n["tagSpacing"].as<double>();
  } else if (c.targetType == "checkerboard") {
    c.targetRows = n["targetRows"].as<int>();
    c.targetCols = n["targetCols"].as<int>();
    c.rowSpacingMeters = n["rowSpacingMeters"].as<double>();
    c.colSpacingMeters = n["colSpacingMeters"].as<double>();
  } else if (c.targetType == "circlegrid") {
    c.targetRows = n["targetRows"].as<int>();
    c.targetCols = n["targetCols"].as<int>();
    c.spacingMeters = n["spacingMeters"].as<double>();
    c.asymmetricGrid = n["asymmetricGrid"] ? n["asymmetricGrid"].as<bool>() : false;
  } else {
    throw std::runtime_error("Unknown calibration target type: " + c.targetType);
  }
  return c;
}

void TargetConfig::printDetails(std::ostream& os) const {
  os << "  Type: " << targetType << "\n";
  if (targetType == "aprilgrid") {
    os << "  Tags: \n";
    os << "    Rows: " << tagRows << "\n";
    os << "    Cols: " << tagCols << "\n";
    os << "    Size: " << tagSize << " [m]\n";
    os << "    Spacing " << tagSize * tagSpacing << " [m]\n";
  } else if (targetType == "checkerboard") {
    os << "  Rows\n    Count: " << targetRows << "\n    Distance: " << rowSpacingMeters << " [m]\n";
    os << "  Cols\n    Count: " << targetCols << "\n    Distance: " << colSpacingMeters << " [m]\n";
  } else {
    os << "  Rows: " << targetRows << "\n  Cols: " << targetCols << "\n  Spacing: " << spacingMeters
       << " [m]\n  Asymmetric: " << asymmetricGrid << "\n";
  }
}

aslam::cameras::GridCalibrationTargetBase::Ptr TargetConfig::createTarget(bool showExtraction) const {
  using namespace aslam::cameras;
  if (targetType == "checkerboard") {
    GridCalibrationTargetCheckerboard::CheckerboardOptions o;
    o.filterQuads = true;
    o.normalizeImage = true;
    o.useAdaptiveThreshold = true;
    o.performFastCheck = false;
    o.windowWidth = 5;
    o.showExtractionVideo = showExtraction;
    return GridCalibrationTargetBase::Ptr(
        new GridCalibrationTargetCheckerboard(targetRows, targetCols, rowSpacingMeters, colSpacingMeters, o));
  } else if (targetType == "circlegrid") {
    GridCalibrationTargetCirclegrid::CirclegridOptions o;
    o.showExtractionVideo = showExtraction;
    o.useAsymmetricCirclegrid = asymmetricGrid;
    return GridCalibrationTargetBase::Ptr(
        new GridCalibrationTargetCirclegrid(targetRows, targetCols, spacingMeters, o));
  } else if (targetType == "aprilgrid") {
    GridCalibrationTargetAprilgrid::AprilgridOptions o;
    // enforce more than one row --> pnp solution can be bad if all points are almost on a line
    o.minTagsForValidObs = static_cast<unsigned>(std::max(tagRows, tagCols) + 1);
    o.showExtractionVideo = showExtraction;
    return GridCalibrationTargetBase::Ptr(
        new GridCalibrationTargetAprilgrid(tagRows, tagCols, tagSize, tagSpacing, o));
  }
  throw std::runtime_error("Unknown calibration target type!");
}

}  // namespace kalibr
