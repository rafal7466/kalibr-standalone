// Readers for the EuRoC / ASL dataset format (replacement of the rosbag readers
// kalibr_common/ImageDatasetReader.py and ImuDatasetReader.py).
//
//   <mav0>/cam0/data.csv   : #timestamp [ns],filename
//   <mav0>/cam0/data/*.png
//   <mav0>/imu0/data.csv   : #timestamp [ns],w_x,w_y,w_z [rad s^-1],a_x,a_y,a_z [m s^-2]
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <aslam/Time.hpp>

namespace kalibr {

struct ImageEntry {
  int64_t stamp_ns;
  std::string path;
  aslam::Time time() const;
};

class ImageFolderReader {
 public:
  /// \param folder     camera folder (contains data.csv and data/)
  /// \param fromTo     optional [start,end] in seconds relative to the first image (like --bag-from-to)
  /// \param freq       optional subsampling frequency in Hz (like --bag-freq), <=0 disables
  ImageFolderReader(const std::string& folder, const std::vector<double>& fromTo = {},
                    double freq = -1.0);
  size_t numImages() const { return entries_.size(); }
  const std::vector<ImageEntry>& entries() const { return entries_; }
  const std::string& folder() const { return folder_; }
  /// name of the stream, e.g. "cam0"
  const std::string& name() const { return name_; }
  /// loads an image as 8 bit grayscale (16 bit images are scaled like kalibr does)
  cv::Mat readImage(size_t idx) const;

 private:
  std::string folder_, name_;
  std::vector<ImageEntry> entries_;
};

struct ImuMeasurementRaw {
  int64_t stamp_ns;
  Eigen::Vector3d omega;  // [rad/s]
  Eigen::Vector3d alpha;  // [m/s^2]
};

class ImuCsvReader {
 public:
  ImuCsvReader(const std::string& folderOrCsv, const std::vector<double>& fromTo = {});
  const std::vector<ImuMeasurementRaw>& data() const { return data_; }
  const std::string& name() const { return name_; }
 private:
  std::string name_;
  std::vector<ImuMeasurementRaw> data_;
};

/// splits a csv line (also handles spaces after commas)
std::vector<std::string> splitCsv(const std::string& line);

}  // namespace kalibr
