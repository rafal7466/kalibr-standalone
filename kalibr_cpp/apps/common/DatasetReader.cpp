#include "common/DatasetReader.hpp"
#include "common/Common.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include <boost/filesystem.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace kalibr {

namespace fs = boost::filesystem;

aslam::Time ImageEntry::time() const {
  return aslam::Time(static_cast<uint32_t>(stamp_ns / 1000000000LL),
                     static_cast<uint32_t>(stamp_ns % 1000000000LL));
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else if (c != '\r' && c != '\n') {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  for (auto& s : out) {
    size_t a = s.find_first_not_of(" \t");
    size_t b = s.find_last_not_of(" \t");
    s = (a == std::string::npos) ? std::string() : s.substr(a, b - a + 1);
  }
  return out;
}

static std::string streamName(const std::string& folder) {
  fs::path p(folder);
  if (p.filename() == "." || p.filename().empty()) p = p.parent_path();
  return p.filename().string();
}

static std::vector<size_t> truncateFromTo(const std::vector<int64_t>& stamps,
                                          const std::vector<double>& fromTo,
                                          const std::string& what) {
  std::vector<size_t> valid;
  if (stamps.empty()) return valid;
  if (fromTo.size() != 2) {
    for (size_t i = 0; i < stamps.size(); ++i) valid.push_back(i);
    return valid;
  }
  if (fromTo[0] >= fromTo[1]) throw std::runtime_error("--bag-from-to: start time must be smaller than end time");
  int64_t start = *std::min_element(stamps.begin(), stamps.end());
  int64_t end = *std::max_element(stamps.begin(), stamps.end());
  double length = (end - start) * 1e-9;
  if (fromTo[1] > length)
    KLOG_WARN("End time of " << fromTo[1] << " s is bigger than the total length of " << length << " s");
  for (size_t i = 0; i < stamps.size(); ++i) {
    double t = (stamps[i] - start) * 1e-9;
    if (t >= fromTo[0] && t <= fromTo[1]) valid.push_back(i);
  }
  KLOG_WARN(what << ": truncated " << (stamps.size() - valid.size()) << " / " << stamps.size()
                 << " (from-to).");
  return valid;
}

ImageFolderReader::ImageFolderReader(const std::string& folder, const std::vector<double>& fromTo,
                                     double freq)
    : folder_(folder), name_(streamName(folder)) {
  std::string csv = joinPath(folder, "data.csv");
  std::ifstream f(csv);
  std::vector<ImageEntry> all;
  if (f.good()) {
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      auto tok = splitCsv(line);
      if (tok.size() < 2) continue;
      ImageEntry e;
      e.stamp_ns = std::stoll(tok[0]);
      e.path = joinPath(joinPath(folder, "data"), tok[1]);
      all.push_back(e);
    }
  } else {
    // fallback: data/<timestamp_ns>.png without csv
    fs::path dataDir = fs::path(folder) / "data";
    if (!fs::exists(dataDir)) throw std::runtime_error("No data.csv and no data/ folder in " + folder);
    for (auto& it : fs::directory_iterator(dataDir)) {
      if (!fs::is_regular_file(it.path())) continue;
      std::string ext = it.path().extension().string();
      std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
      if (ext != ".png" && ext != ".jpg" && ext != ".jpeg" && ext != ".pgm" && ext != ".bmp") continue;
      ImageEntry e;
      e.stamp_ns = std::stoll(it.path().stem().string());
      e.path = it.path().string();
      all.push_back(e);
    }
  }
  std::sort(all.begin(), all.end(), [](const ImageEntry& a, const ImageEntry& b) { return a.stamp_ns < b.stamp_ns; });
  if (all.empty()) throw std::runtime_error("No images found in " + folder);

  std::vector<int64_t> stamps;
  for (auto& e : all) stamps.push_back(e.stamp_ns);
  std::vector<size_t> idx = truncateFromTo(stamps, fromTo, "ImageFolderReader(" + name_ + ")");

  if (freq > 0.0) {
    std::vector<size_t> kept;
    double last = -1.0;
    for (size_t i : idx) {
      double t = all[i].stamp_ns * 1e-9;
      if (last < 0.0 || (t - last) >= 1.0 / freq) {
        last = t;
        kept.push_back(i);
      }
    }
    KLOG_WARN("ImageFolderReader(" << name_ << "): truncated " << (idx.size() - kept.size()) << " / "
                                  << idx.size() << " images (frequency)");
    idx = kept;
  }
  for (size_t i : idx) entries_.push_back(all[i]);
}

cv::Mat ImageFolderReader::readImage(size_t idx) const {
  cv::Mat img = cv::imread(entries_.at(idx).path, cv::IMREAD_UNCHANGED);
  if (img.empty()) throw std::runtime_error("Could not read image " + entries_[idx].path);
  if (img.depth() == CV_16U) img.convertTo(img, CV_8U, 1.0 / 256.0);
  if (img.channels() == 3) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  if (img.channels() == 4) cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
  return img;
}

ImuCsvReader::ImuCsvReader(const std::string& folderOrCsv, const std::vector<double>& fromTo) {
  std::string csv = folderOrCsv;
  if (fs::is_directory(folderOrCsv)) csv = joinPath(folderOrCsv, "data.csv");
  name_ = streamName(fs::path(csv).parent_path().string());
  std::ifstream f(csv);
  if (!f.good()) throw std::runtime_error("Could not open IMU csv " + csv);
  std::string line;
  std::vector<ImuMeasurementRaw> all;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    auto tok = splitCsv(line);
    if (tok.size() < 7) continue;
    ImuMeasurementRaw m;
    m.stamp_ns = std::stoll(tok[0]);
    m.omega << std::stod(tok[1]), std::stod(tok[2]), std::stod(tok[3]);
    m.alpha << std::stod(tok[4]), std::stod(tok[5]), std::stod(tok[6]);
    all.push_back(m);
  }
  std::sort(all.begin(), all.end(), [](const ImuMeasurementRaw& a, const ImuMeasurementRaw& b) { return a.stamp_ns < b.stamp_ns; });
  std::vector<int64_t> stamps;
  for (auto& m : all) stamps.push_back(m.stamp_ns);
  for (size_t i : truncateFromTo(stamps, fromTo, "ImuCsvReader(" + name_ + ")")) data_.push_back(all[i]);
  if (data_.empty()) throw std::runtime_error("No IMU measurements in " + csv);
}

}  // namespace kalibr
