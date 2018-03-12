//
// Created by yongqi on 17-11-29.
//

#ifndef VFORCE_DETECTOR_HPP
#define VFORCE_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

namespace VForce {

struct ObjectInfo {
  cv::Rect rect_;
  cv::Mat mask_;
  int id_;
  bool valid_mask_;
  Eigen::Matrix4f pose_;
  bool valid_pose_;
};

typedef std::vector<ObjectInfo> DetectorResults;

class Detector {
 public:
  Detector(const std::string& cfg_root = ".", const std::string &cfg_file = "Detector.yml") :
      cfg_root_(cfg_root), cfg_file_(cfg_file) {}

  /**
   * Detect objects in image
   * @param img input image
   * @param results vector of detected objects
   * @return return false if no object detected
   */
  virtual bool Detect(const cv::Mat &img, DetectorResults &results) = 0;

  /** Visualize detect results
   * @param img input image
   * @param results detect results
   * @return visualized image
   */
  cv::Mat VisualizeResults(const cv::Mat &img, const DetectorResults &results) const;

 protected:
  std::string cfg_root_, cfg_file_;
};

}
#endif //VFORCE_DETECTOR_HPP
