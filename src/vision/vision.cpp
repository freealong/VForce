//
// Created by yongqi on 17-12-5.
//

#include <glog/logging.h>
#include "vision.hpp"
#include "detector/frcnndetector.hpp"
#include "detector/mrcnndetector.hpp"
#include "matcher/randomicpmatcher.hpp"
#include "utils/pointcloudutils.hpp"
#include "utils/timer.hpp"

using namespace std;

namespace VForce {

Vision::Vision(std::string cfg_file) : transformed_model_(new pcl::PointCloud<pcl::PointXYZ>) {
  cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
  // load detector
  auto detector_node = fs["detector"];
  string detector_name, detector_config;
  detector_node["name"] >> detector_name;
  detector_node["config"] >> detector_config;
  if (detector_name == "FRCNN") {
    detector_ = std::shared_ptr<Detector>(new FRCNNDetector(detector_config));
  }
  else if (detector_name == "MRCNN") {
    detector_ = std::shared_ptr<Detector>(new MRCNNDetector(detector_config));
  }
  else {
    LOG(ERROR) << "Unrecognized detector name(which should be FRCNN or MRCNN): " << detector_name;
  }
  // load matcher
  auto matcher_node = fs["matcher"];
  string matcher_name, matcher_config;
  matcher_node["name"] >> matcher_name;
  matcher_node["config"] >> matcher_config;
  if (matcher_name == "RandomICP") {
    matcher_ = std::shared_ptr<Matcher>(new RandomICPMatcher(matcher_config));
  }
  else {
    LOG(ERROR) << "Unrecognized matcher name(which should be RandomICP): " << detector_name;
  }
  // load params
  fs["max_match_error"] >> max_match_error_;
  DLOG(INFO) << "max_match_error: " << max_match_error_;
}

bool Vision::Process(const cv::Mat &color,
                     const cv::Mat &depth,
                     const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
  // run detect
  Utils::Timer timer;
  // @TODO: convert rgb in numpy array
  cv::Mat rgb;
  cv::cvtColor(color, rgb, cv::COLOR_BGR2RGB);
  DetectorResults objects;
  if (!detector_->Detect(rgb, objects)) {
    DLOG(WARNING) << "No object detected";
    return false;
  }
  cv::cvtColor(color, rgb, cv::COLOR_RGB2BGR);
#ifdef DEBUG
  cv::imwrite("detect_result.png", detector_->VisualizeResults(color, objects));
#endif
#ifdef ENABLE_TIMING
  LOG(INFO) << "Detection time: " << timer;
  timer.reset();
#endif
  // get target pointcloud based on detect results
  vector<Candidate> candidates;
  for (auto &r : objects) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    if (r.valid_mask_) {
      Utils::mask_cloud(cloud, r.rect_, r.mask_, target);
    }
    else {
      Utils::rect_cloud(cloud, r.rect_, target);
    }
    auto center = Utils::calculate_cloud_center(target);
    Candidate c(r, target, center.z);
    candidates.emplace_back(c);
  }
  // sort target clouds based on target height
  std::sort(candidates.begin(), candidates.end(), [](Candidate &c1, Candidate &c2) {
    return c1.mean_z < c2.mean_z;
  });
#ifdef ENABLE_TIMING
  LOG(INFO) << "Selection time: " << timer;
  timer.reset();
#endif
  // run match
  for (int i = 0; i < candidates.size(); ++i) {
//    DLOG(INFO) << "begin match candidate with z mean: " << candidates[i].mean_z;
    auto error = matcher_->EstimatePose(candidates[i].target, cMo_, transformed_model_);
    if (error < max_match_error_) {
      object_ = candidates[i].object;
      DLOG(INFO) << "Candidate " << i << " match successfully with error: " << error;
#ifdef ENABLE_TIMING
      LOG(INFO) << "Match time: " << timer;
#endif
      return true;
    }
    else {
      DLOG(WARNING) << "Candidate " << i << " match failed with error: " << error;
    }
  }
  return false;
}

}