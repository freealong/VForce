//
// Created by yongqi on 17-12-5.
//

#include <glog/logging.h>
#include "Vision.hpp"
#include "vision/detector/FRCNNDetector.hpp"
#include "vision/detector/MRCNNDetector.hpp"
#include "vision/matcher/RandomICPMatcher.hpp"
#include "utils/PointCloudUtils.hpp"
#include "utils/Timer.hpp"

using namespace std;

namespace VForce {

Vision::Vision(std::string cfg_root, std::string cfg_file) : transformed_model_(new pcl::PointCloud<pcl::PointXYZ>) {
  cv::FileStorage fs(cfg_root + "/"  + cfg_file, cv::FileStorage::READ);
  // load detector
  string detector_name;
  fs["detector_name"] >> detector_name;
  if (detector_name == "FRCNNDetector") {
    detector_ = std::shared_ptr<Detector>(new FRCNNDetector(cfg_root));
  }
  else if (detector_name == "MRCNNDetector") {
    detector_ = std::shared_ptr<Detector>(new MRCNNDetector(cfg_root));
  }
  else {
    LOG(ERROR) << "Unrecognized detector name(which should be FRCNNDetector or MRCNNDetector): " << detector_name;
  }
  // load matcher
  string matcher_name;
  fs["matcher_name"] >> matcher_name;
  if (matcher_name == "RandomICPMatcher") {
    matcher_ = std::shared_ptr<Matcher>(new RandomICPMatcher(cfg_root));
  }
  else {
    LOG(ERROR) << "Unrecognized matcher name(which should be RandomICPMatcher): " << detector_name;
  }
  // load params
  fs["max_match_error"] >> max_match_error_;
  DLOG(INFO) << "max_match_error: " << max_match_error_;
}

bool Vision::Process(const cv::Mat &color,
                     const cv::Mat &depth,
                     const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
  LOG(INFO) << "\nProcessing...";
  // run detect
  Utils::Timer<std::chrono::milliseconds> timer;
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
  LOG(INFO) << "Detection time: " << timer << "ms";
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
  LOG(INFO) << "Selection time: " << timer << "ms";
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
      LOG(INFO) << "Match time: " << timer << "ms";
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