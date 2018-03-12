//
// Created by yongqi on 17-12-5.
//

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include "Vision.hpp"
#include "vision/detector/FRCNNDetector.hpp"
#include "vision/detector/MRCNNDetector.hpp"
#include "vision/matcher/RandomICPMatcher.hpp"
#include "utils/PointCloudUtils.hpp"
#include "utils/Timer.hpp"

using namespace std;

namespace VForce {

Vision::Vision(std::string cfg_root, std::string cfg_file) {
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
  fs["model_uniform_radius"] >> model_uniform_radius_;
  DLOG(INFO) << "model_uniform_radius: " << model_uniform_radius_;
  estimate_all_ = static_cast<int>(fs["estimate_all"]) != 0;
  // load target models info
  auto targets = fs["targets"];
  for (auto it = targets.begin(); it != targets.end(); ++it) {
    cv::FileNode node = *it;
    Model model;
    int id;
    node["id"] >> id;
    if (models_.count(id) > 0) {
      LOG(WARNING) << "Found same target id, model info replaced with the later one.";
    }
    node["name"] >> model.name;
    node["path"] >> model.path;
    auto size_node = node["size"];
    size_node["x"] >> model.size[0];
    size_node["y"] >> model.size[1];
    size_node["z"] >> model.size[2];
    models_[id] = model;
    DLOG(INFO) << "Read target: {id: " << id
               << ", name: " << model.name
               << ", path: " << model.path
               << ", size: " << model.size[0]
               << ", " << model.size[1]
               << ", " << model.size[2] << "}";
  }
}

bool Vision::Init() {
  for (auto iter = models_.begin(); iter != models_.end(); ++iter) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_pc(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(iter->second.path, *model_pc) == -1) {
      LOG(ERROR) << "Load object model failed: " << iter->second.path;
      return false;
    }
    Utils::sampling_cloud(model_pc, model_pc, model_uniform_radius_);
    iter->second.model = model_pc;
  }
  return true;
}

bool Vision::Process(const cv::Mat &color,
                     const cv::Mat &depth,
                     const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) {
  LOG(INFO) << "\nProcessing...";
  // clear results
  results_.clear();
  transformed_models_.clear();
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
    int id = candidates[i].object.id_;
    auto& model = models_[id];
    Eigen::Matrix4f cMo;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model;
    auto error = matcher_->EstimatePose(model.model, model.size, candidates[i].target, cMo, transformed_model);
    if (error < max_match_error_) {
      auto object = candidates[i].object;
      object.pose_ = cMo;
      object.valid_pose_ = true;
      results_.emplace_back(object);
      transformed_models_.emplace_back(transformed_model);
      DLOG(INFO) << "Candidate " << i << " match successfully with error: " << error;
      if (!estimate_all_) {
        break;
      }
    }
    else {
      DLOG(WARNING) << "Candidate " << i << " match failed with error: " << error;
    }
  }
#ifdef ENABLE_TIMING
  LOG(INFO) << "Match time: " << timer << "ms";
#endif
  return !results_.empty();
}

}