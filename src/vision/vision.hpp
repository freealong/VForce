//
// Created by yongqi on 17-12-5.
//

#ifndef VFORCE_VISION_HPP
#define VFORCE_VISION_HPP

#include "detector/detector.hpp"
#include "matcher/matcher.hpp"

namespace VForce {

class Vision {
 public:
  Vision(std::string cfg_file = "vision.yml");

  bool Process(const cv::Mat &color, const cv::Mat &depth,
               const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

  Eigen::Matrix4f GetObjectTransfomation() const {
    return cMo_;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr GetTransformedModel() const {
    return transformed_model_;
  }

  ObjectInfo GetObjectInfo() const {
    return object_;
  }

 private:
  struct Candidate {
    ObjectInfo object;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
    float mean_z;
    Candidate(ObjectInfo o, pcl::PointCloud<pcl::PointXYZ>::Ptr t, float z) :
        object(o), target(t), mean_z(z) {}
  };

  std::shared_ptr<Detector> detector_;
  std::shared_ptr<Matcher> matcher_;

  Eigen::Matrix4f cMo_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model_;
  ObjectInfo object_;

  double max_match_error_;
};

}
#endif //VFORCE_VISION_HPP
