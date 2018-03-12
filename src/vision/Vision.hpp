//
// Created by yongqi on 17-12-5.
//

#ifndef VFORCE_VISION_HPP
#define VFORCE_VISION_HPP

#include "vision/detector/Detector.hpp"
#include "vision/matcher/Matcher.hpp"

namespace VForce {

class Vision {
 public:
  Vision(std::string cfg_root = ".", std::string cfg_file = "Vision.yml");

  bool Init();

  bool Process(const cv::Mat &color, const cv::Mat &depth,
               const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);

  const std::vector<ObjectInfo> & GetVisionResults() const {
    return results_;
  }

  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> GetTransformedModels() const {
    return transformed_models_;
  }

 private:
  struct Candidate {
    ObjectInfo object;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
    float mean_z;
    Candidate(ObjectInfo o, pcl::PointCloud<pcl::PointXYZ>::Ptr t, float z) :
        object(o), target(t), mean_z(z) {}
  };

  struct Model {
    std::string name;
    std::string path;
    float size[3];
    pcl::PointCloud<pcl::PointXYZ>::Ptr model;
  };

  std::shared_ptr<Detector> detector_;
  std::shared_ptr<Matcher> matcher_;

  bool estimate_all_;
  std::map<int, Model> models_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformed_models_;
  std::vector<ObjectInfo> results_;

  double max_match_error_;
  float model_uniform_radius_;
};

}
#endif //VFORCE_VISION_HPP
