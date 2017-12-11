//
// Created by yongqi on 17-12-4.
//

#ifndef VFORCE_RANDOMICPMATCHER_HPP
#define VFORCE_RANDOMICPMATCHER_HPP

#include "matcher.hpp"

namespace VForce {

class RandomICPMatcher : public Matcher {
 public:
  RandomICPMatcher(const std::string &cfg_file = "RandomICPMatcher.yml");

  virtual bool LoadConfig(const std::string &cfg_file);

  virtual double EstimatePose(const PointTCloudPtr &target, Eigen::Matrix4f &tf, PointTCloudPtr &final);

 private:
  float model_size_[3];
  float uniform_radius_;
  int divide_num_;
  int iter_num_;
  PointTCloudPtr sampled_model_;
};

}
#endif //VFORCE_RANDOMICPMATCHER_HPP
