//
// Created by yongqi on 17-12-4.
//

#ifndef VFORCE_RANDOMICPMATCHER_HPP
#define VFORCE_RANDOMICPMATCHER_HPP

#include "Matcher.hpp"

namespace VForce {

class RandomICPMatcher : public Matcher {
 public:
  RandomICPMatcher(const std::string &cfg_root = ".", const std::string &cfg_file = "RandomICPMatcher.yml");

  virtual bool LoadConfig(const std::string &cfg_file);

  virtual double EstimatePose(const PointTCloudPtr &model, const float* model_size,
                              const PointTCloudPtr &target,
                              Eigen::Matrix4f &tf, PointTCloudPtr &final);

 private:
  float uniform_radius_;
  int divide_num_;
  int iter_num_;
};

}
#endif //VFORCE_RANDOMICPMATCHER_HPP
