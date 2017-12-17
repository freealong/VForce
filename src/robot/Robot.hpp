//
// Created by yongqi on 17-12-4.
//

#ifndef VFORCE_ROBOT_HPP
#define VFORCE_ROBOT_HPP

#include <Eigen/Eigen>
#include "Pose.hpp"

namespace VForce {

class Robot {
 public:
  Robot(const std::string &cfg_root = ".", const std::string &cfg_file = "Robot.yml");

  /**
   * Calculate robot catch pose
   * @param cMo tf from camera to object
   * @param pose robot catch pose
   * @param id robot catch id
   */
  void CalculatePose(const Eigen::Matrix4f &cMo, Pose &pose, int &id);

 private:
  /**
 * Decide using which pose to catch the object based on the object pose
 * @param rMo object pose in the robot coordinate
 * @return catch pose id
 */
  int GetCatchId(Eigen::Matrix4f &rMo);

  // constant after calibration
  Eigen::Matrix4f rMc_;
  std::vector<Eigen::Matrix4f> oMe_vec_;

  Eigen::Matrix4f rMo_;
  Eigen::Matrix4f rMe_;

  bool relative_motion_;
  Eigen::Matrix4f eMr_base_;
  Eigen::Matrix4f eMe_; // rMe_base to rMe, use when relative_motion_ is true
};

}
#endif //VFORCE_ROBOT_HPP
