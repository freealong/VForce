//
// Created by yongqi on 17-12-4.
//

#ifndef VFORCE_POSE_HPP
#define VFORCE_POSE_HPP

#include <ostream>
#include "utils/MathUtils.hpp"

namespace VForce {

struct Pose {
  float x_, y_, z_; // in meter
  float roll_, pitch_, yaw_; // in radian

  Pose() = default;

  Pose(float x, float y, float z, float roll, float pitch, float yaw) :
      x_(x), y_(y), z_(z), roll_(roll), pitch_(pitch), yaw_(yaw) {}

  Pose(const Eigen::Matrix4f &tf) {
    Utils::matrix2pose(tf, x_, y_, z_, roll_, pitch_, yaw_);
  }

  friend std::ostream& operator<<(std::ostream &os, const Pose &p) {
    os << "x: " << p.x_ * 1000. << "mm y: " << p.y_ * 1000. << "mm z: " << p.z_ * 1000.
       << "mm roll: " << Utils::rad2deg(p.roll_)
       << "° pitch: " << Utils::rad2deg(p.pitch_)
       << "° yaw: " << Utils::rad2deg(p.yaw_) << "°";
  }
};

}
#endif //VFORCE_POSE_HPP
