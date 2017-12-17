//
// Created by yongqi on 17-12-4.
//

#ifndef VFORCE_MESSAGE_HPP
#define VFORCE_MESSAGE_HPP

#include <boost/asio.hpp>
#include "robot/Pose.hpp"

struct Message {
  double flag;
  double x, y, z; // in mm
  double roll, pitch, yaw; // in degree

  Message(const VForce::Pose& pose, int id) {
    flag = static_cast<double>(id);
    x = static_cast<double>(pose.x_);
    y = static_cast<double>(pose.y_);
    z = static_cast<double>(pose.z_);
    roll = static_cast<double>(pose.roll_);
    pitch = static_cast<double>(pose.pitch_);
    yaw = static_cast<double>(pose.yaw_);
  }

  friend std::ostream& operator<<(std::ostream &os, const Message &m);
};

std::ostream &operator<<(std::ostream &os, const Message &m) {
  os << "flag: " << m.flag << ", x: " << m.x << ", y: " << m.y << ", z: " << m.z
     << ", R: " << m.roll << ", P: " << m.pitch << ", Y: " << m.yaw;
}
#endif //VFORCE_MESSAGE_HPP
