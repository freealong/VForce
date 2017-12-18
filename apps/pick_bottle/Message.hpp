//
// Created by yongqi on 17-12-4.
//

#ifndef VFORCE_MESSAGE_HPP
#define VFORCE_MESSAGE_HPP

#include <boost/asio.hpp>
#include <iomanip>
#include "robot/Pose.hpp"
#include "utils/MathUtils.hpp"

using namespace VForce;

struct Message {
  char msg_[44];

  Message(const VForce::Pose& pose, int id) {
    assert(id < 10 && id > -1); // client demand
    // @TODO: ugly code
    sprintf(msg_, "%2f", pose.x_ * 1000);
    msg_[6] = ',';
    sprintf(msg_ + 7, "%2f", pose.y_ * 1000);
    msg_[13] = ',';
    sprintf(msg_ + 14, "%2f", pose.z_ * 1000);
    msg_[20] = ',';
    sprintf(msg_ + 21, "%2f", Utils::rad2deg(pose.roll_));
    msg_[27] = ',';
    sprintf(msg_ + 28, "%2f", Utils::rad2deg(pose.pitch_));
    msg_[34] = ',';
    sprintf(msg_ + 35, "%2f", Utils::rad2deg(pose.yaw_));
    msg_[41] = ',';
    msg_[42] = (char)(48 + id); // convert int to char
    msg_[43] = 0;
  }

  friend std::ostream& operator<<(std::ostream &os, const Message &m);
};

std::ostream &operator<<(std::ostream &os, const Message &m) {
  os << m.msg_;
}
#endif //VFORCE_MESSAGE_HPP
