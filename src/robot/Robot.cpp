//
// Created by yongqi on 17-12-4.
//

#include "Robot.hpp"
#include <glog/logging.h>
#include "utils/CVUtils.hpp"

namespace VForce {

using namespace Utils;

Robot::Robot(const std::string &cfg_root, const std::string &cfg_file) {
  cv::FileStorage fs(cfg_root + "/" + cfg_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Robot config file not found: " << cfg_root + "/" + cfg_file;
  }
  // load hand eye matrix
  std::string hand_eye_type;
  fs["hand_eye_type"] >> hand_eye_type;
  if (hand_eye_type == "eye_in_hand") {
    Eigen::Matrix4f rMe_camera, eMc;
    fs[hand_eye_type]["rMe_camera"] >> rMe_camera;
    DLOG(INFO) << "rMe_camera:\n" << rMe_camera.format(IOF);
    fs[hand_eye_type]["eMc"] >> eMc;
    DLOG(INFO) << "eMc:\n" << eMc.format(IOF);
    rMc_ = rMe_camera * eMc;
  }
  else if (hand_eye_type == "eye_to_hand") {
    fs[hand_eye_type]["rMc"] >> rMc_;
    DLOG(INFO) << "rMc:\n" << rMc_.format(IOF);
  }
  else {
    LOG(ERROR) << "Unrecognized eye-hand type: " << hand_eye_type;
  }
  // get robot move type
  fs["relative_motion"] >> relative_motion_;
  if (relative_motion_) {
    Eigen::Matrix4f rMe_base;
    fs["rMe_base"] >> rMe_base;
    DLOG(INFO) << "rMe_base:\n" << rMe_base.format(IOF);
    eMr_base_ = rMe_base.inverse();
  }
  // get oMe_vec
  auto catch_pose = fs["catch_pose"];
  oMe_vec_.resize(catch_pose.size());
  for (auto it = catch_pose.begin(); it != catch_pose.end(); ++it) {
    cv::FileNode node = *it;
    int id;
    node["id"] >> id;
    Eigen::Matrix4f cMo, rMe;
    node["cMo"] >> cMo;
    node["rMe"] >> rMe;
    DLOG(INFO) << "cMo " << id << " :" << cMo.format(IOF);
    DLOG(INFO) << "rMe " << id << " :" << rMe.format(IOF);
    oMe_vec_[id] = (rMc_ * cMo).inverse() * rMe;
  }
}

void Robot::CalculatePose(const Eigen::Matrix4f &cMo, Pose &pose, int &id) {
  // calculate object pose
  rMo_ = rMc_ * cMo;
  assert(rMo_(2, 2) < 0); // object z-axis should point robot -z-axis
  // calculate robot pose
  id = GetCatchId(rMo_);
  rMe_ = rMo_ * oMe_vec_[id - 1]; // wMe = wMo * oMe, oMe is constant = oMw_0 * wMe_0
  Pose robot_pose(rMe_);
  LOG(INFO) << robot_pose;
  if (relative_motion_) {
    eMe_ = eMr_base_ * rMe_;
    pose = Pose(eMe_);
  }
  else
    pose = robot_pose;
}

int Robot::GetCatchId(Eigen::Matrix4f &rMo) {
//  Eigen::Vector3f X(1, 0, 0);
//  Eigen::Vector3f XX = rMo_.block<3, 3>(0, 0) * X;
//  float x = XX(0);
//  float y = XX(1);
  // same result with the code comment out above
  float x = rMo(0, 0);
  float y = rMo(1, 0);
  if (fabs(x) > fabs(y)) {
    if (x > 0) {
      // model x-axis is around robot x-axis
      LOG(INFO) << "choose catch pose 1";
      return 1;
    } else {
      // model x-axis is around robot -x-axis
      LOG(INFO) << "choose catch pose 2";
      return 2;
    }
  } else {
    if (y > 0) {
      // model x-axis is around robot y-axis
      LOG(INFO) << "choose catch pose 3";
      return 3;
    } else {
      // model x-axis is around robot -y-axis
      LOG(INFO) << "choose catch pose 4";
      return 4;
    }
  }
}

}
