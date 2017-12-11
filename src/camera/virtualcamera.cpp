//
// Created by yongqi on 17-5-22.
//

#include "virtualcamera.hpp"
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>
#include <utils/cvutils.hpp>

namespace VForce {

using namespace std;
using namespace cv;
using namespace pcl;

VirtualCamera::VirtualCamera(std::string p, std::string type) :
    _frame_id(0), _path(p), _camera_type(type) {
}

bool VirtualCamera::Start() {
  return LoadCalibration(_path + "/camParam.yml");
}

void VirtualCamera::Stop() {

}

void VirtualCamera::Update() {
  _frame_id++;
  string cloud_name = _path + "/cloud" + to_string(_frame_id) + ".pcd";
  ifstream f(cloud_name);
  if (!f.good() && _frame_id == 1) {
    LOG(WARNING) << cloud_name << " not found" << endl;
    return;
  }
  if (!f.good()) {
    LOG(WARNING) << "reaching end of the last frame, start from the beginning." << endl;
    _frame_id = 1;
  }
}

void VirtualCamera::FetchColor(cv::Mat &color) {
  _color = color = cv::imread(_path + "/color" + to_string(_frame_id) + ".png");
}

void VirtualCamera::FetchDepth(cv::Mat &depth) {
  _depth = depth = Utils::read_depth(_path + "/depth" + to_string(_frame_id) + ".png");
}

void VirtualCamera::FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr) {
  io::loadPCDFile(_path + "/cloud" + to_string(_frame_id) + ".pcd", *cloud_ptr);
}

void VirtualCamera::FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr) {
  io::loadPCDFile(_path + "/cloud" + to_string(_frame_id) + ".pcd", *cloud_ptr);
}

bool VirtualCamera::LoadCalibration(const std::string &cfg_file) {
  cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Can't Open file: " << cfg_file << endl;
    return false;
  }
  fs["r_intrin"] >> _r_intrin;
  fs["r_coeffs"] >> _r_coeffs;
  if (_camera_type == "struct_light") {
    fs["d_intrin"] >> _d_intrin;
    fs["d_coeffs"] >> _d_coeffs;
  } else {
    fs["reprojection"] >> _depth_reprojection;
  }
  fs["dMr"] >> _dMr;
  _rMd = _dMr.inv();
  fs.release();
  return true;
}

bool VirtualCamera::SaveCalibration(const std::string &cfg_file) {
  return false;
}

}
