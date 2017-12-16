//
// Created by yongqi on 17-12-13.
//

#include <librealsense/rs.hpp>
using rs::context;
using rs::device;

#include "stereorealsensecamera.hpp"
#include <glog/logging.h>

namespace VForce {

using namespace std;

StereoRealsenseCamera::StereoRealsenseCamera(const std::string &cfg_file, const std::string &cfg_root) {
  context_ = std::shared_ptr<context>(new context);
  int device_count = context_->get_device_count();
  if (device_count < 2) {
    LOG(ERROR) << "Stereo Realsense Camera needs two cameras, " << device_count << " camera found.";
  }
  left_dev_ = context_->get_device(0);
  right_dev_ = context_->get_device(1);
  cfg_root_ = cfg_root;
  LoadCalibration(cfg_file);
}

bool StereoRealsenseCamera::Start() {
  if (running_)
    return true;
  if (left_dev_ == nullptr || right_dev_ == nullptr) {
    LOG(ERROR) << "left and right camera are not all connected.";
    return false;
  }
  left_dev_->enable_stream(rs::stream::depth, rs::preset::best_quality);
  left_dev_->enable_stream(rs::stream::color, rs::preset::best_quality);
  left_dev_->start();
  right_dev_->enable_stream(rs::stream::depth, rs::preset::best_quality);
  right_dev_->enable_stream(rs::stream::color, rs::preset::best_quality);
  right_dev_->start();
  // warm up camera
  for (int i = 0; i < 30; ++i) {
    left_dev_->wait_for_frames();
    right_dev_->wait_for_frames();
  }
  depth_ = new float[left_color_width_ * left_color_height_];
  left_aligned_depth_ = new uint16_t[left_color_width_ * left_color_height_];
  right_aligned_depth_ = new uint16_t[left_color_width_ * left_color_height_];
  left_depth_scale_ = left_dev_->get_depth_scale();
  right_depth_scale_ = right_dev_->get_depth_scale();
  running_ = true;
  return true;
}

void StereoRealsenseCamera::Stop() {
  if (running_) {
    left_dev_->stop();
    right_dev_->stop();
    delete[] depth_;
    delete[] left_aligned_depth_;
    delete[] right_aligned_depth_;
    running_ = false;
  }
}

void StereoRealsenseCamera::Update() {
  if (running_) {
    // close right laser, open left laser;
    right_dev_->set_option(rs::option::f200_laser_power, 0.0);
    left_dev_->set_option(rs::option::f200_laser_power, 1.0);
    // get raw depth after skip few frames
    for (int i = 0; i < 5; ++i)
      left_dev_->wait_for_frames();
    uint16_t *left_raw_depth = (uint16_t  *)left_dev_->get_frame_data(rs::stream::depth);
    left_color_ = (uint8_t *)left_dev_->get_frame_data(rs::stream::color);
    // close left laser, open right laser;
    left_dev_->set_option(rs::option::f200_laser_power, 0.0);
    right_dev_->set_option(rs::option::f200_laser_power, 1.0);
    // get raw depth after skip few frames
    for (int i = 0; i < 5; ++i)
      right_dev_->wait_for_frames();
    uint16_t *right_raw_depth = (uint16_t  *)right_dev_->get_frame_data(rs::stream::depth);
    // align left raw depth to left color
    Align2Other(left_raw_depth, left_depth_height_, left_depth_width_,
                left_depth_matrix_, left_depth_coeffs_,
                left_aligned_depth_, left_color_height_, left_color_width_,
                left_color_matrix_, left_color_coeffs_,
                left_rMd_, left_depth_scale_);
    // align right raw depth to left color
    Align2Other(right_raw_depth, right_depth_height_, right_depth_width_,
                right_depth_matrix_, right_depth_coeffs_,
                right_aligned_depth_, left_color_height_, left_color_width_,
                left_color_matrix_, left_color_coeffs_,
                lMr_ * right_rMd_, right_depth_scale_);
    FuseDepth(left_aligned_depth_, right_aligned_depth_, depth_);
  }
  else {
    LOG(ERROR) << "start the camera first !!!";
  }
}

void StereoRealsenseCamera::FetchColor(cv::Mat &color) {
  // Create color image
  cv::Mat color_rgb(left_color_height_,
                    left_color_width_,
                    CV_8UC3,
                    (uchar *) left_color_);
  cvtColor(color_rgb, color, cv::COLOR_BGR2RGB);
}

void StereoRealsenseCamera::FetchDepth(cv::Mat &depth) {
  // Create depth image
  cv::Mat depth_m(left_color_height_,
                   left_color_width_,
                   CV_32F,
                   depth_);
  depth_m.copyTo(depth);
}

void StereoRealsenseCamera::FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr) {
  cloud_ptr->clear();
  cloud_ptr->height = static_cast<unsigned>(left_color_height_);
  cloud_ptr->width = static_cast<unsigned>(left_color_width_);
  cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);
  // convert depth to pointcloud
  for (int dy = 0; dy < cloud_ptr->height; ++dy) {
    for (int dx = 0; dx < cloud_ptr->width; ++dx) {
      float depth = depth_[dy * cloud_ptr->width + dx];
      pcl::PointXYZ p;
      // set nan if depth missing
      if (depth <= 0) {
        p.x = numeric_limits<float>::quiet_NaN();
        p.y = numeric_limits<float>::quiet_NaN();
        p.z = numeric_limits<float>::quiet_NaN();
      } else {
        // Retrieve the 16-bit depth value and map it into a depth in meters
        float pixel[2] = {(float) dx, (float) dy};
        float point[3] = {0, 0, 0};
        Deproject(pixel, depth, left_color_matrix_, left_color_coeffs_, point);
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
      }
      cloud_ptr->points[dy * cloud_ptr->width + dx] = p;
    }
  }
}

void StereoRealsenseCamera::FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr) {
  cloud_ptr->clear();
  cloud_ptr->height = static_cast<unsigned>(left_color_height_);
  cloud_ptr->width = static_cast<unsigned>(left_color_width_);
  cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);
  // convert depth to pointcloud
  for (int dy = 0; dy < cloud_ptr->height; ++dy) {
    for (int dx = 0; dx < cloud_ptr->width; ++dx) {
      float depth = depth_[dy * cloud_ptr->width + dx];
      pcl::PointXYZRGBA p;
      // set nan if depth missing
      if (depth <= 0) {
        p.x = numeric_limits<float>::quiet_NaN();
        p.y = numeric_limits<float>::quiet_NaN();
        p.z = numeric_limits<float>::quiet_NaN();
        p.r = p.g = p.b = 0;
      } else {
        // Retrieve the 16-bit depth value and map it into a depth in meters
        float pixel[2] = {(float) dx, (float) dy};
        float point[3] = {0, 0, 0};
        Deproject(pixel, depth, left_color_matrix_, left_color_coeffs_, point);
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
        p.r = *(left_color_ + (dy * left_color_width_ + dx) * 3);
        p.g = *(left_color_ + (dy * left_color_width_ + dx) * 3 + 1);
        p.b = *(left_color_ + (dy * left_color_width_ + dx) * 3 + 2);
      }
      cloud_ptr->points[dy * cloud_ptr->width + dx] = p;
    }
  }
}

bool StereoRealsenseCamera::LoadCalibration(const std::string &cfg_file) {
  cv::FileStorage fs(cfg_root_ + "/" + cfg_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(ERROR) << "No such config file: " << cfg_file << endl;
    return false;
  }
  string left_camera_cfg, right_camera_cfg, stereo_cfg;
  fs["left_camera"] >> left_camera_cfg;
  cv::FileStorage left_camera(cfg_root_ + "/" + left_camera_cfg, cv::FileStorage::READ);
  if (!left_camera.isOpened()) {
    LOG(ERROR) << "No such config file: " << left_camera_cfg << endl;
    return false;
  }
  fs["right_camera"] >> right_camera_cfg;
  cv::FileStorage right_camera(cfg_root_ + "/" + right_camera_cfg, cv::FileStorage::READ);
  if (!right_camera.isOpened()) {
    LOG(ERROR) << "No such config file: " << right_camera_cfg << endl;
    return false;
  }
  fs["stereo"] >> stereo_cfg;
  cv::FileStorage stereo(cfg_root_ + "/" + stereo_cfg, cv::FileStorage::READ);
  if (!stereo.isOpened()) {
    LOG(ERROR) << "No such config file: " << stereo_cfg << endl;
    return false;
  }
  left_camera["r_width"] >> left_color_width_;
  left_camera["r_height"] >> left_color_height_;
  left_camera["r_intrin"] >> left_color_matrix_;
  left_camera["r_coeffs"] >> left_color_coeffs_;
  left_camera["d_width"] >> left_depth_width_;
  left_camera["d_height"] >> left_depth_height_;
  left_camera["d_intrin"] >> left_depth_matrix_;
  left_camera["d_coeffs"] >> left_depth_coeffs_;
  left_camera["dMr"] >> left_rMd_;
  left_rMd_ = left_rMd_.inv();
  right_camera["r_width"] >> right_color_width_;
  right_camera["r_height"] >> right_color_height_;
  right_camera["r_intrin"] >> right_color_matrix_;
  right_camera["r_coeffs"] >> right_color_coeffs_;
  right_camera["d_width"] >> right_depth_width_;
  right_camera["d_height"] >> right_depth_height_;
  right_camera["d_intrin"] >> right_depth_matrix_;
  right_camera["d_coeffs"] >> right_depth_coeffs_;
  right_camera["dMr"] >> right_rMd_;
  right_rMd_ = right_rMd_.inv();
  stereo["lMr"] >> lMr_;
  fs.release();
  return true;
}

bool StereoRealsenseCamera::SaveCalibration(const std::string &cfg_file) {
  LOG(WARNING) << "Not implemented";
  return true;
}

void StereoRealsenseCamera::FuseDepth(uint16_t *depth1, uint16_t *depth2, float *depth_fused) {
  int size = left_color_width_ * left_color_height_;
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < size; ++i) {
    float d1 = depth1[i] * left_depth_scale_;
    float d2 = depth2[i] * right_depth_scale_;
    if (depth1[i] == 0)
      depth_fused[i] = d2;
    else if (depth2[i] != 0) { // both depth1 and depth2 are valid
      if (fabs(d1 - d2) > 0.05)
        depth_fused[i] = 0;
      else
        depth_fused[i] = 0.5f * depth1[i] * left_depth_scale_ +  0.5f * depth2[i] * right_depth_scale_;
    }
    else
      depth_fused[i] = d1;
  }
}

}
