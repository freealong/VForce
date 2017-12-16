//
// Created by yongqi on 3/28/17.
//

#include <librealsense/rs.hpp>
using rs::context;
using rs::device;

#include "realsensecamera.hpp"
#include <glog/logging.h>

namespace VForce {

using namespace std;
using namespace cv;

RealSenseCamera::RealSenseCamera(int id) : user_calibration_(false) {
  // get device
  context_ = std::shared_ptr<context>(new context);
  dev_ = context_->get_device(id);
}

bool RealSenseCamera::Start() {
  if (running_)
    return true;
  // start camera
  if (dev_ == nullptr) {
    LOG(ERROR) << "device " << " is not connected" << endl;
    return false;
  }
  dev_->enable_stream(rs::stream::depth, rs::preset::best_quality);
  dev_->enable_stream(rs::stream::color, rs::preset::best_quality);
  dev_->start();
  // warm up camera
  for (int i = 0; i < 30; ++i)
    dev_->wait_for_frames();
  // get camera params.
  // If user_calibration then we have already load the camera params,
  // else we should load camera params from the camera.
  if (user_calibration_)
    depth_image_ = new uint16_t[r_width_ * r_height_];
  else
    ReadCameraParams();
  depth_scale_ = dev_->get_depth_scale();
  running_ = true;
  return true;
}

void RealSenseCamera::Stop() {
  if (running_) {
    dev_->stop();
    if (user_calibration_ && depth_image_ != nullptr)
      delete[] depth_image_;
    running_ = false;
  }
}

void RealSenseCamera::Update() {
  if (dev_->is_streaming()) {
    dev_->wait_for_frames();
    color_image_ = (uint8_t *) dev_->get_frame_data(rs::stream::color);
    if (!user_calibration_)
      depth_image_ = (uint16_t *) dev_->get_frame_data(rs::stream::depth_aligned_to_color);
    else {
      Align2Other((uint16_t *) dev_->get_frame_data(rs::stream::depth), d_height_, d_width_,
                  d_intrin_, d_coeffs_, depth_image_, r_height_, r_width_, r_intrin_, r_coeffs_,
                  rMd_, depth_scale_);
    }
  } else {
    LOG(ERROR) << "start the camera first !!!" << endl;
  }
}

void RealSenseCamera::FetchColor(cv::Mat &color) {
  // Create color image
  cv::Mat color_rgb(r_height_,
                    r_width_,
                    CV_8UC3,
                    (uchar *) color_image_);
  cvtColor(color_rgb, color, cv::COLOR_BGR2RGB);
}

void RealSenseCamera::FetchDepth(cv::Mat &depth) {
  // Create depth image
  cv::Mat depth_mm(d_height_,
                   d_width_,
                   CV_16U,
                   depth_image_);
  depth_mm.convertTo(depth, CV_32F, depth_scale_);
}

void RealSenseCamera::FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr) {
  cloud_ptr->clear();
  cloud_ptr->height = static_cast<unsigned>(d_height_);
  cloud_ptr->width = static_cast<unsigned>(d_width_);
  cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);
  // convert depth to pointcloud
  for (int dy = 0; dy < cloud_ptr->height; ++dy) {
    for (int dx = 0; dx < cloud_ptr->width; ++dx) {
      uint16_t depth_value = depth_image_[dy * cloud_ptr->width + dx];
      pcl::PointXYZ p;
      // set nan if depth missing
      if (depth_value == 0) {
        p.x = numeric_limits<float>::quiet_NaN();
        p.y = numeric_limits<float>::quiet_NaN();
        p.z = numeric_limits<float>::quiet_NaN();
      } else {
        // Retrieve the 16-bit depth value and map it into a depth in meters
        float depth_in_meters = depth_value * depth_scale_;
        float pixel[2] = {(float) dx, (float) dy};
        float point[3] = {0, 0, 0};
        Deproject(pixel, depth_in_meters, r_intrin_,r_coeffs_, point);
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
      }
      cloud_ptr->points[dy * cloud_ptr->width + dx] = p;
    }
  }
}

void RealSenseCamera::FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr) {
  cloud_ptr->clear();
  cloud_ptr->height = static_cast<unsigned>(d_height_);
  cloud_ptr->width = static_cast<unsigned>(d_width_);
  cloud_ptr->points.resize(cloud_ptr->width * cloud_ptr->height);
  // convert depth to pointcloud
  for (int dy = 0; dy < cloud_ptr->height; ++dy) {
    for (int dx = 0; dx < cloud_ptr->width; ++dx) {
      uint16_t depth_value = depth_image_[dy * cloud_ptr->width + dx];
      pcl::PointXYZRGBA p;
      // set nan if depth missing
      if (depth_value == 0) {
        p.x = numeric_limits<float>::quiet_NaN();
        p.y = numeric_limits<float>::quiet_NaN();
        p.z = numeric_limits<float>::quiet_NaN();
        p.r = p.g = p.b = 0;
      } else {
        // Retrieve the 16-bit depth value and map it into a depth in meters
        float depth_in_meters = depth_value * depth_scale_;
        float pixel[2] = {(float) dx, (float) dy};
        float point[3] = {0, 0, 0};
        Deproject(pixel, depth_in_meters, r_intrin_, r_coeffs_, point);
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
        p.r = *(color_image_ + (dy * r_width_ + dx) * 3);
        p.g = *(color_image_ + (dy * r_width_ + dx) * 3 + 1);
        p.b = *(color_image_ + (dy * r_width_ + dx) * 3 + 2);
      }
      cloud_ptr->points[dy * cloud_ptr->width + dx] = p;
    }
  }
}

bool RealSenseCamera::LoadCalibration(const std::string &cfg_file) {
  cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(ERROR) << "No Such Calibration file: " << cfg_file << endl;
    user_calibration_ = false;
    return false;
  }
  fs["r_width"] >> r_width_;
  fs["r_height"] >> r_height_;
  fs["r_intrin"] >> r_intrin_;
  fs["r_coeffs"] >> r_coeffs_;
  fs["d_width"] >> d_width_;
  fs["d_height"] >> d_height_;
  fs["d_intrin"] >> d_intrin_;
  fs["d_coeffs"] >> d_coeffs_;
  fs["dMr"] >> dMr_;
  rMd_ = dMr_.inv();
  fs.release();
  user_calibration_ = true;
  return true;
}

bool RealSenseCamera::SaveCalibration(const std::string &cfg_file) {
  cv::FileStorage fs(cfg_file, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Can't Open file: " << cfg_file << endl;
    return false;
  }
  fs << "r_width" << r_width_;
  fs << "r_height" << r_height_;
  fs << "r_intrin" << r_intrin_;
  fs << "r_coeffs" << r_coeffs_;
  fs << "d_width" << d_width_;
  fs << "d_height" << d_height_;
  fs << "d_intrin" << d_intrin_;
  fs << "d_coeffs" << d_coeffs_;
  fs << "dMr" << dMr_;
  fs.release();
  return true;
}

void RealSenseCamera::ReadCameraParams() {
  // get color intrin
  auto color_intrin = dev_->get_stream_intrinsics(rs::stream::color);
  r_width_ = color_intrin.width;
  r_height_ = color_intrin.height;
  r_intrin_.at<double>(0, 0) = color_intrin.fx;
  r_intrin_.at<double>(0, 2) = color_intrin.ppx;
  r_intrin_.at<double>(1, 1) = color_intrin.fy;
  r_intrin_.at<double>(1, 2) = color_intrin.ppy;
  r_intrin_.at<double>(0, 1) = 0;
  r_intrin_.at<double>(1, 0) = 0;
  r_intrin_.at<double>(2, 0) = 0;
  r_intrin_.at<double>(2, 1) = 0;
  r_intrin_.at<double>(2, 2) = 1;
  // get depth intrin
  auto depth_intrin = dev_->get_stream_intrinsics(rs::stream::depth);
  d_width_ = depth_intrin.width;
  d_height_ = depth_intrin.height;
  d_intrin_.at<double>(0, 0) = depth_intrin.fx;
  d_intrin_.at<double>(0, 2) = depth_intrin.ppx;
  d_intrin_.at<double>(1, 1) = depth_intrin.fy;
  d_intrin_.at<double>(1, 2) = depth_intrin.ppy;
  d_intrin_.at<double>(0, 1) = 0;
  d_intrin_.at<double>(1, 0) = 0;
  d_intrin_.at<double>(2, 0) = 0;
  d_intrin_.at<double>(2, 1) = 0;
  d_intrin_.at<double>(2, 2) = 1;
  // get color coeffs
  r_coeffs_.at<double>(0, 0) = color_intrin.coeffs[0];
  r_coeffs_.at<double>(0, 1) = color_intrin.coeffs[1];
  r_coeffs_.at<double>(0, 2) = color_intrin.coeffs[2];
  r_coeffs_.at<double>(0, 3) = color_intrin.coeffs[3];
  r_coeffs_.at<double>(0, 4) = color_intrin.coeffs[4];
  // get depth coeffs
  d_coeffs_.at<double>(0, 0) = depth_intrin.coeffs[0];
  d_coeffs_.at<double>(0, 1) = depth_intrin.coeffs[1];
  d_coeffs_.at<double>(0, 2) = depth_intrin.coeffs[2];
  d_coeffs_.at<double>(0, 3) = depth_intrin.coeffs[3];
  d_coeffs_.at<double>(0, 4) = depth_intrin.coeffs[4];
  auto depth_to_color = dev_->get_extrinsics(rs::stream::depth, rs::stream::color);
  float *d2c_rot = depth_to_color.rotation;
  float *d2c_tra = depth_to_color.translation;
  /** @NOTICE dMr means the matrix transforms depth coordinate to color coordinate
      depth_to_color means the matrix transforms point coordinate from depth to color;
  **/
  rMd_.at<double>(3, 0) = rMd_.at<double>(3, 1) = rMd_.at<double>(3, 2) = 0;
  rMd_.at<double>(3, 3) = 1;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      rMd_.at<double>(r, c) = d2c_rot[c * 3 + r];
    }
    rMd_.at<double>(r, 3) = d2c_tra[r];
  }
  dMr_ = rMd_.inv();
}

}
