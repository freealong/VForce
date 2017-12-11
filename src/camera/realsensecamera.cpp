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
  // read camera params
  ReadCameraParams();
  depth_scale_ = dev_->get_depth_scale();
  if (user_calibration_)
    depth_image_ = new uint16_t[r_width_ * r_height_];
  running_ = true;
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
      Align2Color((uint16_t *) dev_->get_frame_data(rs::stream::depth), depth_image_);
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
  cloud_ptr->height = d_height_;
  cloud_ptr->width = d_width_;
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
  cloud_ptr->height = d_height_;
  cloud_ptr->width = d_width_;
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
  fs["r_intrin"] >> r_intrin_;
  fs["r_coeffs"] >> r_coeffs_;
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
  fs << "r_intrin" << r_intrin_;
  fs << "r_coeffs" << r_coeffs_;
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

void RealSenseCamera::Align2Color(uint16_t *depth_image, uint16_t *aligned_depth_image) {
  assert(depth_image != nullptr);
  assert(aligned_depth_image != nullptr);
  assert(depth_image != aligned_depth_image);
  int size = r_width_ * r_height_;
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < size; ++i)
    aligned_depth_image[i] = 0;
#pragma omp parallel for schedule(dynamic)
  for (int dy = 0; dy < d_height_; ++dy) {
    int dxy = dy * d_width_;
    for (int dx = 0; dx < d_width_; ++dx, ++dxy) {
      if (float depth = depth_image[dxy]) {
        // convert depth pixel(u,v) to depth point(x,y,z)
        // Map the top-left corner of the depth pixel onto the other image
        float depth_in_meter = depth * depth_scale_;
        float depth_pixel[2] = {dx - 0.5f, dy - 0.5f}, depth_point[3], other_point[3], other_pixel[2];
        Deproject(depth_pixel, depth_in_meter, d_intrin_, d_coeffs_, depth_point);
        Transform(depth_point, rMd_, other_point);
        Project(other_point, r_intrin_, r_coeffs_, other_pixel);
        const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
        const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

        // Map the bottom-right corner of the depth pixel onto the other image
        depth_pixel[0] = dx + 0.5f;
        depth_pixel[1] = dy + 0.5f;
        Deproject(depth_pixel, depth_in_meter, d_intrin_, d_coeffs_, depth_point);
        Transform(depth_point, rMd_, other_point);
        Project(other_point, r_intrin_, r_coeffs_, other_pixel);
        const int other_x1 = static_cast<int>(other_pixel[0] + 0.5f);
        const int other_y1 = static_cast<int>(other_pixel[1] + 0.5f);

        if (other_x0 < 0 || other_y0 < 0 || other_x1 >= r_width_ || other_y1 >= r_height_)
          continue;

        // Transfer between the depth pixels and the pixels inside the rectangle on the other image
        for (int y = other_y0; y <= other_y1; ++y)
          for (int x = other_x0; x <= other_x1; ++x)
            aligned_depth_image[y * r_width_ + x] = depth_image[dxy];
      }
    }
  }
}

void RealSenseCamera::Deproject(float *pixel,
                                float depth,
                                const cv::Mat &intrin,
                                const cv::Mat &coeffs,
                                float *point) const {
  float x = (pixel[0] - intrin.at<double>(0, 2)) / intrin.at<double>(0, 0);
  float y = (pixel[1] - intrin.at<double>(1, 2)) / intrin.at<double>(1, 1);
  {
    float r2 = x * x + y * y;
    float f = 1 + coeffs.at<double>(0) * r2 + coeffs.at<double>(1) * r2 * r2 + coeffs.at<double>(4) * r2 * r2 * r2;
    float ux = x * f + 2 * coeffs.at<double>(2) * x * y + coeffs.at<double>(3) * (r2 + 2 * x * x);
    float uy = y * f + 2 * coeffs.at<double>(3) * x * y + coeffs.at<double>(2) * (r2 + 2 * y * y);
    x = ux;
    y = uy;
  }
  point[0] = depth * x;
  point[1] = depth * y;
  point[2] = depth;
}

void RealSenseCamera::Project(float *point, const cv::Mat &intrin, const cv::Mat &coeffs, float *pixel) const {
  float x = point[0] / point[2], y = point[1] / point[2];
  {
    float r2 = x * x + y * y;
    float f = 1 + coeffs.at<double>(0) * r2 + coeffs.at<double>(1) * r2 * r2 + coeffs.at<double>(4) * r2 * r2 * r2;
    x *= f;
    y *= f;
    float dx = x * f + 2 * coeffs.at<double>(2) * x * y + coeffs.at<double>(3) * (r2 + 2 * x * x);
    float dy = y * f + 2 * coeffs.at<double>(3) * x * y + coeffs.at<double>(2) * (r2 + 2 * y * y);
    x = dx;
    y = dy;
  }
  pixel[0] = x * intrin.at<double>(0, 0) + intrin.at<double>(0, 2);
  pixel[1] = y * intrin.at<double>(1, 1) + intrin.at<double>(1, 2);
}

void RealSenseCamera::Transform(float *point1, const cv::Mat &extrin, float *point2) const {
  point2[0] = extrin.at<double>(0, 0) * point1[0] + extrin.at<double>(0, 1) * point1[1] +
      extrin.at<double>(0, 2) * point1[2] + extrin.at<double>(0, 3);
  point2[1] = extrin.at<double>(1, 0) * point1[0] + extrin.at<double>(1, 1) * point1[1] +
      extrin.at<double>(1, 2) * point1[2] + extrin.at<double>(1, 3);
  point2[2] = extrin.at<double>(2, 0) * point1[0] + extrin.at<double>(2, 1) * point1[1] +
      extrin.at<double>(2, 2) * point1[2] + extrin.at<double>(2, 3);
}

}
