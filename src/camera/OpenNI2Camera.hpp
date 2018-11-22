//
// Created by yongqi on 3/28/17.
//

#ifndef VFORCE_OpenNI2CAMERA_HPP
#define VFORCE_OpenNI2CAMERA_HPP

#include "Camera.hpp"
#include <pcl/io/openni2_grabber.h>

namespace VForce {

class OpenNI2Camera : public Camera {
 public:
  explicit OpenNI2Camera(std::string cfg_root, std::string cfg_file, bool user_calibration = false);
  bool Start() override;
  void Stop() override;
  void Update() override;
  void FetchColor(cv::Mat &color) override;
  void FetchDepth(cv::Mat &depth) override;
  void FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr) override;
  void FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr) override;
  bool LoadCalibration(const std::string &cfg_file) override;
  bool SaveCalibration(const std::string &cfg_file) override;

  ~OpenNI2Camera() override {
    Stop();
  }

 private:
  void cloud_callback(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>> &cloud) {
    boost::mutex::scoped_lock lock(cloud_mutex_);
    callback_cloud_ = cloud;
  }

  void image_callback(const boost::shared_ptr<pcl::io::openni2::Image> &image) {
    boost::mutex::scoped_lock lock(image_mutex_);
    callback_image_ = image;
  }

  void depth_callback(const boost::shared_ptr<pcl::io::openni2::DepthImage> &depth) {
    boost::mutex::scoped_lock lock(depth_mutex_);
    callback_depth_ = depth;
  }

 private:
  std::shared_ptr<pcl::io::OpenNI2Grabber> grabber_;
  boost::mutex cloud_mutex_;
  boost::mutex image_mutex_;
  boost::mutex depth_mutex_;
  bool user_calibration_;
  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>> callback_cloud_;
  boost::shared_ptr<pcl::io::Image> callback_image_;
  boost::shared_ptr<pcl::io::DepthImage> callback_depth_;
  boost::signals2::connection cloud_connection_;
  boost::signals2::connection image_connection_;
  boost::signals2::connection depth_connection_;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>> cloud_;
  cv::Mat image_;
  cv::Mat depth_;
  float depth_scale_; // scale factor from camera raw depth to depth in meter
  int r_width_, r_height_;
  int d_width_, d_height_;
  cv::Mat r_intrin_{3, 3, CV_64F};
  cv::Mat r_coeffs_{1, 5, CV_64F};
  cv::Mat d_intrin_{3, 3, CV_64F};
  cv::Mat d_coeffs_{1, 5, CV_64F};
  cv::Mat dMr_{4, 4, CV_64F};
  cv::Mat rMd_{4, 4, CV_64F};
};

}
#endif //VFORCE_OpenNI2CAMERA_HPP
