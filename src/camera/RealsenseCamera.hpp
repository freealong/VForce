//
// Created by yongqi on 3/28/17.
//

#ifndef VFORCE_REALSENSECAMERA_HPP
#define VFORCE_REALSENSECAMERA_HPP

#include "Camera.hpp"

// Forward Declaration
class context;
class device;

namespace VForce {

class RealsenseCamera : public Camera {
 public:
  RealsenseCamera(bool manual_calibration = true,
                  const std::string &cfg_root = ".",
                  const std::string &cfg_file = "RealsenseCamera.yml");
  virtual bool Start();
  virtual void Stop();
  virtual void Update();
  virtual void FetchColor(cv::Mat &color);
  virtual void FetchDepth(cv::Mat &depth);
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr);
  virtual bool LoadCalibration(const std::string &cfg_file);
  virtual bool SaveCalibration(const std::string &cfg_file);

  ~RealsenseCamera() {
    if (running_)
      Stop();
  }

 private:
  // read camera params from camera flash
  void ReadCameraParams();

 private:
  std::shared_ptr<context> context_;
  device *dev_;
  bool user_calibration_;
  uint16_t *depth_image_; // depth map aligned to color
  uint8_t *color_image_;
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
#endif //VFORCE_REALSENSECAMERA_HPP
