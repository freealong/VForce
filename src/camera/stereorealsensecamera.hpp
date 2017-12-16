//
// Created by yongqi on 17-12-13.
//

#ifndef VFORCE_STEREOREALSENSECAMERA_HPP
#define VFORCE_STEREOREALSENSECAMERA_HPP

#include "camera.hpp"

// Forward Declaration
class context;
class device;

namespace VForce {

class StereoRealsenseCamera : public Camera {
 public:
  StereoRealsenseCamera(const std::string &cfg_file = "StereoRealsenseCamera.yml", const std::string &cfg_root = ".");
  virtual bool Start();
  virtual void Stop();
  virtual void Update();
  virtual void FetchColor(cv::Mat &color);
  virtual void FetchDepth(cv::Mat &depth);
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr);
  virtual bool LoadCalibration(const std::string &cfg_file);
  virtual bool SaveCalibration(const std::string &cfg_file);

 private:
  /**
   * Fuse two depth map into one
   * @param depth1
   * @param depth2
   * @param depth_fused
   */
  void FuseDepth(uint16_t *depth1, uint16_t *depth2, float *depth_fused);

  std::shared_ptr<context> context_;
  device *left_dev_, *right_dev_;
  float *depth_;
  uint8_t *color_;
  uint16_t *left_aligned_depth_, *right_aligned_depth_; // depth aligned to color
  uint8_t *left_color_, *right_color_;
  float left_depth_scale_, right_depth_scale_;
  int left_color_width_, left_color_height_;
  int left_depth_width_, left_depth_height_;
  int right_color_width_, right_color_height_;
  int right_depth_width_, right_depth_height_;
  cv::Mat left_color_matrix_, right_color_matrix_;
  cv::Mat left_color_coeffs_, right_color_coeffs_;
  cv::Mat left_depth_matrix_, right_depth_matrix_;
  cv::Mat left_depth_coeffs_, right_depth_coeffs_;
  cv::Mat left_rMd_, right_rMd_; // color to depth transformation
  cv::Mat lMr_; // left color to right color transformation
};

}
#endif //VFORCE_STEREOREALSENSECAMERA_HPP
