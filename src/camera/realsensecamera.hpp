//
// Created by yongqi on 3/28/17.
//

#ifndef VFORCE_REALSENSECAMERA_HPP
#define VFORCE_REALSENSECAMERA_HPP

#include "camera.hpp"

// Forward Declaration
class context;
class device;

namespace VForce {

class RealSenseCamera : public Camera {
 public:
  RealSenseCamera(int decive_id = 0);
  virtual bool Start();
  virtual void Stop();
  virtual void Update();
  virtual void FetchColor(cv::Mat &color);
  virtual void FetchDepth(cv::Mat &depth);
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr);
  virtual bool LoadCalibration(const std::string &cfg_file);
  virtual bool SaveCalibration(const std::string &cfg_file);

  ~RealSenseCamera() {
    if (running_)
      Stop();
  }

 private:
  // read camera params from camera flash
  void ReadCameraParams();

  /**
   * Align depth image to color image coordinate
   * @param depth_image
   * @param aligned_depth_image
   */
  void Align2Color(uint16_t *depth_image, uint16_t *aligned_depth_image);

  /**
   * deproject pixel to 3d point
   * @param pixel pixel in image coordinate
   * @param depth pixel depth
   * @param intrin image intrin matrix
   * @param coeffs image coeffs
   * @param point 3d point in camera coordinate
   */
  void Deproject(float pixel[2], float depth, const cv::Mat &intrin, const cv::Mat &coeffs, float point[3]) const;

  /**
   * project 3d point to pixel
   * @param point  3d point in camera coordinate
   * @param intrin
   * @param coeffs
   * @param pixel pixel in image coordinate
   */
  void Project(float point[3], const cv::Mat &intrin, const cv::Mat &coeffs, float pixel[2]) const;

  /**
   * transform transform 3d point based on given tranformation
   * @param point1 input 3d point
   * @param extrin given tranformation matrix
   * @param point2 output 3d point
   */
  void Transform(float point1[3], const cv::Mat &extrin, float point2[3]) const;

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
