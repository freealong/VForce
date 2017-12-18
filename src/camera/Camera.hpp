//
// Created by yongqi on 12/2/16.
//

#ifndef VFORCE_CAMERA_HPP
#define VFORCE_CAMERA_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace VForce {

class Camera {
 public:
  Camera(const std::string& cfg_root = ".", const std::string &cfg_file = "Camera.yml") :
      running_(false), cfg_root_(cfg_root), cfg_file_(cfg_file) {}
  virtual bool Start() = 0;
  virtual void Stop() = 0;
  /**
   * update frames from cameras
   */
  virtual void Update() = 0;
  virtual void FetchColor(cv::Mat &color) = 0;
  virtual void FetchDepth(cv::Mat &depth) = 0;
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr) = 0;
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr) = 0;
  virtual bool LoadCalibration(const std::string &cfg_file) = 0;
  virtual bool SaveCalibration(const std::string &cfg_file) = 0;
  virtual ~Camera() {}

  // @TODO: make sure z in reasonable range

 protected:
  /**
   * Align depth to other image coordinate
   * @param depth raw depth image
   * @param depth_size
   * @param Kd depth image camera matrix
   * @param Dd depth image camera coeffs
   * @param other aligned depth
   * @param other_size
   * @param Ko other image camera matrix
   * @param Do other image camera coeffs
   * @param oMd transformation from other to depth
   */
  void Align2Other(uint16_t *depth, int d_height, int d_width, const cv::Mat &Kd, const cv::Mat &Dd,
                   uint16_t *other, int o_height, int o_width, const cv::Mat &Ko, const cv::Mat &Do,
                   const cv::Mat &oMd, float depth_scale);

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

  bool running_;
  std::string cfg_root_, cfg_file_;
};

}
#endif //VFORCE_CAMERA_HPP
