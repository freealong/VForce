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
  Camera() : running_(false) {}
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
  bool running_;
};

}
#endif //VFORCE_CAMERA_HPP
