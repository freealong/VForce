//
// Created by yongqi on 17-5-22.
//

#ifndef VFORCE_VIRTUALCAMERA_HPP
#define VFORCE_VIRTUALCAMERA_HPP

#include "camera.hpp"

namespace VForce {

class VirtualCamera : public Camera {
 public:
  VirtualCamera(std::string p = ".", std::string type = "Struct_light");
  virtual bool Start();
  virtual void Stop();
  virtual void Update();
  virtual void FetchColor(cv::Mat &color);
  virtual void FetchDepth(cv::Mat &depth);
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);
  virtual void FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr);
  virtual bool LoadCalibration(const std::string &cfg_file);
  virtual bool SaveCalibration(const std::string &cfg_file);

  virtual ~VirtualCamera() {
    if (running_)
      Stop();
  }

 private:
  std::string _path, _camera_type;
  int _frame_id;
  cv::Mat _color;
  cv::Mat _depth;
  cv::Mat _r_intrin{3, 3, CV_64F};
  cv::Mat _r_coeffs{1, 5, CV_64F};
  cv::Mat _d_intrin{3, 3, CV_64F};
  cv::Mat _d_coeffs{1, 5, CV_64F};
  cv::Mat _dMr{4, 4, CV_64F};
  cv::Mat _rMd{4, 4, CV_64F};
  cv::Mat _depth_reprojection{4, 4, CV_64F};
};

}
#endif //VFORCE_VIRTUALCAMERA_HPP
