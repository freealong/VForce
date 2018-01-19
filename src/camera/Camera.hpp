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
  static void Align2Other(uint16_t *depth, int d_height, int d_width, const cv::Mat &Kd, const cv::Mat &Dd,
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
  static void Deproject(float pixel[2], float depth, const cv::Mat &intrin, const cv::Mat &coeffs, float point[3]);

  /**
   * project 3d point to pixel
   * @param point  3d point in camera coordinate
   * @param intrin
   * @param coeffs
   * @param pixel pixel in image coordinate
   */
  static void Project(float point[3], const cv::Mat &intrin, const cv::Mat &coeffs, float pixel[2]);

  /**
   * transform transform 3d point based on given tranformation
   * @param point1 input 3d point
   * @param extrin given tranformation matrix
   * @param point2 output 3d point
   */
  static void Transform(float point1[3], const cv::Mat &extrin, float point2[3]);

  /**
   * Reproject disparity to depth
   * @param disparity disparity map
   * @param w disparity map width
   * @param h disparity map height
   * @param Q Stereo Reprojection Matrix
   * @param depth Output depth map, should already allocated with size w*h
   */
  static void Reproject(const float *disparity, int w, int h, const cv::Mat &Q, float *depth);

  /**
   * Convert depth to HHA(disparity, height, angle)
   * @param depth input aligned depth map, CV_32FC1
   * @param K color camera matrix, CV_64FC1 with 3*3 size
   * @param D color distcoeffs, CV_64FC1 with 1*5 size
   * @param cMw camera to world transformation matrix, CV_64C1 with 4*4 size
   * @param wMb world to box transformation matrix, CV_64C1 with 4*4 size
   * @param depth_floor min depth from camera base
   * @param depth_ceil max depth
   * @param height_floor min height from world base
   * @param height_ceil max height
   * @return HHA map, CV_8UC3 with depht.size()
   */
  static cv::Mat Depth2HHA(const cv::Mat &depth, const cv::Mat &K, const cv::Mat &D,
                           const cv::Mat &cMw, const cv::Mat &wMb,
                           float depth_floor, float depth_ceil,
                           float height_floor, float height_ceil);

  /**
   * Convert cloud to HHA
   * @param cloud
   * @param cMw
   * @param wMb
   * @param depth_floor
   * @param depth_ceil
   * @param height_floor
   * @param height_ceil
   * @return
   */
  static cv::Mat Cloud2HHA(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                           const cv::Mat &cMw, const cv::Mat &wMb,
                           float depth_floor, float depth_ceil,
                           float height_floor, float height_ceil);
  /**
   * Convert depth to point cloud
   * @param depth
   * @param K
   * @param D
   * @return
   */
  static pcl::PointCloud<pcl::PointXYZ>::Ptr Depth2Cloud(const cv::Mat &depth, const cv::Mat &K, const cv::Mat &D);

  /**
   * Compute cloud normals based on a region of points
   * @param cloud
   * @param R region radius, region length is 2*R + 1
   * @return cloud of normals
   */
  static pcl::PointCloud<pcl::Normal>::Ptr
  ComputeNormalSquareSupport(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int R);

 protected:
  bool running_;
  std::string cfg_root_, cfg_file_;
};

}
#endif //VFORCE_CAMERA_HPP
