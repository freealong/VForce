//
// Created by yongqi on 17-12-4.
//

#ifndef VFORCE_POINTCLOUDUTILS_HPP
#define VFORCE_POINTCLOUDUTILS_HPP

#include <pcl/pcl_config.h>
#if PCL_VERSION_COMPARE(<, 1, 8, 0)
#include <pcl/keypoints/uniform_sampling.h>
#else
#include <pcl/filters/uniform_sampling.h>
#endif
#include <opencv2/opencv.hpp>

namespace VForce {
namespace Utils {

using namespace std;
using namespace pcl;

/**
 * Uniform Sampling Point Cloud
 * @tparam PointT point type
 * @param cloud input point cloud
 * @param keypoints output sampled point cloud
 * @param radius sampling radius
 */
template<typename PointT>
void sampling_cloud(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud,
                    boost::shared_ptr<pcl::PointCloud<PointT>> &keypoints,
                    float radius) {
  UniformSampling<PointT> uniform_sampling;
  uniform_sampling.setInputCloud(cloud);
  uniform_sampling.setRadiusSearch(radius);
#if PCL_VERSION_COMPARE(<, 1, 8, 0)
  PointCloud<int> keypointsIndices;
  uniform_sampling.compute(keypointsIndices);
  copyPointCloud(*cloud, keypointsIndices.points, *keypoints);
#else
  uniform_sampling.filter(*keypoints);
#endif
}

/**
 * Calculate Point Cloud Center
 * @tparam PointT point type
 * @param cloud input point cloud
 * @return point cloud center
 */
template<typename PointT>
pcl::PointXYZ calculate_cloud_center(const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud) {
  auto points = cloud->points;
  PointXYZ mean_p(0, 0, 0);
  // @TODO using openmp
//#pragma omp parallel for reduction(+:mean_p)
  for (int i = 0; i < points.size(); ++i) {
    mean_p.x += points[i].x;
    mean_p.y += points[i].y;
    mean_p.z += points[i].z;
  }
  double size = static_cast<double>(points.size());
  mean_p.x /= size;
  mean_p.y /= size;
  mean_p.z /= size;
  return mean_p;
}

/**
 * Shift Point Cloud
 * @tparam PointT point type
 * @param cloud point cloud will be shifted
 * @param shift shift distance
 */
template<typename PointT>
void shift_cloud(boost::shared_ptr<pcl::PointCloud<PointT>> &cloud, const PointT &shift) {
#pragma omp parallel for
  for (int i = 0; i < cloud->size(); ++i) {
    auto &p = cloud->at(i);
    p.x += shift.x;
    p.y += shift.y;
    p.z += shift.z;
  }
}

/**
 * rect out the target point cloud based on given rect and mask
 * @tparam PointIT
 * @tparam PointOT
 * @param cloud_in organized cloud from camera
 * @param rect target rect
 * @param cloud_out unorganized target cloud inside the rect
 */
template<typename PointIT, typename PointOT>
void rect_cloud(const boost::shared_ptr<pcl::PointCloud<PointIT>> &cloud_in,
                const cv::Rect &rect,
                boost::shared_ptr<pcl::PointCloud<PointOT>> &cloud_out) {
  cloud_out->clear();
  for (int y = rect.y; y < rect.y + rect.height; ++y) {
    for (int x = rect.x; x < rect.x + rect.width; ++x) {
      auto point_in = cloud_in->at(x, y);
      if (!std::isnan(point_in.z)) {
        PointOT point_out;
        pcl::copyPoint(point_in, point_out);
        cloud_out->push_back(point_out);
      }
    } // for (int x ...
  } // for (int y ...
}

/**
 * mask out the target point cloud based on given rect and mask
 * @tparam PointIT
 * @tparam PointOT
 * @param cloud_in organized cloud from camera
 * @param rect target rect
 * @param rect_mask target mask inside rect
 * @param cloud_out unorganized target cloud correspond to the mask
 */
template<typename PointIT, typename PointOT>
void mask_cloud(const boost::shared_ptr<pcl::PointCloud<PointIT>> &cloud_in,
                const cv::Rect &rect,
                const cv::Mat &rect_mask,
                boost::shared_ptr<pcl::PointCloud<PointOT>> &cloud_out) {
  cloud_out->clear();
  for (int y = rect.y; y < rect.y + rect.height; ++y) {
    for (int x = rect.x; x < rect.x + rect.width; ++x) {
      if (rect_mask.at<uchar>(y - rect.y, x - rect.x) > 0) {
        PointIT &point_in = cloud_in->at(x, y);
        if (!std::isnan(point_in.z)) {
          PointOT point_out;
          pcl::copyPoint(point_in, point_out);
          cloud_out->push_back(point_out);
        }
      } // if mask
    } // for x
  } // for y
}

/**
 * transform point of vector
 * @tparam PointT
 * @param points_in
 * @param point_out
 * @param tf
 */
template <typename PointT>
void transform_points(const std::vector<PointT> &points_in, std::vector<PointT> &point_out, Eigen::Matrix4f tf) {
  point_out.clear();
  for (auto &p : points_in) {
    auto p_out = p;
    p_out.x = tf(0, 0) * p.x + tf(0, 1) * p.y + tf(0, 2) * p.z + tf(0, 3);
    p_out.y = tf(1, 0) * p.x + tf(1, 1) * p.y + tf(1, 2) * p.z + tf(1, 3);
    p_out.z = tf(2, 0) * p.x + tf(2, 1) * p.y + tf(2, 2) * p.z + tf(2, 3);
    point_out.emplace_back(p_out);
  }
}

}
}
#endif //VFORCE_POINTCLOUDUTILS_HPP
