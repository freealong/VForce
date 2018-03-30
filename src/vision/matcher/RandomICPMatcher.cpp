//
// Created by yongqi on 17-12-4.
//

#include "RandomICPMatcher.hpp"
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include "utils/PointCloudUtils.hpp"

namespace VForce {

using namespace std;

RandomICPMatcher::RandomICPMatcher(const string &cfg_root, const string &cfg_file) :
    Matcher(cfg_root, cfg_file),
    uniform_radius_(0.001),
    divide_num_(1) {
  LoadConfig(cfg_root + "/" + cfg_file);
}

bool RandomICPMatcher::LoadConfig(const std::string &cfg_file) {
  init_ = false;
  // open cfg file
  LOG(INFO) << "Load config file from: " << cfg_file;
  cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Open config file failed: " << cfg_file;
    return false;
  }
  // load cfg
  fs["uniform_radius"] >> uniform_radius_;
  DLOG(INFO) << "uniform_radius: " << uniform_radius_;
  fs["divide_num"] >> divide_num_;
  DLOG(INFO) << "divide_num_: " << divide_num_;
  fs["iter_num"] >> iter_num_;
  DLOG(INFO) << "iter_num_: " << iter_num_;
  init_ = true;
  return true;
}

double RandomICPMatcher::EstimatePose(const PointTCloudPtr &model, const float* model_size,
                                      const PointTCloudPtr &target,
                                      Eigen::Matrix4f &tf, PointTCloudPtr &final) {
  if (!init_) {
    LOG(ERROR) << "Matcher not initialized";
    return std::numeric_limits<double>::max();
  }
  if (target->empty()) {
    return 1;
  }
  PointTCloudPtr sampled_target(new PointTCloud);
  Utils::sampling_cloud(target, sampled_target, uniform_radius_);
  final = PointTCloudPtr(new PointTCloud);
#ifdef DEBUG
  pcl::io::savePCDFile("debug_target.pcd", *sampled_target);
#endif
  pcl::PointXYZ center = Utils::calculate_cloud_center(sampled_target);
  // shift sampled_target to origin
  Utils::shift_cloud(sampled_target, {-center.x, -center.y, -center.z});

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(0.2);
//  icp.setRANSACOutlierRejectionThreshold (0.05);
  icp.setTransformationEpsilon(1e-6);
//  icp.setEuclideanFitnessEpsilon(0.00001);
  icp.setMaximumIterations(300);
  icp.setInputSource(model);
  icp.setInputTarget(sampled_target);
  // find the best initial tf
  Eigen::Matrix4f best_initial_tf;
  double min_error = std::numeric_limits<double>::max();
  Eigen::Matrix4f guess_tf = Eigen::Matrix4f::Identity();
  float phi;
  // @TODO: using openmp
  for (int i = 0; i < divide_num_; ++i) {
    phi = static_cast<float>(i * 2 * M_PI / 12);
    guess_tf(0, 0) = cos(phi);
    guess_tf(0, 1) = -sin(phi);
    guess_tf(1, 0) = sin(phi);
    guess_tf(1, 1) = cos(phi);
    icp.align(*final, guess_tf);
    auto error = icp.getFitnessScore();
    if (error < min_error) {
      best_initial_tf = icp.getFinalTransformation();
      min_error = error;
    }
  }
  // make sure transformed model's z points to the origin of the target
  Eigen::Vector3f oz(0, 0, 1);
  Eigen::Vector3f transformed_oz = best_initial_tf.block<3, 3>(0, 0) * oz;
  if (transformed_oz(2) < 0) {
    DLOG(INFO) << "Rotate 180° around aixs y" << endl;
    Eigen::AngleAxisf rotation(M_PI, Eigen::Vector3f{0, 1, 0});
    Eigen::Matrix4f rot_tf = Eigen::Matrix4f::Identity();
    rot_tf.block<3, 3>(0, 0) = rotation.toRotationMatrix();
    rot_tf(2, 3) = model_size[2];
    best_initial_tf *= rot_tf;
  }
  // run icp again based on best initial tf
  tf = best_initial_tf;
  icp.setMaxCorrespondenceDistance(0.01);
  icp.setTransformationEpsilon(1e-6);
  icp.setMaximumIterations(2);
  transformPointCloud(*model, *final, tf);
  Eigen::Matrix4f prev = Eigen::Matrix4f::Identity();
  for (int i = 0; i < iter_num_; ++i) {
    icp.setInputSource(final);
    icp.align(*final);
    if (fabs((icp.getLastIncrementalTransformation() - prev).sum() < icp.getTransformationEpsilon()))
      icp.setMaxCorrespondenceDistance(icp.getMaxCorrespondenceDistance() - 0.001);
    prev = icp.getLastIncrementalTransformation();
    tf = icp.getFinalTransformation() * tf;
  }
  tf(0, 3) += center.x;
  tf(1, 3) += center.y;
  tf(2, 3) += center.z;
  pcl::transformPointCloud(*model, *final, tf);
  Utils::shift_cloud(sampled_target, center);
  return CalculateMatchError(final, sampled_target);
}

}
