//
// Created by yongqi on 3/28/17.
//

#include "OpenNI2Camera.hpp"
#include <glog/logging.h>

namespace VForce {

using namespace std;
using namespace cv;

OpenNI2Camera::OpenNI2Camera(string cfg_root, string cfg_file, bool user_calibration) :
    Camera(std::move(cfg_root), std::move(cfg_file)),
    user_calibration_(user_calibration) {
  FileStorage fs(cfg_root_ + '/' + cfg_file_, FileStorage::READ);
  std::string id;
  fs["id"] >> id;
  grabber_ = std::make_shared<pcl::io::OpenNI2Grabber>(id);
  if (user_calibration) {
    // @TODO: support use user calibration params
    LOG(ERROR) << "using user's calibration params is not implement.";
  }
}

bool OpenNI2Camera::Start() {
  if (grabber_->isRunning()) {
    return true;
  }
  try {
    boost::function<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>> &)> cloud_cb =
        boost::bind(&OpenNI2Camera::cloud_callback, this, _1);
    cloud_connection_ = grabber_->registerCallback(cloud_cb);
    boost::function<void(const boost::shared_ptr<pcl::io::openni2::Image> &)> image_cb =
        boost::bind(&OpenNI2Camera::image_callback, this, _1);
    image_connection_ = grabber_->registerCallback(image_cb);
    boost::function<void(const boost::shared_ptr<pcl::io::openni2::DepthImage> &)> depth_cb =
        boost::bind(&OpenNI2Camera::depth_callback, this, _1);
    depth_connection_ = grabber_->registerCallback(depth_cb);
    grabber_->start();
    // make sure callback functions have been called
    while (!(callback_cloud_ and callback_image_ and callback_depth_)) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
    // Allocate space
    cloud_ =
        boost::make_shared < pcl::PointCloud < pcl::PointXYZRGBA >> (callback_cloud_->width, callback_cloud_->height);
    cloud_->header = callback_cloud_->header;
    cloud_->width = callback_cloud_->width;
    cloud_->height = callback_cloud_->height;
    cloud_->is_dense = callback_cloud_->is_dense;
    cloud_->sensor_orientation_ = callback_cloud_->sensor_orientation_;
    cloud_->sensor_origin_ = callback_cloud_->sensor_origin_;
    image_ = cv::Mat(callback_image_->getHeight(), callback_image_->getWidth(), CV_8UC3);
    depth_ = cv::Mat(callback_depth_->getHeight(), callback_depth_->getWidth(), CV_32F);
    return true;
  } catch (...) {
    LOG(ERROR) << "start camera failed.";
    return false;
  }
}

void OpenNI2Camera::Stop() {
  if (grabber_->isRunning()) {
    grabber_->stop();
    cloud_connection_.disconnect();
    image_connection_.disconnect();
    depth_connection_.disconnect();
  }
}

void OpenNI2Camera::Update() {
  if (grabber_->isRunning()) {
    cloud_mutex_.lock();
    memcpy(&cloud_->points[0], &callback_cloud_->points[0], callback_cloud_->points.size() * sizeof(pcl::PointXYZRGBA));
    cloud_mutex_.unlock();
    image_mutex_.lock();
    callback_image_->fillRGB(static_cast<unsigned>(image_.cols), static_cast<unsigned>(image_.rows),
                             image_.data, static_cast<unsigned>(image_.step));
    image_mutex_.unlock();
    depth_mutex_.lock();
    callback_depth_->fillDepthImage(static_cast<unsigned>(depth_.cols), static_cast<unsigned>(depth_.rows),
                                    reinterpret_cast<float *>(depth_.data), static_cast<unsigned>(depth_.step));
    depth_mutex_.unlock();
    cvtColor(image_, image_, cv::COLOR_BGR2RGB);
  } else {
    LOG(ERROR) << "start the camera first !!!" << endl;
  }
}

void OpenNI2Camera::FetchColor(cv::Mat &color) {
  color = image_;
}

void OpenNI2Camera::FetchDepth(cv::Mat &depth) {
  depth = depth_;
}

void OpenNI2Camera::FetchPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr) {
  // @TODO: fetch xyz cloud
  LOG(ERROR) << "fetch xyz point cloud is not implement.";
}

void OpenNI2Camera::FetchPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_ptr) {
  cloud_ptr = cloud_;
}

bool OpenNI2Camera::LoadCalibration(const std::string &cfg_file) {
  cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(ERROR) << "No Such Calibration file: " << cfg_file << endl;
    user_calibration_ = false;
    return false;
  }
  fs["r_width"] >> r_width_;
  fs["r_height"] >> r_height_;
  fs["r_intrin"] >> r_intrin_;
  fs["r_coeffs"] >> r_coeffs_;
  fs["d_width"] >> d_width_;
  fs["d_height"] >> d_height_;
  fs["d_intrin"] >> d_intrin_;
  fs["d_coeffs"] >> d_coeffs_;
  fs["dMr"] >> dMr_;
  rMd_ = dMr_.inv();
  fs.release();
  user_calibration_ = true;
  return true;
}

bool OpenNI2Camera::SaveCalibration(const std::string &cfg_file) {
  cv::FileStorage fs(cfg_file, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Can't Open file: " << cfg_file << endl;
    return false;
  }
  fs << "r_width" << r_width_;
  fs << "r_height" << r_height_;
  fs << "r_intrin" << r_intrin_;
  fs << "r_coeffs" << r_coeffs_;
  fs << "d_width" << d_width_;
  fs << "d_height" << d_height_;
  fs << "d_intrin" << d_intrin_;
  fs << "d_coeffs" << d_coeffs_;
  fs << "dMr" << dMr_;
  fs.release();
  return true;
}

}
