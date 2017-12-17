//
// Created by yongqi on 17-12-11.
//

#include "CVUtils.hpp"

namespace VForce {
namespace Utils {

cv::Scalar random_color() {
  static cv::RNG rng(1);
  int icolor = (unsigned) rng;
  return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}

void apply_mask(cv::Mat &img, const cv::Mat &mask, const cv::Scalar &color) {
  assert(img.size == mask.size);
  float alpha = 0.5;
  for (int y = 0; y < mask.rows; ++y)
    for (int x = 0; x < mask.cols; ++x) {
      if (mask.at<uchar>(y, x) > 0) {
        auto &vec = img.at<cv::Vec3b>(y, x);
        vec[0] = static_cast<uchar>(alpha * color[0] + (1 - alpha) * vec[0]);
        vec[1] = static_cast<uchar>(alpha * color[1] + (1 - alpha) * vec[1]);
        vec[2] = static_cast<uchar>(alpha * color[2] + (1 - alpha) * vec[2]);
      }
    }
}

void write_depth(const std::string &depth_file, const cv::Mat &depth_meter) {
  cv::Mat depth_mat(depth_meter.size(), CV_16UC1);
  for (int y = 0; y < depth_mat.rows; y++)
    for (int x = 0; x < depth_mat.cols; x++) {
      unsigned short depth_short = static_cast<unsigned short>(std::round(depth_meter.at<float>(y, x) * 10000.));
      depth_short = (depth_short >> 13 | depth_short << 3);
      depth_mat.at<unsigned short>(y, x) = depth_short;
    }
  std::vector<int> compression_params;
  compression_params.emplace_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.emplace_back(9);
  cv::imwrite(depth_file, depth_mat, compression_params);
}

cv::Mat read_depth(const std::string &depth_file) {
  cv::Mat depth_mat = cv::imread(depth_file, CV_16UC1);
  cv::Mat depth_meter(depth_mat.size(), CV_32F);
  for (int y = 0; y < depth_mat.rows; ++y)
    for (int x = 0; x < depth_mat.cols; ++x) {
      auto depth_short = depth_mat.at<unsigned short>(y, x);
      depth_short = (depth_short << 13 | depth_short >> 3);
      depth_meter.at<float>(y, x) = depth_short / 10000.f;
    }
  return depth_meter;
}

}
}