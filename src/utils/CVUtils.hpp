//
// Created by yongqi on 17-12-7.
//

#ifndef VFORCE_IMAGEUTILS_HPP
#define VFORCE_IMAGEUTILS_HPP

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

namespace VForce {
namespace Utils {

/**
 * generate random color
 * @return color scalar
 */
cv::Scalar random_color();

/**
 * apply mask to image
 * @param img rgb image
 * @param mask uchar mat
 * @param color random rgb color
 */
void apply_mask(cv::Mat &img, const cv::Mat &mask, const cv::Scalar &color);

/**
 * Eigen Matrix cout format;
 * [ 1,  2,  3,  4,
 *   5,  6,  7,  8,
 *   9, 10, 11, 12,
 *  13, 14, 15, 16]
 */
static Eigen::IOFormat IOF(0, 0, ",", ",\n", "", "", "[", "]");

/**
 * Read Eigen Matrix from yaml file
 * @tparam T
 * @tparam Row
 * @tparam Col
 * @param node
 * @param tf
 */
template<typename T, int Row, int Col>
static inline void operator>>(const cv::FileNode &node, Eigen::Matrix<T, Row, Col> &tf) {
  if (node.empty())
    tf = Eigen::Matrix<T, Row, Col>::Identity();
  else {
    auto it = node.begin();
    for (auto y = 0; y < Row; ++y)
      for (auto x = 0; x < Col; ++x)
        tf(y, x) = static_cast<T>(*it++);
  }
}

/**
 * write depth in meter into visualized png picture
 * @param depth_file save file name
 * @param depth_meter depth in meter Mat
 */
void write_depth(const std::string &depth_file, const cv::Mat &depth_meter);

/**
 * read visualized png depth map into depth in meter
 * @param depth_file read file name
 * @return depth in meter mat
 */
cv::Mat read_depth(const std::string &depth_file);

}
}

#endif //VFORCE_IMAGEUTILS_HPP
