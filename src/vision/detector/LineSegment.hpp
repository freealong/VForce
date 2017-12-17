//
// Created by yongqi on 17-12-7.
//

#ifndef BING_PICKING_LINESEGMENT_HPP
#define BING_PICKING_LINESEGMENT_HPP

#include <opencv2/opencv.hpp>

namespace VForce {

class LineSegment {
 public:
  cv::Mat GetSegmentMask(const cv::Mat &color, bool show = false);

 private:
  /**
   * Merge parallel lines depend on rho
   * @param lines lines with similar theta
   * @param min_rho merge lines if rho1 - rho2 < min_rho
   * @return parallel lines whith different rho
   */
  std::vector<cv::Vec2f> merge_lines(const std::vector<cv::Vec2f> &lines, float min_rho);

  /**
   * Mask picture based on line
   * @param line
   * @param mask
   */
  void mask_pic(const cv::Vec2f &line, cv::Mat &mask);
};

}
#endif //BING_PICKING_LINESEGMENT_HPP
