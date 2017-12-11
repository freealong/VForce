//
// Created by yongqi on 17-12-7.
//

#include "linesegment.hpp"
#include <glog/logging.h>

namespace VForce {

using namespace std;
using namespace cv;

cv::Mat LineSegment::GetSegmentMask(const cv::Mat &color, bool show) {
  Mat mask(color.rows, color.cols, CV_8U, Scalar(255));
//  LOG(INFO) << "mask size: " << mask.size() << endl;

  Mat gray;
  cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
  cv::Mat edge;
  cv::Canny(gray, edge, 200, 400);
//  cv::dilate(edge, edge, Mat(), Point(-1, -1));

  // find lines
  const float min_merge_rho = min(color.cols, color.rows) / 5.f;
  vector<cv::Vec2f> lines, another_lines;
  cv::HoughLines(edge, lines, 1, CV_PI / 90, 80, 0, 0, 5. / 180 * CV_PI, 85. / 180 * CV_PI);
  cv::HoughLines(edge, another_lines, 1, CV_PI / 90, 80, 0, 0, 95. / 180 * CV_PI, 175. / 180 * CV_PI);
//  if (lines.size() > another_lines.size()) {
  {
    // merge lines depend on rho
    std::sort(lines.begin(), lines.end(), [](cv::Vec2f &l1, cv::Vec2f &l2) -> bool {
      return l1[0] < l2[0];
    });
    lines = merge_lines(lines, min_merge_rho);
    if (show)
      LOG(INFO) << "line between 30-60 size: " << lines.size() << endl;
    // mask pic based on lines
    for (auto &line : lines) {
      mask_pic(line, mask);
    }
  }
//  else if (lines.size() < another_lines.size()) {
  {
    // merge lines depend on rho
    std::sort(another_lines.begin(), another_lines.end(), [](cv::Vec2f &l1, cv::Vec2f &l2) -> bool {
      return l1[0] < l2[0];
    });
    lines = merge_lines(another_lines, min_merge_rho);
    if (show)
      LOG(INFO) << "line between 120-150 size: " << another_lines.size() << endl;
    // mask pic based on lines
    for (auto &line : another_lines) {
      mask_pic(line, mask);
    }
  }

  if (show) {
    imshow("edge", edge);
    cv::Mat masked_gray;
    gray.copyTo(masked_gray, mask);
    imshow("mask gray", masked_gray);
    waitKey();
  }
  return mask;
}

std::vector<cv::Vec2f> LineSegment::merge_lines(const std::vector<cv::Vec2f> &lines, float min_rho) {
  std::vector<cv::Vec2f> res_lines;
  if (lines.empty())
    return res_lines;
  res_lines.push_back(lines[0]);
  for (int i = 1; i < lines.size(); ++i) {
//    LOG(INFO) << lines[i][0] << endl;
    if (lines[i][0] - res_lines[res_lines.size() - 1][0] > min_rho) {
      res_lines.push_back(lines[i]);
    }
  }
  return res_lines;
}

void LineSegment::mask_pic(const cv::Vec2f &line, cv::Mat &mask) {
  int w = mask.cols, h = mask.rows;
  float rho = line[0];
  float theta = line[1];
  double a = cos(theta), b = sin(theta);
  float x0 = rho / a; // y = 0
  float y0 = rho / b; // x = 0
  float x1 = (rho - h * b) / a; // y = h
  float y1 = (rho - w * a) / b; // x = w
  //    1
  //   ----
  // 2 |  | 4
  //   ----
  //    3
  // points on side 1 and 2
  if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
    DLOG(INFO) << "points on side 1 and 2" << endl;
    for (int y = 0; y < y0; ++y)
      for (int x = 0; x < x0; ++x)
        if (x0 * y + y0 * x < x0 * y0)
          mask.at<uchar>(y, x) = 0;
    if (x0 < w * 0.7 && y0 < h * 0.7) {
      for (int y = h - y0; y < h; ++y)
        for (int x = w - x0; x < w; ++x)
          if (y0 * x + x0 * y > w * y0 + h * x0 - x0 * y0)
            mask.at<uchar>(y, x) = 0;
    }
  }
  // side 1 and 4
  else if (x0 >= 0 && x0 < w && y1 >= 0 && y1 < h) {
    DLOG(INFO) << "points on side 1 and 4" << endl;
    for (int y = 0; y < y1; ++y)
      for (int x = x0; x < w; ++x)
        if (y1 * x - (w - x0) * y > x0 * y1)
          mask.at<uchar>(y, x) = 0;
    if (x0 > w * 0.3 && y1 < h * 0.7) {
      for (int y = h - y1; y < h; ++y)
        for (int x = 0; x < w - x0; ++x)
          if ((w - x0) * y - y1 * x > (h - y1) * (w - x0))
            mask.at<uchar>(y, x) = 0;
    }
  }
  // side 2 and 3
  else if (y0 >= 0 && y0 < h && x1 >= 0 && x1 < w) {
    DLOG(INFO) << "points on side 2 and 3" << endl;
    for (int y = y0; y < h; ++y)
      for (int x = 0; x < x1; ++x)
        if (x1 * y - (h - y0) * x > y0 * x1)
          mask.at<uchar>(y, x) = 0;
    if (x1 < w * 0.7 && y0 > h * 0.3) {
      for (int y = 0; y < h - y0; ++y)
        for (int x = w - x1; x < w; ++x)
          if ((h - y0) * x - x1 * y > (w - x1) * (h - y0))
            mask.at<uchar>(y, x) = 0;
    }
  }
  // side 3 and 4
  else if (x1 >= 0 && x1 < w && y1 >= 0 && y1 < h) {
    DLOG(INFO) << "points on side 3 and 4" << endl;
    for (int y = y1; y < h; ++y)
      for (int x = x1; x < w; ++x)
        if ((h - y1) * x + (w - x1) * y > w * h - x1 * y1)
          mask.at<uchar>(y, x) = 0;
    if (x1 > w * 0.3 && y1 > h * 0.3) {
      for (int y = 0; y < h - y1; ++y)
        for (int x = 0; x < w - x1; ++x)
          if ((w - x1) * y + (h - y1) * x < (w - x1) * (h - y1))
            mask.at<uchar>(y, x) = 0;
    }
  }
  // side 1 and 3
  else if (x0 >= 0 && x0 < w && x1 >= 0 && x1 < w) {
    DLOG(INFO) << "points on side 1 and 3" << endl;
    if (x0 + x1 < w) {
      for (int y = 0; y < h; ++y)
        for (int x = 0; x < max(x0, x1); ++x)
          if ((h * x - (x1 - x0) * y - h * x0) < 0)
            mask.at<uchar>(y, x) = 0;
    } else {
      for (int y = 0; y < h; ++y)
        for (int x = min(x0, x1); x < w; ++x)
          if ((h * x - (x1 - x0) * y - h * x0) > 0)
            mask.at<uchar>(y, x) = 0;
    }
  }
  // side 2 and 4
  else if (y0 >= 0 && y0 < h && y1 >= 0 && y1 < h) {
    DLOG(INFO) << "points on side 2 and 4" << endl;
    if (y0 + y1 < h) {
      for (int y = 0; y < max(y0, y1); ++y)
        for (int x = 0; x < w; ++x)
          if ((y1 - y0) * x - w * (y - y0) > 0)
            mask.at<uchar>(y, x) = 0;
    } else {
      for (int y = min(y0, y1); y < h; ++y)
        for (int x = 0; x < w; ++x)
          if (((y1 - y0) * x - w * (y - y0)) < 0)
            mask.at<uchar>(y, x) = 0;
    }
  }
}

}
