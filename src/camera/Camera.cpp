//
// Created by yongqi on 17-12-15.
//

#include "Camera.hpp"

namespace VForce {

void Camera::Align2Other(uint16_t *depth,
                         int d_height,
                         int d_width,
                         const cv::Mat &Kd,
                         const cv::Mat &Dd,
                         uint16_t *other,
                         int o_height,
                         int o_width,
                         const cv::Mat &Ko,
                         const cv::Mat &Do,
                         const cv::Mat &oMd,
                         float depth_scale) {
  assert(depth != nullptr);
  assert(other != nullptr);
  assert(depth != other);
  int size = o_height * o_width;
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < size; ++i)
    other[i] = 0;
#pragma omp parallel for schedule(dynamic)
  for (int dy = 0; dy < d_height; ++dy) {
    int dxy = dy * d_width;
    for (int dx = 0; dx < d_width; ++dx, ++dxy) {
      if (float d = depth[dxy]) {
        // convert depth pixel(u,v) to depth point(x,y,z)
        // Map the top-left corner of the depth pixel onto the other image
        float depth_in_meter = d * depth_scale;
        float depth_pixel[2] = {dx - 0.5f, dy - 0.5f}, depth_point[3], other_point[3], other_pixel[2];
        Deproject(depth_pixel, depth_in_meter, Kd, Dd, depth_point);
        Transform(depth_point, oMd, other_point);
        Project(other_point, Ko, Do, other_pixel);
        const int other_x0 = static_cast<int>(other_pixel[0] + 0.5f);
        const int other_y0 = static_cast<int>(other_pixel[1] + 0.5f);

        // Map the bottom-right corner of the depth pixel onto the other image
        depth_pixel[0] = dx + 0.5f;
        depth_pixel[1] = dy + 0.5f;
        Deproject(depth_pixel, depth_in_meter, Kd, Dd, depth_point);
        Transform(depth_point, oMd, other_point);
        Project(other_point, Ko, Do, other_pixel);
        const int other_x1 = static_cast<int>(other_pixel[0] + 0.5f);
        const int other_y1 = static_cast<int>(other_pixel[1] + 0.5f);

        if (other_x0 < 0 || other_y0 < 0 || other_x1 >= o_width || other_y1 >= o_height)
          continue;

        // Transfer between the depth pixels and the pixels inside the rectangle on the other image
        for (int y = std::min(other_y0, other_y1); y <= std::max(other_y1, other_y0); ++y)
          for (int x = std::min(other_x0, other_x1); x <= std::max(other_x1, other_x0); ++x)
            other[y * o_width + x] = depth[dxy];
      }
    }
  }
}
void Camera::Deproject(float *pixel,
                       float depth,
                       const cv::Mat &intrin,
                       const cv::Mat &coeffs,
                       float *point) const {
  double x = (pixel[0] - intrin.at<double>(0, 2)) / intrin.at<double>(0, 0);
  double y = (pixel[1] - intrin.at<double>(1, 2)) / intrin.at<double>(1, 1);
  {
    double r2 = x * x + y * y;
    double f = 1 + coeffs.at<double>(0) * r2 + coeffs.at<double>(1) * r2 * r2 + coeffs.at<double>(4) * r2 * r2 * r2;
    double ux = x * f + 2 * coeffs.at<double>(2) * x * y + coeffs.at<double>(3) * (r2 + 2 * x * x);
    double uy = y * f + 2 * coeffs.at<double>(3) * x * y + coeffs.at<double>(2) * (r2 + 2 * y * y);
    x = ux;
    y = uy;
  }
  point[0] = static_cast<float>(depth * x);
  point[1] = static_cast<float>(depth * y);
  point[2] = depth;
}

void Camera::Project(float *point, const cv::Mat &intrin, const cv::Mat &coeffs, float *pixel) const {
  double x = point[0] / point[2], y = point[1] / point[2];
  {
    double r2 = x * x + y * y;
    double f = 1 + coeffs.at<double>(0) * r2 + coeffs.at<double>(1) * r2 * r2 + coeffs.at<double>(4) * r2 * r2 * r2;
    x *= f;
    y *= f;
    double dx = x * f + 2 * coeffs.at<double>(2) * x * y + coeffs.at<double>(3) * (r2 + 2 * x * x);
    double dy = y * f + 2 * coeffs.at<double>(3) * x * y + coeffs.at<double>(2) * (r2 + 2 * y * y);
    x = dx;
    y = dy;
  }
  pixel[0] = static_cast<float>(x * intrin.at<double>(0, 0) + intrin.at<double>(0, 2));
  pixel[1] = static_cast<float>(y * intrin.at<double>(1, 1) + intrin.at<double>(1, 2));
}

void Camera::Transform(float *point1, const cv::Mat &extrin, float *point2) const {
  point2[0] = static_cast<float>(extrin.at<double>(0, 0) * point1[0] + extrin.at<double>(0, 1) * point1[1] +
      extrin.at<double>(0, 2) * point1[2] + extrin.at<double>(0, 3));
  point2[1] = static_cast<float>(extrin.at<double>(1, 0) * point1[0] + extrin.at<double>(1, 1) * point1[1] +
      extrin.at<double>(1, 2) * point1[2] + extrin.at<double>(1, 3));
  point2[2] = static_cast<float>(extrin.at<double>(2, 0) * point1[0] + extrin.at<double>(2, 1) * point1[1] +
      extrin.at<double>(2, 2) * point1[2] + extrin.at<double>(2, 3));
}

}