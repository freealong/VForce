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

        int y_beg = other_y0 < other_y1 ? other_y0 : other_y1;
        int y_end = other_y0 < other_y1 ? other_y1 : other_y0;
        int x_beg = other_x0 < other_x1 ? other_x0 : other_x1;
        int x_end = other_x0 < other_x1 ? other_x1 : other_x0;
        if (x_beg < 0 || y_beg < 0 || x_end >= o_width || y_end >= o_height)
          continue;

        // Transfer between the depth pixels and the pixels inside the rectangle on the other image
        for (int y = y_beg; y < y_end; ++y)
          for (int x = x_beg; x < x_end; ++x)
            other[y * o_width + x] = depth[dxy];
      }
    }
  }
}

// @FIXME: There is something wrong with distortion, distortion should based on camera distortion model
void Camera::Deproject(float *pixel,
                       float depth,
                       const cv::Mat &intrin,
                       const cv::Mat &coeffs,
                       float *point) {
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

void Camera::Project(float *point, const cv::Mat &intrin, const cv::Mat &coeffs, float *pixel) {
  double x = point[0] / point[2], y = point[1] / point[2];
  {
    double r2 = x * x + y * y;
    double f = 1 + coeffs.at<double>(0) * r2 + coeffs.at<double>(1) * r2 * r2 + coeffs.at<double>(4) * r2 * r2 * r2;
    x *= f;
    y *= f;
    double dx = x + 2 * coeffs.at<double>(2) * x * y + coeffs.at<double>(3) * (r2 + 2 * x * x);
    double dy = y + 2 * coeffs.at<double>(3) * x * y + coeffs.at<double>(2) * (r2 + 2 * y * y);
    x = dx;
    y = dy;
  }
  pixel[0] = static_cast<float>(x * intrin.at<double>(0, 0) + intrin.at<double>(0, 2));
  pixel[1] = static_cast<float>(y * intrin.at<double>(1, 1) + intrin.at<double>(1, 2));
}

void Camera::Transform(float *point1, const cv::Mat &extrin, float *point2) {
  point2[0] = static_cast<float>(extrin.at<double>(0, 0) * point1[0] + extrin.at<double>(0, 1) * point1[1] +
      extrin.at<double>(0, 2) * point1[2] + extrin.at<double>(0, 3));
  point2[1] = static_cast<float>(extrin.at<double>(1, 0) * point1[0] + extrin.at<double>(1, 1) * point1[1] +
      extrin.at<double>(1, 2) * point1[2] + extrin.at<double>(1, 3));
  point2[2] = static_cast<float>(extrin.at<double>(2, 0) * point1[0] + extrin.at<double>(2, 1) * point1[1] +
      extrin.at<double>(2, 2) * point1[2] + extrin.at<double>(2, 3));
}

void Camera::Reproject(const float *disparity, const int w, const int h, const cv::Mat &Q, float *depth) {
#pragma omp parallel for schedule(dynamic)
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      int id = y * w + x;
//      double X = x + Q.at<double>(0, 3);
//      double Y = y + Q.at<double>(1, 3);
      double Z = Q.at<double>(2, 3);
      double W = Q.at<double>(3, 2) * disparity[id] + Q.at<double>(3, 3);
      depth[id] = static_cast<float>(Z / W);
    }
}

cv::Mat Camera::Depth2HHA(const cv::Mat &depth,
                          const cv::Mat &K,
                          const cv::Mat &D,
                          const cv::Mat &cMw,
                          const cv::Mat &wMb,
                          float depth_floor,
                          float depth_ceil,
                          float height_floor = -0.05f,
                          float height_ceil = 0.5f) {
  auto cloud = Depth2Cloud(depth, K, D);
  return Cloud2HHA(cloud, cMw, wMb, depth_floor, depth_ceil, height_floor, height_ceil);
}

cv::Mat Camera::Cloud2HHA(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                          const cv::Mat &cMw,
                          const cv::Mat &wMb,
                          float depth_floor,
                          float depth_ceil,
                          float height_floor,
                          float height_ceil) {
  cv::Mat hha(cloud->height, cloud->width, CV_8UC3);
  // prepare computing disparity params
  float reciprocal_floor = 1 / depth_floor;
  float reciprocal_ceil = 1 / depth_ceil;
  // prepare computing height params
  cv::Mat bMc = (cMw * wMb).inv();
  // prepare computing angle params
  float y_dir[3] = {(float)(-cMw.at<double>(0, 2)), (float)(-cMw.at<double>(1, 2)), (float)(-cMw.at<double>(2, 2))};
  auto normals = ComputeNormalSquareSupport(cloud, 3);
  // calculate hha
  for (int y = 0; y < cloud->height; ++y) {
    for (int x = 0; x < cloud->width; ++x) {
      auto &p = cloud->at(x, y);
      if (p.z == 0 || std::isnan(p.z)) {
        hha.at<cv::Vec3b>(y, x)[0] = 0;
        hha.at<cv::Vec3b>(y, x)[1] = 0;
        hha.at<cv::Vec3b>(y, x)[2] = 38;
      }
      else {
        // disparity
        float disparity = 1 / p.z;
        disparity = (disparity - reciprocal_ceil) / (reciprocal_floor - reciprocal_ceil);
        disparity = std::min(std::max(disparity, 0.f), 1.f); // crop to 0-1
        hha.at<cv::Vec3b>(y, x)[0] = static_cast<uchar>(disparity * 255.f);
        // height
        float height = static_cast<float>(bMc.at<double>(2, 0) * p.x + bMc.at<double>(2, 1) * p.y +
            bMc.at<double>(2, 2) * p.z + bMc.at<double>(2, 3));
        height = (height - height_floor) / (height_ceil - height_floor);
        height = std::min(std::max(height, 0.f), 1.f); // crop to 0-1
        hha.at<cv::Vec3b>(y, x)[1] = static_cast<uchar>(height * 255.f);
        // angle
        auto &norm = normals->at(x, y);
        float cos_angle = norm.normal_x * y_dir[0] + norm.normal_y * y_dir[1] + norm.normal_z * y_dir[2];
        cos_angle = std::min(1.f, std::max(-1.f, cos_angle));
        hha.at<cv::Vec3b>(y,x)[2] = static_cast<uchar>((acosf(cos_angle)) / M_PI * 180.f + 38);
      }
    }
  }
  return hha;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Camera::Depth2Cloud(const cv::Mat &depth, const cv::Mat &K, const cv::Mat &D) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->height = static_cast<uint32_t >(depth.rows);
  cloud->width = static_cast<uint32_t >(depth.cols);
  cloud->resize(cloud->height * cloud->width);
  for (int y = 0; y < depth.rows; ++y) {
    for (int x = 0; x < depth.cols; ++x) {
      auto &p = cloud->at(x, y);
      float d = depth.at<float>(y, x);
      if (d == 0 || std::isnan(d)) {
        p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
        continue;
      }
      float pixel[2] = {(float)x, (float)y};
      Deproject(pixel, d, K, D, p.data);
      p.data[3] = 1.0f; // Don't know why, just follow the Construct function of PointXYZ
    }
  }
  return cloud;
}

pcl::PointCloud<pcl::Normal>::Ptr Camera::ComputeNormalSquareSupport(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                                     int R) {
  cv::Mat X(cloud->height, cloud->width, CV_64F), Y(X.size(), CV_64F), Z(X.size(), CV_64F);
  cv::Mat One_Z(X.size(), CV_64F), X_Z(X.size(), CV_64F), Y_Z(X.size(), CV_64F), One(X.size(), CV_64F), X_ZZ(X.size(), CV_64F), Y_ZZ(X.size(), CV_64F);
  cv::Mat XX_ZZ(X.size(), CV_64F), YY_ZZ(X.size(), CV_64F), XY_ZZ(X.size(), CV_64F);
  // prepare AtARaw xx_zz, xy_zz, x_z, yy_zz, y_z, one
  // AtbRaw x_zz, y_zz, one_z
  for (int y = 0; y < cloud->height; ++y)
    for (int x = 0; x < cloud->width; ++x) {
      auto &p = cloud->at(x, y);
      // convert to cm
      X.at<double>(y, x) = p.x * 100;
      Y.at<double>(y, x) = p.y * 100;
      Z.at<double>(y, x) = p.z * 100;
      if (std::isnan(p.z)) {
        One_Z.at<double>(y, x) =  X_Z.at<double>(y, x) = Y_Z.at<double>(y, x) =
        One.at<double>(y, x) = X_ZZ.at<double>(y, x) = Y_ZZ.at<double>(y, x) =
        XX_ZZ.at<double>(y, x) = YY_ZZ.at<double>(y, x) = XY_ZZ.at<double>(y, x) = 0;
        continue;
      }
      One_Z.at<double>(y, x) = 1 / p.z;
      X_Z.at<double>(y, x) = p.x / p.z;
      Y_Z.at<double>(y, x) = p.y / p.z;
      One.at<double>(y, x) = std::isnan(p.z) ? p.z : 1;
      X_ZZ.at<double>(y, x) = X_Z.at<double>(y, x) / Z.at<double>(y, x);
      Y_ZZ.at<double>(y, x) = Y_Z.at<double>(y, x) / Z.at<double>(y, x);
      XX_ZZ.at<double>(y, x) = X_Z.at<double>(y, x) * X_Z.at<double>(y, x);
      YY_ZZ.at<double>(y, x) = Y_Z.at<double>(y, x) * Y_Z.at<double>(y, x);
      XY_ZZ.at<double>(y, x) = X_Z.at<double>(y, x) * Y_Z.at<double>(y, x);
    }
  // for fast access
  double *a1 = (double *)XX_ZZ.data;
  double *a2 = (double *)XY_ZZ.data;
  double *a3 = (double *)X_Z.data;
  double *a4 = (double *)YY_ZZ.data;
  double *a5 = (double *)Y_Z.data;
  double *a6 = (double *)One.data;
  double *b1 = (double *)X_ZZ.data;
  double *b2 = (double *)Y_ZZ.data;
  double *b3 = (double *)One_Z.data;
  // debug
//  int id = 384*640+342;
//  cout << "point: " << X.at<double>(384, 342) << " " << Y.at<double>(384, 342) << " " << Z.at<double>(384, 342) << endl;
//  cout << "AtA: " << a1[id] << " " << a2[id] << " " << a3[id] << " " << a4[id] << " " << a5[id] << " " << a6[id] << endl;
  // filter2d to AtaRaw, AtbRaw
  auto B = cv::Mat::ones(2*R+1, 2*R+1, CV_64F);
  filter2D(XX_ZZ, XX_ZZ, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
  filter2D(XY_ZZ, XY_ZZ, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
  filter2D(X_Z, X_Z, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
  filter2D(YY_ZZ, YY_ZZ, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
  filter2D(Y_Z, Y_Z, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
  filter2D(One, One, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
  filter2D(X_ZZ, X_ZZ, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
  filter2D(Y_ZZ, Y_ZZ, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
  filter2D(One_Z, One_Z, -1, B, cv::Point(-1, -1), 0, cv::BORDER_ISOLATED);
//  cout << "AtA: " << a1[id] << " " << a2[id] << " " << a3[id] << " " << a4[id] << " " << a5[id] << " " << a6[id] << endl;
  // calculate N and b;
  double a, d, g, e, h, k;
  double deltaa;
  double x1, x2, x3;
  double len;
  pcl::PointCloud<pcl::Normal>::Ptr N(new pcl::PointCloud<pcl::Normal>(cloud->width, cloud->height));
  std::vector<double> b(cloud->size());
  for (int i = 0; i < cloud->size(); ++i) {
    a = a4[i] * a6[i] - a5[i] * a5[i];
    d = -(a2[i] * a6[i] - a3[i] * a5[i]);
    g = a2[i] * a5[i] - a3[i] * a4[i];
    e = a1[i] * a6[i] - a3[i] * a3[i];
    h = -(a1[i] * a5[i] - a2[i] * a3[i]);
    k = a1[i] * a4[i] - a2[i] * a2[i];
    deltaa = a1[i] * a + a2[i] * d + a3[i] *g;
    x1 = a * b1[i] + d * b2[i] + g * b3[i];
    x2 = d * b1[i] + e * b2[i] + h * b3[i];
    x3 = g * b1[i] + h * b2[i] + k * b3[i];
//    if (i == id){
//      cout << "x: " << x1 << " " << x2 << " " << x3 << endl;
//    }
    len = sqrt(x1 * x1 + x2 * x2 + x3 * x3);
    N->points[i].normal_x = static_cast<float>(x1 / len);
    N->points[i].normal_y = static_cast<float>(x2 / len);
    N->points[i].normal_z = static_cast<float>(x3 / len);
    b[i] = -deltaa / len;
    if (x3 < 0) {
      N->points[i].normal_x *= -1;
      N->points[i].normal_y *= -1;
      N->points[i].normal_z *= -1;
      b[i] *= -1;
    }
    double sign = N->points[i].normal_x * ((double *)X.data)[i] + N->points[i].normal_y * ((double *)Y.data)[i]
        + N->points[i].normal_z * ((double *)Z.data)[i];
    if (sign < 0) {
      N->points[i].normal_x *= -1;
      N->points[i].normal_y *= -1;
      N->points[i].normal_z *= -1;
      b[i] *= -1;
    }
  }
  return N;
}

}
