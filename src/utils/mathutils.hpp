//
// Created by yongqi on 17-12-4.
//

#ifndef VFORCE_MATHUTILS_HPP
#define VFORCE_MATHUTILS_HPP

#include <cmath>
#include <Eigen/Eigen>

namespace VForce {
namespace Utils {
/**
 * make angle between [-pi, pi)
 * @param x input angle in radius
 * @return normalized angle
 */
inline double normalize_angle(double x) {
  x = fmod(x + M_PI, 2 * M_PI);
  return x < 0 ? x += 2 * M_PI : x - M_PI;
}

inline float normalize_anlge(float x) {
  x = fmod(x + M_PI, 2 * M_PI);
  return x < 0 ? x += 2 * M_PI : x - M_PI;
}

/**
 * radian to degree
 * @tparam T
 * @param x
 * @return
 */
template<typename T>
inline T rad2deg(T x) {
  return x / M_PI * 180.;
}

/**
 * degree to radian
 * @tparam T
 * @param x
 * @return
 */
template<typename T>
inline T deg2rad(T x) {
  return x * M_PI / 180.;
}

/**
 * Convert pose(in Rz(yaw)Ry(pitch)Rx(roll) order) into transformation matrix
 * @tparam T
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 * @param tf
 */
template<typename T>
void pose2matrix(const T &x, const T &y, const T &z,
                 const T &roll, const T &pitch, const T &yaw,
                 Eigen::Matrix<T, 4, 4> &t) {
  T A = cos(yaw), B = sin(yaw), C = cos(pitch), D = sin(pitch),
      E = cos(roll), F = sin(roll), DE = D * E, DF = D * F;

  t(0, 0) = A * C;
  t(0, 1) = A * DF - B * E;
  t(0, 2) = B * F + A * DE;
  t(0, 3) = x;
  t(1, 0) = B * C;
  t(1, 1) = A * E + B * DF;
  t(1, 2) = B * DE - A * F;
  t(1, 3) = y;
  t(2, 0) = -D;
  t(2, 1) = C * F;
  t(2, 2) = C * E;
  t(2, 3) = z;
  t(3, 0) = 0;
  t(3, 1) = 0;
  t(3, 2) = 0;
  t(3, 3) = 1;
}

/**
 * Convert transformation matrix into pose
 * @tparam T
 * @param matrix
 * @param x
 * @param y
 * @param z
 * @param roll
 * @param pitch
 * @param yaw
 */
template<typename T>
void matrix2pose(const Eigen::Matrix<T, 4, 4> &t,
                 T &x, T &y, T &z,
                 T &roll, T &pitch, T &yaw) {
  x = t(0, 3);
  y = t(1, 3);
  z = t(2, 3);
  roll = atan2(t(2, 1), t(2, 2));
  pitch = asin(-t(2, 0));
  yaw = atan2(t(1, 0), t(0, 0));
}

}
}

#endif //VFORCE_MATHUTILS_HPP
