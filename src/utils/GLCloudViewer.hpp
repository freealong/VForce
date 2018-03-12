//
// Created by yongqi on 1/4/17.
//

#ifndef VFORCE_OPENGLCLOUDVIEW_HPP
#define VFORCE_OPENGLCLOUDVIEW_HPP

#include <string>
#include <vector>
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace VForce {
namespace Utils {

class GLCloudViewer {
 public:
  GLCloudViewer(int width = 640, int height = 480, std::string name = "main");

  void InitWindow();

  void DestoryWindow() {
    glfwDestroyWindow(_win);
  }

  inline bool NotStop() {
    return !glfwWindowShouldClose(_win);
  }

  inline void SwapBuffer() {
    glfwSwapBuffers(_win);
  }

  void ProcessEvents();

  void ShowCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, unsigned char* color);

  void ShowCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, int color = 0);

  void Draw3DBox(const std::vector<pcl::PointXYZ> &box, int color = 2);

  void DrawAxis(const std::vector<pcl::PointXYZ> &points);

  inline bool ReadyToUpdate() const {
    return _ready;
  }

  inline bool NeedToSave() const {
    return saveData;
  }

  inline void FinishSave() {
    saveData = false;
  }

 private:
  GLFWwindow *_win;
  int _width, _height;
  std::string _name;
  bool _ready;

  static void on_mouse_button(GLFWwindow *win, int button, int action, int mods);
  static void on_mouse_scroll(GLFWwindow *win, double x, double y);
  static void on_cursor_pos(GLFWwindow *win, double x, double y);
  static void on_char_button(GLFWwindow *win, unsigned int c);
  static void on_key_button(GLFWwindow *win, int key, int scancode, int action, int mods);
  static double yaw, pitch, lastX, lastY;
  static double eyeZ;
  static double moveX, moveY;
  static int ml;
  static int status;
  static bool nextFrame, saveData;
};

}
}
#endif //VFORCE_OPENGLCLOUDVIEW_HPP
