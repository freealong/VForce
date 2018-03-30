//
// Created by yongqi on 1/4/17.
//

#include "GLCloudViewer.hpp"

namespace VForce {
namespace Utils {

// helper functions
double clamp(double val, double lo, double hi) {
  return val < lo ? lo : val > hi ? hi : val;
}

GLCloudViewer::GLCloudViewer(int width, int height, std::string name)
    : _width(width), _height(height), _name(name), _ready(false) {}

void GLCloudViewer::InitWindow() {
  glfwInit();
  _win = glfwCreateWindow(_width, _height, _name.c_str(), nullptr, nullptr);
  glfwSetCursorPosCallback(_win, on_cursor_pos);
  glfwSetScrollCallback(_win, on_mouse_scroll);
  glfwSetMouseButtonCallback(_win, on_mouse_button);
  glfwSetCharCallback(_win, on_char_button);
  glfwSetKeyCallback(_win, on_key_button);
  glfwMakeContextCurrent(_win);
}

void GLCloudViewer::ProcessEvents() {
  glfwPollEvents();

  // Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(60, (float) _width / _height, 0.01f, 20.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  gluLookAt(0, 0, eyeZ, 0, 0, 1, 0, -1, 0);
  glTranslatef(0, 0, +0.5f);
  glRotated(pitch, 1, 0, 0);
  glRotated(yaw, 0, 1, 0);
  glTranslatef(moveX, moveY, -0.5f);

  switch (status) {
    case 0:glfwSetWindowShouldClose(_win, 1);
      break;
    case 1:_ready = true;
      break;
    case -1:_ready = nextFrame;
      nextFrame = false;
      break;
    default:break;
  }
}

void GLCloudViewer::ShowCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, unsigned char* color) {
  if (cloud == nullptr || cloud->empty())
    return;
  glPointSize(2);
  glEnable(GL_DEPTH_TEST);
  glBegin(GL_POINTS);
  for (auto p : *cloud) {
    glColor3ub(color[0], color[1], color[2]);
    glVertex3f(p.x, p.y, p.z);
  }
  glEnd();
}

void GLCloudViewer::ShowCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, int color) {
  if (cloud == nullptr || cloud->empty())
    return;
  glPointSize(2);
  glEnable(GL_DEPTH_TEST);
  glBegin(GL_POINTS);
  for (auto p : *cloud) {
    if (std::isnan(p.z)) continue;
    switch (color) {
      case 0:glColor3ub(p.r, p.g, p.b);
        break;
      case 1:glColor3ub(255, 0, 0);
        break;
      case 2:glColor3ub(0, 255, 0);
        break;
      case 3:glColor3ub(0, 0, 255);
        break;
      default:glColor3ub(100, 100, 100);
        break;
    }
    glVertex3f(p.x, p.y, p.z);
  }
  glEnd();
}

void GLCloudViewer::Draw3DBox(const std::vector<pcl::PointXYZ> &box, int color) {
  if (box.size() != 8)
    return;

  glLineWidth(5);
  switch (color) {
    case 1:glColor3ub(255, 0, 0);
      break;
    case 2:glColor3ub(0, 255, 0);
      break;
    case 3:glColor3ub(0, 0, 255);
      break;
    default:glColor3ub(255, 255, 255);
      break;
  }

  glBegin(GL_LINE_LOOP);
  glVertex3f(box[0].x, box[0].y, box[0].z);
  glVertex3f(box[1].x, box[1].y, box[1].z);
  glVertex3f(box[2].x, box[2].y, box[2].z);
  glVertex3f(box[3].x, box[3].y, box[3].z);
  glEnd();
  glBegin(GL_LINE_LOOP);
  glVertex3f(box[4].x, box[4].y, box[4].z);
  glVertex3f(box[5].x, box[5].y, box[5].z);
  glVertex3f(box[6].x, box[6].y, box[6].z);
  glVertex3f(box[7].x, box[7].y, box[7].z);
  glEnd();

  glBegin(GL_LINES);
  glVertex3f(box[0].x, box[0].y, box[0].z);
  glVertex3f(box[4].x, box[4].y, box[4].z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(box[1].x, box[1].y, box[1].z);
  glVertex3f(box[5].x, box[5].y, box[5].z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(box[2].x, box[2].y, box[2].z);
  glVertex3f(box[6].x, box[6].y, box[6].z);
  glEnd();
  glBegin(GL_LINES);
  glVertex3f(box[3].x, box[3].y, box[3].z);
  glVertex3f(box[7].x, box[7].y, box[7].z);
  glEnd();
}

void GLCloudViewer::DrawAxis(const std::vector<pcl::PointXYZ> &points) {
  assert(points.size() == 4);
  auto &origin = points[0], &x_axis = points[1], &y_axis = points[2], &z_axis = points[3];
  glLineWidth(5);
  glBegin(GL_LINES);
  glColor3ub(255, 0, 0);
  glVertex3f(origin.x, origin.y, origin.z);
  glVertex3f(x_axis.x, x_axis.y, x_axis.z);
  glEnd();
  glBegin(GL_LINES);
  glColor3ub(0, 255, 0);
  glVertex3f(origin.x, origin.y, origin.z);
  glVertex3f(y_axis.x, y_axis.y, y_axis.z);
  glEnd();
  glBegin(GL_LINES);
  glColor3ub(0, 0, 255);
  glVertex3f(origin.x, origin.y, origin.z);
  glVertex3f(z_axis.x, z_axis.y, z_axis.z);
  glEnd();
}

// static members
void GLCloudViewer::on_mouse_button(GLFWwindow *win, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT)
    ml = action == GLFW_PRESS;
}

void GLCloudViewer::on_mouse_scroll(GLFWwindow *win, double x, double y) {
  eyeZ += y * 0.02;
}

void GLCloudViewer::on_cursor_pos(GLFWwindow *win, double x, double y) {
  if (ml) {
    yaw = clamp(yaw - (x - lastX), -120, 120);
    pitch = clamp(pitch + (y - lastY), -80, 80);
  }
  lastX = x;
  lastY = y;
}

void GLCloudViewer::on_char_button(GLFWwindow *win, unsigned int c) {
  switch (c) {
    // "q"
    case 113:status = 0;
      break;
      // " "
    case 32 :status = -status;
      nextFrame = false;
      break;
      // "n"
    case 110:nextFrame = true;
      break;
      // "s"
    case 115:saveData = true;
      break;
    default:break;
  }
}

void GLCloudViewer::on_key_button(GLFWwindow *win, int key, int scancode, int action, int mods) {
//  std::cout << key << " " << scancode << " " << action << " " << mods << std::endl;
  switch (key) {
    case 262: // right
      moveX -= 0.02;
      break;
    case 263: //left
      moveX += 0.02;
      break;
    case 264: // down
      moveY -= 0.02;
      break;
    case 265: // up
      moveY += 0.02;
      break;
  }
}

double GLCloudViewer::yaw = 0;
double GLCloudViewer::pitch = 0;
double GLCloudViewer::lastY = 0;
double GLCloudViewer::lastX = 0;
double GLCloudViewer::eyeZ = 0;
double GLCloudViewer::moveX = 0;
double GLCloudViewer::moveY = 0;
int GLCloudViewer::ml = 0;
int GLCloudViewer::status = -1; // -1 step mode, 1 running mode, 0 quit
bool GLCloudViewer::nextFrame = true;
bool GLCloudViewer::saveData = false;

}
}
