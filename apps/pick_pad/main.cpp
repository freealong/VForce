//
// Created by yongqi on 17-12-11.
//

// System

// STL
#include <iostream>
#include <chrono>
#include <thread>
#include <condition_variable>

// 3rd Libraries
#include <pcl/io/pcd_io.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

// User Defined
#include "utils/CVUtils.hpp"
#include "utils/GLCloudViewer.hpp"
#include "utils/SyncTCPServer.hpp"
#include "utils/PointCloudUtils.hpp"
#include "camera/RealsenseCamera.hpp"
#include "camera/StereoRealsenseCamera.hpp"
#include "camera/VirtualCamera.hpp"
#include "vision/Vision.hpp"
#include "robot/Robot.hpp"
#include "Message.hpp"

using namespace std;
using namespace pcl;
using namespace VForce;

typedef PointXYZRGBA PointType;

// command line flags
DEFINE_bool(gui, true, "pointcloud visualization");
DEFINE_bool(tcp, false, "run tcp server");
DEFINE_bool(virtual_camera, false, "read data from file system instead of a really RGB-D camera");
DEFINE_string(virtual_camera_path, "test_data", "data path which virtual camera will read from");
DEFINE_string(config_root, "../apps/pick_pad/config", "program will read configs under this path");

// global mutex and condition_variable
mutex mu;
condition_variable tcp_cv, gui_cv;
bool g_close = false;

// Request Type between threads
enum class RequestType : int {
  NONE = 0, VIP = 111,
  IN1 = 1, OUT1 = -1,
  IN2 = 2, OUT2 = -2
};

/**
 * TCP server thread
 * @param address tcp server address
 * @param port tcp server port
 * @param request client request type
 * @param flag target flag
 * @param target target will send to client
 */
void TCPThread(string address, int port, RequestType &request, int &flag, Pose &target) {
  LOG(INFO) << "Initializing TCP thread...";
  while (true) {
    if (g_close)
      break;
    Utils::SyncTCPServer<char, Message> tcp_server(address, static_cast<unsigned short>(port));
    LOG(INFO) << "Setup TCP server successfully";
    tcp_server.WaitingClient();
    LOG(INFO) << "Client connected";
    while (true) {
      if (g_close)
        break;
      try {
        char recv_msg;
        tcp_server.RecvMsg(recv_msg);
        LOG(INFO) << "RecvMsg: " << recv_msg;
        unique_lock<mutex> lk(mu);
        if (recv_msg == 'a')
          request = RequestType::IN1;
        tcp_cv.wait(lk, [&]{return request == RequestType::OUT1 || request == RequestType::VIP;});
        Message send_msg(target, 0x00); // client need flag = 0
        LOG(INFO) << "SendMsg: " << send_msg;
        tcp_server.SendMsg(send_msg);
      }
      catch (std::exception& e){
        LOG(INFO) << "Client disconnected";
        break;
      }
    }
  }
  LOG(INFO) << "TCP Thread Closed";
}

/**
 * GUI Thread visualize the vision output
 * @param width window width
 * @param height window height
 * @param name window name
 * @param cloud raw cloud from the camera
 * @param transformed_models a map store transformed models
 * @param results vision results
 */
void GUIThread(int width, int height, string name, RequestType &request,
               PointCloud<PointType>::Ptr &cloud,
               std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &transformed_models,
               std::vector<ObjectInfo> &results) {
  LOG(INFO) << "Initializing GUI thread...";
  Utils::GLCloudViewer viewer(width, height, name);
  viewer.InitWindow();
//  vector<PointXYZ> origin_box = ();
//  vector<PointXYZ> target_box(origin_box);
  vector<PointXYZ> origin_axis{{0, 0, 0}, {0.06, 0, 0}, {0, 0.06, 0}, {0, 0, 0.06}};
  vector<vector<PointXYZ>> target_axis_buffer;
  vector<vector<unsigned char>> target_color_buffer;

  LOG(INFO) << "Initialized GUI thread successfully";
  while (viewer.NotStop()) {
    if (g_close) {
      break;
    }
    viewer.ProcessEvents();
    unique_lock<mutex> lk(mu);
    if (request == RequestType::VIP || viewer.ReadyToUpdate()) {
      if (viewer.ReadyToUpdate()) {
        request = RequestType::IN1;
        gui_cv.wait(lk, [&]{return request == RequestType::OUT1 || request == RequestType::VIP;});
      }
      target_axis_buffer.clear();
      target_color_buffer.clear();
      for (auto &res : results) {
        vector<PointXYZ> target_axis(origin_axis);
        Utils::transform_points(origin_axis, target_axis, res.pose_);
        target_axis_buffer.emplace_back(target_axis);
//        vector<unsigned char> target_color{rand()%256, rand()%256, rand()%256};
        vector<unsigned char> target_color{255, 0, 0};
        target_color_buffer.emplace_back(target_color);
      }
//      target_box = TransformPoints(cMo, origin_box);
      if (request == RequestType::VIP)
        request = RequestType::NONE;
    }
    viewer.DrawAxis(origin_axis); // camera origin
    viewer.ShowCloud(cloud);
    for (auto &target_axis : target_axis_buffer)
      viewer.DrawAxis(target_axis);
    for (int i = 0; i < target_axis_buffer.size(); ++i)
      viewer.ShowCloud(transformed_models[i], target_color_buffer[i].data());
//      viewer.Draw3DBox(target_box);
    viewer.SwapBuffer();
  }
  viewer.DestoryWindow();
  LOG(INFO) << "GUI Thread Closed";
}

/**
 * Vision Thread detect the target and calcuate the target's pose
 * @param camera_name
 * @param vision_cfg
 * @param robot_cfg
 * @param tcp_request tcp request signal
 * @param flag
 * @param target_pose target pose in robot coordinate
 * @param gui_request gui request signal
 * @param cloud raw point cloud from camera
 * @param transformed_models a map store transformed models
 * @param results vision results
 */
void VisionThread(string camera_name, string vision_cfg, string robot_cfg,
                  RequestType &tcp_request, int &flag, Pose &target_pose,
                  RequestType &gui_request, PointCloud<PointType>::Ptr &cloud,
                  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &transformed_models,
                  std::vector<ObjectInfo> &results) {
  LOG(INFO) << "Initializing vision thread...";
  shared_ptr<Camera> camera;
  if (FLAGS_virtual_camera)
    camera = shared_ptr<Camera>(new VirtualCamera(FLAGS_virtual_camera_path));
  else {
    if (camera_name == "Realsense") {
      camera = shared_ptr<Camera>(new RealsenseCamera(true, FLAGS_config_root));
//    camera->LoadCalibration(camera_param);
    }
    else if (camera_name == "StereoRealsense") {
      camera = shared_ptr<Camera>(new StereoRealsenseCamera(FLAGS_config_root));
    }
    else {
      LOG(ERROR) << "Unrecognized Camera name(should be Realsense or StereoRealsense): " << camera_name;
      return;
    }
  }
  camera->Start();
  cv::Mat color, depth;

  Vision vision(FLAGS_config_root, vision_cfg);
  if (!vision.Init()) {
    LOG(ERROR) << "Vision init failed";
    return;
  }
  Robot robot(FLAGS_config_root, robot_cfg);

  LOG(INFO) << "Vision thread initialized successfully";
  while (true) {
    if (g_close)
      break;
    unique_lock<mutex> lk(mu);
    if (tcp_request == RequestType::IN1 || gui_request == RequestType::IN1) {
      // update frame
      camera->Update();
      camera->FetchDepth(depth);
      camera->FetchColor(color);
      camera->FetchPointCloud(cloud);
      if (cloud->empty()) {
        LOG(WARNING) << "cloud is empty";
        continue;
      }
      if (vision.Process(color, depth, cloud)) {
        results = vision.GetVisionResults();
        auto cMo = results[0].pose_;
        LOG(INFO) << "cMo:\n" << cMo.format(Utils::IOF);
        transformed_models = vision.GetTransformedModels();
        robot.CalculatePose(cMo, target_pose, flag);
        if (tcp_request == RequestType::IN1) {
          tcp_request = RequestType::OUT1;
          gui_request = RequestType::VIP;
          lk.unlock();
          tcp_cv.notify_all();
          gui_cv.notify_all();
        } else if (gui_request == RequestType::IN1) {
          gui_request = RequestType::OUT1;
          lk.unlock();
          gui_cv.notify_all();
        }
      }
    }
  }
  LOG(INFO) << "Vision thread Closed";
}


/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();

  // read conifg
  LOG(INFO) << "Load config from " << FLAGS_config_root + "/main.yml";
  cv::FileStorage fs(FLAGS_config_root + "/main.yml", cv::FileStorage::READ);
  int width, height;
  fs["gui"]["width"] >> width;
  fs["gui"]["height"] >> height;
  string server_address;
  int server_port;
  fs["tcp"]["ip"] >> server_address;
  fs["tcp"]["port"] >> server_port;
  string camera_name, vision_cfg, robot_cfg;
  fs["core"]["camera_name"] >> camera_name;
  fs["core"]["vision_cfg"] >> vision_cfg;
  fs["core"]["robot_cfg"] >> robot_cfg;

  // share variable between TCPThread and VisionThread
  RequestType tcp_request = RequestType::NONE;
  int flag;
  Pose target_pose;
  // share variable between GUIThread and VisionThread
  RequestType gui_request = RequestType::NONE;
  PointCloud<PointType>::Ptr cloud(new PointCloud<PointType>);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> transformed_models;
  std::vector<ObjectInfo> results;

  shared_ptr<thread> vision_thread(new thread(VisionThread, camera_name, vision_cfg, robot_cfg,
                                              std::ref(tcp_request), std::ref(flag), std::ref(target_pose),
                                              std::ref(gui_request), std::ref(cloud), std::ref(transformed_models),
                                              std::ref(results)));
  shared_ptr<thread> gui_thread, tcp_thread;
  if (FLAGS_gui) {
    gui_thread = shared_ptr<thread>(new thread(GUIThread, width, height, argv[0], std::ref(gui_request),
                                               std::ref(cloud), std::ref(transformed_models), std::ref(results)));
  }

  if (FLAGS_tcp) {
    tcp_thread = shared_ptr<thread>(new thread(TCPThread, server_address, server_port,
                                               std::ref(tcp_request), std::ref(flag), std::ref(target_pose)));
  }

  if (FLAGS_gui)
    gui_thread->detach();
  if (FLAGS_tcp)
    tcp_thread->detach();
  vision_thread->detach();

  char key;
  while (cin >> key) {
    if (key == 'q') {
      g_close = true;
      LOG(INFO) << "closing......";
      this_thread::sleep_for(chrono::seconds(1));
      break;
    }
  }

  return 0;
}