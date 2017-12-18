// System

// STL
#include <iostream>

// 3rd Libraries
#include <pcl/io/pcd_io.h>

// User Defined
#include "camera/RealsenseCamera.hpp"
#include "camera/StereoRealsenseCamera.hpp"
#include "utils/GLCloudViewer.hpp"
#include "utils/CVUtils.hpp"

/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace pcl;
using namespace VForce;

int main(int argc, char **argv) {
  typedef PointXYZRGBA PointType;
  int save_id = 1;
  string save_path(".");
  if (argc == 2)
    save_path = argv[1];

  cout << "begin to init..." << endl;

//  LinemodDetector detector;
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr template_cloud;
  cout << "initialize PointCloudHandler successfully" << endl;

  // using camera variables
//  shared_ptr<Camera> camera(new RealsenseCamera(true, "../config"));
  shared_ptr<Camera> camera(new StereoRealsenseCamera("../config"));
  cv::Mat color, depth;
  camera->Start();
  cout << "load camera successfully..." << endl;

  // Open a GLFW window to display our output
  Utils::GLCloudViewer viewer(1280, 960, "Data Collector");
  viewer.InitWindow();
  cout << "load opengl successfully..." << endl;

  cout << "begin to loop..." << endl;
  while (viewer.NotStop()) {
    viewer.ProcessEvents();

    if (viewer.ReadyToUpdate()) {
      // update frame
      camera->Update();
      camera->FetchDepth(depth);
      camera->FetchColor(color);
      camera->FetchPointCloud(cloud);
    }

    // draw
    viewer.ShowCloud(cloud);
//    viewer.ShowCloud(template_cloud, 1);

    // save cloud
    if (viewer.NeedToSave()) {
      pcl::io::savePCDFileBinaryCompressed(save_path + "/cloud" + to_string(save_id) + ".pcd", *cloud);
      Utils::write_depth(save_path + "/depth" + to_string(save_id) + ".png", depth);
      cv::imwrite(save_path + "/color" + to_string(save_id) + ".png", color);
      viewer.FinishSave();
      ++save_id;
      cout << "data saved " << to_string(save_id) << endl;
    }

    viewer.SwapBuffer();
  }

  return 0;
}
