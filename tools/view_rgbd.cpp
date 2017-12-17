//
// Created by yongqi on 12/7/16.
//

// System

// STL
#include <iostream>

// 3rd Libraries

// User Defined
#include "camera/RealsenseCamera.hpp"
#include "camera/StereoRealsenseCamera.hpp"
#include "utils/CVUtils.hpp"

using namespace std;
using namespace VForce;

/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  shared_ptr<Camera> camera(new RealsenseCamera(true, "../config"));
  camera->Start();

  cv::namedWindow("Depth Frame", 1);
  cv::namedWindow("RGB Frame", 1);

  cv::Mat color;
  cv::Mat depth;
  int save_id = 1;
  while (true) {
//    this_thread::sleep_for(std::chrono::milliseconds(1000));
    camera->Update();
    camera->FetchColor(color);
    camera->FetchDepth(depth);

    imshow("Depth Frame", depth);
    imshow("RGB Frame", color);
    char key = cv::waitKey(10);
    if (key > 0)
      cout << "pressed key: " << key << endl;
    if (key == 27) // key == esc
      break;
    else if (key == 115) {
      Utils::write_depth("depth" + to_string(save_id) + ".png", depth);
      cv::imwrite("color" + to_string(save_id) + ".png", color);
      save_id++;
    } // key = s
  }

//  camera->Stop();
  cv::destroyAllWindows();

  return 0;
}
