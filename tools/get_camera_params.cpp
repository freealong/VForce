//
// Created by yongqi on 17-5-4.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include "camera/realsensecamera.hpp"

using namespace std;
using namespace cv;
using namespace VForce;

int main(int argc, char *argv[]) {
  string output("camParam.yml");
  if (argc > 1)
    output = argv[1];
  RealSenseCamera camera;
  camera.Start();
  camera.SaveCalibration(output);
  return 0;
}