//
// Created by yongqi on 17-5-4.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include "camera/RealsenseCamera.hpp"

using namespace std;
using namespace cv;
using namespace VForce;

int main(int argc, char *argv[]) {
  string output("camParams.yml");
  if (argc > 1)
    output = argv[1];
  RealsenseCamera camera(false, "../config");
  camera.Start();
  camera.SaveCalibration(output);
  return 0;
}