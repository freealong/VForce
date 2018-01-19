//
// Created by yongqi on 18-1-19.
//

#include <pcl/io/pcd_io.h>
#include "camera/Camera.hpp"

using namespace std;
using namespace VForce;

int main(int argc, char **argv) {
  if (argc < 4) {
    cout << "Usage: ./cloud2hha data_path first_frame_id, last_frame_id [0|1 show hha]" << endl;
  }
  string path = argv[1];
  int beg_id = stoi(argv[2]);
  int end_id = stoi(argv[3]);
  bool show = false;
  if (argc == 5)
    show = stoi(argv[4]) != 0;
  cv::Mat wMb = cv::Mat::eye(4, 4, CV_64F);
  cv::Mat cMw = wMb.clone();
  cMw.at<double>(1, 1) = cMw.at<double>(2, 2) = -1;
  cMw.at<double>(2, 3) = 0.5;
  for (int i = beg_id; i <= end_id; ++i) {
    std::ostringstream frame_prefix;
    frame_prefix << std::setw(6) << std::setfill('0') << i;
    std::string cloud_frame_filename = path + "/frame-" + frame_prefix.str() + "_cloud.pcd";
    std::string hha_frame_filename = path + "/frame-" + frame_prefix.str() + "_hha.png";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(cloud_frame_filename, *cloud) == -1) {
      continue;
    }
    cv::Mat hha = Camera::Cloud2HHA(cloud, cMw, wMb, 0.2f, 1.f, -0.05f, 0.3f);
    cv::imwrite(hha_frame_filename, hha);
    float progress = (float)(i - beg_id + 1) / (end_id -beg_id + 1);
    std::cout << "Completed: " << (int)(progress * 100) << "%\r";
    std::cout.flush();
    if (show) {
      std::vector<cv::Mat> vec_hha(3);
      cv::split(hha, vec_hha);
      cv::imshow("disparity", vec_hha[0]);
      cv::imshow("height", vec_hha[1]);
      cv::imshow("angle", vec_hha[2]);
      cv::waitKey();
    }
  }
  return 0;
}
