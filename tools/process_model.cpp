//
// Created by yongqi on 1/9/17.
//

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "utils/PointCloudUtils.hpp"

using namespace std;
using namespace pcl;

typedef pcl::PointXYZ PointType;

void normalize_cloud(pcl::PointCloud<PointType>::Ptr cloud) {
  auto center = VForce::Utils::calculate_cloud_center(cloud);
  cout << center << endl;
}

void tranform_cloud(pcl::PointCloud<PointType>::Ptr cloud) {
  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
  float phi = 30.f / 180.f * 3.14f;
  tf(0, 0) = cos(phi);
  tf(0, 1) = -sin(phi);
  tf(1, 0) = sin(phi);
  tf(1, 1) = cos(phi);
  pcl::transformPointCloud(*cloud, *cloud, tf);
}

void zoom_cloud(pcl::PointCloud<PointType>::Ptr cloud, float scale) {
  for (auto &p : *cloud) {
//    cout << p << endl;
    p.x *= scale;
    p.y *= scale;
    p.z *= scale;
  }
}

void sampling_cloud(pcl::PointCloud<PointType>::Ptr cloud, float radius) {
  UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud(cloud);
  uniform_sampling.setRadiusSearch(radius);
#if PCL_VERSION_COMPARE(<, 1, 8, 0)
  PointCloud<int> keypointsIndices;
  uniform_sampling.compute(keypointsIndices);
  copyPointCloud(*cloud, keypointsIndices.points, *cloud);
#else
  uniform_sampling.filter(*cloud);
#endif
}

int main(int argc, char **argv) {
  string input, output;
  input = argv[1];
  output = argv[2];
  int id = 0;
  if (argc > 3) {
    id = stoi(argv[3]);
  }

  pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>);
  pcl::io::loadPCDFile(input, *model);

  switch (id) {
    case 0:
      // normalize cloud
      cout << "Normalize Cloud: " << input << " to " << output << endl;
      normalize_cloud(model);
      break;
    case 1:
      // tranform cloud
      cout << "Transform Cloud: " << input << " to " << output << endl;
      tranform_cloud(model);
      break;
    case 2:zoom_cloud(model, 0.001);
      break;
    case 3:cout << "before size: " << model->size() << endl;
      sampling_cloud(model, 0.001);
      cout << "after size: " << model->size() << endl;
      break;
  }

  pcl::io::savePCDFile(output, *model);
  return 1;
}
