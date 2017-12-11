//
// Created by yongqi on 17-12-11.
//

#include "matcher.hpp"
#include <pcl/search/impl/kdtree.hpp>

using namespace std;

namespace VForce {

double Matcher::CalculateMatchError(const PointTCloudPtr &model, const PointTCloudPtr &target, double max_dist) {
  if (model->size() == 0 || target->size() == 0)
    return 1;

  // initialize a Kd tree for searching
  pcl::search::KdTree<PointT> search;
  search.setInputCloud(target);

  vector<int> nn_indices;
  vector<float> nn_dists;
  set<int> indices;

  double fitness_score = 0;
  int nr = 0;
  // for each point, find the closest point in the other pointcloud
  for (PointT p: model->points) {
    search.nearestKSearch(p, 1, nn_indices, nn_dists);
    if (nn_dists[0] <= max_dist) {
      fitness_score += nn_dists[0];
      nr++;
    }
  }

  if (nr > 0)
    return (fitness_score / (double) nr);
  else
    return (std::numeric_limits<double>::max());
}

}