//
// Created by yongqi on 17-7-18.
//

#ifndef VFORCE_FRCNNDETECTOR_HPP
#define VFORCE_FRCNNDETECTOR_HPP

#include "Detector.hpp"
#include "NDArrayConverter.hpp"
#include "LineSegment.hpp"

namespace VForce {

class FRCNNDetector : public Detector {
 public:
  FRCNNDetector(const std::string &cfg_root = ".", const std::string &cfg_file = "FRCNNDetector.yml");

  virtual bool Detect(const cv::Mat &img, DetectorResults &results);

 private:
  PyObject *pModule_, *pfun_;
  LineSegment segment_;
};

}
#endif //VFORCE_FRCNNDETECTOR_HPP
