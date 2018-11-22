//
// Created by yongqi on 17-11-29.
//

#ifndef VFORCE_MRCNNDETECTOR_HPP
#define VFORCE_MRCNNDETECTOR_HPP

#include "Detector.hpp"
#include "Python.h"

namespace VForce {

class MRCNNDetector : public Detector {
 public:
  MRCNNDetector(const std::string &cfg_root = ".", const std::string &cfg_file = "MRCNNDetector.yml");

  virtual bool Detect(const cv::Mat &img, DetectorResults &results);

 private:
  PyObject *pModule_, *pfun_;
};

}
#endif //VFORCE_MRCNNDETECTOR_HPP
