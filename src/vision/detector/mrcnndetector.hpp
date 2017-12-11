//
// Created by yongqi on 17-11-29.
//

#ifndef VFORCE_MRCNNDETECTOR_HPP
#define VFORCE_MRCNNDETECTOR_HPP

#include "detector.hpp"
#include "pythonconversion.hpp"

namespace VForce {

class MRCNNDetector : public Detector {
 public:
  MRCNNDetector(const std::string &cfg_file);

  virtual bool Detect(const cv::Mat &img, DetectorResults &results);

 private:
  PyObject *pModule_, *pfun_;
};

}
#endif //VFORCE_MRCNNDETECTOR_HPP
