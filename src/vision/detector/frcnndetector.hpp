//
// Created by yongqi on 17-7-18.
//

#ifndef VFORCE_FRCNNDETECTOR_HPP
#define VFORCE_FRCNNDETECTOR_HPP

#include "detector.hpp"
#include "pythonconversion.hpp"
#include "linesegment.hpp"

namespace VForce {

class FRCNNDetector : public Detector {
 public:
  FRCNNDetector(const std::string &cfg_file);

  virtual bool Detect(const cv::Mat &img, DetectorResults &results);

 private:
  PyObject *pModule_, *pfun_;
  LineSegment segment_;
};

}
#endif //VFORCE_FRCNNDETECTOR_HPP
