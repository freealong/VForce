//
// Created by yongqi on 17-12-7.
//

#include "Detector.hpp"
#include "utils/CVUtils.hpp"

namespace VForce {

cv::Mat Detector::VisualizeResults(const cv::Mat &img, const DetectorResults &results) const {
  cv::Mat color = img.clone();
  for (const auto &r: results) {
    cv::rectangle(color, r.rect_, cv::Scalar(255, 0, 0));
    cv::putText(color, std::to_string(r.id_), cv::Point(r.rect_.x, r.rect_.y),
                cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255));
    if (r.valid_mask_) {
      cv::Mat roi = color(r.rect_);
      Utils::apply_mask(roi, r.mask_, Utils::random_color());
    }
  }
  return color;
}

}
