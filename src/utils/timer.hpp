//
// Created by yongqi on 17-12-10.
//

#ifndef VFORCE_TIMER_HPP
#define VFORCE_TIMER_HPP

#include <iostream>
#include <chrono>

namespace VForce {
namespace Utils {

class Timer {
 public:
  typedef std::chrono::high_resolution_clock clock;
  typedef std::chrono::nanoseconds timestep;

  Timer() {
    reset();
  }

  void reset() {
    beg_ = clock::now();
  }

  inline timestep elapsed() const {
    return std::chrono::duration_cast<timestep>(clock::now() - beg_);
  }

  friend std::ostream& operator<<(std::ostream &os, const Timer &t) {
    return os << t.elapsed().count();
  }

 private:
  clock::time_point beg_;
};

}
}
#endif //VFORCE_TIMER_HPP
