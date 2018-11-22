//
// Created by yongqi on 17-12-10.
//

#ifndef VFORCE_TIMER_HPP
#define VFORCE_TIMER_HPP

#include <iostream>
#include <chrono>

namespace VForce {
namespace Utils {

template <typename timestep>
class Timer {
 public:
  typedef std::chrono::high_resolution_clock clock;

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

#if ENABLE_TIMING
#define TIMING_INIT   static auto start_timing = std::chrono::high_resolution_clock::now();\
    std::chrono::duration<double> elapsed_timing(0)

#define START_TIMING  start_timing = std::chrono::high_resolution_clock::now()
#define END_TIMING(text_timing)   elapsed_timing = std::chrono::high_resolution_clock::now() - start_timing;\
    text_timing << " used: " << elapsed_timing.count()*1000.0 << " ms\n"
#else
#define TIMING_INIT
#define START_TIMING
#define END_TIMING(text_timing)
#endif

}
}
#endif //VFORCE_TIMER_HPP
