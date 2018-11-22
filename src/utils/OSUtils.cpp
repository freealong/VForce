//
// Created by yongqi on 18-10-24.
//

#include "OSUtils.hpp"

namespace VForce {
namespace Utils {

bool split_path(const std::string &full_path, std::string &path, std::string &name) {
  if (full_path.empty()) {
    return false;
  }
  auto pos = full_path.rfind('/');
  if (pos == std::string::npos) {
    path.assign(".");
    name = full_path;
  } else {
    path = full_path.substr(0, pos);
    name = full_path.substr(pos);
  }
  return true;
}

}
}
