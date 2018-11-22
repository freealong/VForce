//
// Created by yongqi on 18-10-24.
//

#ifndef PICKPACK_OSUTILS_HPP
#define PICKPACK_OSUTILS_HPP

#include <string>
#include <memory>

namespace VForce {
namespace Utils {

/**
 * split a file's full path name into path and filename
 * @param full_path
 * @param path
 * @param name
 * @return
 */
bool split_path(const std::string &full_path, std::string &path, std::string &name);

/**
 * format string using c style
 * @tparam Args
 * @param format
 * @param args
 * @return
 */
template<typename ... Args>
std::string string_format(const std::string &format, Args ... args) {
  int size = snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
  std::unique_ptr<char[]> buf(new char[size]);
  snprintf(buf.get(), size, format.c_str(), args ...);
  return std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
}

}
}

#endif //PICKPACK_OSUTILS_HPP
