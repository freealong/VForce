//
// Created by yongqi on 17-11-29.
//

#include <glog/logging.h>
#include "MRCNNDetector.hpp"

namespace VForce {

using namespace std;

MRCNNDetector::MRCNNDetector(const std::string &cfg_root, const std::string &cfg_file) : Detector(cfg_root, cfg_file) {
  auto cfg_path = cfg_root_ + "/" + cfg_file_;
  LOG(INFO) << "Load config file from: " << cfg_path;
  cv::FileStorage fs(cfg_path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Open config file failed: " << cfg_path;
  }
  string script_path, script_name, model;
  fs["script_path"] >> script_path;
  fs["script_name"] >> script_name;
  fs["model"] >> model;
  Py_Initialize();
  if (!script_path.empty()) {
    PyRun_SimpleString("import sys");
    string append_path = "sys.path.append('" + script_path + "')";
    PyRun_SimpleString(append_path.c_str());
  }
  // import module
  PyObject *pName;
  pName = PyUnicode_FromString(script_name.c_str());
  pModule_ = PyImport_Import(pName);
  // load model
  auto fun = PyObject_GetAttrString(pModule_, "load_model");
  PyObject *pArgs = PyTuple_New(1);
  PyTuple_SetItem(pArgs, 0, PyUnicode_FromString(model.c_str()));
  PyObject_CallObject(fun, pArgs);
  // load detect function
  pfun_ = PyObject_GetAttrString(pModule_, "detect");
}

// @FIXME: mask is not right
bool MRCNNDetector::Detect(const cv::Mat &img, DetectorResults &results) {
  NDArrayConverter converter;
  PyObject *img_ndarray = converter.toNDArray(img);
  PyObject *pArgs = PyTuple_New(1);
  PyTuple_SetItem(pArgs, 0, img_ndarray);
  PyObject *pRes = PyObject_CallObject(pfun_, pArgs);
  int size = static_cast<int>(PyList_Size(pRes));
  Py_DecRef(img_ndarray);
  if (size == 0) {
    return false;
  } else {
    results.clear();
    for (int i = 0; i < size; ++i) {
      ObjectInfo object;
      PyObject *res = PyList_GetItem(pRes, i);
      // get rect
      cv::Rect &rect = object.rect_;
      PyObject *box = PyList_GetItem(res, 0);
      rect.x = static_cast<int>(PyLong_AsLong(PyList_GetItem(box, 0)));
      rect.y = static_cast<int>(PyLong_AsLong(PyList_GetItem(box, 1)));
      rect.width = static_cast<int>(PyLong_AsLong(PyList_GetItem(box, 2)));
      rect.height = static_cast<int>(PyLong_AsLong(PyList_GetItem(box, 3)));
      assert(rect.x >= 0 && rect.y >= 0 && rect.x + rect.width < img.cols && rect.y + rect.height < img.rows);
      // get mask
      PyObject *mask = PyList_GetItem(res, 1);
      object.mask_ = converter.toMat(mask);
      object.valid_mask_ = true;
      // get class_id
      PyObject *class_id = PyList_GetItem(res, 2);
      object.id_ = static_cast<int>(PyLong_AsLong(class_id));
      results.emplace_back(object);
    }
  }
  return true;
}

}
