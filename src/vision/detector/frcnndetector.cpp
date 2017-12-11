//
// Created by yongqi on 17-7-18.
//

#include <glog/logging.h>
#include "frcnndetector.hpp"

namespace VForce {

using namespace std;

FRCNNDetector::FRCNNDetector(const string &cfg_file) {
  LOG(INFO) << "Load config file from: " << cfg_file;
  cv::FileStorage fs(cfg_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    LOG(ERROR) << "Open config file failed: " << cfg_file;
  }
  string script_path, script_name;
  fs["script_path"] >> script_path;
  fs["script_name"] >> script_name;
  Py_Initialize();
  // add script_path to python path
  if (!script_path.empty()) {
    PyRun_SimpleString("import sys");
    string append_path = "sys.path.append('" + script_path + "')";
    PyRun_SimpleString(append_path.c_str());
  }
  // import FRCNNDetector.py
  PyObject *pName;
  pName = PyUnicode_FromString(script_name.c_str());
  pModule_ = PyImport_Import(pName);
  // load function detect
  pfun_ = PyObject_GetAttrString(pModule_, "detect");
}

bool FRCNNDetector::Detect(const cv::Mat &img, DetectorResults &reslults) {
  NDArrayConverter converter;
  PyObject *img_ndarray = converter.toNDArray(img);
  PyObject *pArgs = PyTuple_New(1);
  PyTuple_SetItem(pArgs, 0, img_ndarray);
  PyObject *pRes = PyObject_CallObject(pfun_, pArgs);
  int size = PyList_Size(pRes);
  Py_DecRef(img_ndarray);
  if (size == 0) {
    return false;
  } else {
    reslults.clear();
    for (int i = 0; i < size; ++i) {
      ObjectInfo object;
      // get rect
      auto &rect = object.rect_;
      int xmin, ymin, xmax, ymax;
      PyObject *box = PyList_GetItem(pRes, i);
      xmin = PyLong_AsLong(PyList_GetItem(box, 0));
      ymin = PyLong_AsLong(PyList_GetItem(box, 1));
      xmax = PyLong_AsLong(PyList_GetItem(box, 2));
      ymax = PyLong_AsLong(PyList_GetItem(box, 3));
      // make index start with 0
      rect.x = xmin - 1;
      rect.y = ymin - 1;
      rect.width = xmax - xmin;
      rect.height = ymax - ymin;
      assert(rect.x >= 0 && rect.y >= 0 && rect.x + rect.width < img.cols && rect.y + rect.height < img.rows);
      // @TODO: class id are not available
      object.valid_mask_ = false;
      object.id_ = 1;
      reslults.emplace_back(object);
    }
  }
  // using segment to get object mask
  for (auto &r : reslults) {
    r.mask_ = segment_.GetSegmentMask(img(r.rect_));
    r.valid_mask_ = true;
  }
  return true;
}

}
