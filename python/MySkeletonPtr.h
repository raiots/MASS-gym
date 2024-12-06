#ifndef __MySKELETONPTR_H__
#define __MySKELETONPTR_H__
#include "dart/dart.hpp"

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <utility>
namespace py = pybind11;

using namespace dart::dynamics;


class MySkeletonPtr: public SkeletonPtr{
public:
    MySkeletonPtr();
};

#endif
