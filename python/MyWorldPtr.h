#ifndef __MyWORLDPTR_H__
#define __MyWORLDPTR_H__
#include "dart/dart.hpp"

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <utility>
namespace py = pybind11;

using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;

namespace MASS
{
    class MyWorldPtr: public dart::simulation::WorldPtr{
        
    };


};

#endif
