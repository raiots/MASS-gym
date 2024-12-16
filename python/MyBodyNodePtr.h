#ifndef __MYBODYNODEPTR_H__
#define __MYBODYNODEPTR_H__

#include <Eigen/Core>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <utility>
#include <initializer_list>
#include "dart/dart.hpp"

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace dart::dynamics;

class MyBodyNodePtr
{
    protected:
    BodyNode* ptr;

    public:
        MyBodyNodePtr(BodyNode *);
        BodyNode* getPtr() { return this->ptr;}
        Eigen::Vector3d getCOM() const{
            return (this->ptr)->getCOM();
        }

};


#endif