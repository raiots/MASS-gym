#ifndef __MYISOMETRY3D__
#define __MYISOMETRY3D__

#include <Eigen/Core>
#include <Eigen/Geometry>
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


class MyIsometry3d
{
    protected:
    Eigen::Isometry3d _m;

    public:
    MyIsometry3d(Eigen::Isometry3d);

    MyIsometry3d();
    void reset_translation_y(){
        this->_m.translation()[1] = 0;
    }
    MyIsometry3d inverse(){
        return MyIsometry3d(this->_m.inverse());
    }

    Eigen::Matrix4d to_matrix(){
        Eigen::Matrix4d trans;
        trans.block<3, 3>(0, 0) = this->_m.linear();
        trans.block<3, 1>(0, 3) = this->_m.translation();
        return trans;
    }

};


#endif
