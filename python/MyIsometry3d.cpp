#include "MyIsometry3d.h"

MyIsometry3d::MyIsometry3d(Eigen::Isometry3d m){
    this->_m = m;
}

MyIsometry3d::MyIsometry3d(){
    this->_m = Eigen::Isometry3d::Identity();
}

PYBIND11_MODULE(pyMyIsometry3d, m)
{
	py::class_<MyIsometry3d>(m, "pyMyIsometry3d")
    .def(py::init())
	.def(py::init<Eigen::Isometry3d>())
    .def("inverse", &MyIsometry3d::inverse)
    .def("to_matrix", &MyIsometry3d::to_matrix)
    .def("reset_translation_y", &MyIsometry3d::reset_translation_y);
}

