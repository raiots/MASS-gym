#include "MyWorldPtr.h"

using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;


MyWorldPtr::MyWorldPtr():WorldPtr(std::make_shared<World>()){

}

PYBIND11_MODULE(pyMyWorldPtr, m)
{
	py::class_<MyWorldPtr>(m, "pyMyWorldPtr")
    .def(py::init())
    .def("setGravity", &MyWorldPtr::setGravity)
    .def("setTimeStep", &MyWorldPtr::setTimeStep)
    .def("getConstraintSolverSetDetector", &MyWorldPtr::getConstraintSolverSetDetector)
    .def("addSkeleton", &MyWorldPtr::addSkeleton)
    .def("setTime", &MyWorldPtr::setTime)
    .def("getTime", &MyWorldPtr::getTime)
    .def("step", &MyWorldPtr::step)
    .def("reset", &MyWorldPtr::reset);
}

