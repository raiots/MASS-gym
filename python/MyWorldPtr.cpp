#include "MyWorldPtr.h"

using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;
using namespace MASS;


MyWorldPtr::MyWorldPtr(){
    WorldPtr(std::make_shared<World>());
}

PYBIND11_MODULE(pyMyWorldPtr, m)
{
	py::class_<MyWorldPtr>(m, "pyMyWorldPtr")
    .def(py::init());
}

