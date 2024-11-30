#include "MyWorldPtr.h"

using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;
using namespace MASS;


using namespace MASS;

MyWorldPtr::MyWorldPtr(){
    this->WorldPtr(std::make_shared<World>());
}


PYBIND11_MODULE(pyWorldPtr, m)
{
	py::class_<Environment>(m, "pyWorldPtr")
    .def(py::init());
}

