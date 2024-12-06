#include "MySkeletonPtr.h"

using namespace dart::dynamics;

MySkeletonPtr::MySkeletonPtr(){
    MySkeletonPtr();
}


PYBIND11_MODULE(pyMySkeletonPtr, m)
{
	py::class_<MySkeletonPtr>(m, "pyMySkeletonPtr")
    .def(py::init());
}

