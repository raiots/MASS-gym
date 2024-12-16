#include "MyBodyNodePtr.h"

MyBodyNodePtr::MyBodyNodePtr(BodyNode *ptr){
	this->ptr = ptr;
}

PYBIND11_MODULE(pyMyBodyNodePtr, m)
{
	py::class_<MyBodyNodePtr>(m, "pyMyBodyNodePtr")
	.def(py::init<BodyNode*>())
	.def("getCOM", &MyBodyNodePtr::getCOM);
}