#include "MyBodyNodePtr.h"

MyBodyNodePtr::MyBodyNodePtr(BodyNode *ptr){
	this->ptr = ptr;
}

MySkeletonPtr MyBodyNodePtr::getSkeleton(){
    return MySkeletonPtr(this->ptr->getSkeleton());
}

PYBIND11_MODULE(pyMyBodyNodePtr, m)
{
	py::class_<MyBodyNodePtr>(m, "pyMyBodyNodePtr")
	.def(py::init<BodyNode*>())
	.def("getCOM", &MyBodyNodePtr::getCOM)
	.def("isNull", &MyBodyNodePtr::isNull)
	.def("getParentBodyNode", &MyBodyNodePtr::getParentBodyNode)
	.def("getIndexInSkeleton", &MyBodyNodePtr::getIndexInSkeleton)
	.def("addExtForce", &MyBodyNodePtr::addExtForce)
	.def("getTransform", &MyBodyNodePtr::getTransform)
	.def("getSkeleton", &MyBodyNodePtr::getSkeleton)
	.def("getTransform_inverse", &MyBodyNodePtr::getTransform_inverse);
}