#include "MySkeletonPtr.h"

using namespace dart::dynamics;

MySkeletonPtr::MySkeletonPtr():SkeletonPtr(){
    
}

MySkeletonPtr::MySkeletonPtr(const SkeletonPtr& ptr): SkeletonPtr(ptr)
{
   
}


PYBIND11_MODULE(pyMySkeletonPtr, m)
{
	py::class_<MySkeletonPtr>(m, "pyMySkeletonPtr")
    .def(py::init())
    .def("getNumDofs", &MySkeletonPtr::getNumDofs)
    .def("clearConstraintImpulses", &MySkeletonPtr::clearConstraintImpulses)
    .def("clearInternalForces", &MySkeletonPtr::clearInternalForces)
    .def("clearExternalForces", &MySkeletonPtr::clearExternalForces)
    .def("setPositions", &MySkeletonPtr::setPositions)
    .def("setVelocities", &MySkeletonPtr::setVelocities)
    .def("getPositions", &MySkeletonPtr::getPositions)
    .def("getVelocities", &MySkeletonPtr::getVelocities)
    .def("computeForwardKinematics", &MySkeletonPtr::computeForwardKinematics)
    .def("getBodyNode0TransformTranslation_y", &MySkeletonPtr::getBodyNode0TransformTranslation_y)
    .def("getBodyNode_i_getCOM_root", &MySkeletonPtr::getBodyNode_i_getCOM_root)
    .def("getBodyNode_0_getCOMLinearVelocity", &MySkeletonPtr::getBodyNode_0_getCOMLinearVelocity)
    .def("getBodyNode_i_getCOMLinearVelocity", &MySkeletonPtr::getBodyNode_i_getCOMLinearVelocity)
    .def("getRootBodyNode_getCOM_y", &MySkeletonPtr::getRootBodyNode_getCOM_y)
    .def("getNumBodyNodes", &MySkeletonPtr::getNumBodyNodes)
    .def("setForces", &MySkeletonPtr::setForces)
    .def("getPositionDifferences", &MySkeletonPtr::getPositionDifferences)
    .def("getRootBodyNodeParentJointType", &MySkeletonPtr::getRootBodyNodeParentJointType)
    .def("getBodyNodeByName_getParentJoint_getIndexInSkeleton", &MySkeletonPtr::getBodyNodeByName_getParentJoint_getIndexInSkeleton)
    .def("getBodyNodeByName_getParentJoint_getType", &MySkeletonPtr::getBodyNodeByName_getParentJoint_getType)
    .def("getCOM", &MySkeletonPtr::getCOM)
    .def("getBodyNode", &MySkeletonPtr::getBodyNode)
    .def("getTimeStep", &MySkeletonPtr::getTimeStep)
    .def("getMassMatrix", &MySkeletonPtr::getMassMatrix)
    .def("getConstraintForces", &MySkeletonPtr::getConstraintForces)
    .def("getCoriolisAndGravityForces", &MySkeletonPtr::getCoriolisAndGravityForces);
}

