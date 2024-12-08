#ifndef __MyWORLDPTR_H__
#define __MyWORLDPTR_H__
#include "dart/dart.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/bullet/BulletCollisionDetector.hpp"
#include "MySkeletonPtr.h"

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <utility>

#include <string>
namespace py = pybind11;

using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;
using namespace dart::constraint;
using namespace dart::collision;


class MyWorldPtr: public WorldPtr{
public:
    MyWorldPtr();   
    void setGravity(const Eigen::Vector3d& _gravity){
        (*this)->setGravity(_gravity);
    }

    void setTimeStep(double _timeStep){
        (*this)->setTimeStep(_timeStep);
    }

    void getConstraintSolverSetDetector(){
        return (*this)->getConstraintSolver()->setCollisionDetector(BulletCollisionDetector::create());
    }

    std::string addSkeleton(const MySkeletonPtr& _skeleton){
        return (*this)->addSkeleton(_skeleton);
    }   

    void reset()
    {
        (*this)->reset();
    }

    void setTime(double _time)
    {
        (*this)->setTime(_time);
    }

    double getTime(){
        return (*this)->getTime();
    }

    void step(){
        bool _resetCommand = true;
        return (*this)->step(_resetCommand); 
    }
    
    
};




#endif
