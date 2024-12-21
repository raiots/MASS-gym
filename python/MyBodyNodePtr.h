#ifndef __MYBODYNODEPTR_H__
#define __MYBODYNODEPTR_H__

#include <Eigen/Core>
#include <string>
#include <fstream>
#include <vector>
#include <map>
#include <utility>
#include <initializer_list>
#include <stdexcept>
#include "dart/dart.hpp"
#include "MySkeletonPtr.h"

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace dart::dynamics;
class MySkeletonPtr;

class MyBodyNodePtr
{
    protected:
    BodyNode* ptr;

    public:
        MyBodyNodePtr(BodyNode *);
        BodyNode* getPtr() { return this->ptr;}
        Eigen::Vector3d getCOM() const{
            return (this->ptr)->getCOM();
        }
        MyBodyNodePtr getParentBodyNode(){
            return MyBodyNodePtr(this->ptr->getParentBodyNode());
        }

        bool isNull()
        {
            return this->ptr == nullptr;
        }
        std::size_t getIndexInSkeleton() const
        {
            if(this->ptr)
                return (this->ptr->getIndexInSkeleton());
            else
            {
                std::string info = "ERROR: the pointer to the body node is null, can't get the index in skeleton.";
                std::cout << info << std::endl;
                throw std::invalid_argument(info);
            }
        }

        Eigen::Matrix4d getTransform(){
            Eigen::Isometry3d i3d = this->ptr->getTransform();
            Eigen::Matrix4d m;
            m.block<3, 3>(0, 0) = i3d.linear();
            m.block<3, 1>(0, 3) = i3d.translation();
            return m;
        }

        Eigen::Matrix4d getTransform_inverse()const
        {
            Eigen::Isometry3d i3d = (this->ptr->getTransform().inverse());
            Eigen::Matrix4d m;
            m.block<3, 3>(0, 0) = i3d.linear();
            m.block<3, 1>(0, 3) = i3d.translation();
            return m;
        }

        void addExtForce(const Eigen::Vector3d& _force, const Eigen::Vector3d &_offset, bool _isForceLocal, bool _isOffsetLocal)
        {
            this->ptr->addExtForce(_force, _offset, _isForceLocal, _isOffsetLocal);
        }

        MySkeletonPtr getSkeleton ();
};


#endif