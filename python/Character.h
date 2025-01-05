#ifndef __MASS_CHARACTER_H__
#define __MASS_CHARACTER_H__
#include "dart/dart.hpp"
#include "BVH.h"
#include "Muscle.h"
#include "MySkeletonPtr.h"
#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <utility>
namespace py = pybind11;

using namespace MASS;

class MASS::BVH;

class Character
{
public:
	Character();

	void LoadSkeleton(const std::string& path,bool create_obj = false);
	void LoadMuscles(const std::string& path);
	void LoadBVH(const std::string& path,bool cyclic=true);

	void Reset();	
	void SetPDParameters(double kp, double kv);
	void AddEndEffector(const std::string& body_name){mEndEffectors.push_back(mSkeleton->getBodyNode(body_name));}
	Eigen::VectorXd GetSPDForces(const Eigen::VectorXd& p_desired);

	Eigen::VectorXd GetTargetPositions(double t,double dt);
	std::pair<Eigen::VectorXd,Eigen::VectorXd> GetTargetPosAndVel(double t,double dt);
	
	
	MySkeletonPtr& GetSkeleton(){return mSkeleton;}
	int GetNumOfMuscles() {
		return mMuscles.size();
	}
	const std::vector<Muscle*>& GetMuscles() 
	{
		return mMuscles;
	}

	const Muscle& getMuscleAt(unsigned i){
		return *(mMuscles[i]);
	}

	void setMuscleAt(unsigned i, const Muscle& m){
		*mMuscles[i] = m;
	}

	bool hasMuscles() {return true;}

	const std::vector<dart::dynamics::BodyNode*>& GetEndEffectors(){return mEndEffectors;}
	BVH* GetBVH(){return mBVH;}
	double GetBVH_GetMaxTime(){
		return this->GetBVH()->GetMaxTime();
	}

	const std::map<std::string,std::string>& GetBVH_GetBVHMap(){
		return this->GetBVH()->GetBVHMap();
	}

	std::size_t GetEndEffectors_size(){
		return this->GetEndEffectors().size();
	}

	Eigen::Vector3d GetEndEffectors_i_getCOM(int i){
		return this->GetEndEffectors()[i]->getCOM();
	}

public:
	MySkeletonPtr mSkeleton;
	BVH* mBVH;
	Eigen::Isometry3d mTc;

	std::vector<Muscle*> mMuscles;
	std::vector<dart::dynamics::BodyNode*> mEndEffectors;

	Eigen::VectorXd mKp, mKv;

};


#endif
