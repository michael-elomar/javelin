#include "javelin.hpp"
#include <iostream>

namespace javelin {

Model *Model::create(double mass, Eigen::Matrix3d inertia)
{
	if (mass == 0.0) {
		std::cerr << "Mass cannot be 0" << std::endl;
		return nullptr;
	}

	if (inertia.determinant() == 0.0) {
		std::cerr << "Invalid inertia matrix: it must be invertible"
			  << std::endl;
		return nullptr;
	}

	return new Model(mass, inertia);
}

Model::Model(double mass, Eigen::Matrix3d inertia)
	: mMass(mass), mInertia(inertia)
{
}

Model::~Model() {}

void Model::addWorldForce(Eigen::Vector3d &force)
{
	mTotalForce += force;
}

void Model::addWorldTorque(Eigen::Vector3d &torque)
{
	mTotalTorque += torque;
}

bool Model::isStatic()
{
	return mIsStatic;
}

void Model::update()
{
	mPos += mVel * mTimeStep;
	mVel += (mTotalForce / mMass) * mTimeStep;

	std::cout << "Position: " << mPos << std::endl;
}

} // namespace javelin
