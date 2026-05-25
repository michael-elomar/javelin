#include "javelin.hpp"
#include <iostream>

namespace javelin {

Model::Model() {}

Model::~Model() {}

void Model::addForce(Eigen::Vector3d &force)
{
	totalForce += force;
}

bool Model::isStatic()
{
	return mIsStatic;
}

void Model::update()
{
	pos += vel * mTimeStep;
	vel += (totalForce / mass) * mTimeStep;

	std::cout << "Position: " << pos << std::endl;
}

} // namespace javelin
