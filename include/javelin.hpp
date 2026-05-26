#pragma once

#include <Eigen/Eigen>
#include <vector>

namespace javelin {
class Model {

private:
	Model(double mass, Eigen::Matrix3d inertia);

public:
	~Model();
	static Model *create(double mass, Eigen::Matrix3d inertia);
	void addWorldForce(Eigen::Vector3d &force);
	void addWorldTorque(Eigen::Vector3d &torque);
	bool isStatic();

	void preUpdate();
	void update();
	void postUpdate();

private:
	Eigen::Vector3d mVel{0, 0, 0};
	Eigen::Vector3d mPos{0, 0, 0};
	Eigen::Vector3d mAttitude{0, 0, 0};
	Eigen::Vector3d mTotalForce{0, 0, 0};
	Eigen::Vector3d mTotalTorque{0, 0, 0};

	double mMass;
	Eigen::Matrix3d mInertia;
	double mTimeStep{1e-3};
	uint32_t id{0};
	bool mIsStatic{false};
};

class QuadCopter : public Model {
};

class Plane : public Model {
};

class World {
public:
	World();
	~World();

	bool insertModel(Model *modelPtr);

	bool step();

private:
	std::vector<Model *> mModels;
	double mTimeStep{1e-3};
};

} // namespace javelin
