#pragma once

#include <Eigen/Eigen>
#include <vector>

namespace javelin {
class Model {
public:
	Model();
	~Model();

	void addForce(Eigen::Vector3d &force);
	bool isStatic();

	void update();

private:
	Eigen::Vector3d vel{0, 0, 0}, pos{0, 0, 0};
	Eigen::Vector3d totalForce{0, 0, 0};
	double mass{1};
	double mTimeStep{1e-3};
	uint32_t id{0};
	bool mIsStatic{false};
};

class QuadCopter : public Model {};

class Plane : public Model {};

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
