#include "javelin.hpp"
#include <iostream>

namespace javelin {

World::World()
{
	mLogger = new TelemetryLogger();
}

World::~World()
{
	delete mLogger;
}

bool World::insertModel(Model *model)
{
	mModels.push_back(model);
	model->setWorld(this);
	model->setLogger(mLogger);
	return true;
}

double World::getSimTime()
{
	return mSimTime;
}

bool World::step()
{
	for (auto modelPtr : mModels) {
		if (!modelPtr->isStatic()) {
			modelPtr->preUpdate();
			modelPtr->update();
			modelPtr->postUpdate();
		}
	}
	mSimTime += mTimeStep;
	return true;
}

bool World::spin()
{
	bool ret = false;
	while (true) {
		ret = step();
		if (!ret) {
			std::cerr << "Simulation failed\n";
			return false;
		}
	}
	return false;
}

bool World::spin(double ts)
{
	bool ret = false;
	while (mSimTime <= ts) {
		ret = step();
		if (!ret) {
			std::cerr << "Simulation failed prematurely\n";
			return false;
		}
	}
	return true;
}
} // namespace javelin
