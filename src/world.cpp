#include "javelin.hpp"
#include <iostream>
#include <signal.h>

namespace javelin {

class SimTimerHandler : public neutron::Timer::Handler {
public:
	SimTimerHandler(World *world) : mWorld(world) {}
	virtual ~SimTimerHandler() override {}
	inline virtual void processTimer() override
	{
		mWorld->step();
	}

private:
	World *mWorld;
};

World::World()
{
	World::sInstance = this;

	mLogger = new TelemetryLogger();

	mLoop = new neutron::Loop();
	mHandler = new SimTimerHandler(this);
	mTimer = new neutron::Timer(mLoop, mHandler);
	mTimer->set(1, 1);
}

World::~World()
{
	delete mLogger;
	delete mHandler;
	delete mTimer;
	delete mLoop;
}

void World::sigHandler(int signum)
{
	sInstance->mRunning = false;
	sInstance->mLoop->wakeup();
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

void World::updateForces(Model *model)
{
	model->addWorldForce({0, 0, 1});
}

bool World::step()
{
	for (auto modelPtr : mModels) {
		if (!modelPtr->isStatic()) {
			updateForces(modelPtr);
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
	int ret = 0;

	signal(SIGINT, &sigHandler);
	signal(SIGTERM, &sigHandler);
	mRunning = true;

	while (mRunning) {
		ret = mLoop->spin();
		if (ret > 0) {
			std::cout << "Error processing neutron loop\n";
			return false;
		}
	}
	return false;
}

bool World::spin(double ts)
{
	int ret = 0;

	signal(SIGINT, &sigHandler);
	signal(SIGTERM, &sigHandler);
	mRunning = true;

	while (mSimTime <= ts && mRunning) {
		ret = mLoop->spin();
		if (ret > 0) {
			std::cout << "Simulation failed prematurely\n";
			return false;
		}
	}
	mRunning = false;
	return true;
}
} // namespace javelin
