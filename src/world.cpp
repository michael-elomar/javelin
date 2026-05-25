#include "javelin.hpp"

namespace javelin {

World::World() {}

World::~World() {}

bool World::insertModel(Model *model)
{
	mModels.push_back(model);
	return true;
}

bool World::step()
{
	for (auto modelPtr : mModels) {

		if (!modelPtr->isStatic())
			modelPtr->update();
	}
	return true;
}
} // namespace javelin
