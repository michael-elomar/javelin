#include <javelin.hpp>

int main(int argc, char *argv[])
{
	javelin::World world;
	javelin::Model model;

	world.insertModel(&model);

	Eigen::Vector3d force{0, 0, 1};

	model.addForce(force);

	while (true) {
		world.step();
	}

	return 0;
}
