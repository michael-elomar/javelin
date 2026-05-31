#include <javelin.hpp>

int main(int argc, char *argv[])
{
	Eigen::Matrix3d inertia{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	};

	javelin::World world;
	javelin::Model *model = javelin::Model::create(1.0, inertia, "body");

	world.insertModel(model);

	Eigen::Vector3d force{0, 0, 1}, point{1, 0, 0}, torque{0, 0, 1};

	model->addRelativeTorque(torque);

	world.spin(10);

	delete model;

	return 0;
}
