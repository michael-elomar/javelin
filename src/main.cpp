#include <cmath>
#include <javelin.hpp>
#include <math.h>

int main(int argc, char *argv[])
{
	Eigen::Matrix3d inertia{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	};

	javelin::World *world = new javelin::World();
	javelin::Model *model = javelin::Model::create(1.0, inertia, "body");

	// javelin::Sensor *imu = new javelin::ImuSensor("@unix:imu");
	// model->addSensor(imu);

	world->insertModel(model);

	world->spin(10);

	delete model;
	delete world;

	return 0;
}
