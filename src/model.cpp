#include "javelin.hpp"
#include <iostream>

namespace javelin {

static Eigen::Vector4d updateQuaternion(const Eigen::Vector3d &angularVel,
					const Eigen::Vector4d &quat,
					double timeStep)
{
	Eigen::Matrix4d updateMatrix{
		{0, -angularVel[0], -angularVel[1], -angularVel[2]},
		{angularVel[0], 0, angularVel[2], -angularVel[1]},
		{angularVel[1], -angularVel[2], 0, angularVel[0]},
		{angularVel[2], angularVel[1], -angularVel[0], 0},
	};
	return 0.5 * (updateMatrix * quat) * timeStep;
}

static Eigen::Vector3d attitudeFromQuaternion(const Eigen::Vector4d &quat)
{

	double roll =
		std::atan2(2 * (quat[0] * quat[1] + quat[2] * quat[3]),
			   quat[0] * quat[0] + quat[3] * quat[3]
				   - quat[1] * quat[1] - quat[2] * quat[2]);
	double pitch = std::asin(2 * (quat[0] * quat[2] - quat[1] * quat[3]));
	double yaw =
		std::atan2(2 * (quat[0] * quat[3] + quat[1] * quat[2]),
			   quat[0] * quat[0] + quat[1] * quat[1]
				   - quat[2] * quat[2] - quat[3] * quat[3]);

	return Eigen::Vector3d{roll, pitch, yaw};
}

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

void Model::setLogger(TelemetryLogger *logger)
{
	if (logger != nullptr)
		mLogger = logger;
}

void Model::setWorld(World *parentWorld)
{
	mParentWorld = parentWorld;
	/* sync time step between world and model */
}

Model::~Model() {}

void Model::addRelativeForce(const Eigen::Vector3d &force)
{
	mTotalForce += force;
}

void Model::addRelativeTorque(const Eigen::Vector3d &torque)
{
	mTotalTorque += torque;
}

void Model::addRelativeForceAtPoint(const Eigen::Vector3d &force,
				    const Eigen::Vector3d &point)
{
	mTotalForce += force;
	mTotalTorque += point.cross(force);
}

void Model::addWorldForce(const Eigen::Vector3d &force)
{
	mTotalForce += mRotateToBodyFrame * force;
}

void Model::addWorldForceAtPoint(const Eigen::Vector3d &force,
				 const Eigen::Vector3d &point)
{
	mTotalForce += mRotateToBodyFrame * force;
	mTotalTorque += (mRotateToBodyFrame * (point - mWorldPos))
				.cross(mRotateToBodyFrame * force);
}

void Model::addWorldTorque(const Eigen::Vector3d &torque)
{
	mTotalTorque += mRotateToBodyFrame * torque;
}

bool Model::isStatic()
{
	return mIsStatic;
}

void Model::preUpdate()
{
	/* TODO
	 *	1. Calculate forces and torques from add-ons
	 *	2. update rotation matrices
	 * */

	mWorldAttitude = attitudeFromQuaternion(mWorldAttitudeQuaternion);

	double q0 = mWorldAttitudeQuaternion[0];
	double q1 = mWorldAttitudeQuaternion[1];
	double q2 = mWorldAttitudeQuaternion[2];
	double q3 = mWorldAttitudeQuaternion[3];

	mRotateToInertialFrame = Eigen::Matrix3d({
		{
			q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
			2 * (q1 * q2 - q0 * q3),
			2 * (q1 * q3 + q0 * q2),
		},
		{
			2 * (q1 * q2 + q0 * q3),
			q2 * q2 + q0 * q0 - q1 * q1 - q3 * q3,
			2 * (q2 * q3 - q0 * q1),
		},
		{
			2 * (q1 * q3 - q0 * q2),
			2 * (q2 * q3 + q0 * q1),
			q3 * q3 + q0 * q0 - q1 * q1 - q2 * q2,
		},
	});

	mRotateToBodyFrame = mRotateToInertialFrame.inverse();

	/* log all telemetry */
}

void Model::update()
{
	mAcc = mTotalForce / mMass - mAngularVel.cross(mVel);
	mAngularAcc =
		mInertia.inverse()
		* (mTotalTorque - mAngularVel.cross(mInertia * mAngularVel));
	mVel += mAcc * mTimeStep;
	mAngularVel += mAngularAcc * mTimeStep;

	mWorldLinearAcc = mRotateToInertialFrame * mAcc;
	mWorldAngularAcc = mRotateToInertialFrame * mAngularAcc;

	mWorldAttitudeQuaternion += updateQuaternion(
		mAngularVel, mWorldAttitudeQuaternion, mTimeStep);

	mWorldLinearVel = mRotateToInertialFrame * mVel;
	mWorldPos += mWorldLinearVel * mTimeStep;
}

void Model::postUpdate()
{
	/* TODO
	 *	1. Update sensors
	 * */
}
} // namespace javelin
