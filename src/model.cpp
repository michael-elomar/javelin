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

Model *Model::create(const double mass,
		     const Eigen::Matrix3d &inertia,
		     const std::string &name)
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

	return new Model(mass, inertia, name);
}

Model::Model(const double mass,
	     const Eigen::Matrix3d &inertia,
	     const std::string &name)
	: mMass(mass), mInertia(inertia), mName(name)
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

void Model::logTelemetry()
{
	mLogger->log(mName + ".world_position.x", mWorldPos[0]);
	mLogger->log(mName + ".world_position.y", mWorldPos[1]);
	mLogger->log(mName + ".world_position.z", mWorldPos[2]);

	mLogger->log(mName + ".world_vel.x", mWorldLinearVel[0]);
	mLogger->log(mName + ".world_vel.y", mWorldLinearVel[1]);
	mLogger->log(mName + ".world_vel.z", mWorldLinearVel[2]);

	mLogger->log(mName + ".world_acc.x", mWorldLinearAcc[0]);
	mLogger->log(mName + ".world_acc.y", mWorldLinearAcc[1]);
	mLogger->log(mName + ".world_acc.z", mWorldLinearAcc[2]);

	mLogger->log(mName + ".world_attitude.x", mWorldAttitude[0]);
	mLogger->log(mName + ".world_attitude.y", mWorldAttitude[1]);
	mLogger->log(mName + ".world_attitude.z", mWorldAttitude[2]);

	mLogger->log(mName + ".world_angular_vel.x", mWorldAngularVel[0]);
	mLogger->log(mName + ".world_angular_vel.y", mWorldAngularVel[1]);
	mLogger->log(mName + ".world_angular_vel.z", mWorldAngularVel[2]);

	mLogger->log(mName + ".angular_vel.x", mAngularVel[0]);
	mLogger->log(mName + ".angular_vel.y", mAngularVel[1]);
	mLogger->log(mName + ".angular_vel.z", mAngularVel[2]);

	mLogger->log(mName + ".angular_acc.x", mAngularAcc[0]);
	mLogger->log(mName + ".angular_acc.y", mAngularAcc[1]);
	mLogger->log(mName + ".angular_acc.z", mAngularAcc[2]);

	mLogger->log(mName + ".attitude_quat.w", mWorldAttitudeQuaternion[0]);
	mLogger->log(mName + ".attitude_quat.x", mWorldAttitudeQuaternion[1]);
	mLogger->log(mName + ".attitude_quat.y", mWorldAttitudeQuaternion[2]);
	mLogger->log(mName + ".attitude_quat.z", mWorldAttitudeQuaternion[3]);
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

Eigen::Vector3d Model::LinearAcc()
{
	return mAcc;
}

Eigen::Vector3d Model::AngularVel()
{
	return mAngularVel;
}

void Model::addWorldTorque(const Eigen::Vector3d &torque)
{
	mTotalTorque += mRotateToBodyFrame * torque;
}

void Model::addSensor(Sensor *sensor)
{
	mSensors.push_back(sensor);
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
	logTelemetry();
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
	mWorldAngularVel = mRotateToInertialFrame * mAngularVel;
	mWorldPos += mWorldLinearVel * mTimeStep;
}

void Model::postUpdate()
{
	/* TODO
	 *	1. Update sensors
	 * */

	/* clear forces and torques for next time step */
	mTotalForce = {0, 0, 0};
	mTotalTorque = {0, 0, 0};

	// for (auto sensor : mSensors) {
	// 	nexus::Message *msg = sensor->aquireData();
	// 	sensor->publishData(msg);
	// 	delete msg;
	// }
}
} // namespace javelin
