#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <fstream>
#include <mutex>
#include <neutron.hpp>

namespace javelin {

class Model;
class World;

class TelemetryLogger {
private:
	std::ofstream mLogFile;
	std::mutex mLogMutex;
	std::string mFilename;

	// Get current timestamp as microseconds since epoch (8B)
	uint64_t getCurrentTimestamp();

public:
	TelemetryLogger();
	TelemetryLogger(const std::string &filename);
	~TelemetryLogger();

	// Log a single field (binary format)
	void log(const std::string &field, double value);

	// Log multiple fields at once
	void log(const std::map<std::string, double> &fields);
};

struct TelemetryEntry {
	uint64_t timestamp; // Microseconds since epoch
	std::string field;  // Field name (e.g., "altitude")
	double value;       // Field value
};

class TelemetryReader {
public:
	// Constructor: Open a binary log file for reading
	TelemetryReader(const std::string &filename);
	~TelemetryReader();

	// Read all entries from the log file
	std::vector<TelemetryEntry> readAll();

	// Read entries one at a time (for streaming)
	bool readNext(TelemetryEntry &entry);

	// Reset the reader to the beginning of the file
	void reset();

private:
	std::ifstream logFile;
	std::string filename;

	// Helper: Read a single entry from the file
	bool readEntry(TelemetryEntry &entry);
};

class Model {

private:
	Model(const double mass,
	      const Eigen::Matrix3d &inertia,
	      const std::string &name);
	void setWorld(World *parentWorld);
	void setLogger(TelemetryLogger *logger);
	void logTelemetry();

public:
	~Model();
	static Model *create(const double mass,
			     const Eigen::Matrix3d &inertia,
			     const std::string &name);

	/* expressed in world inertial frame */
	void addWorldForce(const Eigen::Vector3d &force);
	void addWorldForceAtPoint(const Eigen::Vector3d &force,
				  const Eigen::Vector3d &point);
	void addWorldTorque(const Eigen::Vector3d &torque);

	/* expressed in relative body frame attached to model */
	void addRelativeForce(const Eigen::Vector3d &force);
	void addRelativeForceAtPoint(const Eigen::Vector3d &force,
				     const Eigen::Vector3d &point);
	void addRelativeTorque(const Eigen::Vector3d &torque);

	/* getters */
	Eigen::Vector3d LinearAcc();
	Eigen::Vector3d AngularVel();

	/* returns true if model is not meant for be moved */
	bool isStatic();

	void preUpdate();
	void update();
	void postUpdate();

	/* util transformations */
	Eigen::Vector3d toBodyFrame(const Eigen::Vector3d &vec);

	Eigen::Vector3d toInertialFrame(const Eigen::Vector3d &vec);

	friend class World;

private:
	/* Linear velocity expressed in body frame */
	Eigen::Vector3d mVel{0, 0, 0};

	/* Linear acceleration expressed in body frame */
	Eigen::Vector3d mAcc{0, 0, 0};

	/* anuglar velocity expressed in body frame */
	Eigen::Vector3d mAngularVel{0, 0, 0};

	/* angular acceleration expressed in body frame */
	Eigen::Vector3d mAngularAcc{0, 0, 0};

	/* position with respect to inertial frame */
	Eigen::Vector3d mWorldPos{0, 0, 0};

	/* World linear velocity */
	Eigen::Vector3d mWorldLinearVel{0, 0, 0};

	/* World linear acceleration */
	Eigen::Vector3d mWorldLinearAcc{0, 0, 0};

	/* quaternion representing attitude with respect to inertial frame */
	Eigen::Vector4d mWorldAttitudeQuaternion{1, 0, 0, 0};

	/* euler angles with respect to inertial frame */
	Eigen::Vector3d mWorldAttitude{0, 0, 0};

	/* World linear velocity */
	Eigen::Vector3d mWorldAngularVel{0, 0, 0};

	/* World linear velocity */
	Eigen::Vector3d mWorldAngularAcc{0, 0, 0};

	/* total external forces expressed in body frame */
	Eigen::Vector3d mTotalForce{0, 0, 0};

	/*total external torque expressed in body frame */
	Eigen::Vector3d mTotalTorque{0, 0, 0};

	Eigen::Matrix3d mRotateToBodyFrame = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d mRotateToInertialFrame = Eigen::Matrix3d::Identity();

	Eigen::Matrix3d mMapAngularVelToEuler;

	const double mMass;
	const Eigen::Matrix3d mInertia;

	double mTimeStep{1e-3};
	uint32_t id{0};
	const std::string mName;

	World *mParentWorld;
	TelemetryLogger *mLogger;

	bool mIsStatic{false};
};

class World {
public:
	World();
	~World();

	static void sigHandler(int signum);

	bool insertModel(Model *modelPtr);
	double getSimTime();

	void updateForces(Model *model);
	bool step();

	bool spin();
	bool spin(double ts);

private:
	neutron::Loop *mLoop;
	neutron::Timer *mTimer;
	neutron::Timer::Handler *mHandler;

	TelemetryLogger *mLogger;
	std::vector<Model *> mModels;
	double mTimeStep{1e-3};
	double mSimTime{0.0};

	bool mRunning;
	inline static World *sInstance;
};

class QuadCopter : public Model {
};

class Plane : public Model {
};
} // namespace javelin
