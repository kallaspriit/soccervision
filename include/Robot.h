#ifndef ROBOT_H
#define ROBOT_H

#include "Maths.h"
#include "Config.h"
#include "Odometer.h"
#include "ParticleFilterLocalizer.h"
#include "Vision.h"
#include "Tasks.h"
#include "Communication.h"
#include "Command.h"

#include <string>

class Wheel;
class Dribbler;
class Coilgun;
class Task;
class Communication;
class OdometerLocalizer;

class Robot : public Communication::Listener, public Command::Listener {

public:
    Robot(Communication* com);
    ~Robot();

    void setup();

	void step(float dt, Vision::Results* visionResults);

    const Math::Position getPosition() const { return Math::Position(x, y, orientation);  }
    float getOrientation() const { return orientation; }
	float getVelocity() { return velocity; }
	float getLastVelocity() { return lastVelocity; }
	bool isAccelerating() { return velocity > lastVelocity; }
	bool isBraking() { return velocity < lastVelocity; }
	bool isStalled();
	bool hasTasks() { return getCurrentTask() != NULL; }

    void setTargetDir(float x, float y, float omega = 0.0f);
    void setTargetDir(const Math::Angle& dir, float speed = 1.0f, float omega = 0.0f);
	void setTargetOmega(float omega) { targetOmega = omega; }
	void spinAroundDribbler(bool reverse = false, float period = Config::robotSpinAroundDribblerPeriod, float radius = Config::robotSpinAroundDribblerRadius, float forwardSpeed = Config::robotSpinAroundDribblerForwardSpeed);
    void setPosition(float x, float y, float orientation);
	void stop();
	void clearTasks() { tasks.clear(); }
    void handleTasks(float dt);

	void lookAt(Object* object);
	void turnBy(float angle, float speed = 1.0f);
    void driveTo(float x, float y, float orientation, float speed = 1.0f);
    void driveFacing(float targetX, float targetY, float faceX, float faceY, float speed = 1.0f);
    void drivePath(const Math::PositionQueue positions, float speed = 1.0f);
	void stopRotation();
	void jumpAngle(float angle = 0.35f, float speed = 13.0f);
	void setTargetDirFor(float x, float y, float omega, float duration);

    void addTask(Task* task) { tasks.push_back(task); }
    Task* getCurrentTask();
    TaskQueue getTasks() { return tasks; }
	Odometer::Movement getMovement() { return movement; }
	const ParticleFilterLocalizer::Measurements& getMeasurements() const { return measurements; } // TODO Here?

	void handleCommunicationMessage(std::string message);
	bool handleCommand(const Command& cmd);

	std::string getJSON() { return json; }

	Wheel* wheelFL;
    Wheel* wheelFR;
    Wheel* wheelRL;
    Wheel* wheelRR;
	Dribbler* dribbler;
	Coilgun* coilgun;
	ParticleFilterLocalizer* robotLocalizer;
	OdometerLocalizer* odometerLocalizer;
	
private:
	void setupWheels();
	void setupDribbler();
	void setupCoilgun();
	void setupOdometer();
	void setupRobotLocalizer();
	void setupOdometerLocalizer();
    void updateWheelSpeeds();
	void updateMeasurements();

    float x;
    float y;
    float orientation;
	float velocity;
	float lastVelocity;

	double lastCommandTime;
    float lastDt;
    float totalTime;
	bool coilgunCharged;

    TaskQueue tasks;
	Math::Vector targetDir;
    float targetOmega;
	bool frameTargetSpeedSet;

	Communication* com;
	Vision::Results* visionResults;
	Odometer* odometer;
	Odometer::Movement movement;
	ParticleFilterLocalizer::Measurements measurements;

	std::string json;
};

#endif // ROBOT_H
