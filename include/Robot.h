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

class Robot : public Communication::Listener, public Command::Listener {

public:
    Robot(Communication* com);
    ~Robot();

    void setup();
	void setupWheels();
	void setupDribbler();
	void setupCoilgun();
	void setupOdometer();
	void setupLocalizer();

	void step(float dt, Vision::Results* visionResults);

    inline const Math::Position getPosition() const { return Math::Position(x, y, orientation);  }
    inline float getOrientation() const { return orientation; }
	inline float getVelocity() { return velocity; }
	inline float getLastVelocity() { return lastVelocity; }
	inline bool isAccelerating() { return velocity > lastVelocity; }
	inline bool isBraking() { return velocity < lastVelocity; }
	inline bool usingAutostop() { return autostop; }
	bool isStalled();
	bool hasTasks() { return getCurrentTask() != NULL; }

	inline void setAutostop(bool mode) { autostop = mode; }
    void setTargetDir(float x, float y, float omega = 0.0f, bool fluid = false);
    void setTargetDir(const Math::Angle& dir, float speed = 1.0f, float omega = 0.0f, bool fluid = false);
	void spinAroundDribbler(bool reverse = false, float period = Config::robotSpinAroundDribblerPeriod, float radius = Config::robotSpinAroundDribblerRadius, float forwardSpeed = Config::robotSpinAroundDribblerForwardSpeed, bool fluid = false);
    void setPosition(float x, float y, float orientation);
	void stop();
	void clearTasks() { tasks.clear(); }
    void handleTasks(float dt);

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
	Wheel* getWheelFL() const { return wheelFL; }
    Wheel* getWheelFR() const { return wheelFR; }
    Wheel* getWheelRL() const { return wheelRL; }
    Wheel* getWheelRR() const { return wheelRR; }
	Dribbler* getDribbler() const { return dribbler; }
	Coilgun* getCoilgun() const { return coilgun; }
	Localizer* getRobotLocalizer() { return robotLocalizer; }
	const ParticleFilterLocalizer::Measurements& getMeasurements() const { return measurements; } // TODO Here?

	void handleCommunicationMessage(std::string message);
	bool handleCommand(const Command& cmd);
	
private:
    void updateWheelSpeeds();
	void updateMeasurements();

    float x;
    float y;
    float orientation;
	float velocity;
	float lastVelocity;

	float lastCommandTime;
    float lastDt;
    float totalTime;
	bool coilgunCharged;
	bool autostop;

    TaskQueue tasks;
	Math::Vector targetDir;
    float targetOmega;
	bool frameTargetSpeedSet;

	// TODO Still need this?
	bool fluidMovement;
	float fluidTargetX;
	float fluidTargetY;
	float fluidTargetOmega;

    Wheel* wheelFL;
    Wheel* wheelFR;
    Wheel* wheelRL;
    Wheel* wheelRR;
	Dribbler* dribbler;
	Coilgun* coilgun;

	Communication* com;
	Vision::Results* visionResults;
	Odometer* odometer;
	Odometer::Movement movement;
    ParticleFilterLocalizer* robotLocalizer;
	ParticleFilterLocalizer::Measurements measurements;
};

#endif // ROBOT_H
