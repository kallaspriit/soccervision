#include "Robot.h"
#include "Wheel.h"
#include "Dribbler.h"
#include "Coilgun.h"
#include "Odometer.h"
#include "Util.h"
#include "Tasks.h"
#include "Config.h"

#include <iostream>
#include <map>
#include <sstream>

Robot::Robot(Communication* com) : com(com), wheelFL(NULL), wheelFR(NULL), wheelRL(NULL), wheelRR(NULL), coilgun(NULL), robotLocalizer(NULL), odometer(NULL), visionResults(NULL) {
    targetOmega = 0;
    targetDir = Math::Vector(0, 0);
   
	fluidTargetX = 0.0f;
	fluidTargetY = 0.0f;
	fluidTargetOmega = 0.0f;

    x = 0.0f;
    y = 0.0f;
    orientation = 0.0f;

    lastCommandTime = -1;
	fluidMovement = false;
	frameTargetSpeedSet = false;
	coilgunCharged = false;
	autostop = true;
}

Robot::~Robot() {
    if (wheelRR != NULL) delete wheelRR; wheelRR = NULL;
    if (wheelRL != NULL) delete wheelRL; wheelRL = NULL;
    if (wheelFR != NULL) delete wheelFR; wheelFR = NULL;
    if (wheelFL != NULL) delete wheelFL; wheelFL = NULL;
	if (coilgun != NULL) delete coilgun; coilgun = NULL;
	if (dribbler != NULL) delete dribbler; dribbler = NULL;
	if (odometer != NULL) delete odometer; odometer = NULL;
	if (robotLocalizer != NULL) delete robotLocalizer; robotLocalizer = NULL;

    while (tasks.size() > 0) {
        delete tasks.front();

        tasks.pop_front();
    }
}

void Robot::setup() {
    setupWheels();
	setupDribbler();
	setupCoilgun();
	setupOdometer();
	setupLocalizer();
}

void Robot::setupWheels() {
    wheelFL = new Wheel(Config::wheelFLId);
    wheelFR = new Wheel(Config::wheelFRId);
    wheelRL = new Wheel(Config::wheelRLId);
    wheelRR = new Wheel(Config::wheelRRId);
}

void Robot::setupDribbler() {
	dribbler = new Dribbler(Config::dribblerId);
}

void Robot::setupCoilgun() {
	coilgun = new Coilgun();
}

void Robot::setupOdometer() {
	odometer = new Odometer(
		Config::robotWheelAngle1,
		Config::robotWheelAngle2,
		Config::robotWheelAngle3,
		Config::robotWheelAngle4,
		Config::robotWheelOffset,
		Config::robotWheelRadius
	);
}

void Robot::setupLocalizer() {
	robotLocalizer = new ParticleFilterLocalizer();

	robotLocalizer->addLandmark(
		"yellow-center",
		0.0f,
		Config::fieldHeight / 2.0f
	);

	robotLocalizer->addLandmark(
		"blue-center",
		Config::fieldWidth,
		Config::fieldHeight / 2.0f
	);
}

void Robot::step(float dt, Vision::Results* visionResults) {
	this->visionResults = visionResults;

	lastDt = dt;
    totalTime += dt;

	if (!coilgunCharged) {
		coilgun->charge();

		coilgunCharged = true;
	}

    handleTasks(dt);
    updateWheelSpeeds();

    wheelFL->step(dt);
    wheelFR->step(dt);
    wheelRL->step(dt);
    wheelRR->step(dt);
	coilgun->step(dt);
	dribbler->step(dt);

	// send target speeds
	com->send("speeds:"
		+ Util::toString((int)Math::round(wheelFL->getTargetSpeed())) + ":"
		+ Util::toString((int)Math::round(wheelFR->getTargetSpeed())) + ":"
		+ Util::toString((int)Math::round(wheelRL->getTargetSpeed())) + ":"
		+ Util::toString((int)Math::round(wheelRR->getTargetSpeed())) + ":"
		+ Util::toString((int)Math::round(dribbler->getTargetSpeed()))
	);

	movement = odometer->calculateMovement(
		wheelFL->getRealOmega(),
		wheelFR->getRealOmega(),
		wheelRL->getRealOmega(),
		wheelRR->getRealOmega()
	);

	// this is basically odometer localizer
	orientation = Math::floatModulus(orientation + movement.omega * dt, Math::TWO_PI);

    if (orientation < 0.0f) {
        orientation += Math::TWO_PI;
    }

    float globalVelocityX = movement.velocityX * Math::cos(orientation) - movement.velocityY * Math::sin(orientation);
    float globalVelocityY = movement.velocityX * Math::sin(orientation) + movement.velocityY * Math::cos(orientation);

    x += globalVelocityX * dt;
    y += globalVelocityY * dt;

	// particle filter localizer
	/*updateMeasurements();

	robotLocalizer->update(measurements);
	robotLocalizer->move(movement.velocityX, movement.velocityY, movement.omega, dt/, measurements.size() == 0 ? true : false/);

	Math::Position position = robotLocalizer->getPosition();

	x = position.x;
	y = position.y;
	orientation = position.orientation;*/

    //std::cout << "Vx: " << movement.velocityX << "; Vy: " << movement.velocityY << "; omega: " << movement.omega << std::endl;


	// TODO Review this..
	/*if (autostop) {
		if (!frameTargetSpeedSet) {
			stop();
		}
	} else if (lastCommandTime != -1 && Util::duration(lastCommandTime) > 0.5f) {
        std::cout << "! No movement command for 500ms, stopping for safety" << std::endl;

        stop();

        lastCommandTime = -1.0f;
    }*/

	frameTargetSpeedSet = false;
}

void Robot::setTargetDir(float x, float y, float omega, bool fluid) {
	//std::cout << "! Setting robot target direction: " << x << "x" << y << " @ " << omega << (fluid ? " (fluid)" : "") << std::endl;

	fluidMovement = fluid;

	if (fluidMovement) {
		// TODO Do it acceleration based (multiply diff) or perhaps remove
		if (fluidTargetX < x) {
			fluidTargetX = Math::min(fluidTargetX + Config::robotfluidSpeedStep * lastDt, x);
		} else {
			fluidTargetX = Math::max(fluidTargetX - Config::robotfluidSpeedStep * lastDt, x);
		}

		if (fluidTargetY < y) {
			fluidTargetY = Math::min(fluidTargetY + Config::robotfluidSpeedStep * lastDt, y);
		} else {
			fluidTargetY = Math::max(fluidTargetY - Config::robotfluidSpeedStep * lastDt, y);
		}

		if (fluidTargetOmega < omega) {
			fluidTargetOmega = Math::min(fluidTargetOmega + Config::robotfluidOmegaStep * lastDt, omega);
		} else {
			fluidTargetOmega = Math::max(fluidTargetOmega - Config::robotfluidOmegaStep * lastDt, omega);
		}

		targetDir = Math::Vector(fluidTargetX, fluidTargetY);
		targetOmega = fluidTargetOmega;
	} else {
		targetDir = Math::Vector(x, y);
		targetOmega = omega;
	}

    lastCommandTime = Util::millitime();
	frameTargetSpeedSet = true;
}

void Robot::setTargetDir(const Math::Angle& dir, float speed, float omega, bool fluid) {
    Math::Vector dirVector = Math::Vector::createForwardVec(dir.rad(), speed);

    setTargetDir(dirVector.x, dirVector.y, omega, fluid);
}

void Robot::spinAroundDribbler(bool reverse, float period, float radius, float forwardSpeed, bool fluid) {
	float speed = (2 * Math::PI * radius) / period;
	float omega = (2 * Math::PI) / period;

	if (reverse) {
		speed *= -1.0f;
		omega *= -1.0f;
	}

	setTargetDir(forwardSpeed, -speed, omega, fluid);
}

bool Robot::isStalled() {
	return wheelFL->isStalled()
		|| wheelFR->isStalled()
		|| wheelRL->isStalled()
		|| wheelRR->isStalled();
}

void Robot::stop() {
	//std::cout << "! Stopping robot" << std::endl;

	setTargetDir(0, 0, 0, fluidMovement);
	dribbler->stop();
}

void Robot::setPosition(float x, float y, float orientation) {
    this->x = x;
    this->y = y;
	this->orientation = Math::floatModulus(orientation, Math::TWO_PI);

	robotLocalizer->setPosition(x, y, orientation);
}

Task* Robot::getCurrentTask() {
    if (tasks.size() == 0) {
        return NULL;
    }

    return tasks.front();
}

void Robot::turnBy(float angle, float speed) {
    addTask(new TurnByTask(angle, speed));
}

void Robot::driveTo(float x, float y, float orientation, float speed) {
    addTask(new DriveToTask(x, y, orientation, speed));
}

void Robot::driveFacing(float targetX, float targetY, float faceX, float faceY, float speed) {
    addTask(new DriveFacingTask(targetX, targetY, faceX, faceY, speed));
}

void Robot::drivePath(const Math::PositionQueue positions, float speed) {
    addTask(new DrivePathTask(positions, speed));
}

void Robot::stopRotation() {
    addTask(new StopRotationTask());
}

void Robot::jumpAngle(float angle, float speed) {
	addTask(new JumpAngleTask(angle, speed));
}

void Robot::setTargetDirFor(float x, float y, float omega, float duration) {
	addTask(new DriveForTask(x, y, omega, duration));
}

void Robot::handleTasks(float dt) {
    Task* task = getCurrentTask();

    if (task == NULL) {
        return;
    }

    if (!task->isStarted()) {
        task->onStart(*this, dt);

        task->setStarted(true);
    }

    if (task->onStep(*this, dt) == false) {
        task->onEnd(*this, dt);

        delete task;

        tasks.pop_front();

        handleTasks(dt);
    }
}

void Robot::updateWheelSpeeds() {
	Odometer::WheelSpeeds wheelSpeeds = odometer->calculateWheelSpeeds(targetDir.x, targetDir.y, targetOmega);

	//std::cout << "! Updating wheel speeds: " << wheelSpeeds.FL << ", " << wheelSpeeds.FR << ", " << wheelSpeeds.RL << ", " << wheelSpeeds.RR << std::endl;

	wheelFL->setTargetOmega(wheelSpeeds.FL);
	wheelFR->setTargetOmega(wheelSpeeds.FR);
    wheelRL->setTargetOmega(wheelSpeeds.RL);
    wheelRR->setTargetOmega(wheelSpeeds.RR);
}

void Robot::updateMeasurements() {
	measurements.clear();

	Object* yellowGoal = visionResults->getLargestGoal(Side::YELLOW);
	Object* blueGoal = visionResults->getLargestGoal(Side::BLUE);

	if (yellowGoal != NULL) {
		measurements["yellow-center"] = ParticleFilterLocalizer::Measurement(yellowGoal->distance, yellowGoal->angle);
	}

	if (blueGoal != NULL) {
		measurements["blue-center"] = ParticleFilterLocalizer::Measurement(blueGoal->distance, blueGoal->angle);
	}
}

void Robot::handleCommunicationMessage(std::string message) {
	if (Command::isValid(message)) {
        Command command = Command::parse(message);

		handleCommand(command);
	}
}

bool Robot::handleCommand(const Command& cmd) {
	bool handled = false;

	if (wheelFL->handleCommand(cmd)) handled = true;
	if (wheelFR->handleCommand(cmd)) handled = true;
	if (wheelRL->handleCommand(cmd)) handled = true;
	if (wheelRR->handleCommand(cmd)) handled = true;
	if (dribbler->handleCommand(cmd)) handled = true;

	return handled;
}