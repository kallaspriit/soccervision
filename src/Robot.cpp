#include "Robot.h"
#include "Wheel.h"
#include "Dribbler.h"
#include "Coilgun.h"
#include "Odometer.h"
#include "OdometerLocalizer.h"
#include "Util.h"
#include "Tasks.h"
#include "Config.h"

#include <iostream>
#include <map>
#include <sstream>

Robot::Robot(Communication* com) : com(com), wheelFL(NULL), wheelFR(NULL), wheelRL(NULL), wheelRR(NULL), coilgun(NULL), robotLocalizer(NULL), odometerLocalizer(NULL), ballLocalizer(NULL), odometer(NULL), visionResults(NULL) {
    targetOmega = 0;
    targetDir = Math::Vector(0, 0);
   
    x = 0.0f;
    y = 0.0f;
    orientation = 0.0f;
	velocity = 0.0f;
	lastVelocity = 0.0f;

    lastCommandTime = -1;
	frameTargetSpeedSet = false;
	coilgunCharged = false;

	json = "null";
}

Robot::~Robot() {
    if (wheelRR != NULL) delete wheelRR; wheelRR = NULL;
    if (wheelRL != NULL) delete wheelRL; wheelRL = NULL;
    if (wheelFR != NULL) delete wheelFR; wheelFR = NULL;
    if (wheelFL != NULL) delete wheelFL; wheelFL = NULL;
	if (coilgun != NULL) delete coilgun; coilgun = NULL;
	if (dribbler != NULL) delete dribbler; dribbler = NULL;
	if (odometer != NULL) delete odometer; odometer = NULL;
	if (ballLocalizer != NULL) delete ballLocalizer; ballLocalizer = NULL;
	if (robotLocalizer != NULL) delete robotLocalizer; robotLocalizer = NULL;
	if (odometerLocalizer != NULL) delete odometerLocalizer; odometerLocalizer = NULL;

    while (tasks.size() > 0) {
        delete tasks.front();

        tasks.pop_front();
    }
}

void Robot::setup() {
	setupCameraFOV();
	setupRobotLocalizer();
	setupBallLocalizer();
	setupOdometerLocalizer();
    setupWheels();
	setupDribbler();
	setupCoilgun();
	setupOdometer();

	setPosition(Config::fieldWidth / 2.0f, Config::fieldHeight / 2.0f, 0.0f);
}

void Robot::setupCameraFOV() {
	Math::PointList points;

	points.push_back(Math::Point(0, 0));
	points.push_back(Math::Point(Config::cameraFovDistance, -Config::cameraFovWidth / 2.0f));
	points.push_back(Math::Point(Config::cameraFovDistance, Config::cameraFovWidth / 2.0f));
	points.push_back(Math::Point(0, 0));
	points.push_back(Math::Point(-Config::cameraFovDistance, -Config::cameraFovWidth / 2.0f));
	points.push_back(Math::Point(-Config::cameraFovDistance, Config::cameraFovWidth / 2.0f));
	points.push_back(Math::Point(0, 0));

	cameraFOV = Math::Polygon(points);
}

void Robot::setupRobotLocalizer() {
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

void Robot::setupBallLocalizer() {
	ballLocalizer = new BallLocalizer();
}

void Robot::setupOdometerLocalizer() {
	odometerLocalizer = new OdometerLocalizer();
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
	coilgun = new Coilgun(com);
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
    updateBallLocalizer(visionResults, dt);

    wheelFL->step(dt);
    wheelFR->step(dt);
    wheelRL->step(dt);
    wheelRR->step(dt);
	coilgun->step(dt);
	dribbler->step(dt);

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

	Math::Vector velocityVec(movement.velocityX, movement.velocityY);
	lastVelocity = velocity;
	velocity = velocityVec.getLength();

	updateMeasurements();
	updateBallLocalizer(visionResults, dt);

	robotLocalizer->update(measurements);
	robotLocalizer->move(movement.velocityX, movement.velocityY, movement.omega, dt, measurements.size() == 0 ? true : false);
	odometerLocalizer->move(movement.velocityX, movement.velocityY, movement.omega, dt);

	Math::Position localizerPosition = robotLocalizer->getPosition();
	Math::Position odometerPosition = odometerLocalizer->getPosition();

	x = localizerPosition.x;
	y = localizerPosition.y;
	orientation = localizerPosition.orientation;

	std::stringstream stream;

	stream << "\"localizerX\":" << localizerPosition.x << ",";
    stream << "\"localizerY\":" << localizerPosition.y << ",";
    stream << "\"localizerOrientation\":" << localizerPosition.orientation << ",";
	stream << "\"odometerX\":" << odometerPosition.x << ",";
    stream << "\"odometerY\":" << odometerPosition.y << ",";
    stream << "\"odometerOrientation\":" << odometerPosition.orientation << ",";
    stream << "\"gotBall\":" << (dribbler->gotBall() ? "true" : "false") << ",";

    stream << "\"wheelFL\": {";
	stream << "\"stalled\":" << (wheelFL->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << wheelFL->getTargetOmega() << ",";
    stream << "\"realOmega\":" << wheelFL->getRealOmega();
    stream << "},";

    stream << "\"wheelFR\": {";
	stream << "\"stalled\":" << (wheelFR->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << wheelFR->getTargetOmega() << ",";
    stream << "\"realOmega\":" << wheelFR->getRealOmega();
    stream << "},";

    stream << "\"wheelRL\": {";
	stream << "\"stalled\":" << (wheelRL->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << wheelRL->getTargetOmega() << ",";
    stream << "\"realOmega\":" << wheelRL->getRealOmega();
    stream << "},";

    stream << "\"wheelRR\": {";
	stream << "\"stalled\":" << (wheelRR->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << wheelRR->getTargetOmega() << ",";
    stream << "\"realOmega\":" << wheelRR->getRealOmega();
    stream << "},";

	debugBallList("ballsRaw", stream, visibleBalls);
	debugBallList("ballsFiltered", stream, ballLocalizer->balls);

	stream << "\"cameraFOV\":" << currentCameraFOV.toJSON() << ",";

	stream << "\"tasks\": [";

    bool first = true;

    for (TaskQueueIt it = tasks.begin(); it != tasks.end(); it++) {
        Task* task = *it;

        if (!first) {
            stream << ",";
        } else {
            first = false;
        }

        stream << "{";
        stream << "\"started\": " << (task->isStarted() ? "true" : "false") << ",";
        stream << "\"percentage\": " << task->getPercentage() << ",";
        stream << "\"status\": \"" << task->toString() << "\"";
        stream << "}";
    }

    stream << "]";

	json = stream.str();

	frameTargetSpeedSet = false;
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

void Robot::updateBallLocalizer(Vision::Results* visionResults, float dt) {
	// delete balls from previous frame
	for (BallLocalizer::BallListIt it = visibleBalls.begin(); it != visibleBalls.end(); it++) {
		delete (*it);
	}

	visibleBalls.clear();

	if (visionResults == NULL) {
		return;
	}

	BallLocalizer::BallList frontBalls;
	BallLocalizer::BallList rearBalls;
	
	if (visionResults->front != NULL) {
		frontBalls = ballLocalizer->extractBalls(
			visionResults->front->balls,
			x,
			y,
			orientation
		);
	}

	if (visionResults->rear != NULL) {
		rearBalls = ballLocalizer->extractBalls(
			visionResults->rear->balls,
			x,
			y,
			orientation
		);
	}
	
	visibleBalls.reserve(frontBalls.size() + rearBalls.size());
	visibleBalls.insert(visibleBalls.end(), frontBalls.begin(), frontBalls.end());
	visibleBalls.insert(visibleBalls.end(), rearBalls.begin(), rearBalls.end());

	currentCameraFOV = cameraFOV.getRotated(orientation).getTranslated(x, y);

	ballLocalizer->update(visibleBalls, currentCameraFOV, dt);

	//std::cout << "@ UP front: " << frontBalls.size() << ", rear: " << rearBalls.size() << ", merged: " << visibleBalls.size() << std::endl;
}

void Robot::setTargetDir(float x, float y, float omega) {
	//std::cout << "! Setting robot target direction: " << x << "x" << y << " @ " << omega << std::endl;

	targetDir = Math::Vector(x, y);
	targetOmega = omega;

    lastCommandTime = Util::millitime();
	frameTargetSpeedSet = true;
}

void Robot::setTargetDir(const Math::Angle& dir, float speed, float omega) {
    Math::Vector dirVector = Math::Vector::createForwardVec(dir.rad(), speed);

    setTargetDir(dirVector.x, dirVector.y, omega);
}

void Robot::spinAroundDribbler(bool reverse, float period, float radius, float forwardSpeed) {
	float speed = (2 * Math::PI * radius) / period;
	float omega = (2 * Math::PI) / period;

	if (reverse) {
		speed *= -1.0f;
		omega *= -1.0f;
	}

	setTargetDir(forwardSpeed, -speed, omega);
}

bool Robot::isStalled() {
	return wheelFL->isStalled()
		|| wheelFR->isStalled()
		|| wheelRL->isStalled()
		|| wheelRR->isStalled();
}

void Robot::stop() {
	//std::cout << "! Stopping robot" << std::endl;

	setTargetDir(0, 0, 0);
	dribbler->stop();
}

void Robot::kick(int microseconds) {
	coilgun->kick(microseconds);
}

void Robot::setPosition(float x, float y, float orientation) {
    this->x = x;
    this->y = y;
	this->orientation = Math::floatModulus(orientation, Math::TWO_PI);

	robotLocalizer->setPosition(x, y, orientation);
	odometerLocalizer->setPosition(x, y, orientation);
}

Task* Robot::getCurrentTask() {
    if (tasks.size() == 0) {
        return NULL;
    }

    return tasks.front();
}

void Robot::lookAt(Object* object) {
    if (object == NULL) {
		return;
	}

	lookAt(Math::Rad(object->angle));
}

void Robot::lookAt(const Math::Angle& angle) {
	setTargetOmega(Math::limit(angle.rad() * Config::lookAtP, Config::lookAtMaxOmega));
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

void Robot::handleCommunicationMessage(std::string message) {
	if (Command::isValid(message)) {
        Command command = Command::parse(message);

		handleCommand(command);
	}
}

bool Robot::handleCommand(const Command& cmd) {
	bool handled = false;

	if (cmd.name == "discharged") {
		std::cout << "@ RECEIVED DISCHARGED, CHARGING" << std::endl;

		com->send("charge");
	}

	if (wheelFL->handleCommand(cmd)) handled = true;
	if (wheelFR->handleCommand(cmd)) handled = true;
	if (wheelRL->handleCommand(cmd)) handled = true;
	if (wheelRR->handleCommand(cmd)) handled = true;
	if (dribbler->handleCommand(cmd)) handled = true;

	return handled;
}

void Robot::debugBallList(std::string name, std::stringstream& stream, BallLocalizer::BallList balls) {
	BallLocalizer::Ball* ball;
	bool first = true;

	stream << "\"" << name << "\": [";

	for (BallLocalizer::BallListIt it = balls.begin(); it != balls.end(); it++) {
		ball = *it;

		if (!first) {
            stream << ",";
        } else {
            first = false;
        }

		stream << "{";
		stream << "\"x\": " << ball->x << ",";
		stream << "\"y\": " << ball->y << ",";
		stream << "\"velocityX\": " << ball->velocityX << ",";
		stream << "\"velocityY\": " << ball->velocityY << ",";
		stream << "\"createdTime\": " << ball->createdTime << ",";
		stream << "\"updatedTime\": " << ball->updatedTime << ",";
		stream << "\"shouldBeRemoved\": " << (ball->shouldBeRemoved() ? "true" : "false") << ",";
		stream << "\"visible\": " << (ball->visible ? "true" : "false") << ",";
		stream << "\"inFOV\": " << (ball->inFOV ? "true" : "false");
		stream << "}";
	}

    stream << "],";
}