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

Robot::Robot(AbstractCommunication* com) : com(com), wheelFL(NULL), wheelFR(NULL), wheelRL(NULL), wheelRR(NULL), coilgun(NULL), robotLocalizer(NULL), odometerLocalizer(NULL), ballLocalizer(NULL), odometer(NULL), visionResults(NULL), chipKickRequested(false), requestedChipKickLowerDribbler(false), requestedChipKickDistance(0.0f), lookAtPid(0.35f, 0.0f, 0.0012f, 0.016f) {
    targetOmega = 0;
    targetDir = Math::Vector(0, 0);
   
    x = 0.0f;
    y = 0.0f;
    orientation = 0.0f;
	velocity = 0.0f;
	lastVelocity = 0.0f;
	omega = 0.0f;
	travelledDistance = 0.0f;
	travelledRotation = 0.0f;

    lastCommandTime = -1;
	lastDriveBehindBallTime = -1;
	frameTargetSpeedSet = false;
	coilgunCharged = false;

	json = "null";

	float lookAtLimit = 10.0f;

	// lookAtPid.setTunings(0.35f, 0.0f, 0.0012f); // 2014
	lookAtPid.setTunings(0.4f, 0.0f, 0.002f);
	lookAtPid.setInputLimits(-Config::lookAtMaxSpeedAngle, Config::lookAtMaxSpeedAngle);
	lookAtPid.setOutputLimits(-lookAtLimit, lookAtLimit);
	lookAtPid.setMode(AUTO_MODE);
	lookAtPid.setBias(0.0f);
	lookAtPid.reset();
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
	setupCoilgun();
	setupDribbler();
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

void Robot::setupCoilgun() {
	coilgun = new Coilgun(com);
}

void Robot::setupDribbler() {
	dribbler = new Dribbler(Config::dribblerId, com, coilgun);
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

	com->setSpeeds(
		(int)Math::round(wheelFL->getTargetSpeed()),
		(int)Math::round(wheelFR->getTargetSpeed()),
		(int)Math::round(wheelRL->getTargetSpeed()),
		(int)Math::round(wheelRR->getTargetSpeed()),
		(int)Math::round(dribbler->getTargetSpeed())
	);

	// TODO restore once serial is working correctly
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

	omega = movement.omega;

	travelledDistance += velocity * dt;
	travelledRotation += movement.omega * dt;

	updateMeasurements();
	updateBallLocalizer(visionResults, dt);
	handleQueuedChipKickRequest();

	robotLocalizer->update(measurements);
	robotLocalizer->move(movement.velocityX, movement.velocityY, movement.omega, dt, measurements.size() == 0 ? true : false);
	odometerLocalizer->move(movement.velocityX, movement.velocityY, movement.omega, dt);

	Math::Position localizerPosition = robotLocalizer->getPosition();
	Math::Position odometerPosition = odometerLocalizer->getPosition();

	// use localizer position
	x = localizerPosition.x;
	y = localizerPosition.y;
	orientation = localizerPosition.orientation;

	// use odometer position
	/*x = odometerPosition.x;
	y = odometerPosition.y;
	orientation = odometerPosition.orientation;*/

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
	stream << "\"filteredTargetOmega\":" << wheelFL->getFilteredTargetOmega() << ",";
    stream << "\"realOmega\":" << wheelFL->getRealOmega();
    stream << "},";

    stream << "\"wheelFR\": {";
	stream << "\"stalled\":" << (wheelFR->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << wheelFR->getTargetOmega() << ",";
	stream << "\"filteredTargetOmega\":" << wheelFR->getFilteredTargetOmega() << ",";
    stream << "\"realOmega\":" << wheelFR->getRealOmega();
    stream << "},";

    stream << "\"wheelRL\": {";
	stream << "\"stalled\":" << (wheelRL->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << wheelRL->getTargetOmega() << ",";
	stream << "\"filteredTargetOmega\":" << wheelRL->getFilteredTargetOmega() << ",";
    stream << "\"realOmega\":" << wheelRL->getRealOmega();
    stream << "},";

    stream << "\"wheelRR\": {";
	stream << "\"stalled\":" << (wheelRR->isStalled() ? "true" : "false") << ",";
    stream << "\"targetOmega\":" << wheelRR->getTargetOmega() << ",";
	stream << "\"filteredTargetOmega\":" << wheelRR->getFilteredTargetOmega() << ",";
    stream << "\"realOmega\":" << wheelRR->getRealOmega();
    stream << "},";

	stream << "\"dribbler\": {";
	stream << "\"isRaised\":" << (dribbler->isRaised() ? "true" : "false") << ",";
	stream << "\"isLowered\":" << (dribbler->isLowered() ? "true" : "false");
	stream << "},";

	stream << "\"coilgun\": {";
	stream << "\"voltage\":" << coilgun->getVoltage() << ",";
	stream << "\"timeSinceLastKicked\":" << coilgun->getTimeSinceLastKicked() << ",";
	stream << "\"isLowVoltage\":" << coilgun->isLowVoltage();
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
	float speed = (2.0f * Math::PI * radius) / period;
	float omega = (2.0f * Math::PI) / period;

	if (reverse) {
		speed *= -1.0f;
		omega *= -1.0f;
	}

	setTargetDir(forwardSpeed, -speed, omega);
}

float Robot::getTimeSincLastDroveBehindBall() {
	return (float)Util::duration(lastDriveBehindBallTime); 
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
	// temporarily use chip-kicker instead
	//chipKick(1.5f);
	coilgun->kick(microseconds);
	dribbler->onKick();

	// TODO Remove this hack once hardware issue is resolved
	/*Command::Parameters lostBallParams;
	lostBallParams.push_back("0");
	Command lostBallCmd("ball", lostBallParams);
	dribbler->handleCommand(lostBallCmd);*/
}

bool Robot::chipKick(float distance, bool lowerDribblerAfterwards) {
	if (dribbler->isRaised()) {
		//std::cout << "! Dribbler is raised, chip-kicking immediately targeting " << distance << " meters" << std::endl;

		coilgun->chipKick(distance);

		if (lowerDribblerAfterwards) {
			dribbler->useNormalLimits();
		}

		return true;
	}

	if (chipKickRequested) {
		requestedChipKickLowerDribbler = lowerDribblerAfterwards;
		requestedChipKickDistance = distance;

		return false;
	}

	//std::cout << "! Dribbler is not raised, queuing chip-kicking targeting " << distance << " meters" << std::endl;

	chipKickRequested = true;
	requestedChipKickLowerDribbler = lowerDribblerAfterwards;
	requestedChipKickDistance = distance;

	dribbler->useChipKickLimits();

	return false;
}

void Robot::handleQueuedChipKickRequest() {
	if (!chipKickRequested) {
		return;
	}

	if (dribbler->isRaised()) {
		//std::cout << "! Dribbler is now raised, chip-kicking targeting distance of " << requestedChipKickDistance << " meters" << std::endl;

		chipKick(requestedChipKickDistance, requestedChipKickLowerDribbler);

		chipKickRequested = false;
		requestedChipKickDistance = 0.0f;
	}
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

void Robot::lookAt(Object* object, float lookAtP, bool stare) {
    if (object == NULL) {
		return;
	}

	// aim state for example sets stare to true so it really focuses on it always
	if (stare != true && (object->type == Side::BLUE || object->type == Side::YELLOW)) {
		int halfWidth = Config::cameraWidth / 2;
		int leftEdge = object->x - object->width / 2;
		int rightEdge = object->x + object->width / 2;
		int goalWidth = object->width;
		int goalHalfWidth = goalWidth / 2;
		int goalKickThresholdPixels = (int)((float)goalHalfWidth * (1.0f - Config::goalKickThreshold));

		if (
			leftEdge + goalKickThresholdPixels < halfWidth
			&& rightEdge - goalKickThresholdPixels > halfWidth
		) {
			return;
		}
	}

	lookAt(Math::Rad(object->angle), lookAtP);
}

void Robot::lookAt(const Math::Angle& angle, float lookAtP) {
	// simple P-controller
	//setTargetOmega(Math::limit(angle.rad() * lookAtP, Math::degToRad(Config::lookAtMaxSpeedAngle) * Config::lookAtP));

	// PID controller
	lookAtPid.setSetPoint(0.0f);
	lookAtPid.setProcessValue(angle.deg());

	float targetOmega = lookAtPid.compute();

	setTargetOmega(-targetOmega);

	/*lookAtPid.setProcessValue(angle.deg());

	float omega = lookAtPid.compute();

	setTargetOmega(-Math::degToRad(omega));*/
}

void Robot::lookAtBehind(Object* object) {
    if (object == NULL) {
		return;
	}

	lookAtBehind(Math::Rad(object->angle));
}

void Robot::lookAtBehind(const Math::Angle& angle) {
	float targetAngle;

	if (angle.rad() > 0.0f) {
		targetAngle = angle.rad() - Math::PI;
	} else {
		targetAngle = angle.rad() + Math::PI;
	}

	lookAt(Math::Rad(targetAngle));
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

void Robot::driveBehindBall(float ballDistance, float targetAngle, float speed, float offsetDistance, float side) {
	addTask(new DriveBehindBallTask(ballDistance, targetAngle, speed, offsetDistance, side));

	lastDriveBehindBallTime = Util::millitime();
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
		std::cout << "! Received discharged, charging" << std::endl;

		// TODO Add back
		com->send("charge");
	}

	if (wheelFL->handleCommand(cmd)) handled = true;
	if (wheelFR->handleCommand(cmd)) handled = true;
	if (wheelRL->handleCommand(cmd)) handled = true;
	if (wheelRR->handleCommand(cmd)) handled = true;

	if (dribbler->handleCommand(cmd)) handled = true;
	if (coilgun->handleCommand(cmd)) handled = true;

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