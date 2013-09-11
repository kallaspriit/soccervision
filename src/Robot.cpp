#include "Robot.h"
#include "Wheel.h"
#include "Dribbler.h"
#include "Coilgun.h"
#include "ParticleFilterLocalizer.h"
#include "Util.h"
#include "Tasks.h"
#include "Config.h"

#include <iostream>
#include <map>
#include <sstream>

Robot::Robot() : wheelFL(NULL), wheelFR(NULL), wheelRL(NULL), wheelRR(NULL), coilgun(NULL), robotLocalizer(NULL), visionResults(NULL) {
	wheelAngles[0] = Math::degToRad(Config::robotWheelAngle1);
    wheelAngles[1] = Math::degToRad(Config::robotWheelAngle2);
    wheelAngles[2] = Math::degToRad(Config::robotWheelAngle3);
    wheelAngles[3] = Math::degToRad(Config::robotWheelAngle4);

    targetOmega = 0;
    targetDir = Math::Vector(0, 0);
    wheelOffset = Config::robotWheelOffset;
    wheelRadius = Config::robotWheelRadius;
    wheelRadiusInv = 1.0f / wheelRadius;

	fluidTargetX = 0.0f;
	fluidTargetY = 0.0f;
	fluidTargetOmega = 0.0f;

    x = Config::robotStartX;
    y = Config::robotStartY;
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
	setupLocalizer();
}

void Robot::setupWheels() {
	omegaMatrix = Math::Matrix4x3(
        -Math::sin(wheelAngles[0]), Math::cos(wheelAngles[0]), wheelOffset,
		-Math::sin(wheelAngles[1]), Math::cos(wheelAngles[1]), wheelOffset,
		-Math::sin(wheelAngles[2]), Math::cos(wheelAngles[2]), wheelOffset,
		-Math::sin(wheelAngles[3]), Math::cos(wheelAngles[3]), wheelOffset
    );
    omegaMatrixInvA = Math::Matrix3x3(
        -Math::sin(wheelAngles[0]), Math::cos(wheelAngles[0]), wheelOffset,
		-Math::sin(wheelAngles[1]), Math::cos(wheelAngles[1]), wheelOffset,
		-Math::sin(wheelAngles[2]), Math::cos(wheelAngles[2]), wheelOffset
    ).getInversed();
    omegaMatrixInvB = Math::Matrix3x3(
        -Math::sin(wheelAngles[0]), Math::cos(wheelAngles[0]), wheelOffset,
		-Math::sin(wheelAngles[1]), Math::cos(wheelAngles[1]), wheelOffset,
		-Math::sin(wheelAngles[3]), Math::cos(wheelAngles[3]), wheelOffset
    ).getInversed();
    omegaMatrixInvC = Math::Matrix3x3(
        -Math::sin(wheelAngles[0]), Math::cos(wheelAngles[0]), wheelOffset,
		-Math::sin(wheelAngles[2]), Math::cos(wheelAngles[2]), wheelOffset,
		-Math::sin(wheelAngles[3]), Math::cos(wheelAngles[3]), wheelOffset
    ).getInversed();
    omegaMatrixInvD = Math::Matrix3x3(
		-Math::sin(wheelAngles[1]), Math::cos(wheelAngles[1]), wheelOffset,
		-Math::sin(wheelAngles[2]), Math::cos(wheelAngles[2]), wheelOffset,
		-Math::sin(wheelAngles[3]), Math::cos(wheelAngles[3]), wheelOffset
    ).getInversed();

    // positive omega means that FL turns to the left and all others follow the same direction
    wheelFL = new Wheel(1);
    wheelFR = new Wheel(2);
    wheelRL = new Wheel(3);
    wheelRR = new Wheel(4);
}

void Robot::setupDribbler() {
	dribbler = new Dribbler();
}

void Robot::setupCoilgun() {
	coilgun = new Coilgun();
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

	updateMovement();

	// TODO move the odometer to seperate class
	orientation = Math::floatModulus(orientation + movement.omega * dt, Math::TWO_PI);

    if (orientation < 0.0f) {
        orientation += Math::TWO_PI;
    }

    float globalVelocityX = movement.velocityX * Math::cos(orientation) - movement.velocityY * Math::sin(orientation);
    float globalVelocityY = movement.velocityX * Math::sin(orientation) + movement.velocityY * Math::cos(orientation);

    x += globalVelocityX * dt;
    y += globalVelocityY * dt;

	// using localization
	/*updateMeasurements();

	robotLocalizer->update(measurements);
	robotLocalizer->move(movement.velocityX, movement.velocityY, movement.omega, dt/, measurements.size() == 0 ? true : false/);

	Math::Position position = robotLocalizer->getPosition();

	x = position.x;
	y = position.y;
	orientation = position.orientation;*/

    //std::cout << "Vx: " << movement.velocityX << "; Vy: " << movement.velocityY << "; omega: " << movement.omega << std::endl;

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

	dribbler->step(dt);
	coilgun->step(dt);

	if (autostop) {
		if (!frameTargetSpeedSet) {
			stop();
		}
	} else if (lastCommandTime != -1 && Util::duration(lastCommandTime) > 0.5f) {
        std::cout << "! No movement command for 500ms, stopping for safety" << std::endl;

        stop();

        lastCommandTime = -1;
    }

	frameTargetSpeedSet = false;
}

void Robot::setTargetDir(float x, float y, float omega, bool fluid) {
	fluidMovement = fluid;

	if (fluidMovement) {
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

void Robot::setTargetDirFor(float x, float y, float omega, double duration) {
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
    Math::Matrix3x1 targetMatrix(
        targetDir.x,
        targetDir.y,
        targetOmega
    );

    Math::Matrix4x1 resultMatrix = omegaMatrix
        .getMultiplied(wheelRadiusInv)
        .getMultiplied(targetMatrix);

    wheelRL->setTargetOmega(-resultMatrix.a11);
    wheelFL->setTargetOmega(-resultMatrix.a21);
    wheelFR->setTargetOmega(-resultMatrix.a31);
    wheelRR->setTargetOmega(-resultMatrix.a41);
}

void Robot::updateMovement() {
    Math::Matrix3x1 wheelMatrixA = Math::Matrix3x1(
        wheelRL->getRealOmega(),
        wheelFL->getRealOmega(),
        wheelFR->getRealOmega()
    );
    Math::Matrix3x1 wheelMatrixB = Math::Matrix3x1(
        wheelRL->getRealOmega(),
        wheelFL->getRealOmega(),
        wheelRR->getRealOmega()
    );
    Math::Matrix3x1 wheelMatrixC = Math::Matrix3x1(
        wheelRL->getRealOmega(),
        wheelFR->getRealOmega(),
        wheelRR->getRealOmega()
    );
    Math::Matrix3x1 wheelMatrixD = Math::Matrix3x1(
        wheelFL->getRealOmega(),
        wheelFR->getRealOmega(),
        wheelRR->getRealOmega()
    );

    Math::Matrix3x1 movementA = omegaMatrixInvA.getMultiplied(wheelMatrixA).getMultiplied(wheelRadius);
    Math::Matrix3x1 movementB = omegaMatrixInvB.getMultiplied(wheelMatrixB).getMultiplied(wheelRadius);
    Math::Matrix3x1 movementC = omegaMatrixInvC.getMultiplied(wheelMatrixC).getMultiplied(wheelRadius);
    Math::Matrix3x1 movementD = omegaMatrixInvD.getMultiplied(wheelMatrixD).getMultiplied(wheelRadius);

    float avgVelocityX = -(movementA.a11 + movementB.a11 + movementC.a11 + movementD.a11) / 4.0f;
    float avgVelocityY = -(movementA.a21 + movementB.a21 + movementC.a21 + movementD.a21) / 4.0f;
    float avgOmega = -(movementA.a31 + movementB.a31 + movementC.a31 + movementD.a31) / 4.0f;

    float avgDiffA = Math::abs(movementA.a11 - avgVelocityX) + Math::abs(movementA.a21 - avgVelocityY) + Math::abs(movementA.a31 - avgOmega);
    float avgDiffB = Math::abs(movementB.a11 - avgVelocityX) + Math::abs(movementB.a21 - avgVelocityY) + Math::abs(movementB.a31 - avgOmega);
    float avgDiffC = Math::abs(movementC.a11 - avgVelocityX) + Math::abs(movementC.a21 - avgVelocityY) + Math::abs(movementC.a31 - avgOmega);
    float avgDiffD = Math::abs(movementD.a11 - avgVelocityX) + Math::abs(movementD.a21 - avgVelocityY) + Math::abs(movementD.a31 - avgOmega);

    float diffs[] = {avgDiffA, avgDiffB, avgDiffC, avgDiffD};
    float largestDiff = 0;
    int largestDiffIndex = -1;

    for (int i = 0; i < 4; i++) {
        if (diffs[i] > largestDiff) {
            largestDiff = diffs[i];
            largestDiffIndex = i;
        }
    }

    if (largestDiffIndex != -1) {
        switch (largestDiffIndex) {
            case 0:
                avgVelocityX = -(movementB.a11 + movementC.a11 + movementD.a11) / 3.0f;
                avgVelocityY = -(movementB.a21 + movementC.a21 + movementD.a21) / 3.0f;
                avgOmega = -(movementB.a31 + movementC.a31 + movementD.a31) / 3.0f;
            break;

            case 1:
                avgVelocityX = -(movementA.a11 + movementC.a11 + movementD.a11) / 3.0f;
                avgVelocityY = -(movementA.a21 + movementC.a21 + movementD.a21) / 3.0f;
                avgOmega = -(movementA.a31 + movementC.a31 + movementD.a31) / 3.0f;
            break;

            case 2:
                avgVelocityX = -(movementA.a11 + movementB.a11 + movementD.a11) / 3.0f;
                avgVelocityY = -(movementA.a21 + movementB.a21 + movementD.a21) / 3.0f;
                avgOmega = -(movementA.a31 + movementB.a31 + movementD.a31) / 3.0f;
            break;

            case 3:
                avgVelocityX = -(movementA.a11 + movementB.a11 + movementC.a11) / 3.0f;
                avgVelocityY = -(movementA.a21 + movementB.a21 + movementC.a21) / 3.0f;
                avgOmega = -(movementA.a31 + movementB.a31 + movementC.a31) / 3.0f;
            break;
        }
    }

	movement.velocityX = avgVelocityX;
	movement.velocityY = avgVelocityY;
	movement.omega = avgOmega;

	Math::Vector velocityVector(movement.velocityX, movement.velocityY);

	lastVelocity = velocity;
	velocity = velocityVector.getLength() / lastDt;
}

void Robot::updateMeasurements() {
	measurements.clear();

	Object* yellowGoal = visionResults->getLargestGoal(Side::YELLOW);
	Object* blueGoal = visionResults->getLargestGoal(Side::BLUE);

	if (yellowGoal != NULL) {
		measurements["yellow-center"] = Measurement(yellowGoal->distance, yellowGoal->angle);
	}

	if (blueGoal != NULL) {
		measurements["blue-center"] = Measurement(blueGoal->distance, blueGoal->angle);
	}
}