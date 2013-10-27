#include "Tasks.h"
#include "Robot.h"
#include "Util.h"

#include <iostream>

// TurnBy angle task
void TurnByTask::onStart(Robot& robot, float dt) {
	speed = Math::abs(speed);
    startAngle = robot.getOrientation();
	targetAngle = Math::floatModulus(startAngle + turnAngle, Math::TWO_PI);
	dir = turnAngle < 0.0f ? -1.0f : 1.0f;
	startTime = -1;
	maxTurnTime = Math::abs(turnAngle / speed) * 1.5f;
	lastDiff = -1.0f;
}

bool TurnByTask::onStep(Robot& robot, float dt) {
	if (startTime == -1) {
		startTime = Util::millitime();
	}

    float currentAngle = robot.getOrientation();

	//float newDiff = Math::abs(currentAngle - targetAngle);

	float newDiff;

	if (dir == 1.0f) {
		newDiff = targetAngle - currentAngle;
	} else {
		newDiff = currentAngle - targetAngle;
	}

	if (newDiff < 0.0f) {
		newDiff += Math::TWO_PI;
	}

	if (
		newDiff < threshold
		//|| (lastDiff != -1 && newDiff - lastDiff > threshold)
		|| Util::duration(startTime) > maxTurnTime
	) {
		return false;
	}

	lastDiff = newDiff;

	float useSpeed = speed * dir;

    robot.setTargetDir(0.0f, 0.0f, useSpeed);

    return true;
}

void TurnByTask::onEnd(Robot& robot, float dt) {
    robot.stop();
}

float TurnByTask::getPercentage() {
    if (!started) {
        return 0.0f;
    }

    return Math::max(100.0f - (lastDiff * 100.0f / Math::abs(turnAngle)), 0.0f);
}

std::string TurnByTask::toString() {
    return "TurnBy " + Util::toString(Math::round(Math::radToDeg(turnAngle), 1)) + " deg";
}

// DriveTo coordinates task
void DriveToTask::onStart(Robot& robot, float dt) {
    Math::Position pos = robot.getPosition();

    startX = pos.x;
    startY = pos.y;
    startOrientation = pos.orientation;

    startDistance = Math::distanceBetween(pos.x, pos.y, targetX, targetY);
}

bool DriveToTask::onStep(Robot& robot, float dt) {
    Math::Position pos = robot.getPosition();
    Math::Vector target(targetX, targetY);

    currentDistance = Math::distanceBetween(pos.x, pos.y, targetX, targetY);

    float currentOrientation = pos.orientation;
    float orientationDiff = Math::getAngleDiff(currentOrientation, targetOrientation);

    if (currentDistance <= positionThreshold && Math::abs(orientationDiff) < orientationThreshold) {
        return false;
    }

    float omega = orientationDiff / (currentDistance * 0.5f);
    float useSpeed = speed;

    if (currentDistance < 0.2f) {
        useSpeed = speed * currentDistance * 5.0f;
    }

    Math::Vector globalDir = Math::Vector::createDirVec(target, pos).getNormalized().getScaled(useSpeed);
    Math::Vector localDir = globalDir.getRotated(-currentOrientation);

    //std::cout << "[" << Util::round(Math::radToDeg(currentOrientation)) << "] " << Util::round(globalDir.x, 1) << "x" << Util::round(globalDir.y) << " > " << Util::round(localDir.x, 1) << "x" << Util::round(localDir.y) << std::endl;
    //std::cout << "diff: " << Math::radToDeg(orientationDiff) << "; current: " << Math::radToDeg(currentOrientation) << "; to: " << Math::radToDeg(targetOrientation) << "; omega: " << omega << std::endl;
    //std::cout << "omega: " << omega << std::endl;

    if (omega < -3.0f) {
        omega = -3.0f;
    } else if (omega > 3.0f) {
        omega = 3.0f;
    }

    robot.setTargetDir(localDir.x, localDir.y, omega);

    return true;
}

void DriveToTask::onEnd(Robot& robot, float dt) {
    robot.setTargetDir(0, 0, 0);
}

float DriveToTask::getPercentage() {
    if (!started) {
        return 0.0f;
    }

    return 100.0f - (currentDistance * 100.0f / startDistance);
}

std::string DriveToTask::toString() {
    return "DriveTo " + Util::toString(targetX) + "x" + Util::toString(targetY) + " @ " + Util::toString(Math::round(Math::radToDeg(targetOrientation), 1)) + " deg";
}


// Follow path of coordinates task
void DrivePathTask::onStart(Robot& robot, float dt) {
    Math::Position pos = robot.getPosition();

    startX = pos.x;
    startY = pos.y;
    startOrientation = pos.orientation;
}

bool DrivePathTask::onStep(Robot& robot, float dt) {
    if (positions.size() == 0) {
        return false;
    }

    Math::Position currentPos = robot.getPosition();
    Math::Position targetPos = positions.front();

    float currentDistance = Math::distanceBetween(currentPos.x, currentPos.y, targetPos.x, targetPos.y);
    float orientationDiff = Math::getAngleDiff(currentPos.orientation, targetPos.orientation);

    if (currentDistance <= positionThreshold && Math::abs(orientationDiff) < orientationThreshold) {
        positions.pop();

        if (positions.size() == 0) {
            return false;
        } else {
            return onStep(robot, dt);
        }
    }

    float omega = orientationDiff / (currentDistance * 0.5f);
    float useSpeed = speed;

    if (currentDistance < 0.2f) {
        useSpeed = speed * currentDistance * 5.0f;
    }

    Math::Vector globalDir = Math::Vector::createDirVec(targetPos, currentPos).getNormalized().getScaled(useSpeed);
    Math::Vector localDir = globalDir.getRotated(-currentPos.orientation);

    if (omega < -3.0f) {
        omega = -3.0f;
    } else if (omega > 3.0f) {
        omega = 3.0f;
    }

    robot.setTargetDir(localDir.x, localDir.y, omega);

    return true;
}

void DrivePathTask::onEnd(Robot& robot, float dt) {
    robot.setTargetDir(0, 0, 0);
}

float DrivePathTask::getPercentage() {
    if (!started) {
        return 0.0f;
    }

    return 100.0f - (positions.size() * 100.0f / startPositionCount);
}

std::string DrivePathTask::toString() {
    if (positions.size() == 0) {
        return "DrivePath - empty";
    }

    Math::Position targetPos = positions.front();

    return "DrivePath " + Util::toString(startPositionCount - positions.size()) + "/" + Util::toString(startPositionCount) + " - " + Util::toString(targetPos.x) + "x" + Util::toString(targetPos.y) + " @ " + Util::toString(Math::round(Math::radToDeg(targetPos.orientation), 1)) + " deg";
}


// drive to coordinates facing some other coordinates
void DriveFacingTask::onStart(Robot& robot, float dt) {
    Math::Position pos = robot.getPosition();

    startX = pos.x;
    startY = pos.y;
    startOrientation = pos.orientation;

    startDistance = Math::distanceBetween(pos.x, pos.y, targetX, targetY);
}

bool DriveFacingTask::onStep(Robot& robot, float dt) {
    Math::Position pos = robot.getPosition();
    Math::Vector target(targetX, targetY);

    currentDistance = Math::distanceBetween(pos.x, pos.y, targetX, targetY);

    Math::Vector targetVec = Math::Vector::createDirVec(pos, Math::Vector(targetX, targetY));
    float targetOrientation = Math::floatModulus(targetVec.getAngleBetween(Math::Vector(1.0f, 0.0f)) + Math::PI, Math::TWO_PI);
    float currentOrientation = pos.orientation;
    float orientationDiff = Math::getAngleDiff(currentOrientation, targetOrientation);

    if (currentDistance <= positionThreshold && Math::abs(orientationDiff) < orientationThreshold) {
        return false;
    }

    float omega = orientationDiff / (currentDistance * 0.5f);
    float useSpeed = speed;

    if (currentDistance < 0.2f) {
        useSpeed = speed * currentDistance * 5.0f;
    }

    Math::Vector globalDir = Math::Vector::createDirVec(target, pos).getNormalized().getScaled(useSpeed);
    Math::Vector localDir = globalDir.getRotated(-currentOrientation);

    //std::cout << "[" << Util::round(Math::radToDeg(currentOrientation)) << "] " << Util::round(globalDir.x, 1) << "x" << Util::round(globalDir.y) << " > " << Util::round(localDir.x, 1) << "x" << Util::round(localDir.y) << std::endl;
    //std::cout << "diff: " << Math::radToDeg(orientationDiff) << "; current: " << Math::radToDeg(currentOrientation) << "; to: " << Math::radToDeg(targetOrientation) << "; omega: " << omega << std::endl;
    //std::cout << "omega: " << omega << std::endl;

    if (omega < -3.0f) {
        omega = -3.0f;
    } else if (omega > 3.0f) {
        omega = 3.0f;
    }

    robot.setTargetDir(localDir.x, localDir.y, omega);

    return true;
}

void DriveFacingTask::onEnd(Robot& robot, float dt) {
    robot.setTargetDir(0, 0, 0);
}

float DriveFacingTask::getPercentage() {
    if (!started) {
        return 0.0f;
    }

    return 100.0f - (currentDistance * 100.0f / startDistance);
}

std::string DriveFacingTask::toString() {
    return "DriveFacing " + Util::toString(targetX) + "x" + Util::toString(targetY) + " facing " + Util::toString(faceX) + "x" + Util::toString(faceY);
}

// stops rotation
void StopRotationTask::onStart(Robot& robot, float dt) {
	startOmega = robot.getMovement().omega;
	startSign = startOmega > 0 ? 1 : 0;
	startTime = Util::millitime();
}

bool StopRotationTask::onStep(Robot& robot, float dt) {
	currentOmega = robot.getMovement().omega;
	diff = Math::abs(currentOmega - Config::rotationStoppedOmegaThreshold);
	int currentSign = currentOmega > 0 ? 1 : 0;

	if (
		Math::abs(currentOmega) < Config::rotationStoppedOmegaThreshold
		|| currentSign != startSign
		|| Util::duration(startTime) > 0.5f
	) {
		return false;
	}

	if (currentOmega > 0) {
		robot.setTargetDir(0, 0, -diff * Config::rotationCancelMultiplier);
	} else {
		robot.setTargetDir(0, 0, diff * Config::rotationCancelMultiplier);
	}

	return true;
}

void StopRotationTask::onEnd(Robot& robot, float dt) {
	robot.stop();
}

float StopRotationTask::getPercentage() {
    if (!started) {
        return 0.0f;
    }

	return 100.0f - (diff * 100.0f / Math::abs(startOmega));
}

std::string StopRotationTask::toString() {
    return "StopRotation";
}

// jumps by an angle
void JumpAngleTask::onStart(Robot& robot, float dt) {
	startOrientation = robot.getOrientation();
	targetOrientation = Math::floatModulus(startOrientation + angle, Math::TWO_PI);
	breaking = false;
}

bool JumpAngleTask::onStep(Robot& robot, float dt) {
	float currentOrientation = robot.getOrientation();
	float currentOmega = robot.getMovement().omega;
	diff = Math::abs(currentOrientation - targetOrientation);
	
	
	if (
		diff < 0.2f
		|| breaking
	) {
		breaking = true;

		if (
			Math::abs(currentOmega) < Config::rotationStoppedOmegaThreshold * 2
			|| currentOmega < 0.0f
		) {
			return false;
		} else {
			if (currentOmega > 0) {
				robot.setTargetDir(0, 0, -diff * Config::jumpAngleStopMultiplier);
			} else {
				robot.setTargetDir(0, 0, diff * Config::jumpAngleStopMultiplier);
			}
		}
	} else {
		robot.setTargetDir(0, 0, speed);
	}

	return true;
}

void JumpAngleTask::onEnd(Robot& robot, float dt) {
	robot.stop();
}

float JumpAngleTask::getPercentage() {
    if (!started) {
        return 0.0f;
    }

	return 100.0f - (diff * 100.0f / Math::abs(angle));
}

std::string JumpAngleTask::toString() {
    return "JumpAngle";
}


// drives for an duration
void DriveForTask::onStart(Robot& robot, float dt) {
	startTime = Util::millitime();
	endTime = startTime + duration;
}

bool DriveForTask::onStep(Robot& robot, float dt) {
	currentTime = Util::millitime();

	if (currentTime > endTime) {
		robot.stop();

		return false;
	}

	robot.setTargetDir(x, y, omega);

	return true;
}

void DriveForTask::onEnd(Robot& robot, float dt) {
	robot.stop();
}

float DriveForTask::getPercentage() {
    if (!started) {
        return 0.0f;
    }

	double timeRemaining = endTime - currentTime;

	return (float)(100.0 - (timeRemaining * 100.0 / duration));
}

std::string DriveForTask::toString() {
    return "Drive " + Util::toString(x) + "x" + Util::toString(y) + " @ " + Util::toString(omega) + " for " + Util::toString(duration) + "seconds";
}


// drive behind ball
void DriveBehindBallTask::onStart(Robot& robot, float dt) {
	startTime = Util::millitime();
	endTime = startTime + ballDistance / speed + 1.0;
	startSpeed = robot.getVelocity();
}

bool DriveBehindBallTask::onStep(Robot& robot, float dt) {
	currentTime = Util::millitime();

	duration += dt;

	if (currentTime > endTime) {
		robot.stop();

		return false;
	}
	//float sideP = 0.4f;
	float sideAccelerationDuration = 0.5f;
	float deaccelerationDuration = 0.5f;
	double sideSpeedDelay = ballDistance / speed;
	float deacceleratedSpeed = Math::map((float)duration, 0.0f, deaccelerationDuration, startSpeed, speed);
	float sideSpeed = side * Math::map((float)(duration - sideSpeedDelay), 0.0f, sideAccelerationDuration, 0.0f, deacceleratedSpeed);

	Math::Vector dirVector = Math::Vector::createForwardVec(targetAngle, deacceleratedSpeed);

	dirVector.y += sideSpeed;

	robot.setTargetDir(dirVector.x, dirVector.y);
	//robot.lookAt(goal);

	return true;
}

void DriveBehindBallTask::onEnd(Robot& robot, float dt) {
	
}

float DriveBehindBallTask::getPercentage() {
    if (!started) {
        return 0.0f;
    }

	double timeRemaining = endTime - currentTime;

	return (float)(100.0 - (timeRemaining * 100.0 / duration));
}

std::string DriveBehindBallTask::toString() {
    return "Drive behind ball";
}