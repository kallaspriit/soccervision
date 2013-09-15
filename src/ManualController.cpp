#include "ManualController.h"

#include "Robot.h"
#include "Command.h"
#include "Dribbler.h"
#include "Coilgun.h"
#include "Util.h"

ManualController::ManualController(Robot* robot) : Controller(robot) {
	
};

void ManualController::step(float dt, Vision::Results* visionResults) {
	robot->setAutostop(false);
}

bool ManualController::handleRequest(std::string request) {
    return false;
}

bool ManualController::handleCommand(const Command& cmd) {
    if (cmd.name == "target-vector" && cmd.params.size() == 3) {
        handleTargetVectorCommand(cmd);
    } else if (cmd.name == "target-dir" && cmd.params.size() == 3) {
        handleTargetDirCommand(cmd);
    } else if (cmd.name == "set-dribbler" && cmd.params.size() == 1) {
        handleSetDribblerCommand(cmd);
    } else if (cmd.name == "kick" && cmd.params.size() == 1) {
        handleKickCommand(cmd);
    } else {
        return false;
    }

    return true;
}

void ManualController::handleTargetVectorCommand(const Command& cmd) {
    float x = Util::toFloat(cmd.params[0]);
    float y = Util::toFloat(cmd.params[1]);
    float omega = Util::toFloat(cmd.params[2]);

    robot->setTargetDir(x, y, omega);
}

void ManualController::handleTargetDirCommand(const Command& cmd) {
    Math::Deg dir = Math::Deg(Util::toFloat(cmd.params[0]));
    float speed = Util::toFloat(cmd.params[1]);
    float omega = Util::toFloat(cmd.params[2]);

    robot->setTargetDir(dir, speed, omega);
}

void ManualController::handleSetDribblerCommand(const Command& cmd) {
    int strength = Util::toInt(cmd.params[0]);

	robot->getDribbler()->setSpeed(strength);
}

void ManualController::handleKickCommand(const Command& cmd) {
    int strength = Util::toInt(cmd.params[0]);

	robot->getCoilgun()->kick(strength);
}

std::string ManualController::getJSON() {
	std::stringstream stream;

    stream << "null";

    return stream.str();
}
