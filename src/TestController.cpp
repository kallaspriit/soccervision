#include "TestController.h"

#include "Robot.h"
#include "Command.h"
#include "Util.h"

TestController::TestController(Robot* robot, Communication* com) : Controller(robot, com) {

};

void TestController::step(float dt, Vision::Results* visionResults) {
	
}

bool TestController::handleCommand(const Command& cmd) {
    if (cmd.name == "test-find-ball") {
        
    }

    return true;
}

void TestController::handleCommunicationMessage(std::string message) {
	if (Command::isValid(message)) {
        Command command = Command::parse(message);

		handleCommand(command);
	}
}