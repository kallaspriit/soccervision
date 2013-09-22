#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Vision.h"
#include "Communication.h"
#include "Command.h"

#include <string>
#include <map>

class Robot;
class Command;

class Controller : public Communication::Listener, public Command::Listener {

public:
    Controller(Robot* robot, Communication* com) : robot(robot), com(com) {}
	virtual void onEnter() {}
	virtual void onExit() {}
	virtual bool handleRequest(std::string request) { return false; }
	virtual bool handleCommand(const Command& cmd) { return false; }
	virtual void handleCommunicationMessage(std::string message) {}
	virtual void step(float dt, Vision::Results* visionResults) = 0;
	virtual std::string getJSON() { return "null"; }

protected:
    Robot* robot;
	Communication* com;

};

typedef std::map<std::string, Controller*> ControllerMap;

#endif // CONTROLLER_H
