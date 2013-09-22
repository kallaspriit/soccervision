#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Vision.h"
#include "Communication.h"

#include <string>
#include <map>

class Robot;
class Command;

class Controller : public Communication::Listener {

public:
    Controller(Robot* robot) : robot(robot) {}
	virtual void onEnter() {}
	virtual void onExit() {}
	virtual bool handleRequest(std::string request) { return false; }
	virtual bool handleCommand(const Command& cmd) { return false; }
	virtual void handleCommunicationMessage(std::string message) {}
	virtual void step(float dt, Vision::Results* visionResults) = 0;
	virtual std::string getJSON() { return "null"; }

protected:
    Robot* robot;

};

typedef std::map<std::string, Controller*> ControllerMap;

#endif // CONTROLLER_H
