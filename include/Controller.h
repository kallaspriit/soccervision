#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "InfoBoard.h"
#include "Vision.h"

#include <string>
#include <map>

class Robot;
class Command;

class Controller : public InfoBoardListener {

public:
    Controller(Robot* robot) : robot(robot) {}
	virtual void onEnter() {}
	virtual void onExit() {}
	virtual bool handleRequest(std::string request) { return false; }
	virtual bool handleCommand(const Command& cmd) { return false; }
	virtual void onGoRequestedChange(bool isGoRequested) {}
	virtual void onTargetSideChange(Side newTargetSide) {}
	virtual void step(float dt, Vision::Results* visionResults) = 0;
	virtual std::string getJSON() { return "null"; }

protected:
    Robot* robot;

};

typedef std::map<std::string, Controller*> ControllerMap;

#endif // CONTROLLER_H
