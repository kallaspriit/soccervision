#ifndef MANUALCONTROLLER_H
#define MANUALCONTROLLER_H

#include "Controller.h"
#include "Vision.h"
#include "DebouncedButton.h"
#include "Config.h"

class ManualController : public Controller {

public:
	ManualController(Robot* robot, Communication* com);

	void onEnter() { reset(); }
	void onExit() { reset(); }
    bool handleRequest(std::string request);
    bool handleCommand(const Command& cmd);
	void handleCommunicationMessage(std::string message);
    void step(float dt, Vision::Results* visionResults);
	void reset();
	Side getTargetSide() { return targetSide; }
	std::string getJSON();

    void handleToggleSideCommand();
    void handleTargetVectorCommand(const Command& cmd);
    void handleTargetDirCommand(const Command& cmd);
    void handleSetDribblerCommand(const Command& cmd);
    void handleKickCommand(const Command& cmd);

private:
	Side targetSide;
	DebouncedButton toggleSideBtn;

};

#endif // MANUALCONTROLLER_H
