#ifndef MANUALCONTROLLER_H
#define MANUALCONTROLLER_H

#include "Controller.h"
#include "Vision.h"
#include "DebouncedButton.h"

class ManualController : public Controller {

public:
	ManualController(Robot* robot, Communication* com);

    bool handleRequest(std::string request);
    bool handleCommand(const Command& cmd);
	void handleCommunicationMessage(std::string message);
    void step(float dt, Vision::Results* visionResults);
	Side getTargetSide() { return targetSide; }
	std::string getJSON();

    void handleToggleSideCommand();
    void handleToggleGoCommand();
    void handleTargetVectorCommand(const Command& cmd);
    void handleTargetDirCommand(const Command& cmd);
    void handleSetDribblerCommand(const Command& cmd);
    void handleKickCommand(const Command& cmd);

private:
	Side targetSide;
	bool running;
	DebouncedButton toggleSideBtn;
	DebouncedButton toggleGoBtn;

	// TODO Remove these tests
	int dir;
	float speed;

};

#endif // MANUALCONTROLLER_H
