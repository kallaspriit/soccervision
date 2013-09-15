#ifndef MANUALCONTROLLER_H
#define MANUALCONTROLLER_H

#include "Controller.h"
#include "Vision.h"

class ManualController : public Controller {

public:
	ManualController(Robot* robot);

    bool handleRequest(std::string request);
    bool handleCommand(const Command& cmd);
    void step(float dt, Vision::Results* visionResults);
	std::string getJSON();

    void handleTargetVectorCommand(const Command& cmd);
    void handleTargetDirCommand(const Command& cmd);
    void handleSetDribblerCommand(const Command& cmd);
    void handleKickCommand(const Command& cmd);

};

#endif // MANUALCONTROLLER_H
