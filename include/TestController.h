#ifndef TESTCONTROLLER_H
#define TESTCONTROLLER_H

#include "Controller.h"
#include "Vision.h"
#include "DebouncedButton.h"
#include "Config.h"

class TestController : public Controller {

public:
	TestController(Robot* robot, Communication* com);

    bool handleCommand(const Command& cmd);
	void handleCommunicationMessage(std::string message);
    void step(float dt, Vision::Results* visionResults);

};

#endif // TESTCONTROLLER_H
