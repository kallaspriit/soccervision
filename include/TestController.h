#ifndef TESTCONTROLLER_H
#define TESTCONTROLLER_H

#include "BaseAI.h"
#include "Vision.h"
#include "DebouncedButton.h"
#include "Config.h"

class TestController : public BaseAI {

public:
	class State : public BaseAI::State {

	public:
		State(TestController* ai) : BaseAI::State(ai), ai(ai) {}

	protected:
		TestController* ai;

	};

	class IdleState : public State {

	public:
		IdleState(TestController* ai) : State(ai) {}
		void step(float dt, float totalDuration, float stateDuration) {}

	};

	class WatchBallState : public State {

	public:
		WatchBallState(TestController* ai) : State(ai) {}
		void onEnter();
		void onExit();
		void step(float dt, float totalDuration, float stateDuration);

	};

	
	TestController(Robot* robot, Communication* com);

    bool handleCommand(const Command& cmd);
	void handleToggleGoCommand();
    void step(float dt, Vision::Results* visionResults);
	std::string getJSON();

private:
	void setupStates();

	DebouncedButton toggleGoBtn;
	bool running;

};

#endif // TESTCONTROLLER_H
