#ifndef OFFENSIVEAI_H
#define OFFENSIVEAI_H

#include "BaseAI.h"
#include "Vision.h"
#include "DebouncedButton.h"
#include "Config.h"

#include <string>
#include <map>

class OffensiveAI : public BaseAI {

public:
	class State : public BaseAI::State {

	public:
		State(OffensiveAI* ai) : BaseAI::State(ai), ai(ai) {}

	protected:
		OffensiveAI* ai;

	};

	class IdleState : public State {

	public:
		IdleState(OffensiveAI* ai) : State(ai) {}
		void step(float dt, float totalDuration, float stateDuration);

	};

	class FindBallState : public State {

	public:
		FindBallState(OffensiveAI* ai) : State(ai) {}
		void onEnter();
		void onExit();
		void step(float dt, float totalDuration, float stateDuration);

	};

	OffensiveAI(Robot* robot, Communication* com);

	void onEnter();
	void onExit();
    bool handleRequest(std::string request);
    bool handleCommand(const Command& cmd);
	void handleToggleSideCommand();
	void handleToggleGoCommand();
    void step(float dt, Vision::Results* visionResults);
	void reset();
	bool isPlaying() { return running; }
	Side getTargetSide() { return targetSide; }
	std::string getJSON();

private:
	void setupStates();

	Side targetSide;
	std::string startStateName;
	DebouncedButton toggleSideBtn;
	DebouncedButton toggleGoBtn;
	bool running;

};

#endif // OFFENSIVEAI_H