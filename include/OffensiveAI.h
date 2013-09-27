#ifndef OFFENSIVEAI_H
#define OFFENSIVEAI_H

#include "Controller.h"
#include "Vision.h"
#include "DebouncedButton.h"
#include "Config.h"

#include <string>
#include <map>

class OffensiveAI : public Controller {

public:
	class State {

	public:
		State(OffensiveAI* ai) : ai(ai) {}
		virtual void onEnter() {}
		virtual void onExit() {}
		virtual void step(float dt, float totalDuration, float stateDuration) = 0;

	protected:
		OffensiveAI* ai;

	};

	class IdleState : public State {

	public:
		IdleState(OffensiveAI* ai) : State(ai) {}
		void onEnter();
		void onExit();
		void step(float dt, float totalDuration, float stateDuration);

	};

	class FindBallState : public State {

	public:
		FindBallState(OffensiveAI* ai) : State(ai) {}
		void onEnter();
		void onExit();
		void step(float dt, float totalDuration, float stateDuration);

	};

	typedef std::map<std::string, State*> States;
	typedef States::iterator StatesIt;

	OffensiveAI(Robot* robot, Communication* com);
	~OffensiveAI();

	void onEnter();
	void onExit();
	void setState(std::string state);
    bool handleRequest(std::string request);
    bool handleCommand(const Command& cmd);
	void handleToggleSideCommand();
	void handleToggleGoCommand();
	void handleCommunicationMessage(std::string message);
    void step(float dt, Vision::Results* visionResults);
	void reset();
	Side getTargetSide() { return targetSide; }
	std::string getJSON();

private:
	void setupStates();

	Side targetSide;
	DebouncedButton toggleSideBtn;
	DebouncedButton toggleGoBtn;
	States states;
	State* currentState;
	std::string currentStateName;
	bool running;
	float totalDuration;
	float currentStateDuration;

};

#endif