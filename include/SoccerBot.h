#ifndef SOCCERBOT_H
#define SOCCERBOT_H

#include "Vision.h"
#include "Controller.h"
#include "Server.h"
#include "Command.h"
#include <string>

class XimeaCamera;
class Blobber;
class ProcessThread;
class Gui;
class FpsCounter;
class Robot;
class Communication;

class SoccerBot {

public:
	SoccerBot();
	~SoccerBot();

	void setup();
	void run();

	void setupVision();
	void setupProcessors();
	void setupFpsCounter();
	void setupCameras();
	void setupRobot();
	void setupControllers();
	void setupSignalHandler();
	void setupGui();
	void setupServer();
	void setupCommunication();

	void addController(std::string name, Controller* controller);
    Controller* getController(std::string name);
    bool setController(std::string name);
    std::string getActiveControllerName();

	void handleServerMessages();
	void handleServerMessage(Server::Message* message);
	void handleGetControllerCommand(Server::Message* message);
	void handleSetControllerCommand(Command::Parameters parameters, Server::Message* message);
	void handleGetFrameCommand();
	void handleCameraChoiceCommand(Command::Parameters parameters);
	void handleBlobberThresholdCommand(Command::Parameters parameters);
	void handleBlobberClearCommand(Command::Parameters parameters);
	void handleScreenshotCommand();

	void handleCommunicationMessages();
	void handleCommunicationMessage(std::string message);

	std::string getStateJSON();

	bool debugVision;
	bool showGui;

private:
	void setupCamera(std::string name, XimeaCamera* camera);
	bool fetchFrame(XimeaCamera* camera, ProcessThread* processor);
	void broadcastFrame(unsigned char* rgb, unsigned char* classification);

	XimeaCamera* frontCamera;
	XimeaCamera* rearCamera;
	Blobber* frontBlobber;
	Blobber* rearBlobber;
	Vision* frontVision;
	Vision* rearVision;
	ProcessThread* frontProcessor;
	ProcessThread* rearProcessor;
	Gui* gui;
	FpsCounter* fpsCounter;
	Vision::Results* visionResults;
	Server* server;
	Robot* robot;
	Controller* activeController;
	Communication* com;
	ControllerMap controllers;
	std::string activeControllerName;

	bool controllerRequested;
	bool running;
	bool frameRequested;
	float dt;
	double lastStepTime;
	float totalTime;
	Dir debugCameraDir;

	unsigned char* jpegBuffer;
};

#endif // SOCCERBOT_H