#ifndef SOCCERBOT_H
#define SOCCERBOT_H

#include "Vision.h"
#include "Controller.h"
#include <string>

class XimeaCamera;
class Blobber;
class ProcessThread;
class Gui;
class FpsCounter;
class Robot;
class Server;

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

	void addController(std::string name, Controller* controller);
    Controller* getController(std::string name);
    bool setController(std::string name);
    std::string getActiveControllerName();

	bool debugVision;
	bool showGui;

private:
	void setupCamera(std::string name, XimeaCamera* camera);
	bool fetchFrame(XimeaCamera* camera, ProcessThread* processor);

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
	ControllerMap controllers;
	std::string activeControllerName;

	bool controllerRequested;
	bool running;

};

#endif // SOCCERBOT_H