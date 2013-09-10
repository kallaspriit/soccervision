#include "SoccerBot.h"
#include "XimeaCamera.h"
#include "Vision.h"
#include "ProcessThread.h"
#include "Gui.h"
#include "FpsCounter.h"
#include "SignalHandler.h"
#include "Config.h"
#include "Util.h"

#include <iostream>

SoccerBot::SoccerBot() :
	frontCamera(NULL), rearCamera(NULL),
	frontBlobber(NULL), rearBlobber(NULL),
	frontVision(NULL), rearVision(NULL),
	frontProcessor(NULL), rearProcessor(NULL),
	gui(NULL), fpsCounter(NULL), visionResults(NULL),
	running(false), debugVision(false), showGui(false)
{

}

SoccerBot::~SoccerBot() {
	std::cout << "! Releasing all resources" << std::endl;

	if (gui != NULL) delete gui; gui = NULL;
	if (frontCamera != NULL) delete frontCamera; frontCamera = NULL;
	if (rearCamera != NULL) delete rearCamera; rearCamera = NULL;
	if (fpsCounter != NULL) delete fpsCounter; fpsCounter = NULL;
	if (frontProcessor != NULL) frontBlobber->saveOptions(Config::blobberConfigFilename); delete frontProcessor; frontProcessor = NULL;
	if (rearProcessor != NULL) delete rearProcessor; rearProcessor = NULL;
	if (visionResults != NULL) delete visionResults; visionResults = NULL;
	if (frontVision != NULL) delete frontVision; frontVision = NULL;
	if (rearVision != NULL) delete rearVision; rearVision = NULL;
	if (frontBlobber != NULL) delete frontBlobber; frontBlobber = NULL;
	if (rearBlobber != NULL) delete rearBlobber; rearBlobber = NULL;

	std::cout << "! Resources freed" << std::endl;
}

void SoccerBot::setup() {
	std::cout << "-- Initializing --" << std::endl;

	setupVision();
	setupProcessors();
	setupFpsCounter();
	//setupCameras();
	setupSignalHandler();

	if (showGui) {
		setupGui();
	}
}

void SoccerBot::run() {
	std::cout << "! Starting main loop" << std::endl;

	running = true;

	if (frontCamera->isOpened()) {
		frontCamera->startAcquisition();
	}

	if (rearCamera->isOpened()) {
		rearCamera->startAcquisition();
	}

	if (!frontCamera->isOpened() && !rearCamera->isOpened()) {
		std::cout << "! Neither of the cameras was opened, running in test mode" << std::endl;

		while (running) {
			Sleep(100);

			if (SignalHandler::exitRequested) {
				running = false;
			}
		}

		return;
	}

	bool gotFrontFrame, gotRearFrame;

	while (running) {
		//__int64 startTime = Util::timerStart();

		gotFrontFrame = gotRearFrame = false;
		frontProcessor->debug = rearProcessor->debug = debugVision || showGui;

		gotFrontFrame = fetchFrame(frontCamera, frontProcessor);
		gotRearFrame = fetchFrame(rearCamera, rearProcessor);

		if (!gotFrontFrame && !gotRearFrame && fpsCounter->frameNumber > 0) {
			std::cout << "- Didn't get any frames from either of the cameras" << std::endl;

			continue;
		}

		fpsCounter->step();

		if (gotFrontFrame) {
			frontProcessor->start();
		}

		if (gotRearFrame) {
			rearProcessor->start();
		}

		if (gotFrontFrame) {
			frontProcessor->join();
			visionResults->front = frontProcessor->visionResult;
		}

		if (gotRearFrame) {
			rearProcessor->join();
			visionResults->rear = rearProcessor->visionResult;
		}

		if (showGui) {
			if (gui == NULL) {
				setupGui();
			}

			gui->setFps(fpsCounter->getFps());

			if (gotFrontFrame) {
				gui->setFrontImages(
					frontProcessor->rgb,
					frontProcessor->dataYUYV,
					frontProcessor->dataY, frontProcessor->dataU, frontProcessor->dataV,
					frontProcessor->classification
				);
			}

			if (gotRearFrame) {
				gui->setRearImages(
					rearProcessor->rgb,
					rearProcessor->dataYUYV,
					rearProcessor->dataY, rearProcessor->dataU, rearProcessor->dataV,
					rearProcessor->classification
				);
			}

			gui->update();
		}

		if (fpsCounter->frameNumber % 60 == 0) {
			std::cout << "! FPS: " << fpsCounter->getFps() << std::endl;
		}

		if (SignalHandler::exitRequested) {
			running = false;
		}

		//std::cout << "! Total time: " << Util::timerEnd(startTime) << std::endl;
	}

	std::cout << "! Main loop ended" << std::endl;
}

bool SoccerBot::fetchFrame(XimeaCamera* camera, ProcessThread* processor) {
	if (camera->isAcquisitioning()) {
		const BaseCamera::Frame* frame = camera->getFrame();

		if (frame != NULL) {
			if (frame->fresh) {
				processor->setFrame(frame->data);

				return true;
			}
		}
	}

	return false;
}

void SoccerBot::setupVision() {
	std::cout << "! Setting up vision.. ";

	frontBlobber = new Blobber();
	rearBlobber = new Blobber();

	frontBlobber->initialize(Config::cameraWidth, Config::cameraHeight);
	rearBlobber->initialize(Config::cameraWidth, Config::cameraHeight);

	frontBlobber->loadOptions(Config::blobberConfigFilename);
	rearBlobber->loadOptions(Config::blobberConfigFilename);

	frontVision = new Vision(frontBlobber, Dir::FRONT, Config::cameraWidth, Config::cameraHeight);
	rearVision = new Vision(rearBlobber, Dir::REAR, Config::cameraWidth, Config::cameraHeight);

	visionResults = new Vision::Results();

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupProcessors() {
	std::cout << "! Setting up processor threads.. ";

	frontProcessor = new ProcessThread(frontBlobber, frontVision);
	rearProcessor = new ProcessThread(rearBlobber, rearVision);

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupFpsCounter() {
	std::cout << "! Setting up fps counter.. ";

	fpsCounter = new FpsCounter();

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupGui() {
	std::cout << "! Setting up GUI.. ";

	gui = new Gui(
		GetModuleHandle(0),
		frontBlobber, rearBlobber,
		Config::cameraWidth, Config::cameraHeight
	);

	std::cout << "done!" << std::endl;
}

void SoccerBot::setupCameras() {
	std::cout << "! Setting up cameras" << std::endl;

	frontCamera = new XimeaCamera();
	rearCamera = new XimeaCamera();

	frontCamera->open(Config::frontCameraSerial);
	rearCamera->open(Config::rearCameraSerial);

	setupCamera("Front", frontCamera);
	setupCamera("Rear", rearCamera);

	std::cout << "! Cameras ready" << std::endl;
}

void SoccerBot::setupCamera(std::string name, XimeaCamera* camera) {
	camera->setGain(6);
	camera->setExposure(10000);
	camera->setFormat(XI_RAW8);
	camera->setAutoWhiteBalance(false);
	camera->setAutoExposureGain(false);
	camera->setQueueSize(12); // TODO Affects anything?

	std::cout << "! " << name << " camera info:" << std::endl;
	std::cout << "  > Name: " << camera->getName() << std::endl;
	std::cout << "  > Type: " << camera->getDeviceType() << std::endl;
	std::cout << "  > API version: " << camera->getApiVersion() << std::endl;
	std::cout << "  > Driver version: " << camera->getDriverVersion() << std::endl;
	std::cout << "  > Serial number: " << camera->getSerialNumber() << std::endl;
	std::cout << "  > Color: " << (camera->supportsColor() ? "yes" : "no") << std::endl;
	std::cout << "  > Framerate: " << camera->getFramerate() << std::endl;
	std::cout << "  > Available bandwidth: " << camera->getAvailableBandwidth() << std::endl;
}

void SoccerBot::setupSignalHandler() {
	SignalHandler::setup();
}