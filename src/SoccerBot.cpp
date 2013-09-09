#include "SoccerBot.h"
#include "XimeaCamera.h"
#include "Vision.h"
#include "ProcessThread.h"
#include "Gui.h"
#include "FpsCounter.h"
#include "Config.h"

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
	frontBlobber->saveOptions(Config::blobberConfigFilename);

	if (gui != NULL) {
		delete gui;
	}

	delete frontCamera;
	delete rearCamera;
	delete fpsCounter;
	delete frontProcessor;
	delete rearProcessor;
	delete visionResults;
	delete frontVision;
	delete rearVision;
	delete frontBlobber;
	delete rearBlobber;
}

void SoccerBot::setup() {
	 std::cout << "-- Initializing --" << std::endl;

	setupVision();
	setupProcessors();
	setupFpsCounter();
	setupCameras();
}

void SoccerBot::run() {
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
		}

		return;
	}

	bool gotFrontFrame, gotRearFrame;

	while (running) {
		gotFrontFrame = gotRearFrame = false;
		frontProcessor->debug = rearProcessor->debug = debugVision || showGui;

		gotFrontFrame = fetchFrame(frontCamera, frontProcessor);
		gotRearFrame = fetchFrame(rearCamera, rearProcessor);

		if (!gotFrontFrame && !gotRearFrame) {
			std::cout << "- Didn't get any frames from either of the cameras" << std::endl;

			continue;
		}

		fpsCounter->step();

		if (gotFrontFrame) {
			frontProcessor->start();
		} else {
			std::cout << "- No image from front camera" << std::endl;
		}

		if (gotRearFrame) {
			rearProcessor->start();
		} else {
			std::cout << "- No image from rear camera" << std::endl;
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

		//if (fpsCounter->frameNumber % 60) {
			std::cout << "! FPS: " << fpsCounter->getFps() << std::endl;
		//}
	}
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
	frontBlobber = new Blobber();
	rearBlobber = new Blobber();

	frontBlobber->initialize(Config::cameraWidth, Config::cameraHeight);
	rearBlobber->initialize(Config::cameraWidth, Config::cameraHeight);

	frontBlobber->loadOptions(Config::blobberConfigFilename);
	rearBlobber->loadOptions(Config::blobberConfigFilename);

	frontVision = new Vision(frontBlobber, Dir::FRONT, Config::cameraWidth, Config::cameraHeight);
	rearVision = new Vision(rearBlobber, Dir::REAR, Config::cameraWidth, Config::cameraHeight);

	visionResults = new Vision::Results();
}

void SoccerBot::setupProcessors() {
	frontProcessor = new ProcessThread(frontBlobber, frontVision);
	rearProcessor = new ProcessThread(rearBlobber, rearVision);
}

void SoccerBot::setupFpsCounter() {
	fpsCounter = new FpsCounter();
}

void SoccerBot::setupCameras() {
	frontCamera = new XimeaCamera();
	rearCamera = new XimeaCamera();

	frontCamera->open(Config::frontCameraSerial);
	rearCamera->open(Config::rearCameraSerial);

	setupCamera("Front", frontCamera);
	setupCamera("Rear", rearCamera);
}

void SoccerBot::setupGui() {
	gui = new Gui(
		GetModuleHandle(0),
		frontBlobber, rearBlobber,
		Config::cameraWidth, Config::cameraHeight
	);
}

void SoccerBot::setupCamera(std::string name, XimeaCamera* camera) {
	camera->setGain(6);
	camera->setExposure(10000);
	camera->setFormat(XI_RAW8);
	camera->setAutoWhiteBalance(false);
	camera->setAutoExposureGain(false);
	camera->setQueueSize(12); // TODO Affects anything?
}
