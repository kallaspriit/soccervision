#include "ProcessThread.h"
#include "Blobber.h"
#include "ImageProcessor.h"
#include "DebugRenderer.h"
#include "Canvas.h"
#include "Config.h"

#include <iostream>

ProcessThread::ProcessThread(Blobber* blobber, Vision* vision) : Thread(), dir(dir), blobber(blobber), vision(vision), visionResult(NULL), debug(false), done(true), rgbSwapRB(false), classificationSwapRB(false) {
	frame = NULL;
	width = blobber->getWidth();
	height = blobber->getHeight();
	dataY = new unsigned char[width * height];
    dataU = new unsigned char[(width / 2) * (height / 2)];
    dataV = new unsigned char[(width / 2) * (height / 2)];
	dataYUYV = new unsigned char[width * height * 3];
	classification = new unsigned char[width * height * 3];
	argb = new unsigned char[width * height * 4];
	rgb = new unsigned char[width * height * 3];
}

ProcessThread::~ProcessThread() {
	if (visionResult != NULL) {
		delete visionResult;
		visionResult = NULL;
	}

	delete dataY;
	delete dataU;
	delete dataV;
	delete dataYUYV;
	delete classification;
	delete argb;
	delete rgb;
}

void* ProcessThread::run() {
	if (frame == NULL) {
		return NULL;
	}

	if (visionResult != NULL) {
		delete visionResult;
		visionResult = NULL;
	}

	done = false;
	
	//Util::timerStart();
	ImageProcessor::bayerRGGBToI420(
		frame,
		dataY, dataU, dataV,
		width, height
	);
	//std::cout << "  - RGGB > I420: " << Util::timerEnd() << std::endl;

	//Util::timerStart();
	ImageProcessor::I420ToYUYV(
		dataY, dataU, dataV,
		dataYUYV,
		width, height
	);
	//std::cout << "  - I420 > YUYV: " << Util::timerEnd() << std::endl;

	//Util::timerStart();
	blobber->processFrame((Blobber::Pixel*)dataYUYV);
	//std::cout << "  - Process:     " << Util::timerEnd() << " (" << blobber->getBlobCount("ball") << " ball blobs)" << std::endl;

	if (debug) {
		//Util::timerStart();
		blobber->classify((Blobber::Rgb*)classification, (Blobber::Pixel*)dataYUYV);
		//std::cout << "  - Blobber classify: " << Util::timerEnd() << std::endl;

		//Util::timerStart();
		ImageProcessor::YUYVToARGB(dataYUYV, argb, width, height);
		//std::cout << "  - YUYV > ARGB: " << Util::timerEnd() << std::endl;

		//Util::timerStart();
		//ImageProcessor::ARGBToRGB24(
		ImageProcessor::ARGBToRGB(
			argb,
			rgb,
			width, height
		);
		//std::cout << "  - ARGB > RGB: " << Util::timerEnd() << std::endl;

		vision->setDebugImage(rgb, width, height);
	} else {
		vision->setDebugImage(NULL, 0, 0);
	}

	visionResult = vision->process();

	if (debug) {
		DebugRenderer::renderBlobs(classification, blobber, classificationSwapRB);
		DebugRenderer::renderBalls(rgb, visionResult->balls, rgbSwapRB);
		DebugRenderer::renderGoals(rgb, visionResult->goals, rgbSwapRB);
		DebugRenderer::renderObstructions(rgb, visionResult->obstructionSide, rgbSwapRB);
		// TODO Show whether a ball is in the way
	}

	done = true;

	return NULL;
}
