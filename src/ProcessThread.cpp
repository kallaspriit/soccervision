#include "ProcessThread.h"
#include "Blobber.h"
#include "Vision.h"
#include "ImageProcessor.h"
#include "DebugRenderer.h"
#include "ImageBuffer.h"
#include "Config.h"

#include <iostream>

ProcessThread::ProcessThread(Dir dir, int width, int height) : Thread(), dir(dir), blobber(NULL), vision(NULL), width(width), height(height), classify(false), convertRGB(false), renderBlobs(false), done(true) {
	frame = NULL;
	dataY = new unsigned char[width * height];
    dataU = new unsigned char[(width / 2) * (height / 2)];
    dataV = new unsigned char[(width / 2) * (height / 2)];
	dataYUYV = new unsigned char[width * height * 3];
	classification = new unsigned char[width * height * 3];
	argb = new unsigned char[width * height * 4];
	rgb = new unsigned char[width * height * 3];

	blobber = new Blobber();
	blobber->initialize(width, height);
	blobber->loadOptions(Config::blobberConfigFilename);

	vision = new Vision(blobber, dir, width, height);
}

ProcessThread::~ProcessThread() {
	blobber->saveOptions(Config::blobberConfigFilename);

	delete vision;
	delete blobber;
}

void* ProcessThread::run() {
	if (frame == NULL) {
		return NULL;
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

	if (classify) {
		//Util::timerStart();
		blobber->classify((Blobber::Rgb*)classification, (Blobber::Pixel*)dataYUYV);
		//std::cout << "  - Blobber classify: " << Util::timerEnd() << std::endl;

		if (renderBlobs) {
			DebugRenderer::renderBlobs(classification, blobber);
		}
	}

	if (convertRGB) {
		//Util::timerStart();
		ImageProcessor::YUYVToARGB(dataYUYV, argb, width, height);
		//std::cout << "  - YUYV > ARGB: " << Util::timerEnd() << std::endl;

		//Util::timerStart();
		ImageProcessor::ARGBToRGB24(
			argb,
			rgb,
			width, height
		);

		vision->setDebugImage(rgb, width, height);
	}

	vision->process();

	//std::cout << "  - ARGB > RGB: " << Util::timerEnd() << std::endl;

	done = true;

	return NULL;
}
