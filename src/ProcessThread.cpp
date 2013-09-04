#include "ProcessThread.h"
#include "Blobber.h"
#include "ImageProcessor.h"
#include "ImageBuffer.h"

#include <iostream>

ProcessThread::ProcessThread(int width, int height) : Thread(), width(width), height(height), classify(false), convertRGB(false), renderBlobs(false), img(NULL), done(true) {
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
	blobber->loadOptions("config/blobber.cfg");
}

ProcessThread::~ProcessThread() {
	delete blobber;

	if (img != NULL) {
		delete img;
		img = NULL;
	}
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
			renderBlobsTo(classification);
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
	}
	//std::cout << "  - ARGB > RGB: " << Util::timerEnd() << std::endl;

	done = true;

	return NULL;
}

void ProcessThread::renderBlobsTo(unsigned char* image) {
	if (img == NULL) {
		img = new ImageBuffer();
	}

	img->data = image;
	img->width = width;
	img->height = height;

	img->drawText(20, 20, "Colors:");

	for (int i = 0; i < blobber->getColorCount(); i++) {
		Blobber::Color* color = blobber->getColor(i);

		if (color == NULL) {
			continue;
		}

		img->drawText(20, 40 + 20 * i, color->name);
	}
}
