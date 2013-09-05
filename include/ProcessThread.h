#ifndef PROCESSTHREAD_H
#define PROCESSTHREAD_H

#include "Thread.h"
#include "Config.h"
#include "Vision.h"

class Blobber;
class VisionResults;

class ProcessThread : public Thread {

public:
	ProcessThread(Blobber* blobber, Vision* vision);
	~ProcessThread();

	void setFrame(unsigned char* data) { frame = data; };
	bool isDone() { return done; };

	int width;
	int height;
	Dir dir;

	bool classify;
	bool convertRGB;
	bool renderBlobs;

	Blobber* blobber;
	Vision* vision;
	Vision::VisionResult* visionResult;

	unsigned char* frame;
	unsigned char* dataYUYV;
	unsigned char* classification;
	unsigned char* rgb;

private:
	void* run();

	bool done;
	unsigned char* dataY;
    unsigned char* dataU;
    unsigned char* dataV;
	unsigned char* argb;
};

#endif // PROCESSTHREAD_H