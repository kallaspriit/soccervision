#ifndef PROCESSTHREAD_H
#define PROCESSTHREAD_H

#include "Thread.h"

class Blobber;
class ImageBuffer;

class ProcessThread : public Thread {

public:
	ProcessThread(int width, int height);
	~ProcessThread();

	void setFrame(unsigned char* data) { frame = data; };
	bool isDone() { return done; };

	int width;
	int height;

	bool classify;
	bool convertRGB;
	bool renderBlobs;

	Blobber* blobber;

	unsigned char* frame;
	unsigned char* dataYUYV;
	unsigned char* classification;
	unsigned char* rgb;

private:
	void* run();
	void renderBlobsTo(unsigned char* image);

	bool done;
	ImageBuffer* img;
	unsigned char* dataY;
    unsigned char* dataU;
    unsigned char* dataV;
	unsigned char* argb;
};

#endif // PROCESSTHREAD_H