#ifndef PROCESSTHREAD_H
#define PROCESSTHREAD_H

#include "Thread.h"

class Blobber;

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

	Blobber* blobber;

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