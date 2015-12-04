#ifndef CAMERA_H
#define CAMERA_H

#include "FpsCounter.h"

class BaseCamera {

public:
	struct Frame {
        unsigned char* data;
        int size;
        int width;
        int height;
        int number;
		bool fresh;
        double timestamp;
    };

	virtual Frame* getFrame() = 0;
	virtual int getSerial() { return -1; }
	virtual bool open(int serial = 0) { return true; }
	virtual bool isOpened() { return true; }
	virtual bool isAcquisitioning() { return true; }
	virtual void startAcquisition() {}
	virtual void stopAcquisition() {}
	virtual void close() {}
	int getFps() { return fpsCounter.getFps(); }


protected:
	FpsCounter fpsCounter;

};

#endif // CAMERA_H