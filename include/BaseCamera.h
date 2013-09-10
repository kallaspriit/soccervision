#ifndef CAMERA_H
#define CAMERA_H

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
	virtual bool open(int serial = 0) = 0;
	virtual bool isOpened() = 0;
	virtual bool isAcquisitioning() = 0;
	virtual void startAcquisition() = 0;
    virtual void stopAcquisition() = 0;
	virtual void close() = 0;

};

#endif // CAMERA_H