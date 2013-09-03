#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <libyuv.h>

class ImageProcessor {

public:
	static void bayerRGGBToI420(unsigned char* input, unsigned char* outputY, unsigned char* outputU, unsigned char* outputV, int width, int height);
	static void I420ToYUYV(unsigned char* inputY, unsigned char* inputU, unsigned char* inputV, unsigned char* output, int width, int height);
	static void YUYVToARGB(unsigned char* input, unsigned char* output, int width, int height);
	static void ARGBToRGB24(unsigned char* input, unsigned char* output, int width, int height);
};

#endif // IMAGEPROCESSOR_H