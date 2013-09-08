#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <libyuv.h>
#include <string>

class ImageProcessor {

public:
	struct YUYV {
		YUYV() : y1(0), u(0), y2(0), v(0) {}

        int y1, u, y2, v;
    };

	struct YUYVRange {
		int minY, maxY;
		int minU, maxU;
		int minV, maxV;
	};

	static void bayerRGGBToI420(unsigned char* input, unsigned char* outputY, unsigned char* outputU, unsigned char* outputV, int width, int height);
	static void I420ToYUYV(unsigned char* inputY, unsigned char* inputU, unsigned char* inputV, unsigned char* output, int width, int height);
	static void YUYVToARGB(unsigned char* input, unsigned char* output, int width, int height);
	static void ARGBToRGB24(unsigned char* input, unsigned char* output, int width, int height);
	static YUYV* getPixelAt(unsigned char* image, int width, int height, int x, int y);
	static YUYVRange extractColorRange(unsigned char* image, int width, int height, std::string colorName, int centerX, int centerY, int brushRadius, float stdDev);
};

#endif // IMAGEPROCESSOR_H