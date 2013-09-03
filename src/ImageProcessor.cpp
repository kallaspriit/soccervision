#include "ImageProcessor.h"

void ImageProcessor::bayerRGGBToI420(unsigned char* input, unsigned char* outputY, unsigned char* outputU, unsigned char* outputV, int width, int height) {
	int strideY = width;
	int strideU = (width + 1) / 2;
	int strideV = (width + 1) / 2;
	
	libyuv::BayerRGGBToI420(
		input,
		width,
		outputY,
		strideY,
		outputU,
		strideU,
		outputV,
		strideV,
		width,
		height
	);
}

void ImageProcessor::I420ToYUYV(unsigned char* inputY, unsigned char* inputU, unsigned char* inputV, unsigned char* output, int width, int height) {
	int strideY = width;
	int strideU = (width + 1) / 2;
	int strideV = (width + 1) / 2;
	int outputStride = width * 2;
	
	libyuv::I420ToYUY2(
		inputY, strideY,
		inputU, strideU,
		inputV, strideV,
		output,
		outputStride,
		width, height
	);
}

void ImageProcessor::YUYVToARGB(unsigned char* input, unsigned char* output, int width, int height) {
	int inputStride = width * 2;
	int outputStride = width * 4;

	libyuv::YUY2ToARGB(
		input, inputStride,
		output, outputStride,
		width, height
	);
}

void ImageProcessor::ARGBToRGB24(unsigned char* input, unsigned char* output, int width, int height) {
	int inputStride = width * 4;
	int outputStride = width * 3;

	libyuv::ARGBToRGB24(
		input, inputStride,
		output, outputStride,
		width, height
	);
}