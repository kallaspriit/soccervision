#include "CameraTranslator.h"
#include "Maths.h"

#include <iostream>

CameraTranslator::WorldPosition CameraTranslator::getWorldPosition(int cameraX, int cameraY) {
	CameraTranslator::CameraPosition undistorted = CameraTranslator::undistort(cameraX, cameraY);
	//CameraTranslator::CameraPosition undistorted = CameraTranslator::CameraPosition(cameraX, cameraY);

	float pixelVerticalCoord = undistorted.y - this->horizon;
	int pixelRight = undistorted.x - this->cameraWidth / 2;

	float worldY = this->A + this->B / pixelVerticalCoord;
	float worldX = C * (float)pixelRight / pixelVerticalCoord;

	float worldDistance = sqrt(pow(worldX, 2) + pow(worldY, 2));
	float worldAngle = atan2(worldX, worldY);

	return WorldPosition(worldX / 1000.0f, worldY / 1000.0f, worldDistance / 1000.0f, worldAngle);
}

CameraTranslator::CameraPosition CameraTranslator::getCameraPosition(float worldX, float worldY) {
	float pixelVerticalCoord = this->B / ((worldY * 1000.0f) - this->A);
	float pixelRight = (worldX * 1000.0f) * pixelVerticalCoord / this->C;

	float cameraY = pixelVerticalCoord + this->horizon;
	float cameraX = pixelRight + this->cameraWidth / 2;

	//return CameraTranslator::distort((int)Math::round(cameraX, 0), (int)Math::round(cameraY, 0));
	return CameraTranslator::CameraPosition((int)Math::round(cameraX, 0), (int)Math::round(cameraY, 0));
}

void CameraTranslator::setConstants(
	int cameraWidth, int cameraHeight,
	float A, float B, float C, float horizon, 
	float distortionFocus,
	float k1, float k2, float k3
) {
	this->cameraWidth = cameraWidth;
	this->cameraHeight = cameraHeight;
	this->A = A;
	this->B = B;
	this->C = C;
	this->horizon = horizon;
	this->distortionFocus = distortionFocus;
	this->k1 = k1;
	this->k2 = k2;
	this->k3 = k3;
}

CameraTranslator::CameraPosition CameraTranslator::undistort(int distortedX, int distortedY) {

	//Conversion for undistorting  (normalization?)
	float x = (float(distortedX) - cameraWidth / 2.0 ) / distortionFocus;
	float y = (float(distortedY) - cameraHeight / 2.0) / distortionFocus;
	//Undistort
	float r2 = x*x + y*y; // distance squared
	float multipler = 1 + 
		k1 * 1 * r2 +
		k2 * 2 * r2 + 
		k3 * 3 * r2;
	x *= multipler;
	y *= multipler;
	//Convert back
	float undistortedX = x * distortionFocus + cameraWidth / 2.0;
	float undistortedY = y * distortionFocus + cameraHeight / 2.0;

	return CameraPosition(
		int(undistortedX),
		int(undistortedY)
	);
}
