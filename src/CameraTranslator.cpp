#include "CameraTranslator.h"
#include "Maths.h"

#include <iostream>

CameraTranslator::WorldPosition CameraTranslator::getWorldPosition(int cameraX, int cameraY) {
	//CameraTranslator::CameraPosition undistorted = CameraTranslator::undistort(cameraX, cameraY);
	CameraTranslator::CameraPosition undistorted = CameraTranslator::CameraPosition(cameraX, cameraY);

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
	float A, float B, float C,
	float k1, float k2, float k3,
	float horizon, int cameraWidth, int cameraHeight
) {
	this->A = A;
	this->B = B;
	this->C = C;
	this->k1 = k1;
	this->k2 = k2;
	this->k3 = k3;
	this->horizon = horizon;
	this->cameraWidth = cameraWidth;
	this->cameraHeight = cameraHeight;
}

CameraTranslator::CameraPosition CameraTranslator::undistort(int x, int y) {
	/*Math::Matrix3x3 cameraMatrix(
		1203.5723440938634f, 0.0f, 639.5f,
		0.0f, 1203.5723440938634f, 511.5f,
		0.0f, 0.0f, 1.0f
	);*/

	//float dx = (float)x - (float)this->cameraWidth / 2.0f;
	//float dy = (float)y - (float)this->cameraHeight / 2.0f;
	float normalizedX = (2.0f * (float)x - (float)cameraWidth) / cameraWidth;
	float normalizedY = (2.0f * (float)y - (float)cameraHeight) / cameraHeight;

	float r = sqrt(pow(normalizedX, 2) + pow(normalizedY, 2));
	float multipler = 1 + 
		k1 * pow(r, 2) +
		k2 * pow(r, 4) + 
		k3 * pow(r, 6);

	float undistortedNormalizedX = x * multipler;
	float undistortedNormalizedY = y * multipler;

	//std::cout << "@ UNDISTORT " << x << "x" << y << " - dx: " << dx << ", dy: " << dy << ", r: " << r << ", multiplier: " << multipler << std::endl;
	std::cout << "@ UNDISTORT " << x << "x" << y << " - normalizedX: " << normalizedX << ", normalizedY: " << normalizedY << ", r: " << r << ", multiplier: " << multipler << std::endl;

	return CameraPosition(
		(int)Math::round((undistortedNormalizedX + 1) * cameraWidth / 2.0f, 0),
		(int)Math::round((undistortedNormalizedY + 1) * cameraHeight / 2.0f, 0)
	);
}

CameraTranslator::CameraPosition CameraTranslator::distort(int x, int y) {
	float dx = (float)x - (float)this->cameraWidth / 2.0f;
	float dy = (float)y - (float)this->cameraHeight / 2.0f;

	float r = sqrt(pow(dx, 2) + pow(dy, 2));
	float multipler = 1 + 
		this->k1 * pow(r, 2) +
		this->k2 * pow(r, 4) + 
		this->k3 * pow(r, 6);

	return CameraPosition(
		(int)Math::round(x / multipler, 0),
		(int)Math::round(y / multipler, 0)
	);
}
