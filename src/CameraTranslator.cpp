#include "CameraTranslator.h"
#include "Maths.h"

#include <iostream>

CameraTranslator::WorldPosition CameraTranslator::getWorldPosition(int cameraX, int cameraY) {
	CameraPosition undistorted = undistort(cameraX, cameraY);

	float pixelVerticalCoord = undistorted.y - this->horizon;
	int pixelRight = undistorted.x - this->cameraWidth / 2;

	float worldY = this->B + this->A / pixelVerticalCoord;
	float worldX = C * (float)pixelRight / pixelVerticalCoord;

	float worldDistance = sqrt(pow(worldX, 2) + pow(worldY, 2));
	float worldAngle = atan2(worldX, worldY);

	return WorldPosition(worldX, worldY, worldDistance, worldAngle);
}

CameraTranslator::CameraPosition CameraTranslator::getCameraPosition(float worldX, float worldY) {
	float pixelVerticalCoord = this->A / (worldY - this->B);
	float pixelRight = worldX * pixelVerticalCoord / this->C;

	float cameraY = pixelVerticalCoord + this->horizon;
	float cameraX = pixelRight + this->cameraWidth / 2;

	return distort((int)Math::round(cameraX, 0), (int)Math::round(cameraY, 0));
}

void CameraTranslator::setConstants(
	float A, float B, float C,
	float k1, float k2, float k3,
	float horizon, float distortionFocus,
	int cameraWidth, int cameraHeight
) {
	this->A = A;
	this->B = B;
	this->C = C;
	this->k1 = k1;
	this->k2 = k2;
	this->k3 = k3;
	this->horizon = horizon;
	this->distortionFocus = distortionFocus;
	this->cameraWidth = cameraWidth;
	this->cameraHeight = cameraHeight;
}

CameraTranslator::CameraPosition CameraTranslator::getMappingPosition(int x, int y, CameraMap& mapX, CameraMap& mapY) {
	if (x < 0) x = 0;
	if (x > cameraWidth - 1) x = cameraWidth - 1;
	if (y < 0) y = 0;
	if (y > cameraHeight - 1) y = cameraHeight - 1;

	return CameraPosition(
		(int)mapX[y][x],
		(int)mapY[y][x]
	);
}

CameraTranslator::CameraPosition CameraTranslator::getAvgMappingPosition(int x, int y, CameraMap& mapX, CameraMap& mapY) {
	int brushSize = 10;
	int xSum = 0;
	int ySum = 0;
	int sampleCount = 0;

	for (int dx = -brushSize / 2; dx < brushSize / 2; dx++) {
		for (int dy = -brushSize / 2; dy < brushSize / 2; dy++) {
			CameraPosition sample = getMappingPosition(x + dx, y + dy, mapX, mapY);

			xSum += sample.x;
			ySum += sample.y;
			sampleCount++;
		}
	}

	return CameraPosition(
		xSum / sampleCount,
		ySum / sampleCount
	);
}

CameraTranslator::CameraPosition CameraTranslator::undistort(int distortedX, int distortedY) {
	//return getMappingPosition(distortedX, distortedY, undistortMapX, undistortMapY);
	return getAvgMappingPosition(distortedX, distortedY, undistortMapX, undistortMapY);
}

CameraTranslator::CameraPosition CameraTranslator::distort(int undistortedX, int undistortedY) {
	return getMappingPosition(undistortedX, undistortedY, distortMapX, distortMapY);
}

// equasion-based solution
/*CameraTranslator::CameraPosition CameraTranslator::distort(int undistortedX, int undistortedY) {
	// conversion for distorting  (normalization?)
	float x = ((float)undistortedX - (float)cameraWidth / 2.0f) / distortionFocus;
	float y = ((float)undistortedY - (float)cameraHeight / 2.0f) / distortionFocus;

	// distort
	float r2 = x * x + y * y; // distance squared
	float multipler = 1 + 
		k1 *  r2 +
		k2 *  r2 * r2 + 
		k3 *  r2 * r2 * r2;

	x *= multipler;
	y *= multipler;

	// convert back
	int distortedX = (int)(x * distortionFocus + cameraWidth / 2.0f);
	int distortedY = (int)(y * distortionFocus + cameraHeight / 2.0f);

	return CameraPosition(
		distortedX,
		distortedY
	);
}*/

bool CameraTranslator::loadUndistortionMapping(std::string xFilename, std::string yFilename){
	return loadMapping(xFilename, yFilename, undistortMapX, undistortMapY);
}

bool CameraTranslator::loadDistortionMapping(std::string xFilename, std::string yFilename){
	return loadMapping(xFilename, yFilename, distortMapX, distortMapY);
}

bool CameraTranslator::loadMapping(std::string xFilename, std::string yFilename, CameraMap& mapX, CameraMap& mapY){
	std::ifstream fileStream;

	fileStream.open(xFilename);

	if (fileStream.is_open()) {
		fileStream >> mapX;

		if (!fileStream.eof()) {
			std::cout << "- Failed to load x map from " << xFilename << std::endl;

			return false;
		}
	}
	else {
		std::cout << "- Failed to open map file " << xFilename << std::endl;
	}

	fileStream.close();
	fileStream.open(yFilename);

	if (fileStream.is_open()) {
		fileStream >> mapY;

		if (!fileStream.eof()) {
			std::cout << "- Failed to load y map from " << yFilename << std::endl;

			return false;
		}
	}
	else {
		std::cout << "- Failed to open y map file " << yFilename << std::endl;
	}

	fileStream.close();

	return true;
}

std::istream& operator >> (std::istream& inputStream, CameraTranslator::CameraMap& map) {
	map.clear();
	CameraTranslator::CameraMapRow mapRow;

	std::string lineString;
	CameraTranslator::CameraMapItem field;

	while (getline(inputStream, lineString)) {
		mapRow.clear();

		std::stringstream lineStream(lineString);
		std::string fieldString;

		while (getline(lineStream, fieldString, ',')) {
			std::stringstream fieldStream(fieldString);

			fieldStream >> field;

			mapRow.push_back(field);
		}

		map.push_back(mapRow);
	}

	return inputStream;  
}

std::string CameraTranslator::getJSON() {
	std::stringstream stream;

	stream << "{";

	stream << "\"A\": " << A << ",";
	stream << "\"B\": " << B << ",";
	stream << "\"C\": " << C << ",";
	stream << "\"k1\": " << k1 << ",";
	stream << "\"k2\": " << k2 << ",";
	stream << "\"k3\": " << k3 << ",";
	stream << "\"horizon\": " << horizon << ",";
	stream << "\"distortionFocus\": " << distortionFocus << "";

	stream << "}";

	return stream.str();
}