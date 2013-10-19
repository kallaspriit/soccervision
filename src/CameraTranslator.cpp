#include "CameraTranslator.h"

CameraTranslator::WorldPosition CameraTranslator::getWorldPosition(int cameraX, int cameraY) {
	CameraTranslator::CameraPosition undistorted = CameraTranslator::undistort(cameraX, cameraY);

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

	return CameraTranslator::distort((int)Math::round(cameraX, 0), (int)Math::round(cameraY, 0));
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
	int undistortedX = xMap[distortedY][distortedX];
	int undistortedY = yMap[distortedY][distortedX];

	return CameraPosition(
		int(undistortedX),
		int(undistortedY)
	);
}
CameraTranslator::CameraPosition CameraTranslator::distort(int undistortedX, int undistortedY) {
	// Conversion for distorting  (normalization?)
	float x = (float(undistortedX) - cameraWidth / 2.0 ) / distortionFocus;
	float y = (float(undistortedY) - cameraHeight / 2.0) / distortionFocus;
	// distort
	float r2 = x*x + y*y; // distance squared
	float multipler = 1 + 
		k1 *  r2 +
		k2 *  r2 * r2 + 
		k3 *  r2 * r2 * r2;
	x *= multipler;
	y *= multipler;
	//Convert back
	float distortedX = x * distortionFocus + cameraWidth / 2.0;
	float distortedY = y * distortionFocus + cameraHeight / 2.0;

	return CameraPosition(
		int(distortedX),
		int(distortedY)
	);
}

bool CameraTranslator::loadMapping(char* xFileName, char* yFileName){
	std::ifstream fileStream;

	// Load xMap
	fileStream.open(xFileName);
	if(fileStream.is_open()){
		fileStream >> xMap;
		if (!fileStream.eof()) {
			std::cout << "failed to load xMap" << std::endl;
			return false;
		}
	}
	else{
		std::cout << "failed to open xMap file" << std::endl;
	}

	// Load yMap
	fileStream.close();
	fileStream.open(yFileName);
	if(fileStream.is_open()){
		fileStream >> yMap;
		if (!fileStream.eof()) {
			std::cout << "failed to load yMap" << std::endl;
			return false;
		}
	}
	else{
		std::cout << "failed to open yMap file" << std::endl;
	}
	fileStream.close();

	return true;
}

void CameraTranslator::printMapping(){
	printf("0:0 = %i\n",xMap[0][0]);
	std::cout << "X-Map:" << std::endl;
	printMap(xMap);
	std::cout << "Y-Map:" << std::endl;
	printMap(yMap);
}
void CameraTranslator::printMap(cameraMap& map){
	for (cameraMap::const_iterator row = map.begin(); row != map.end(); ++row) {
		for (cameraMapRow::const_iterator col = row->begin(); col != row->end(); ++col) {
			printf("%4i,",*col);
		}
		std::cout << std::endl;
	}
}

std::istream& operator >> ( std::istream& inputStream, CameraTranslator::cameraMap& map) {
	map.clear();
	CameraTranslator::cameraMapRow mapRow;

	//Get a line from input stream..
	std::string lineString;
	while(getline(inputStream, lineString)) {
		mapRow.clear();
		std::stringstream lineStream(lineString);
		//Get a field from a line stream
		std::string fieldString;
		while (getline(lineStream, fieldString, ',')) {
			std::stringstream fieldStream(fieldString);
			//extract the field as an integer
			int field;
			fieldStream >> field;
			mapRow.push_back(field);
		}
		map.push_back(mapRow);
	}

	return inputStream;  
}

