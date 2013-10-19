#ifndef CAMERATRANSLATOR_H
#define CAMERATRANSLATOR_H

#include <math.h>
#include "Maths.h"

#include <string>
#include <vector>

#include <iostream> //cout
#include <fstream>
#include <sstream>

class CameraTranslator {
public:
	typedef std::vector <int> cameraMapRow;
	typedef std::vector <cameraMapRow> cameraMap;

	struct WorldPosition {
		WorldPosition() : dx(0.0f), dy(0.0f), distance(0.0f), angle(0.0f) {}
		WorldPosition(float dx, float dy, float distance, float angle) : dx(dx), dy(dy), distance(distance), angle(angle) {}

		float dx;
		float dy;
		float distance;
		float angle;
	};

	struct CameraPosition {
		CameraPosition() : x(0), y(0) {}
		CameraPosition(int x, int y) : x(x), y(y) {}

		int x;
		int y;
	};

	CameraTranslator() : A(0.0f), B(0.0f), C(0.0f), k1(0.0f), k2(0.0f), k3(0.0f), horizon(0.0f), cameraWidth(0), cameraHeight(0) {}

	void setConstants(
		int cameraWidth, int cameraHeight,
		float A, float B, float C, float horizon, 
		float distortionFocus,
		float k1, float k2, float k3);

	bool loadMapping(char* xFile, char* yFile);
	void CameraTranslator::printMapping();


	WorldPosition getWorldPosition(int cameraX, int cameraY);
	CameraPosition getCameraPosition(float dx, float dy);

	CameraTranslator::CameraPosition CameraTranslator::distort(int x, int y);
	CameraTranslator::CameraPosition CameraTranslator::undistort(int x, int y);
	

private:
	int cameraWidth;
	int cameraHeight;

	//World <--> Camera mapping constants
	float A;
	float B;
	float C;
	float horizon;

	//Distortion constants
	float distortionFocus;
	float k1;
	float k2;
	float k3;

	cameraMap xMap;
	cameraMap yMap;
	void CameraTranslator::printMap(cameraMap& map);

	//Overload for loading cameraMap from file
	friend std::istream& operator >> ( std::istream& inputStream, cameraMap& map);
};

#endif