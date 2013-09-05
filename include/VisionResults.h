#ifndef VISIONRESULTS_H
#define VISIONRESULTS_H

#include "Config.h"
#include "Object.h"

class VisionResults {

public:
	VisionResults();
	~VisionResults();

	void merge(VisionResults* other);

	Object* getClosestBall(bool frontOnly = false);
	Object* getLargestGoal(Side side, bool frontOnly = false);
	Object* getFurthestGoal(bool frontOnly = false);

	ObjectList balls;
	ObjectList goals;
	Obstruction obstructionSide;
	float blackDistance;

private:

};

#endif // VISIONRESULTS_H