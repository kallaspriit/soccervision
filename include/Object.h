#ifndef OBJECT_H
#define OBJECT_H

#include "Config.h"

#include <vector>

class Object {

public:
	Object(int x = 0, int y = 0, int width = 0, int height = 0, int area = 0, float distance = 0.0f, float angle = 0.0f, int type = -1, bool behind = false);
	void copyFrom(const Object* other);
	bool intersects(Object* other, int margin = 0) const;
	bool contains(Object* other) const;
	void updateVisible(float x, float y, float distance, float angle, float dt, float drag = Config::rollingDrag);
    void updateInvisible(float dt, float drag = Config::rollingDrag);
	void applyDrag(float drag, float dt);
	void markForRemoval(float afterSeconds);
    bool shouldBeRemoved();
	Object* mergeWith(Object* other) const;

	static std::vector<Object*> mergeOverlapping(const std::vector<Object*>& set, int margin = 0, bool requireSameType = false);

	int id;
	static int instances;
    int x;
    int y;
    int width;
    int height;
    int area;
    float distance;
    float angle;
	float velocityX;
	float velocityY;
    int type;
	double lastSeenTime;
	double removeTime;
	bool visible;
	bool behind;
	bool processed;
};

typedef std::vector<Object*> ObjectList;
typedef ObjectList::iterator ObjectListIt;
typedef ObjectList::const_iterator ObjectListItc;

#endif // OBJECT_H
