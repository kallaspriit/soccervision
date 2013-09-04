#ifndef PROCESSTHREAD_H
#define PROCESSTHREAD_H

#include "Thread.h"

class ProcessThread : public Thread {

public:
	ProcessThread(int time);

private:
	void* run();

	int time;

};

#endif // PROCESSTHREAD_H