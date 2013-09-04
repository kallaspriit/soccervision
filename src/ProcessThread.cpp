#include "ProcessThread.h"

#include <iostream>
#include <Windows.h>

ProcessThread::ProcessThread(int time) : Thread(), time(time) {}

void* ProcessThread::run() {
	std::cout << "! ProcessThread working " << time << "ms" << std::endl;

	Sleep(time);

	std::cout << "! ProcessThread " << time << "ms done" << std::endl;

	return NULL;
}