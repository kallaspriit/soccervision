#ifndef ABSTRACT_COMMUNICATION_H
#define ABSTRACT_COMMUNICATION_H

#include "Thread.h"

#include <string>
#include <queue>

class AbstractCommunication : public Thread {

public:
	class Listener {

	public:
		virtual void handleCommunicationMessage(std::string message) = 0;
	};

	typedef std::queue<std::string> Messages;
	enum { MAX_SIZE = 4098 };

	virtual void send(std::string message) = 0;
	virtual bool gotMessages() = 0;
	virtual std::string dequeueMessage() = 0;
	//virtual int start() { return 0; };
	virtual void close() = 0;
	virtual void sync() {};

};

#endif // ABSTRACT_COMMUNICATION_H