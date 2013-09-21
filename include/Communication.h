#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Thread.h"

#include <boost/thread/mutex.hpp>
#include <string>
#include <stack>

class Communication : public Thread {

public:
	typedef std::stack<std::string> Messages;

    Communication(std::string host = "127.0.0.1", std::string port = "8042");
	~Communication();
	bool gotMessages();
	std::string popLastMessage();

private:
	void* run();

	std::string host;
	std::string port;
	char request[1024];
	Messages messages;
	mutable boost::mutex messagesMutex;
};

#endif // COMMUNICATION_H