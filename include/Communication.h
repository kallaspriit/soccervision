#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Thread.h"

#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>
#include <string>
#include <stack>

using boost::asio::ip::udp;

class Communication : public Thread {

public:
	typedef std::stack<std::string> Messages;

    Communication(std::string host = "127.0.0.1", int port = 8042);
	~Communication();

	void send(std::string message);
	bool gotMessages();
	std::string popLastMessage();
	void close();

private:
	void* run();

	std::string host;
	int port;
	char request[1024];
	udp::socket* socket;
	udp::resolver::iterator iterator;
	Messages messages;
	bool running;
	mutable boost::mutex messagesMutex;
};

#endif // COMMUNICATION_H