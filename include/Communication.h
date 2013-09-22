#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Thread.h"

#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <stack>

using boost::asio::ip::udp;

class Communication : public Thread {

public:
	class Listener {

	public:
		virtual void onCommunicationMessage(std::string message) = 0;

	};

	typedef std::stack<std::string> Messages;

    Communication(std::string host = "127.0.0.1", int port = 8042);
	~Communication();

	void send(std::string message);
	bool gotMessages();
	std::string popLastMessage();
	void close();

private:
	void* run();
	void onReceive(const boost::system::error_code& error, size_t bytesReceived);
	void onSend(const boost::system::error_code& error, size_t bytesSent);
	void receiveNext();

	std::string host;
	int port;
	char message[1024];
	char request[1024];
	boost::asio::io_service ioService;
	udp::socket* socket;
	udp::endpoint endpoint;
	boost::asio::ip::udp::endpoint remoteEndpoint;
	Messages messages;
	bool running;
	mutable boost::mutex messagesMutex;
};

#endif // COMMUNICATION_H