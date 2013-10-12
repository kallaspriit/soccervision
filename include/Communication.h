#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Thread.h"

#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <vector>
#include <stack>
#include <queue>

using boost::asio::ip::udp;

class Communication : public Thread {

public:
	class Listener {

	public:
		virtual void handleCommunicationMessage(std::string message) = 0;

	};

	typedef std::stack<std::string> Messages;
	enum { MAX_SIZE = 10240 };

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
	char receiveBuffer[MAX_SIZE];
	//boost::array<char, 1024> receiveBuffer;
	char requestBuffer[MAX_SIZE];
	boost::asio::io_service ioService;
	udp::socket* socket;
	udp::endpoint endpoint;
	boost::asio::ip::udp::endpoint remoteEndpoint;
	Messages messages;
	std::queue<std::string> queuedMessages;
	bool running;
	mutable boost::mutex messagesMutex;
};

#endif // COMMUNICATION_H