#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>
#include <string>
#include <stack>

using boost::asio::ip::udp;

class Communication {

public:
	typedef std::stack<std::string> Messages;

    Communication(std::string host = "127.0.0.1", int port = 8042);
	~Communication();

	void send(std::string message);
	bool gotMessages();
	std::string popLastMessage();
	void start();
	void close();

private:
	void onReceive(const boost::system::error_code& error, size_t bytesReceived);
	void onSend(const boost::system::error_code& error, size_t bytesSent);
	void receiveNext();

	std::string host;
	int port;
	char message[1024];
	char request[1024];
	udp::socket* socket;
	udp::endpoint endpoint;
	udp::resolver::iterator iterator;
	Messages messages;
	bool running;
	mutable boost::mutex messagesMutex;
};

#endif // COMMUNICATION_H