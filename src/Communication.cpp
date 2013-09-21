#include "Communication.h"
#include "Util.h"

#include <boost/bind.hpp>

Communication::Communication(std::string host, int port) : host(host), port(port), running(false), socket(NULL) {

}

Communication::~Communication() {
	std::cout << "! Closing communication link.. ";

	close();

	std::cout << "done!" << std::endl;

	if (socket != NULL) delete socket; socket = NULL;
}

void Communication::send(std::string message) {
	if (!running) {
		std::cout << "- Unable to send communication message '" << message << "', connection not open" << std::endl;

		return;
	}

	//socket->send_to(boost::asio::buffer(message), *iterator);

	std::cout << "@ SENDING: " << message << std::endl;

	/*udp::resolver resolver(ioService);
	udp::resolver::query query(udp::v4(), host, Util::toString(port));
	iterator = resolver.resolve(query);*/

	boost::asio::ip::udp::endpoint remoteEndpoint = boost::asio::ip::udp::endpoint(
		boost::asio::ip::address::from_string(host),
		port
	);

	message += "\n";

	try {
		socket->async_send_to(
			//boost::asio::buffer(message + "\n"), *iterator,
			boost::asio::buffer(message.data(), message.length()), remoteEndpoint,
			boost::bind(
				&Communication::onSend,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);
	} catch (std::exception& e) {
		std::cout << "- Communication send error: " << e.what() << std::endl;
	}
}

bool Communication::gotMessages() {
	boost::mutex::scoped_lock lock(messagesMutex);

	return messages.size() > 0;
}

std::string Communication::popLastMessage() {
	boost::mutex::scoped_lock lock(messagesMutex);
	
	if (messages.size() == 0) {
		return "";
	}

	std::string message = messages.top();

	messages.pop();

	return message;
}

void Communication::start() {
	std::cout << "! Starting communication socket connection to " << host << ":" << port << std::endl;

	running = true;

	socket = new udp::socket(ioService, udp::endpoint(udp::v4(), port));

	receiveNext();

	ioService.run();
}

void Communication::receiveNext() {
	std::cout << "@ RECEIVING NEXT" << std::endl;

	send("gs");

	try {
		socket->async_receive_from(
			boost::asio::buffer(message, 1024), endpoint,
			boost::bind(
				&Communication::onReceive,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);
	} catch (std::exception& e) {
		std::cout << "- Communication receive error: " << e.what() << std::endl;
	}
}

void Communication::onReceive(const boost::system::error_code& error, size_t bytesReceived) {
	std::cout << "@ onReceive: " << bytesReceived << ", " << error << std::endl;

	if (!error && bytesReceived > 0) {
		std::cout << "@ RECEIVED: ";
		std::cout.write(message, bytesReceived);
		std::cout << "\n";

		boost::mutex::scoped_lock lock(messagesMutex);
		messages.push(std::string(message, bytesReceived));
	}

	if (running) {
		receiveNext();
	}
}

void Communication::onSend(const boost::system::error_code& error, size_t bytesSent) {
	std::cout << "@ onSend: " << bytesSent << ", " << error << std::endl;

	if (running) {
		receiveNext();
	}
}

void Communication::close() {
	running = false;

	if (socket != NULL) {
		try {
			socket->close();
		} catch (std::exception& e) {
			std::cout << "- Communication close error: " << e.what() << std::endl;
		}
	}
}
