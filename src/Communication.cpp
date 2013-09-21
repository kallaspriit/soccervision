#include "Communication.h"
#include "Util.h"

#include <boost/bind.hpp>

Communication::Communication(std::string host, int port) : host(host), port(port), running(false), socket(NULL) {

}

Communication::~Communication() {
	std::cout << "! Closing communication link.. ";

	close();
	join();

	std::cout << "done!" << std::endl;

	if (socket != NULL) delete socket; socket = NULL;
}

void Communication::send(std::string message) {
	//socket->send_to(boost::asio::buffer(message), *iterator);

	socket->async_send_to(
          boost::asio::buffer(message), *iterator,
          boost::bind(
			&Communication::onSend,
			this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred
		)
	);
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

void* Communication::run() {
	std::cout << "! Starting communication socket connection to " << host << ":" << port << std::endl;

	boost::asio::io_service ioService;
	
	socket = new udp::socket(ioService, udp::endpoint(udp::v4(), 0));

	udp::resolver resolver(ioService);
	udp::resolver::query query(udp::v4(), host, Util::toString(port));
	iterator = resolver.resolve(query);

	//size_t messageLength;

	running = true;

	receiveNext();

	/*while (running) {
		try {
			// TODO Remove test
			socket->send_to(boost::asio::buffer("gs"), *iterator);

			messageLength = socket->receive_from(boost::asio::buffer(message, 1024), endpoint);
			std::cout << "< ";
			std::cout.write(message, messageLength);
			std::cout << "\n";

			boost::mutex::scoped_lock lock(messagesMutex);
			messages.push(std::string(message, messageLength));
		} catch (std::exception& e) {
			std::cout << "- Communication receive error: " << e.what() << std::endl;
		}
	}*/

	return NULL;
}

void Communication::receiveNext() {
	socket->async_receive_from(
        boost::asio::buffer(message, 1024), endpoint,
        boost::bind(
			&Communication::onReceive,
			this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred
		)
	);
}

void Communication::onReceive(const boost::system::error_code& error, size_t bytesReceived) {
	if (!error && bytesReceived > 0) {
		std::cout << "< RECEIVED: ";
		std::cout.write(message, bytesReceived);
		std::cout << "\n";

		boost::mutex::scoped_lock lock(messagesMutex);
		messages.push(std::string(message, bytesReceived));
	} else if (running) {
		receiveNext();
	}
}

void Communication::onSend(const boost::system::error_code& error, size_t bytesSent) {
	if (running) {
		receiveNext();
	}
}

void Communication::close() {
	if (socket != NULL) {
		try {
			socket->cancel();
			socket->close();
		} catch (std::exception& e) {
			std::cout << "- Communication close error: " << e.what() << std::endl;
		}
	}

	running = false;
}
