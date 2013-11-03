#include "Communication.h"
#include "Util.h"

#include <boost/bind.hpp>

Communication::Communication(std::string host, int port) : host(host), port(port), running(false), socket(NULL), receiveBuffer2(boost::asio::buffer(receiveBuffer, MAX_SIZE)) {

}

Communication::~Communication() {
	std::cout << "! Closing communication link.. ";

	close();
	join();

	std::cout << "done!" << std::endl;

	if (socket != NULL) delete socket; socket = NULL;
}

void Communication::send(std::string message) {
	if (message.size() >= MAX_SIZE) {
		std::cout << "- Too big socket message" << std::endl;

		return;
	}

	if (!running) {
		queuedMessages.push(message);

		return;
	}

	while (!queuedMessages.empty()) {
		std::cout << "! Socket queue size: " << queuedMessages.size() << std::endl;

		std::string queuedMessage = queuedMessages.front();
		queuedMessages.pop();

		send(queuedMessage);
	}

	if (message.substr(0, 6) != "speeds" && message.substr(0, 6) != "charge") {
		std::cout << "> " << message << std::endl;
	}

	message += "\n";

	memcpy(requestBuffer, message.c_str(), message.size());
	requestBuffer[message.size()] = 0;

	//boost::shared_ptr<std::string> requestBuffer(new std::string(message));

	try {
		/*boost::asio::ip::udp::endpoint remoteEndpoint = boost::asio::ip::udp::endpoint(
			boost::asio::ip::address::from_string(host),
			port
		);*/

		/*socket->async_send_to(
			boost::asio::buffer(requestBuffer, message.length()),
			remoteEndpoint,
			//boost::asio::buffer(*requestBuffer), remoteEndpoint,
			boost::bind(
				&Communication::onSend,
				this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred
			)
		);*/

		socket->send_to(
			boost::asio::buffer(requestBuffer, message.length()),
			remoteEndpoint
		);
	} catch (std::exception& e) {
		std::cout << "- Communication send error: " << e.what() << std::endl;
	}
}

bool Communication::gotMessages() {
	boost::mutex::scoped_lock lock(messagesMutex);

	return messages.size() > 0;
}

std::string Communication::dequeueMessage() {
	boost::mutex::scoped_lock lock(messagesMutex);
	
	if (messages.size() == 0) {
		return "";
	}

	std::string message = messages.front();

	messages.pop();

	return message;
}

void* Communication::run() {
	std::cout << "! Starting communication socket connection to " << host << ":" << port << std::endl;

	running = true;

	socket = new udp::socket(ioService, udp::endpoint(udp::v4(), port));

	remoteEndpoint = boost::asio::ip::udp::endpoint(
		boost::asio::ip::address::from_string(host),
		port
	);

	//receiveBuffer2 = boost::asio::buffer(receiveBuffer, MAX_SIZE);

	receiveNext();

	ioService.run();

	return NULL;
}

void Communication::receiveNext() {
	try {
		socket->async_receive_from(
			//boost::asio::buffer(message, MAX_SIZE), endpoint,
			//boost::asio::buffer(receiveBuffer, MAX_SIZE),
			receiveBuffer2,
			endpoint,
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
	if (bytesReceived >= MAX_SIZE) {
		std::cout << "- Too large socket message received: " << bytesReceived << std::endl;
	}

	if ((!error || error == boost::asio::error::message_size) && bytesReceived > 0) {
		std::string msg = std::string(receiveBuffer, bytesReceived);
		//std::string msg = std::string(receiveBuffer.data(), bytesReceived);

		if (msg.substr(0, 7) != "<speeds") {
			std::cout << "< " << msg << ", bytesReceived: " << bytesReceived << std::endl;
		}

		boost::mutex::scoped_lock lock(messagesMutex);
		messages.push(msg);
	} else if (error.value() != 995) {
		std::cout << "- Socket receive error: " << error << ", bytesReceived: " << bytesReceived << std::endl;
	}

	if (running) {
		receiveNext();
	}
}

void Communication::onSend(const boost::system::error_code& error, size_t bytesSent) {
	if (error) {
		std::cout << "- Socket send error: " << error << ", bytesSent: " << bytesSent << std::endl;
	}

	if (running) {
		receiveNext();
	}
}

void Communication::close() {
	running = false;

	if (socket != NULL) {
		try {
			boost::system::error_code ec;
			socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
			socket->close();
		} catch (std::exception& e) {
			std::cout << "- Communication close error: " << e.what() << std::endl;
		}
	}
}
