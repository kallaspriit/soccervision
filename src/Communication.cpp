#include "Communication.h"

Communication::Communication(std::string host, std::string port) : host(host), port(port), running(false), socket(NULL) {

}

Communication::~Communication() {
	close();
	join();

	if (socket != NULL) delete socket; socket = NULL;
}

void Communication::send(std::string message) {
	socket->send_to(boost::asio::buffer(message), *iterator);
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
	udp::resolver::query query(udp::v4(), host, port);
	iterator = resolver.resolve(query);

	size_t messageLength;
	udp::endpoint endpoint;
	char message[1024];

	running = true;

	while (running) {
		messageLength = socket->receive_from(boost::asio::buffer(message, 1024), endpoint);
		std::cout << "< ";
		std::cout.write(message, messageLength);
		std::cout << "\n";

		boost::mutex::scoped_lock lock(messagesMutex);
		messages.push(message);
	}

	return NULL;
}