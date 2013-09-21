#include "Communication.h"

#include <boost/asio.hpp>

using boost::asio::ip::udp;

Communication::Communication(std::string host, std::string port) : host(host), port(port) {

}

Communication::~Communication() {

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
	
	udp::socket socket(ioService, udp::endpoint(udp::v4(), 0));

	udp::resolver resolver(ioService);
	udp::resolver::query query(udp::v4(), host, port);
	udp::resolver::iterator iterator = resolver.resolve(query);

	size_t messageLength;
	udp::endpoint endpoint;
	char message[1024];

	while (true) {
		/*std::cout << "> ";
		std::cin.getline(request, 1024);
		size_t requestLength = strlen(request);
		socket.send_to(boost::asio::buffer(request, requestLength), *iterator);*/

		messageLength = socket.receive_from(boost::asio::buffer(message, 1024), endpoint);
		std::cout << "< ";
		std::cout.write(message, messageLength);
		std::cout << "\n";

		boost::mutex::scoped_lock lock(messagesMutex);
		messages.push(message);
	}

	return NULL;
}