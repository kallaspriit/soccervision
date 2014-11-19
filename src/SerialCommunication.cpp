#include "SerialCommunication.h"
#include "enumser.h"

SerialCommunication::SerialCommunication(std::string portName, int baud) :
	portName(portName),
	baud(baud),
	serial(portName, baud)
{
	std::cout << "! Starting communication serial to " << portName << " @ " << baud << std::endl;

	serial.setCallback(boost::bind(&SerialCommunication::received, this, _1, _2));
}

SerialCommunication::~SerialCommunication() {
	std::cout << "! Closing communication link.. ";

	close();
	join();

	std::cout << "done!" << std::endl;
}

SerialCommunication::PortList SerialCommunication::getPortList() {
	PortList portList;

	CEnumerateSerial::UsingSetupAPI1(portList.numbers, portList.names);

	return portList;
}

void SerialCommunication::send(std::string message) {
	if (message.size() >= MAX_SIZE) {
		std::cout << "- Too big socket message" << std::endl;

		return;
	}

	if (!serial.isOpen()) {
		std::cout << "QUEUE " << message << std::endl;

		queuedMessages.push(message);

		return;
	} else if (serial.errorStatus()) {
		std::cout << "Error: serial port unexpectedly closed" << std::endl;

		return;
	}

	while (!queuedMessages.empty()) {
		std::cout << "! Socket queue size: " << queuedMessages.size() << std::endl;

		std::string queuedMessage = queuedMessages.front();
		queuedMessages.pop();

		send(queuedMessage);
	}

	/*if (message.substr(0, 6) != "speeds") {
		// incoming message
		std::cout << "SEND > " << message << std::endl;
	}*/

	message += "\n";

	memcpy(requestBuffer, message.c_str(), message.size());
	requestBuffer[message.size()] = 0;

	try {
		serial.write(requestBuffer, message.size());
	}
	catch (std::exception& e) {
		std::cout << "- SerialCommunication send error: " << e.what() << std::endl;
	}
}

bool SerialCommunication::gotMessages() {
	boost::mutex::scoped_lock lock(messagesMutex);

	return messages.size() > 0;
}

std::string SerialCommunication::dequeueMessage() {
	boost::mutex::scoped_lock lock(messagesMutex);

	if (messages.size() == 0) {
		return "";
	}

	std::string message = messages.front();

	messages.pop();

	if (message.substr(0, 7) != "<speeds" && message.substr(0, 4) != "<adc") {
		std::cout << "RECV < " << message << std::endl;
	}

	return message;
}

void SerialCommunication::close() {
	serial.close();
}

void* SerialCommunication::run() {
	

	return NULL;
}

void SerialCommunication::received(const char *data, unsigned int len) {
	std::vector<char> v(data, data + len);

	for (unsigned int i = 0; i<v.size(); i++) {
		if (v[i] == '\n') {
			boost::mutex::scoped_lock lock(messagesMutex);

			messages.push(partialMessage);

			if (partialMessage.substr(0, 7) != "<speeds" && partialMessage.substr(0, 4) != "<adc") {
				std::cout << "C < " << partialMessage << std::endl;
			}

			partialMessage = "";
		} else {

			

			// remove non-ascii char
			//if (v[i] >= 32 && v[i] < 0x7f) {
			partialMessage += v[i];
			//}
		}
	}

	std::cout << "P < " << partialMessage << std::endl;
}