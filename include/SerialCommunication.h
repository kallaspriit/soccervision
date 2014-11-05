#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include "AbstractCommunication.h"
#include "Serial.h"

class SerialCommunication : public AbstractCommunication {

public:

	SerialCommunication(std::string portName, int baud = 115200);
	~SerialCommunication();

	void send(std::string message);
	bool gotMessages();
	std::string dequeueMessage();
	void close();

private:
	void* run();
	void received(const char *data, unsigned int len);

	CallbackSerial serial;
	std::string portName;
	int baud;
	std::string partialMessage;
	Messages messages;
	Messages queuedMessages;
	char requestBuffer[MAX_SIZE];
	mutable boost::mutex messagesMutex;
	
};

#endif // SERIAL_COMMUNICATION_H