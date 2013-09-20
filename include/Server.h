#ifndef SERVER_H
#define SERVER_H

#include "Thread.h"
#include "WebSocketServer.h"

#include <string>
#include <vector>

class Server : public Thread, WebSocketServer::Listener {

public:
	typedef std::vector<std::string> Messages;

	Server();
	~Server();

	void setPort(int port);

private:
	void* run();
	void onSocketOpen(websocketpp::connection_hdl connection);
	void onSocketClose(websocketpp::connection_hdl connection);
	void onSocketMessage(std::string message, websocketpp::connection_hdl connection, websocketpp::server<websocketpp::config::asio>::message_ptr msg);

	WebSocketServer* ws;
	int port;
	Messages messages;
};

#endif