#include "Server.h"

#include <iostream>

Server::Server() : ws(NULL) {
	ws = new WebSocketServer();
	port = 9002;
}

Server::~Server() {
	if (ws != NULL) delete ws; ws = NULL;
}

void Server::setPort(int port) {
	this->port = port;
}

void* Server::run() {
	std::cout << "! Starting web socket server on port " << port << std::endl;

	ws->listen(port);

	return NULL;
}

void Server::onSocketOpen(websocketpp::connection_hdl connection) {
	std::cout << "CLIENT CONNECTED" << std::endl;
}

void Server::onSocketClose(websocketpp::connection_hdl connection) {
	std::cout << "CLIENT DISCONNECTED" << std::endl;
}

void Server::onSocketMessage(std::string message, websocketpp::connection_hdl connection, websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
	std::cout << "CLIENT MESSAGE: " << message << std::endl;
}