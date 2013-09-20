#include "Server.h"

#include <iostream>

Server::Server() : ws(NULL) {
	ws = new WebSocketServer();
	port = 9002;
	clientCounter = 0;
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

	int id = clientCounter++;

	clients[id] = new Client(id, connection);
}

void Server::onSocketClose(websocketpp::connection_hdl connection) {
	std::cout << "CLIENT DISCONNECTED" << std::endl;

	int id;
	Client* client;

	for (ClientsIt it = clients.begin(); it != clients.end(); it++) {
		id = it->first;
		client = it->second;

		if (client->connection.lock() == connection.lock()) {
			std::cout << "CLIENT #" << id << " DISCONNECTED" << std::endl;

			clients.erase(it);

			break;
		}
	}
}

void Server::onSocketMessage(std::string message, websocketpp::connection_hdl connection, websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
	std::cout << "CLIENT MESSAGE: " << message << std::endl;
}