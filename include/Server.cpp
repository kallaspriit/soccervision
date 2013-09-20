#include "Server.h"

#include <iostream>

Server::Server() : ws(NULL) {
	ws = new WebSocketServer();
	ws->addListener(this);

	port = 9002;
	clientCounter = 0;
}

Server::~Server() {
	ws->stop();

	Client* client;

	for (ClientsIt it = clients.begin(); it != clients.end(); it++) {
		client = it->second;

		delete client;
	}

	clients.clear();

	if (ws != NULL) delete ws; ws = NULL;
}

void Server::setPort(int port) {
	this->port = port;
}

void Server::broadcast(std::string message) {
	ws->broadcast(message);
}

Server::Client* Server::getClientByConnection(websocketpp::connection_hdl connection) {
	int id;
	Client* client;

	for (ClientsIt it = clients.begin(); it != clients.end(); it++) {
		id = it->first;
		client = it->second;

		if (client->connection.lock() == connection.lock()) {
			return client;
		}
	}

	return NULL;
}

void* Server::run() {
	std::cout << "! Starting web socket server on port " << port << std::endl;

	ws->listen(port);

	return NULL;
}

void Server::onSocketOpen(websocketpp::connection_hdl connection) {
	int id = clientCounter++;

	clients[id] = new Client(id, connection);

	std::cout << "! Server client # " << id << " connected" << std::endl;
}

void Server::onSocketClose(websocketpp::connection_hdl connection) {
	int id;
	Client* client;

	for (ClientsIt it = clients.begin(); it != clients.end(); it++) {
		id = it->first;
		client = it->second;

		if (client->connection.lock() == connection.lock()) {
			std::cout << "! Server client #" << id << " disconnected" << std::endl;

			clients.erase(it);

			return;
		}
	}

	std::cout << "- Unknown server client disconnected" << std::endl;
}

void Server::onSocketMessage(std::string message, websocketpp::connection_hdl connection, websocketpp::server<websocketpp::config::asio>::message_ptr msg) {
	Client* client = getClientByConnection(connection);

	if (client == NULL) {
		std::cout << "- Unknown client sent message: " << message << std::endl;

		return;
	}

	messages.push(message);
	
	std::cout << "! Server client #" << client->id << " sent message: " << message << std::endl;
}