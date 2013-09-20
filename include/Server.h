#ifndef SERVER_H
#define SERVER_H

#include "Thread.h"
#include "WebSocketServer.h"

#include <string>
#include <vector>
#include <map>

class Server : public Thread, WebSocketServer::Listener {

public:
	struct Client {
		Client(int id, websocketpp::connection_hdl connection) : id(id), connection(connection) {}

		int id;
		websocketpp::connection_hdl connection;
	};

	typedef std::vector<std::string> Messages;
	typedef Messages::iterator MessagesIt;
	typedef std::map<int, Client*> Clients;
	typedef Clients::iterator ClientsIt;

	Server();
	~Server();

	void setPort(int port);
	Client* getClientByConnection(websocketpp::connection_hdl connection);

private:
	void* run();
	void onSocketOpen(websocketpp::connection_hdl connection);
	void onSocketClose(websocketpp::connection_hdl connection);
	void onSocketMessage(std::string message, websocketpp::connection_hdl connection, websocketpp::server<websocketpp::config::asio>::message_ptr msg);

	WebSocketServer* ws;
	int port;
	int clientCounter;
	Clients clients;
	Messages messages;
};

#endif