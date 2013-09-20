#ifndef SERVER_H
#define SERVER_H

#include "Thread.h"
#include "WebSocketServer.h"

#include <boost/thread/mutex.hpp>
#include <string>
#include <stack>
#include <map>

class Server : public Thread, WebSocketServer::Listener {

public:
	struct Client {
		Client(int id, websocketpp::connection_hdl connection) : id(id), connection(connection) {}

		int id;
		websocketpp::connection_hdl connection;
	};

	typedef std::stack<std::string> Messages;
	typedef std::map<int, Client*> Clients;
	typedef Clients::iterator ClientsIt;

	Server();
	~Server();

	void setPort(int port);
	void broadcast(std::string message);
	std::string popLastMesage();
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
	mutable boost::mutex messagesMutex;
};

#endif