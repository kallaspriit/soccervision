#ifndef WEBSOCKETSERVER_H
#define WEBSOCKETSERVER_H

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <iostream>
#include <string>
#include <vector>

class WebSocketServer {

public:
	class Listener {

	public:
		virtual void onSocketOpen(websocketpp::connection_hdl connection) {}
		virtual void onSocketClose(websocketpp::connection_hdl connection) {}
		virtual void onSocketMessage(std::string message, websocketpp::connection_hdl connection, websocketpp::server<websocketpp::config::asio>::message_ptr msg) {}
	
	};

	typedef std::vector<WebSocketServer::Listener*> Listeners;
	typedef Listeners::const_iterator ListenersIt;
	typedef std::set<websocketpp::connection_hdl> Connections;
	typedef Connections::iterator ConnectionsIt;

	WebSocketServer();
	~WebSocketServer();

	void listen(int port = 9002);
	void stop();
	void addListener(Listener* listener);
	void broadcast(std::string message);
	void send(websocketpp::connection_hdl connection, std::string message);
	
private:
	void onOpen(websocketpp::connection_hdl connection);
	void onClose(websocketpp::connection_hdl connection);
	void onMessage(websocketpp::connection_hdl connection, websocketpp::server<websocketpp::config::asio>::message_ptr msg);

	websocketpp::server<websocketpp::config::asio>* server;
	Listeners listeners;
	Connections connections;

};

#endif