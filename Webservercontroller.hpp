#pragma once

#include<websocketpp/config/asio_no_tls.hpp>
#include<websocketpp/server.hpp>
#include<iostream>
#include<thread>
#include<functional>
#include<string>
#include<mutex>
#include<atomic>

using StoreCallback = std::function<void(const std::string&)>;
using server_t = websocketpp::server<websocketpp::config::asio> ;  

class Webservercontroller {
  
    public:
    Webservercontroller();//constructor
    ~Webservercontroller();//destructor
    void startServer(StoreCallback storeCb);//start the server with a callback function
    void startServer();//start the server without a callback function
    void stopServer(); //stop the server
    void control(StoreCallback storeCb); //Read data from the client (blocks until data is received)
    void control();
    void setMessageCallback(StoreCallback cb);//register the callback function
    
    private:

        void RunServer(StoreCallback storeCb);//Call the function without creating a new thread
        server_t ws_server; 
        void on_message(websocketpp::connection_hdl hdl, server_t::message_ptr msg); //reply to the client
        std::thread control_thread_;    
        std::mutex mutex_;    
        bool running_;
        };                                                                                                                                                                                                   
