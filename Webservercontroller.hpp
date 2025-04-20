#pragma once

#inlude<websocketpp/config/asio_no_tls.hpp>
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
    Webservercontroller();
    ~Webservercontroller();
    void startServer(StoreCallback storeCb);
    void startServer();
    void stopServer(); 
    void control(StoreCallback storeCb); 
    void control();
        
        
    
    private:

        void RunServer(StoreCallback storeCb);
        server_t ws_server; 
        void on_message(websocketpp::connection_hdl hdl, server_t::message_ptr msg) 
        std::thread control_thread_;    
        std::mutex mutex_;    
        bool running_;
        };                                                                                                                                                                                                   
