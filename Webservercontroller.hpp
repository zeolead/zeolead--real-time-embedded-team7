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
    Webservercontroller();
    ~Webservercontroller();
    void startServer(StoreCallback storeCb);
    void startServer();
    void stopServer(); 
    void control(StoreCallback storeCb); 
    void control();
    //{
            // initialize the server
           // ws_server.init_asio();
           // ws_server.listen(8765);
           // ws_server.start_accept();
           // ws_server.set_message_handler([this, storeCb](websocketpp::connection_hdl hdl, server_t::message_ptr msg) {
           //     on_message(hdl, msg);
           //     storeCb(msg->get_payload());
           // });
           // std::cout << "Server started on port 8765" << std::endl;
           // ws_server.run();
    //    }
        
    void setMessageCallback(StoreCallback cb);
    
    private:

        void RunServer(StoreCallback storeCb);
        server_t ws_server; 
        void on_message(websocketpp::connection_hdl hdl, server_t::message_ptr msg); 
       // void on_message(websocketpp::connection_hdl hdl, server_t::message_ptr msg) 
           // std::cout << "Received message: " << msg->get_payload() << std::endl;
            //std::string command = msg->get_payload();
           // std::cout << "Executing command: " << command << std::endl;
        std::thread control_thread_;    
        std::mutex mutex_;    
        bool running_;
        };                                                                                                                                                                                                   
