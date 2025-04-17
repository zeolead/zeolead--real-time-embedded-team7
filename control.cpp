#include<websocketpp/config/asio_no_tls.hpp>
#include<websocketpp/server.hpp>
#include<iostream>
#include<thread>
#include<functional>

typedef websocketpp::server<websocketpp::config::asio> server_t;1

class Webservercontroller {
    public:
        void control（） {
            ws_server.set_message_handler(bind(&Webservercontroller::on_message, this, std::placeholders::_1, std::placeholders::_2));
        }
        ws_server.init_asio();
        ws_server.listen(8765);
        ws_server.start_accept();
        std::cout << "Server started on port 8765" << std::endl;
        ws_server.run();
    
    private:
        
        void on_message(websocketpp::connection_hdl hdl, server_t::message_ptr msg) 
            std::cout << "Received message: " << msg->get_payload() << std::endl;
            std::string command = msg->get_payload();
            std::cout << "Executing command: " << command << std::endl;
        }                                                                                                                                                                                                   

server_t ws_server;    
}
