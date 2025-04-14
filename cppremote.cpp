#include<websocketpp/config/asio_no_tls.hpp>
#include<websocketpp/server.hpp>
#include<iostream>

typedef websocketpp::server<websocketpp::config::asio> server_t;1

void on_message(websocketpp::connection_hdl hdl, server_t::message_ptr msg) {
    std::cout << "Received message: " << msg->get_payload() << std::endl;
}

int main() {
    server_t server;

    server.set_message_handler(&on_message);

    server.init_asio();
    server.set_reuse_addr(true);

    server.listen(9002);
    server.start_accept();

    std::cout << "Server started on port 9002" << std::endl;

    server.run();
    return 0;
}