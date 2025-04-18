#include "websocketcontroller.hpp"
#include <iostream>

void Webservercontroller::control(StoreCallback storeCb) {
  ws_server.init_asio();
  ws_server.listen(8765);
  ws_server.start_accept();
  ws_server.set_message_handler(
    [this, storeCb](auto hdl, auto msg) {
      this->on_message(hdl, msg);
      storeCb(msg->get_payload());
    }
  );
  std::cout << "Server started\n";
  ws_server.run();
}

void Webservercontroller::control() {
  control([](const std::string&){});
}

void Webservercontroller::on_message(server_t::connection_hdl hdl,server_t::message_ptr msg) {
  std::cout << "Received: " << msg->get_payload() << "\n";
  ws_server.send(hdl,"Ack: " + msg->get_payload(),websocketpp::frame::opcode::text);
}