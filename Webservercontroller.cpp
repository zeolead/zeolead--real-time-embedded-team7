#include "Webservercontroller.hpp"
#include <iostream>

static std::function<void(const std::string&)> message_callback;
Webservercontroller::Webservercontroller() : running_(false) {
    // Constructor implementation
}
Webservercontroller::~Webservercontroller() {
    // Destructor implementation
    stopServer();
}
void Webservercontroller::startServer(StoreCallback storeCb) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
        std::cout << "Server is already running\n";
        return;
    }
    running_ = true;
    control_thread_ = std::thread([this, storeCb](){
        this-> control(storeCb);
        });
    }
void Webservercontroller::startServer() {
    startServer(message_callback);
}
void Webservercontroller::setMessageCallback(StoreCallback cb) {
    message_callback = std::move(cb);
}
void Webservercontroller::stopServer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        std::cout << "Server is not running\n";
        return;
    }
    running_ = false;
    ws_server.stop_listening();
    ws_server.stop();
    if (control_thread_.joinable()) {
        control_thread_.join();
    }
    std::cout << "Server stopped\n";
}
void Webservercontroller::RunServer(StoreCallback storeCb) {
    control(storeCb);
}

void Webservercontroller::control(StoreCallback storeCb) {
    ws_server.init_asio();
    std::cout << "initializing\n";
    ws_server.listen(8765);
    ws_server.start_accept();
    ws_server.set_message_handler(
        [this, storeCb](websocketpp::connection_hdl hdl,
                        server_t::message_ptr msg) {
            on_message(hdl, msg);
            storeCb(msg->get_payload());
        }
    );
    std::cout << "Server started\n";
    ws_server.run();
}

void Webservercontroller::control() {
    control([](const std::string&){});
}

void Webservercontroller::on_message(websocketpp::connection_hdl hdl,
                                     server_t::message_ptr msg) {
    std::cout << "Received: " << msg->get_payload() << "\n";
    ws_server.send(hdl,
                   "Ack: " + msg->get_payload(),
                   websocketpp::frame::opcode::text);
}
