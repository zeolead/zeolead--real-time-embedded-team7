#include <iostream>
#include <cstring>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <unistd.h>

int main() {
    struct sockaddr_rc localAddr = {0}, remoteAddr = {0};
    char buf[1024] = {0};

    int server_sock, client_sock;
    socklen_t opt = sizeof(remoteAddr);

    // 创建RFCOMM socket
    server_sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (server_sock < 0) {
        perror("创建socket失败");
        return 1;
    }

    // 绑定蓝牙RFCOMM通道1
    localAddr.rc_family = AF_BLUETOOTH;
    localAddr.rc_bdaddr = *BDADDR_ANY;
    localAddr.rc_channel = 1;  // 通道1

    if (bind(server_sock, (struct sockaddr *)&localAddr, sizeof(localAddr)) < 0) {
        perror("绑定失败");
        return 1;
    }

    // 监听
    if (listen(server_sock, 1) < 0) {
        perror("监听失败");
        return 1;
    }

    std::cout << "等待Windows客户端连接..." << std::endl;

    // 等待客户端连接
    client_sock = accept(server_sock, (struct sockaddr *)&remoteAddr, &opt);
    if (client_sock < 0) {
        perror("接受连接失败");
        return 1;
    }

    char remoteAddrStr[18] = {0};
    ba2str(&remoteAddr.rc_bdaddr, remoteAddrStr);
    std::cout << "已连接客户端: " << remoteAddrStr << std::endl;

    // 通信循环
    while (true) {
        memset(buf, 0, sizeof(buf));
        int bytesRead = read(client_sock, buf, sizeof(buf));
        if (bytesRead <= 0) {
            std::cout << "客户端断开连接" << std::endl;
            break;
        }
        std::cout << "收到: " << buf << std::endl;

        // 发送回显
        std::string reply = "Raspberry Pi收到: " + std::string(buf);
        write(client_sock, reply.c_str(), reply.size());
    }

    // 关闭连接
    close(client_sock);
    close(server_sock);
    return 0;
}