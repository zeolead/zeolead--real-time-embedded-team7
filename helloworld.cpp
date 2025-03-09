#include <windows.h>
#include <iostream>

void sendData(HANDLE hSerial, const std::string& data) {
    DWORD bytesWritten;
    WriteFile(hSerial, data.c_str(), data.size(), &bytesWritten, NULL);
}

std::string readData(HANDLE hSerial) {
    char buffer[1024] = {0};
    DWORD bytesRead;
    ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL);
    return std::string(buffer, bytesRead);
}

int main() {
    HANDLE hSerial = CreateFileA(
        "COM3",  // 请替换为蓝牙对应的虚拟COM端口号
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "无法打开COM口" << std::endl;
        return 1;
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(hSerial, &dcbSerialParams);

    dcbSerialParams.BaudRate = CBR_9600;  // 波特率根据需要调整
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    SetCommState(hSerial, &dcbSerialParams);

    // 发送数据
    sendData(hSerial, "Hello from Windows!\n");

    // 接收数据
    std::string received = readData(hSerial);
    std::cout << "收到来自树莓派的回复: " << received << std::endl;

    CloseHandle(hSerial);
    return 0;
}
