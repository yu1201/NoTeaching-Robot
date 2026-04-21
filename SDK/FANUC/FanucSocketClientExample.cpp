#include <winsock2.h>
#include <ws2tcpip.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#pragma comment(lib, "ws2_32.lib")

namespace
{
    bool InitWinSock()
    {
        WSADATA wsaData = {};
        const int ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (ret != 0)
        {
            std::cerr << "WSAStartup failed, ret=" << ret << std::endl;
            return false;
        }
        return true;
    }

    void CleanupWinSock()
    {
        WSACleanup();
    }

    SOCKET ConnectToServer(const std::string& ip, unsigned short port)
    {
        SOCKET sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock == INVALID_SOCKET)
        {
            std::cerr << "socket() failed, err=" << WSAGetLastError() << std::endl;
            return INVALID_SOCKET;
        }

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
        {
            std::cerr << "inet_pton failed for ip=" << ip << std::endl;
            closesocket(sock);
            return INVALID_SOCKET;
        }

        if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR)
        {
            std::cerr << "connect() failed, err=" << WSAGetLastError() << std::endl;
            closesocket(sock);
            return INVALID_SOCKET;
        }

        std::cout << "Connected to " << ip << ":" << port << std::endl;
        return sock;
    }

    bool SendLine(SOCKET sock, const std::string& text)
    {
        std::string payload = text + "\n";
        const char* data = payload.c_str();
        int totalSent = 0;
        int totalSize = static_cast<int>(payload.size());

        while (totalSent < totalSize)
        {
            const int sent = send(sock, data + totalSent, totalSize - totalSent, 0);
            if (sent == SOCKET_ERROR)
            {
                std::cerr << "send() failed, err=" << WSAGetLastError() << std::endl;
                return false;
            }
            totalSent += sent;
        }

        std::cout << "TX: " << text << std::endl;
        return true;
    }

    bool ReceiveLine(SOCKET sock, std::string& out)
    {
        out.clear();
        char ch = 0;

        while (true)
        {
            const int recved = recv(sock, &ch, 1, 0);
            if (recved == 0)
            {
                std::cerr << "Connection closed by peer." << std::endl;
                return false;
            }
            if (recved == SOCKET_ERROR)
            {
                std::cerr << "recv() failed, err=" << WSAGetLastError() << std::endl;
                return false;
            }

            if (ch == '\n')
            {
                break;
            }
            if (ch != '\r')
            {
                out.push_back(ch);
            }
        }

        std::cout << "RX: " << out << std::endl;
        return true;
    }
}

int main()
{
    // 这里替换成你的 FANUC 控制器 IP 和 KAREL 程序监听端口。
    const std::string robotIp = "192.168.1.100";
    const unsigned short robotPort = 9000;

    if (!InitWinSock())
    {
        return 1;
    }

    SOCKET sock = ConnectToServer(robotIp, robotPort);
    if (sock == INVALID_SOCKET)
    {
        CleanupWinSock();
        return 2;
    }

    // 这是最小联通性测试协议：
    // PC -> HELLO
    // Robot -> OK
    if (!SendLine(sock, "HELLO"))
    {
        closesocket(sock);
        CleanupWinSock();
        return 3;
    }

    std::string response;
    if (!ReceiveLine(sock, response))
    {
        closesocket(sock);
        CleanupWinSock();
        return 4;
    }

    if (response != "OK")
    {
        std::cerr << "Unexpected response: " << response << std::endl;
    }

    // 示例：继续发送一条业务指令
    if (!SendLine(sock, "GET_STATUS"))
    {
        closesocket(sock);
        CleanupWinSock();
        return 5;
    }

    if (!ReceiveLine(sock, response))
    {
        closesocket(sock);
        CleanupWinSock();
        return 6;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    closesocket(sock);
    CleanupWinSock();
    return 0;
}
