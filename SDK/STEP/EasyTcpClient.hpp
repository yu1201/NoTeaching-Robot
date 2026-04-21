#pragma once

// 保证此文件的代码只被编译一次
#ifndef EasyTcpClient_hpp_
#define EasyTcpClient_hpp_
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#include <WinSock2.h>
#include <windows.h>
#pragma comment(lib, "ws2_32.lib") // windows socket2 32的lib库
#else
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#define SOCKET int
#define INVALID_SOCKET (SOCKET)(~0)
#define SOCKET_ERROR (-1)
#include <new>
#include <sys/socket.h>
#include <sys/time.h>

#include <netinet/in.h>
//#include <netinet/tcp.h>
#include <netdb.h>
#include <arpa/inet.h>
#endif
#include <iostream>
#include <mutex> 


/* Set max recv buffer lenth */
constexpr int k_recvBufferLenth = 4096;

class EasyTcpClient {
public:
  EasyTcpClient();
  virtual ~EasyTcpClient();

  // 初始化socket
  int initSocket();

  // 连接服务器
  int ConnectServer(char *ip, unsigned short port);

  // 检查运行状态
  int isRun();

  // 检查运行状态 run return 0
  int runStatus();

  // 关闭socket
  void closeSocket();

  // 查询select网络消息
  bool onRun();

  /* Common send / recv function */
  int sendData(char *data, int size);
  int recvData(char *data, int size);

private:
  // 接收数据  处理粘包、拆分包
  int RecvData();

  bool isConnected = false;//连接状态标志
	#ifdef _WIN32
	  SOCKET _sock;
	  SYSTEMTIME st = { 0 };
	#else
	  uintptr_t _sock;
	#endif
	const static int RECVBUFLENGTH = 4096;
	char m_RecvBuff[RECVBUFLENGTH] = { 0 };
};
#endif
