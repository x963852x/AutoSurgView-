#include "udp.h"

using namespace std;
using chrono::microseconds;
using chrono::seconds;
using chrono::milliseconds;

#pragma region 全局变量
WSADATA wsaData;//初始化
SOCKET RecvSocket;	//监听
SOCKET SendSocket;	//发送，上位机，关节角度信息
SOCKET SendSocket2;	//发送，红云主机
SOCKET SendSocket_debug_info;	//发送，上位机，调试信息
SOCKET SendSocketFlag;     //发送重启下位机信息
sockaddr_in RecvAddr;//服务器地址
SOCKADDR_IN SendAddr;//服务器地址
SOCKADDR_IN SendAddr2;//服务器地址
SOCKADDR_IN SendAddr_debug_info;//服务器地址
SOCKADDR_IN SendAddrFlag;//服务器地址
int Port = 6060;//服务器监听地址

int BufLen = 100;//缓冲区大小

sockaddr_in SenderAddr;
int SenderAddrSize = sizeof(SenderAddr);

std::map<std::string, int> redJS;
int newSend = 0, sendCount = 0;
#pragma endregion

//UDP初始化
void udp_init()
{
	//初始化Socket
	WSAStartup(MAKEWORD(2, 2), &wsaData);

	//接收信息，接收来自上位机与红云主机信息
	RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	//将socket与制定端口和0.0.0.0绑定
	RecvAddr.sin_family = AF_INET;
	RecvAddr.sin_port = htons(Port);
	RecvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	//允许端口复用
	BOOL opt = TRUE;
	int optret = setsockopt(RecvSocket, SOL_SOCKET, SO_BROADCAST, (char* FAR) & opt, sizeof(opt));
	int nRecvBuf = 1;
	optret = setsockopt(RecvSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&nRecvBuf, sizeof(int));

	bind(RecvSocket, (SOCKADDR*)&RecvAddr, sizeof(RecvAddr));


	//发送信息，与上位机通信，发送实时关节角度
	SendSocket = socket(AF_INET, SOCK_DGRAM, 0);
	SendAddr.sin_family = AF_INET;
	SendAddr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	SendAddr.sin_port = htons(3000);

	//发送信息，与上位机通信，发送debug信息
	SendSocket_debug_info = socket(AF_INET, SOCK_DGRAM, 0);
	SendAddr_debug_info.sin_family = AF_INET;
	SendAddr_debug_info.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	SendAddr_debug_info.sin_port = htons(3001);

	//发送信息，与重启下位机程序通信，发送重启标志
	SendSocketFlag = socket(AF_INET, SOCK_DGRAM, 0);
	SendAddrFlag.sin_family = AF_INET;
	SendAddrFlag.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	SendAddrFlag.sin_port = htons(3002);

	printf("receiving datagrams...\n");
}