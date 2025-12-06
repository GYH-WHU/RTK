#include"RTK_Structs.h"

#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)

/*-----------------------------------------------
    打开串口通信
------------------------------------------------*/
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port)
{
    WSADATA wsaData;
    SOCKADDR_IN addrSrv;

    if (!WSAStartup(MAKEWORD(1, 1), &wsaData))
    {
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) != INVALID_SOCKET)
        {
            addrSrv.sin_addr.S_un.S_addr = inet_addr(IP);
            addrSrv.sin_family = AF_INET;
            addrSrv.sin_port = htons(Port);
            connect(sock, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
            return true;
        }
    }
    return false;
}

/*-----------------------------------------------
    关闭串口通信
------------------------------------------------*/
void CloseSocket(SOCKET& sock)
{
    closesocket(sock);
    WSACleanup();
}


#include <time.h>
/*-----------------------------------------------
    采集实时数据为二进制格式
------------------------------------------------*/
bool SaveSocketStreamToFile(const char* ip, unsigned short port, const char* binFilePath, double hour)
{
    SOCKET NetGps;
    if (!OpenSocket(NetGps, ip, port)) {
        printf("Socket 打开失败，IP或端口无效。\n");
        return false;
    }

    FILE* fout = fopen(binFilePath, "wb");
    if (!fout) {
        printf("无法创建文件 %s\n", binFilePath);
        CloseSocket(NetGps);
        return false;
    }

    unsigned char buffer[MAXRAWLEN];
    int lenR = 0;

    DWORD startTick = GetTickCount64();  // 获取开始时间（毫秒）
    printf("开始接收数据（时长：%f 小时）...\n", hour);

    while (true) {
        // 判断是否已到达 hour 小时
        DWORD nowTick = GetTickCount64();
        if (nowTick - startTick >= hour * 60.0 * 60.0 * 1e3) {  // 60000ms = 60s; 9h = 32,400,000ms
            printf("接收已满 %f 小时，保存完毕。\n", hour);
            break;
        }

        Sleep(980);  // 模拟采样间隔

        lenR = recv(NetGps, (char*)buffer, MAXRAWLEN, 0);
        if (lenR > 0) {
            fwrite(buffer, 1, lenR, fout);  // 写入接收到的二进制数据
            fflush(fout);
        }
    }

    fclose(fout);
    CloseSocket(NetGps);
    return true;
}
