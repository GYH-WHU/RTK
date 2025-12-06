#include"RTK_Structs.h"

/*-----------------------------------------------
    字节提取函数
------------------------------------------------*/
double R8(unsigned char* p) // 提取 8 字节 double
{
    double r;
    memcpy(&r, p, 8);
    return r;
}
float R4(unsigned char* p)  // 提取 4 字节 float
{
    float r;
    memcpy(&r, p, 4);
    return r;
}
int I4(unsigned char* p)    // 提取 4 字节有符号整数
{
    int r;
    memcpy(&r, p, 4);
    return r;
}
unsigned int UI4(unsigned char* p)  // 提取 4 字节无符号整数
{
    unsigned int r;
    memcpy(&r, p, 4);
    return r;
}
short I2(unsigned char* p)  // 提取 2 字节有符号整数
{
    short r;
    memcpy(&r, p, 2);
    return r;
}
unsigned short UI2(unsigned char* p)    // 提取 2 字节无符号整数
{
    unsigned short r;
    memcpy(&r, p, 2);
    return r;
}

// CRC32校验函数
unsigned int crc32(const unsigned char* buff, int len)
{
    int i, j;
    unsigned int crc = 0;
    for (i = 0; i < len; i++)
    {
        crc ^= buff[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ POLYCRC32;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/*-----------------------------------------------
    观测值数据解码
------------------------------------------------*/
void DecodeRange(unsigned char* data, EPOCHOBS* obs)
{
    GNSSSys sys;
    int i, n = 0, j, k = 0;
    int Prn;    // 卫星编号
    int Freq;   // 频率
    int ObsNum; // 观测值数量
    double wl;  // 信号的波长
    unsigned int ChanStatus;    // 通道状态
    int PhaseLockFlag, CodeLockedFlag, ParityFlag, SatSystem, SigType;
    unsigned char* p = data + 28;   // 跳过同步头和消息头，直接指向消息体

    //1、从消息头中解码得到观测时刻，该时刻为接收机钟表面时，用GPSTIME结构体表示。
    obs->Time.Week = UI2(data + 14);
    obs->Time.SecOfWeek = UI4(data + 16) * 1E-3;    // 毫秒要乘e-3

    //2、解码得到观测值数量，为所有卫星所有信号观测值的总数。
    ObsNum = UI4(p);  // 观测值数量
    memset(obs->SatObs, 0, MAXCHANNUM * sizeof(SATOBSDATA));    // 将当前历元的所有卫星观测数据清空

    //3、对所有信号观测值进行循环解码
    for (i = 0, p += 4; i < ObsNum; i++, p += 44)   // p+4是因为要跳过ObsNum4个字节，p+44是因为一个观测数据固定长度44个字节
    {
        //① 解码得到跟踪状态标记，从中取出Phase lock flag / Code locked / flag / Parity known flag / Satellite system / signal type等数据
        ChanStatus = UI4(p + 40);
        ParityFlag = (ChanStatus >> 11) & 0x01;
        PhaseLockFlag = (ChanStatus >> 10) & 0x01;
        CodeLockedFlag = (ChanStatus >> 12) & 0x01;
        SatSystem = (ChanStatus >> 16) & 0x07;
        SigType = (ChanStatus >> 21) & 0x1F;

        //② 如果卫星系统不是GPS或BDS，continue至①

        //③ 如果GPS卫星的信号类型不是L1 C / A或者L2P（Y），BDS卫星不是B1I或B3I，continue至①，并记录信号频率类型，第一频率s = 0，第二频率s = 1；
        if (SatSystem == 0) {
            sys = GPS;
            if (SigType == 0) {
                Freq = 0; wl = WL1_GPS;
            }
            else if (SigType == 9) { Freq = 1; wl = WL2_GPS; }
            else continue;
        }
        else if (SatSystem == 4) {
            sys = BDS;
            if (SigType == 0 || SigType == 4) {
                Freq = 0; wl = WL1_BDS;
            }
            else if (SigType == 2 || SigType == 6) { Freq = 1; wl = WL3_BDS; }
            else continue;
        }
        else continue;

        //④ 解码得到卫星号Prn以及卫星系统号，在当前观测值结构体中进行搜索，如果找到相同的卫星，将解码的观测值填充到该卫星对应的数组中；如果在当前已解码的卫星数据中没有发现，则填充到现有数据的末尾。
        Prn = UI2(p);   // 取卫星号
        for (j = 0; j < MAXCHANNUM; j++)
        {
            if (obs->SatObs[j].System == sys && obs->SatObs[j].Prn == Prn) {
                n = j; break;
            }   // 若找到已有系统且prn相同，则找到槽；
            if (obs->SatObs[j].Prn == 0) {
                k = n = j; break;
            }   // 若未找到则找一个prn=0的空槽
        }   // 现在n即为当前卫星观测数据储存的索引

        obs->SatObs[n].Prn = Prn;   // 卫星编号
        obs->SatObs[n].System = sys;    // 所属系统

        obs->SatObs[n].P[Freq] = CodeLockedFlag == 1 ? R8(p + 4) : 0.0;         // 伪距P，伪码锁定==1，否则设为0
        obs->SatObs[n].L[Freq] = -wl * (PhaseLockFlag == 1 ? R8(p + 16) : 0.0); // 载波相位L，相位锁定==1，否则为0
        obs->SatObs[n].D[Freq] = -wl * R4(p + 28);                              // 多普勒频率D
        obs->SatObs[n].cn0[Freq] = R4(p + 32);                                  // 载噪比
        obs->SatObs[n].LockTime[Freq] = R4(p + 36);                             // 锁定时间
        obs->SatObs[n].half[Freq] = ParityFlag;                                 // 半周模糊度标志

    }
    //4. 统计有效的卫星数量
    obs->SatNum = k + 1;
}

/*-----------------------------------------------
    GPS星历解码
------------------------------------------------*/
void DecodeGpsEphem(unsigned char* data, GPSEPHREC geph[])
{
    unsigned char* p = data + 28;  // 跳过28字节头
    int prn;

    //=== 解析 PRN & 检查 =========================================
    prn = UI4(p);  // PRN 在偏移 H+0
    if (prn < 1 || prn > MAXGPSNUM) return;

    GPSEPHREC* eph = geph + (prn - 1);

    eph->PRN = prn;
    eph->System = GPS;

    //=== 时间 =====================================================
    eph->TOE.Week = eph->TOC.Week = UI4(p + 24); // 周编号
    eph->TOE.SecOfWeek = R8(p + 32);             // toe 秒
    eph->TOC.SecOfWeek = R8(p + 164);            // tow（subframe1 时间，先作为 TOC 秒用）

    //=== 星历标识 =================================================
    eph->IODE = UI4(p + 16); // IODE1
    eph->IODC = UI4(p + 20); // IODE2

    //=== 卫星健康 =================================================
    eph->SVHealth = UI4(p + 12); // health

    //=== 轨道六根数 & 改正项 =====================================
    eph->SqrtA = sqrt(R8(p + 40));  // a
    eph->M0 = R8(p + 56);
    eph->e = R8(p + 64);
    eph->omega = R8(p + 72);

    eph->Cuc = R8(p + 80);      // cuc
    eph->Cus = R8(p + 88);      // cus
    eph->Crc = R8(p + 96);      // crc
    eph->Crs = R8(p + 104);     // crs
    eph->Cic = R8(p + 112);     // cic
    eph->Cis = R8(p + 120);     // cis

    eph->DeltaN = R8(p + 48);   // Δn
    eph->OMEGA = R8(p + 144);   // Ω0 升交点经度
    eph->OMEGADot = R8(p + 152);
    eph->i0 = R8(p + 128);      // i0
    eph->iDot = R8(p + 136);    // IDOT

    //=== 钟差参数 =================================================
    eph->ClkBias = R8(p + 180); // a0
    eph->ClkDrift = R8(p + 188);// a1
    eph->ClkDriftRate = R8(p + 196); // a2

    //=== 组延迟 (TGD) =============================================
    eph->TGD1 = R8(p + 172);  // TGD
    //=== Anti-spoofing、URA、N 暂不处理 ==========================
}

/*-----------------------------------------------
    BDS星历解码
------------------------------------------------*/
void DecodeBdsEphem(unsigned char* data, GPSEPHREC beph[])
{
    unsigned char* p = data + 28;  // 跳过同步头+消息头
    int prn;

    //=== 卫星编号 =================================================
    prn = UI4(p);
    if (prn < 1 || prn > MAXBDSNUM) return;

    GPSEPHREC* eph = beph + (prn - 1);

    eph->PRN = prn;
    eph->System = BDS;

    //=== 时间 =====================================================
    eph->TOE.Week = eph->TOC.Week = UI4(p + 4);     // 周数
    eph->TOE.SecOfWeek = UI4(p + 72);               // toe 秒
    eph->TOC.SecOfWeek = UI4(p + 40);               // toc 秒

    //=== 卫星健康状态 =============================================
    eph->SVHealth = UI4(p + 16);

    //=== TGD ======================================================
    eph->TGD1 = R8(p + 20); // tgd1
    eph->TGD2 = R8(p + 28); // tgd2

    //=== 钟差参数 =================================================
    eph->ClkBias = R8(p + 44); // a0
    eph->ClkDrift = R8(p + 52); // a1
    eph->ClkDriftRate = R8(p + 60); // a2

    //=== 星历龄期 =================================================
    eph->IODC = UI4(p + 36); // AODC
    eph->IODE = UI4(p + 68); // AODE

    //=== 轨道参数 =================================================
    eph->SqrtA = R8(p + 76);
    eph->e = R8(p + 84);
    eph->omega = R8(p + 92);  // 近地点幅角
    eph->DeltaN = R8(p + 100);
    eph->M0 = R8(p + 108);
    eph->OMEGA = R8(p + 116); // 升交点赤经
    eph->OMEGADot = R8(p + 124);
    eph->i0 = R8(p + 132);
    eph->iDot = R8(p + 140);

    //=== 轨道摄动改正 =============================================
    eph->Cuc = R8(p + 148);
    eph->Cus = R8(p + 156);
    eph->Crc = R8(p + 164);
    eph->Crs = R8(p + 172);
    eph->Cic = R8(p + 180);
    eph->Cis = R8(p + 188);
}

/*-----------------------------------------------
    NovAtel定位结果数据解码
------------------------------------------------*/
void Decode_psrpos(unsigned char* data, POSRES* pos)
{
    unsigned char* p = data + 28;   // 跳过28字节消息头

    //=== 1、解状态 & 位置类型 =====================================
    pos->sol_status = UI4(p);       // Solution status
    pos->pos_type = UI4(p + 4);     // Position type

    //=== 2️、位置 ================================================
    pos->Pos[1] = R8(p + 8);        // Latitude（度）
    pos->Pos[0] = R8(p + 16);       // Longitude（度）
    pos->Pos[2] = R8(p + 24) + R4(p + 32);       // 高程（m）

    //=== 3️、大地水准面高度差 ===================================
    pos->undulation = R4(p + 32);

    //=== 4️、标准差 =============================================
    pos->lat_sigma = R4(p + 40);
    pos->lon_sigma = R4(p + 44);
    pos->hgt_sigma = R4(p + 48);

    // 组合成三维位置标准差（平方和开根号）
    pos->SigmaPos = sqrt(
        pos->lat_sigma * pos->lat_sigma +
        pos->lon_sigma * pos->lon_sigma +
        pos->hgt_sigma * pos->hgt_sigma
    );

    //=== 5️、卫星数量 ============================================
    pos->SatNum_tracked = *(p + 64); // #SVs
    pos->SatNum_used = *(p + 65);    // #solnSVs

    //=== 6️、未提供字段 ==========================================
    pos->SigmaVel = 0.0; // PSRPOS没有速度标准差
    pos->PDOP = 0.0;     // PSRPOS没有PDOP

    //=== 7️、时间 ================================================
    pos->Time.Week = UI2(data + 14);
    pos->Time.SecOfWeek = UI4(data + 16) * 1E-3;    // 毫秒要乘e-3

}

/*-----------------------------------------------
    解码主函数
------------------------------------------------*/
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos)
{
    int i = 0; // 初始化循环变量
    int MsgLen, MsgID;

    while (i < len)
    {
        // 1、查找同步字符 AA 44 12
        if (i + 3 > len) break; // 剩余不足3字节，不可能有完整同步头
        if (!(buff[i] == 0xAA && buff[i + 1] == 0x44 && buff[i + 2] == 0x12))
        {
            i++; // 未找到同步头，继续查找
            continue;
        }

        // 2、检查是否有足够的字节读取完整的消息头（28字节）
        if (i + 28 > len) break; // 剩余字节不足28，等待下一批数据

        // 从消息头中解码消息长度MsgLen和消息类型MsgID
        MsgID = UI2(buff + i + 4);
        MsgLen = UI2(buff + i + 8);

        // 3、检查整条消息是否完整（头28字节 + MsgLen 字节 + CRC 4字节）
        if (i + 28 + MsgLen + 4 > len) break;

        // 4、CRC检验
        if (crc32(buff + i, 28 + MsgLen) != UI4(buff + i + 28 + MsgLen))
        {
            i += 3;  // 跳过当前AA 44 12
            continue;
        }

        // 5、根据消息 ID 解码
        /*printf("Received message ID: %d\n", MsgID);*/
        int Status = 0;
        switch (MsgID)
        {
        case 43: // 观测数据消息
            DecodeRange(buff + i, obs);
            Status = 1;
            break;

        case 7: // GPS星历
            DecodeGpsEphem(buff + i, geph);
            break;

        case 1696: // BDS星历
            DecodeBdsEphem(buff + i, beph);
            break;

        case 42: // 位置解
            Decode_psrpos(buff + i, pos);
            break;

        default:
            // 未识别的消息，跳过
            //printf("Unknown message ID: %d \n", MsgID);

            break;
        }

        // 5（继续）：跳过整条消息
        i = i + 28 + MsgLen + 4;
        if (Status == 1 && codemode == 1)break; // mode=1时一条一条观测数据处理；mode=0时持续处理所有数据
    }

    // 6、循环结束，处理剩余数据
    memcpy(buff + 0, buff + i, len - i);
    len = len - i;
    return 0;
}