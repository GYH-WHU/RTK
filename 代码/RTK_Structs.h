#pragma once
#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cfloat>
#include <vector>
#include<stdio.h>
#include<windows.h>
#include <math.h>
#include <memory.h>
#include <algorithm> 

using namespace Eigen;
using std::vector;
using namespace std;

/*-----------------------------------------------
    常数定义
------------------------------------------------*/

/* 导航卫星系统定义 */
enum GNSSSys { UNKS = 0, GPS, BDS, GLONASS, GALILEO, QZSS };    // 默认下为位置系统UNKS

/* 弧度-角度转换 */
#define PAI 3.14159265358979323846264338338279                  // 圆周率
#define Rad (PAI / 180.0)                                       // 角度转弧度
#define Deg (180.0 / PAI)                                       // 弧度转角度

/* WGS84坐标系常数 */
#define R_WGS84     6378137.0                                   // 地球半长轴 [米]
#define F_WGS84     (1.0 / 298.257223563)                       // 地球扁率
#define W_WGS84     7.2921151467e-5                             // 地球自转角速度 [弧度/秒]
#define GM_WGS84    3986005e+8                                  // 地心引力常数 [m^3/s^2]

/* CGCS2000坐标系常数 */
#define R_CGCS2000  6378137.0                                   // 地球半长轴 [米]
#define F_CGCS2000  (1.0 / 298.257222101)                       // 地球扁率
#define W_CGCS2000  7.292115e-5                                 // 地球自转角速度 [弧度/秒]
#define GM_CGCS2000 3986004.418e+8                              // 地心引力常数 [m^3/s^2]

/* 数据解码 */
#define C_Light 299792458.0                                     // 光速[米每秒]
#define FILEMODE 1

#define GPST_BDT  14                                            // GPS时与北斗时的差值[s]
#define MAXCHANNUM 36                                           // 接收机最多可同时处理的通道数量
#define MAXGPSNUM  32                                           // GPS系统最大卫星编号
#define MAXBDSNUM 63                                            // 北斗系统最大卫星编号
#define MAXRAWLEN 40960                                         // 原始数据缓冲区的最大长度（单位：字节）
#define POLYCRC32   0xEDB88320u                                 //校验码 

/* 卫星PVT解算 */
#define  FG1_GPS  1575.42E6                                     // L1信号频率
#define  FG2_GPS  1227.60E6                                     // L2信号频率
#define  FG1_BDS  1561.098E6                                    // B1信号的基准频率
#define  FG3_BDS  1268.520E6                                    // B3信号的基准频率

#define  WL1_GPS  (C_Light/FG1_GPS)                             // L1信号波长
#define  WL2_GPS  (C_Light/FG2_GPS)                             // L2信号波长
#define  WL1_BDS  (C_Light/FG1_BDS)                             // B1信号波长
#define  WL3_BDS  (C_Light/FG3_BDS)                             // B3信号波长

/* 误差改正 */
#define  Htrop      15000.0                                     // 对流层顶高度 m
#define  SeaLevel   0                                           // 海平面高度H0
#define  TempT0     15 + 273.16                                 // 温度T0
#define  AtmosPre   1013.25                                     // 气压p0
#define  RelHum     0.50                                        // 相对湿度RH0

#define  GFThres    0.05                                        // GF限差 cm
#define  MWThres    3.0                                         // MW限差 m

#define CFGITEMS 14                                             // 配置项的数量

/* 流动站精确坐标 */
#define PreciseB 30.528754
#define PreciseL 114.357103
#define PreciseH 41.390371
extern int codemode;                                                // 模式选择，事后解算为1，实时解算为0

/*-----------------------------------------------
    数据结构
------------------------------------------------*/

/* 通用时间 (年/月/日 时:分:秒) */
struct COMMONTIME
{
    short Year;
    unsigned short Month;
    unsigned short Day;
    unsigned short Hour;
    unsigned short Minute;
    double Second;

    COMMONTIME()
        : Year(0), Month(0), Day(0),
        Hour(0), Minute(0), Second(0.0) {
    }
};

/* 简化儒略日 (MJD) */
struct MJDTIME
{
    int Days;
    double FracDay;

    MJDTIME() : Days(0), FracDay(0.0) {}
};

/* GPS时间 (周, 周内秒) */
struct GPSTIME
{
    unsigned short Week;
    double SecOfWeek;

    GPSTIME() : Week(0), SecOfWeek(0.0) {}
};

/* 笛卡尔坐标 (X, Y, Z) */
union XYZ
{
    struct {
        double x;
        double y;
        double z;
    };
    double xyz[3];
};

/* 大地坐标 (经度, 纬度, 高程) */
union GEOCOOR
{
    struct {
        double latitude;    // 纬度
        double longitude;   // 经度
        double height;      // 高程
    };
    double Blh[3];
};

/* 测站地平坐标 (北, 东, 天顶) */
union NEU
{
    struct {
        double dE;
        double dN;
        double dU;
    };
    double Neu[3];  // ENU
};

/* 每颗卫星的观测数据  */
struct SATOBSDATA
{
    short Prn;          // 卫星编号
    GNSSSys System;     // 卫星所属系统

    double P[2];        // 伪距观测值（单位：米），最多两频
    double L[2];        // 载波相位观测值（单位：周），最多两频
    double D[2];        // 多普勒观测值（单位：Hz），最多两频

    double cn0[2];      // 载噪比 C/N0（单位：dB-Hz），最多两频
    double LockTime[2]; // 信号锁定时间（单位：秒），最多两频

    unsigned char half[2]; // 半周模糊度标志（通常为 0 或 1），最多两频

    bool Valid;         // 数据有效性标志（true为有效，false为无效）

    SATOBSDATA()
    {
        Prn = 0;
        System = UNKS;
        for (int i = 0; i < 2; i++)
            P[i] = L[i] = D[i] = 0.0;
        cn0[0] = cn0[1] = 0.0;
        LockTime[0] = LockTime[1] = 0.0;
        half[0] = half[1] = 0.0;
        Valid = false;
    }
};

/*  卫星星历结构体   */
struct GPSEPHREC
{
    short PRN;         // 卫星编号
    GNSSSys System;    // 卫星系统

    GPSTIME TOC;       // 钟差参考时间
    GPSTIME TOE;       // 轨道参考时间

    double ClkBias;    // 卫星钟偏差（秒）
    double ClkDrift;   // 卫星钟漂移（秒/秒）
    double ClkDriftRate; // 卫星钟漂移率（秒/秒²）

    double IODE;       // 星历数据编号（轨道部分）
    double IODC;       // 星历数据编号（钟差部分）

    double SqrtA;      // 轨道长半轴的平方根（√a），单位：m^0.5
    double M0;         // 平近点角（弧度），参考时间的值
    double e;          // 轨道偏心率
    double OMEGA;      // 初始升交点经度（弧度）
    double i0;         // 轨道倾角（弧度）
    double omega;      // 升交距角（近地点幅角，弧度）

    double Crs;        // 正弦方向径向轨道改正项（米）
    double Cuc;        // 余弦方向轨道幅角改正项（弧度）
    double Cus;        // 正弦方向轨道幅角改正项（弧度）
    double Cic;        // 余弦方向升交点赤经改正项（弧度）
    double Cis;        // 正弦方向升交点赤经改正项（弧度）
    double Crc;        // 余弦方向径向轨道改正项（米）

    double DeltaN;     // 平均角速度改正值（弧度/秒）
    double OMEGADot;   // 升交点赤经变化率（弧度/秒）
    double iDot;       // 轨道倾角变化率（弧度/秒）

    int SVHealth;      // 卫星健康状态（0=健康，其他=不健康）

    double TGD1;       // 群延迟（秒）
    double TGD2;       // 第二群延迟

    // ====== 构造函数 ======
    GPSEPHREC()
    {
        PRN = 0;
        System = UNKS;  // 或 GNSSSys(0)，默认 未知

        TOC.Week = TOE.Week = 0;
        TOC.SecOfWeek = TOE.SecOfWeek = 0.0;

        ClkBias = 0.0;
        ClkDrift = 0.0;
        ClkDriftRate = 0.0;

        IODE = 0.0;
        IODC = 0.0;

        SqrtA = 0.0;
        M0 = 0.0;
        e = 0.0;
        OMEGA = 0.0;
        i0 = 0.0;
        omega = 0.0;

        Crs = 0.0;
        Cuc = 0.0;
        Cus = 0.0;
        Cic = 0.0;
        Cis = 0.0;
        Crc = 0.0;

        DeltaN = 0.0;
        OMEGADot = 0.0;
        iDot = 0.0;

        SVHealth = 0;

        TGD1 = 0.0;
        TGD2 = 0.0;
    }

};

/* 每个历元的定位结果结构体定义 */
struct POSRES
{
    GPSTIME Time;          // 定位时间（注意：PSRPOS中没有明确时间，需要外部同步）
    double Pos[3];         // 接收机位置（经度、纬度、高程）[度, 度, 米]

    double SigmaPos;       // 三维位置标准差（m）
    double SigmaVel;       // 速度标准差（PSRPOS未提供，置0）
    double PDOP;           // PDOP（PSRPOS未提供，置0）

    unsigned char SatNum_tracked; // 跟踪卫星数量
    unsigned char SatNum_used;    // 参与解算的卫星数量

    float undulation;      // 大地水准面高度差（m）
    float lat_sigma;       // 纬度标准差（m）
    float lon_sigma;       // 经度标准差（m）
    float hgt_sigma;       // 高度标准差（m）

    unsigned int sol_status;  // 解状态
    unsigned int pos_type;    // 位置类型

    POSRES()
    {
        Pos[0] = Pos[1] = Pos[2] = 0.0;

        SigmaPos = 0.0;
        SigmaVel = 0.0;
        PDOP = 0.0;

        SatNum_tracked = 0;
        SatNum_used = 0;

        undulation = 0.0f;
        lat_sigma = 0.0f;
        lon_sigma = 0.0f;
        hgt_sigma = 0.0f;

        sol_status = 0;
        pos_type = 0;
    }
};

/* 每颗卫星位置、速度和钟差等的中间计算结果 */
struct SATPVT
{
    double SatPos[3];     // 卫星在ECEF坐标系下的三维位置 (X, Y, Z), 单位: 米
    double SatVel[3];     // 卫星在ECEF坐标系下的三维速度 (VX, VY, VZ), 单位: 米/秒
    double SatClkOft;     // 卫星钟差, 单位: 秒 (包含卫星钟偏移、漂移及相对论效应)
    double SatClkSft;     // 卫星钟速差, 单位: 无量纲 (用于精密定位中的钟速改正)
    double Elevation;     // 卫星仰角, 单位: 弧度 (范围: 0 ~ π/2)
    double Azimuth;       // 卫星方位角, 单位: 弧度 (范围: 0 ~ 2π, 从北方向顺时针计量)
    double TropCorr;      // 对流层延迟改正量, 单位: 米 (用于消除大气延迟对信号的影响)
    double Tgd1, Tgd2;    // 群延迟改正参数, 单位: 秒 (Tgd1: L1/L2频率间偏差, Tgd2: 其他频率间偏差)
    bool Valid;           // 数据有效性标志: 
    // false=没有可用星历或星历过期/不健康, 
    // true=卫星位置、钟差计算成功且可用

    SATPVT()
    {
        // 初始化位置和速度为0
        SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
        SatVel[0] = SatVel[1] = SatVel[2] = 0.0;

        // 初始仰角设为0
        Elevation = 0;

        // 初始化钟差、钟速差和改正参数为0
        SatClkOft = SatClkSft = 0.0;
        Tgd1 = Tgd2 = TropCorr = 0.0;

        Azimuth = 0.0;
        // 默认标记为无效数据
        Valid = false;
    }
};

/* MWGF组合观测 */
struct MWGF
{
    short Prn;          // 卫星号
    GNSSSys Sys;        // 系统名称
    double MW, GF, PIF; // 组合观测值

    int n;              // 平滑计数

    MWGF()
    {
        Prn = n = 0;
        Sys = UNKS;
        MW = GF = PIF = 0.0;
    }
};

/* 电离层模型参数 */
struct IONOPARA
{
    double alpha[4]; // 电离层振幅参数 alpha0至alpha3
    double beta[4];  // 电离层周期参数 beta0至beta3

    IONOPARA()
    {
        for (int i = 0; i < 4; i++) {
            alpha[i] = 0.0;
            beta[i] = 0.0;
        }
    }
};

/* 每个历元的观测数据定义 */
struct EPOCHOBS
{
    GPSTIME    Time;                           // 当前历元的时间（GPS时间，包括周和周内秒）
    short      SatNum;                         // 当前历元观测到的卫星数量
    SATOBSDATA SatObs[MAXCHANNUM];             // 每颗卫星的观测数据（如伪距、载波等）
    SATPVT     SatPVT[MAXCHANNUM];             // 每颗卫星的位置、速度、钟差计算结果
    MWGF       ComObs[MAXCHANNUM];             // 每颗卫星的组合观测量（如MW组合、GF组合）
    double     Pos[3];                         // 当前历元接收机的位置（XYZ坐标）

    EPOCHOBS()
    {
        SatNum = 0;
        Pos[0] = Pos[1] = Pos[2] = 0.0;
    }
};

/*  每颗卫星的单差观测数据定义  */
struct SDSATOBS
{
    short    Prn;
    GNSSSys  System;
    short    Valid;         // 0代表无效
    double   dP[2], dL[2];  // 伪距和载波相位观测值单差
    short    nBase, nRover; // 存储单差观测值对应的基准和流动站的数值索引号
    unsigned char half[2];  // 半周模糊度标志（0表示有半周），两频

    SDSATOBS()
    {
        Prn = nBase = nRover = 0;
        System = UNKS;
        dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
        half[0] = 0; half[1] = 0;
        Valid = -1;
    }
};

/*  每个历元的单差观测数据定义  */
struct SDEPOCHOBS
{
    GPSTIME    Time;                   // 当前单差观测对应的GPS时间（周内秒、周数等）
    short      SatNum;                 // 本历元参与单差的卫星数（有效配对数）
    SDSATOBS   SdSatObs[MAXCHANNUM];   // 按卫星存放的单差观测（伪距/相位差、系统/PRN、有效标志等）
    MWGF       SdCObs[MAXCHANNUM];     // 保存用于周跳检测的历史MW/GF组合数据（上一历元的参考）

    SDEPOCHOBS()
    {
        SatNum = 0;                    // 构造时默认单差卫星数为0
    }
};

/* 双差解算相关的数据结构定义 */
struct DDCOBS
{
    int RefPrn[2], RefPos[2];                   // 参考星的PRN号与索引位置：[0]=GPS, [1]=BDS
    int Sats, DDSatNum[2];                      // 所有系统的双差卫星数，总数与各系统数量
    double FixedAmb[MAXCHANNUM * 4];            // 固定解的模糊度值（前AmbNum为最优，后为次优）
    double ResAmb[2], Ratio;                    // 模糊度残差与比值检验结果
    float  FixRMS;                              // 固定解的RMS误差（米）
    double dPos[3];                             // 计算得到的基线向量（XYZ）
    bool   bFixed;                              // 是否成功固定模糊度（true = 已固定）

    DDCOBS()
    {
        for (int i = 0; i < 2; i++) {
            DDSatNum[i] = 0;
            RefPrn[i] = RefPos[i] = -1;
        }
        Sats = 0;
        dPos[0] = dPos[1] = dPos[2] = 0.0;
        ResAmb[0] = ResAmb[1] = FixRMS = Ratio = 0.0;
        bFixed = false;
        for (int i = 0; i < MAXCHANNUM * 2; i++) {
            FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
        }
    }
};

/* RTK浮点解数据定义 */
struct FloatResult
{
    double dX[3];               // 基线向量（dx,dy,dz）Rov-Bas
    double N[MAXCHANNUM * 2];   // 双差模糊度
    double sigma;               // 验后单位权中误差
    double DOP[3];              // 基线向量对应的协因数（Qxx,Qyy,Qzz）
    double PDOP, HDOP, VDOP;    // 位置精度因子、水平精度因子、垂直精度因子
    double Qn[(MAXCHANNUM * 2) * (MAXCHANNUM * 2)]; // 模糊度的协因数矩阵 
    int stanum[2], totalsatnum; // 解算所用双差卫星数（不包括两个系统各自的基准星）-> 用于确定Qn的有效元素数量

    FloatResult()
    {
        dX[0] = dX[1] = dX[2] = 0.0;
        PDOP = HDOP = VDOP = DOP[0] = DOP[1] = DOP[2] = 0.0;
        sigma = 0.0;
        stanum[0] = stanum[1] = totalsatnum = 0;
        for (int i = 0; i < MAXCHANNUM * 2; i++)
        {
            N[i] = 0.0;
            for (int j = 0; j < MAXCHANNUM * 2; j++)
            {
                Qn[i * MAXCHANNUM * 2 + j] = 0.0;
            }
        }
    }
};

/* 每个历元的单点定位/测速结果及其精度指标 */
struct PPRESULT
{
    GPSTIME Time;                               // 当前历元时间
    double Position[3];                         // 解算出的XYZ坐标（单位：米）
    double Velocity[3];                         // 解算出的XYZ速度（单位：米/秒）
    double RcvClkOft[2];                        // 接收机钟差：[0]=GPS钟差, [1]=BDS钟差，单位：秒
    double RcvClkSft;                           // 接收机钟漂（钟差变化率，单位：秒/秒）
    double PDOP, SigmaPos, SigmaVel;            // 定位精度因子、位置与速度的标准差（单位：米）
    short  GPSSatNum, BDSSatNum;                // 参与解算的GPS和BDS卫星数
    short  AllSatNum;                           // 所有参与观测的卫星数（总数）
    bool   IsSuccess;                           // 单点定位是否成功（true=成功）

    PPRESULT()
    {
        for (int i = 0; i < 3; i++)Position[i] = Velocity[i] = 0.0;
        RcvClkOft[0] = RcvClkOft[1] = RcvClkSft = 0.0;
        PDOP = SigmaPos = SigmaVel = 999.9;
        GPSSatNum = BDSSatNum = AllSatNum = 0;
        IsSuccess = false;
    }
};

/* RTK定位的数据结构定义 */
struct RAWDAT {
    EPOCHOBS   BasEpk_0;                            // 基准站的上一个观测数据（EPOCHOBS 结构体）
    EPOCHOBS   RovEpk_0;                            // 流动站的上一个观测数据（EPOCHOBS 结构体）
    EPOCHOBS   BasEpk_cur;                          // 基准站的当前观测数据（EPOCHOBS 结构体）
    EPOCHOBS   RovEpk_cur;                          // 流动站的当前观测数据（EPOCHOBS 结构体）
    SDEPOCHOBS SdObs;                               // 单差观测数据（由基准站与流动站的原始观测生成）
    DDCOBS     DDObs;                               // 双差解算结果（包含模糊度固定信息、基线向量等）
    GPSEPHREC  GpsEph[MAXGPSNUM];                   // GPS系统的广播星历（每颗卫星一个 GPSEPHREC 结构）
    GPSEPHREC  BdsEph[MAXBDSNUM];                   // BDS系统的广播星历（每颗卫星一个 GPSEPHREC 结构）
    POSRES BaseBest;                                // 基站的参考位置
    POSRES RoverBest;                               // 流动站的参考位置
};

/* RTK定位后卡尔曼滤波结果 */
struct RTKEKF
{
    GPSTIME Time;
    double X[3 + MAXCHANNUM * 2], P[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];
    int Index[MAXCHANNUM], nSats, nPos[MAXCHANNUM];
    int FixAmb[MAXCHANNUM];           // 时间更新后上个历元已经固定并传递的模糊度， 1=已固定，-1=未固定或有周跳
    DDCOBS DDObs, CurDDObs;           // 上一个历元和当前历元的双差观测值信息
    SDEPOCHOBS SDObs;                 // 上一个历元的单差观测值
    double X0[3 + MAXCHANNUM * 2], P0[(3 + MAXCHANNUM * 2) * (3 + MAXCHANNUM * 2)];  // 状态备份
    double ratio;                     // ratio值
    bool IsInit;                      // 滤波是否初始化
    bool IsFixed;                     // 滤波是否进行了LAMBDA固

    RTKEKF() {
        ratio = 0.0;
        IsInit = false;
        IsFixed = false;
        nSats = 0;
        for (int i = 0; i < MAXCHANNUM; i++) nPos[i] = Index[i] = FixAmb[i] = -1;
        for (int i = 0; i < 3 + MAXCHANNUM * 2; i++) {
            X[i] = X0[i] = 0.0;
            for (int j = 0; j < 3 + MAXCHANNUM * 2; j++) P[i * (3 + MAXCHANNUM * 2) + j] = P0[i * (3 + MAXCHANNUM * 2) + j] = 0.0;
        }
    }
};

/* 流动站配置信息 */
struct ROVERCFGINFO   // 配置信息
{
    int  IsFileData;                     //  1=文件, 0=实时,
    int RTKProcMode;                     // 1=EKF, 2=LSQ
    int    RovPort, RovBaud;             // 流动站COM端口设置
    char   BasNetIP[20], RovNetIP[20];   // 基站、流动站IP地址
    int BasNetPort, RovNetPort;          // 基站、流动站端口
    double CodeNoise, CPNoise;           // 伪距噪声
    double ElevThreshold;                // 高度角阈值
    double RatioThres;                   // Ratio检验阈值

    char  BasObsDatFile[256], RovObsDatFile[256];       //  观测数据的文件名
    char  FloatFile[256], FixedFile[256], SppFile[256], EkfFile[256]; //  结果数据文件名

    ROVERCFGINFO()
    {
        IsFileData = RTKProcMode = -1;
        RovPort = 0;  RovBaud = 0; BasNetPort = 0; RovNetPort = 0;
        CodeNoise = 0;
        CPNoise = 0;
        ElevThreshold = 0.0;
        RatioThres = 0.0;
    }
};

/*-----------------------------------------------
    时间转换函数声明
------------------------------------------------*/
void CommonTimeToMjdTime(const COMMONTIME* ct, MJDTIME* mjd);   /* 通用时 -> 简化儒略日 */
void CommonTimeToGpsTime(const COMMONTIME* ct, GPSTIME* gps);   /* 通用时 -> GPS时间 */
void MjdTimeToCommonTime(const MJDTIME* mjd, COMMONTIME* ct);   /* 简化儒略日 -> 通用时 */
void MjdTimeToGpsTime(const MJDTIME* mjd, GPSTIME* gps);        /* 简化儒略日 -> GPS时间 */
void GpsTimeToCommonTime(const GPSTIME* gps, COMMONTIME* ct);   /* GPS时间 -> 通用时 */
void GpsTimeToMjdTime(const GPSTIME* gps, MJDTIME* mjd);        /* GPS时间 -> 简化儒略日 */

/*-----------------------------------------------
    坐标转换与误差计算 函数声明
------------------------------------------------*/
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F);                      /* 大地坐标 -> 笛卡尔坐标 */
void XYZToBLH(const double XYZ[3], double BLH[3], const double R, const double F);                      /* 笛卡尔坐标 -> 大地坐标 */
void BlhToNeuMat(const GEOCOOR* Blh, Eigen::Matrix3d& Mat);                                             /* 生成 BLH -> NEU 的旋转矩阵 */
void CompEnudPos(const double X0[], const double Xr[], const GEOCOOR* Blh, double dNeu[]);              /* 计算 ENU 坐标差（误差） */
void CompSatElAz(const double Xr[], const double Xs[], const GEOCOOR* Blh, double* Elev, double* Azim); /* 计算卫星高度角与方位角 */

/*-----------------------------------------------
    数据解码 函数声明
------------------------------------------------*/
int DecodeNovOem7Dat(unsigned char buff[], int& len, EPOCHOBS* obs, GPSEPHREC geph[], GPSEPHREC beph[], POSRES* pos);   /* 解码主函数 */
unsigned int crc32(const unsigned char* buff, int len);             /* CRC32校验码 */
void DecodeRange(const unsigned char* msg, EPOCHOBS* obs);          /* 解码二进制观测数据消息 */
void DecodeGpsEphem(const unsigned char* msg, GPSEPHREC geph[]);    /* 解码GPS观测值数据 */
void DecodeBdsEphem(const unsigned char* msg, GPSEPHREC beph[]);    /* 解码BDS观测值数据 */
void Decode_psrpos(const unsigned char* msg, POSRES* pos);          /* 解码单点定位解 */

/*-----------------------------------------------
    定位解算 函数声明
------------------------------------------------*/
bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid);  /* 星历是否过期的判断    不加相对论改正的卫星钟差 */
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATPVT* Mid);                                      /* 计算GPS卫星位置、速度、钟差 */
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATPVT* Mid);                                      /* 计算BDS卫星位置、速度、钟差 */

/*-----------------------------------------------
    误差改正 函数声明
------------------------------------------------*/
double Klobutchar_GPS(const GPSTIME* Time, double Elev, double Azim, double RcvPos[3], const IONOPARA* Ion);    /* 电离层延迟改正 */
double Klobutchar_BDS(const GPSTIME* Time, double Elev, double Azim, double RcvPos[3], const IONOPARA* Ion);
double Hopfield(const double H, const double Elev);                                                             /* 对流层延迟改正 */
void DetectOutlier(EPOCHOBS* Obs);                                                                              /* 粗差探测 */

/*-----------------------------------------------
    SPP & SPV 函数声明
------------------------------------------------*/
void ComputeSatPVTAtSignalTrans(EPOCHOBS* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double UserPos[3]);    /*	信号发射时刻卫星位置计算 */
bool SPP(EPOCHOBS* Epoch, RAWDAT* Raw, PPRESULT* Result);                                                   /*  单点定位SPP解算 */
void SPV(EPOCHOBS* Epoch, PPRESULT* Result);                                                                /*  单点测速SPV解算 */

/*-----------------------------------------------
    Socket网络
------------------------------------------------*/
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);  /*  打开串口通信  */
void CloseSocket(SOCKET& sock); /*  关闭串口通信  */

/*-----------------------------------------------
    配置文件读取
------------------------------------------------*/
bool ReadRTKConfigInfo(const char FName[], ROVERCFGINFO* ANinfo);   /* 读取配置文件 */

/*-----------------------------------------------
    RTK函数
------------------------------------------------*/
int SynRTKObsPP(FILE* FBase, FILE* FRover, RAWDAT* Raw);                                        /* 基站与流动站数据时间同步（文件）*/
int SynRTKObsRT(SOCKET& BaseCom, SOCKET& RoverCom, RAWDAT* Raw);                                /* 基站与流动站数据时间同步（实时）*/
void MarkValid(const EPOCHOBS* EpkLast, EPOCHOBS* EpkCur);                                      /* 观测数据有效性的标记 */
void SDEphObs(const EPOCHOBS* EpkR, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs);                   /* 基站与流动站 站间单差 */
void DetectCycleSlip(SDEPOCHOBS* Obs);                                                          /* 周跳探测函数 */
void DetRefSat(const EPOCHOBS* EpkA, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs);   /* 选取参考卫星 */
/* 最小二乘法 */
bool RTKFloat(RAWDAT* Rawdata, PPRESULT* Base, PPRESULT* Rover, FloatResult* Fres);             /* 相对定位浮点解函数设计 */
void GetQnn(const int satnum, const MatrixXd& N_inv, double* Q);                                /* 提取QNN协因数矩阵 */
bool RTKFix(RAWDAT* Rawdata, PPRESULT* Base, PPRESULT* Rover, FloatResult* Fres);               /* 相对定位固定解 */
/* 卡尔曼滤波法 */
void InitFilter(RAWDAT* Rawdata, PPRESULT* Base, PPRESULT* Rover, RTKEKF* ekf);                 /* 滤波初始化 */
bool EkfPredict(RAWDAT* Rawdata, RTKEKF* ekf);                                                  /* 时间更新函数 */
inline Vector3d LineOfSightDD(const double Rover[3], const double Sat[3], const double Ref[3]); /* 方向余弦计算 */
bool EkfMeasureUpdate(RAWDAT* Rawdata, RTKEKF* ekf);                                            /* 测量更新函数 */

/*-----------------------------------------------
    Lambda函数
------------------------------------------------*/
int lambda(int n, int m, const double* a, const double* Q, double* F, double* s);                       /* lambda/mlambda整数最小二乘估计 */
int MatrixInv(int n, double a[], double b[]);                                                           /* 矩阵求逆 */
bool MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[]); /* 矩阵的乘法 */
int search(int n, int m, const double* L, const double* D, const double* zs, double* zn, double* s);    /* 改进的lambda搜索算法 */
void reduction(int n, double* L, double* D, double* Z);                                                 /* lambda约化 */
void perm(int n, double* L, double* D, int j, double del, double* Z);                                   /* 置换变换 */
void gauss(int n, double* L, double* Z, int i, int j);                                                  /* 整数高斯变换 */
int LD(int n, const double* Q, double* L, double* D);                                                   /* LD分解 */

/*-----------------------------------------------
    结果打印函数
------------------------------------------------*/
void ToFileSppResult(RAWDAT* rawdata, PPRESULT* Fres, double enu[3], FILE* OutputFile);                             /* 输出单点定位结果 */
void ToFileFloatResult(RAWDAT* rawdata, double XYZ[3], double denu[3], FloatResult* Fres, FILE* OutputFile);        /* 输出浮点解定位结果 */
void ToFileFixedResult(RAWDAT* rawdata, double XYZ[3], double denu[3], FILE* OutputFile);                           /* 输出固定解定位结果 */
void ToFileEkfResult(RAWDAT* rawdata, RTKEKF* ekf, double denu[3], FILE* OutputFile);                               /* 输出卡尔曼定位结果 */
void FigureRTKPlot(RAWDAT* rawdata, double FixedBlh[3], RTKEKF* ekf, double EkfBlh[3], FILE* OutputFile, int mode); /* 输出定位结果用于RTKPLOT画图 */
