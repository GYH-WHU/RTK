#include"RTK_Structs.h"
ROVERCFGINFO ANinfo;    // 全局变量 读取配置文件
int codemode = -1;      // 解码格式 1为事后 0为实时(从配置文件中读取)

int main()
{
    // 读取配置文件
    cout << "Please enter the configuration file you want to read:\n0:LSQ_FILE  1:LSQ_COM\n2:EKF_FILE  3:EKF_COM\n";
    int mode = -1; cin >> mode;
    bool IsConfig = false;
    for (int i = 0; i < 4; i++) { if (mode == i) { IsConfig = true; break; } }
    if (!IsConfig) { cout << "Can't recognize the configuration mode: " << mode << endl; return -1; }
    const char* ConfigFile = nullptr;   // 文件指针
    switch (mode)
    {
    case 0:
        ConfigFile = "Data\\RtkConfig_LSQ_FILE.txt";    // mode0 最小二乘文件模式
        cout << "It is now in the mode of post-processing using the least squares solution.\n";
        break;
    case 1:
        ConfigFile = "Data\\RtkConfig_LSQ_COM.txt";     // mode1 最小二乘串口模式
        cout << "It is now the mode of real-time processing using the Least Squares solution.\n";
        break;
    case 2:
        ConfigFile = "Data\\RtkConfig_EKF_FILE.txt";    // mode2 卡尔曼滤波文件模式
        cout << "It is now in the mode of post-processing using the Kalman Filter.\n";
        break;
    case 3:
        ConfigFile = "Data\\RtkConfig_EKF_COM.txt";     // mode3 卡尔曼滤波串口模式
        cout << "It is now in the mode of real-time processing using the Kalman Filter.\n";
        break;
    }
    // 读取配置文件
    if (!ReadRTKConfigInfo(ConfigFile, &ANinfo))cout << "Failed to read Config File!\n";
    codemode = ANinfo.IsFileData;   // 文件模式 或 实时模式

    // 创建文件指针，假设您已经有了文件路径
    FILE* FRTK_EKF = NULL, * FRTK_LSQ = NULL;
    if (ANinfo.RTKProcMode == 1)FRTK_EKF = fopen("Result\\EKF\\RTKPlot.pos", "w");  // 卡尔曼滤波RTK绘图
    if (ANinfo.RTKProcMode == 2)FRTK_LSQ = fopen("Result\\LSQ\\RTKPlot.pos", "w");  // 最小二乘法RTK绘图
    FILE* FBase = fopen(ANinfo.BasObsDatFile, "rb");	// 基站数据文件
    FILE* FRover = fopen(ANinfo.RovObsDatFile, "rb");	// 流动站数据文件
    FILE* FSpp = fopen(ANinfo.SppFile, "w");		    // 单点定位结果数据文件
    FILE* FFloat = NULL, * FFixed = NULL, * FEkf = NULL;
    if (ANinfo.RTKProcMode == 2) FFloat = fopen(ANinfo.FloatFile, "w");		// 浮点解定位结果数据文件
    if (ANinfo.RTKProcMode == 2) FFixed = fopen(ANinfo.FixedFile, "w");		// 固定解定位结果数据文件
    if (ANinfo.RTKProcMode == 1)FEkf = fopen(ANinfo.EkfFile, "w");          // 卡尔曼定位结果数据文件
    
    // 检查文件是否成功打开
    if (ANinfo.IsFileData == 1) { if (FBase == NULL) { printf("Cannot open the BaseFile to read!\n"); return -1; } }
    if (ANinfo.IsFileData == 1) { if (FRover == NULL) { printf("Cannot open the RoverObsFile to read!\n"); return -1; } }
    if (FSpp == NULL) { printf("Cannot open the SppFile to save!\n"); return -1; }
    if (ANinfo.RTKProcMode == 2) {
        if (FFloat == NULL) { printf("Cannot open the FloatFile to save!\n"); return -1; }
        if (FFixed == NULL) { printf("Cannot open the FixedFile to save!\n"); return -1; }
        if (FRTK_LSQ == NULL) { printf("Cannot open the LsqFile to save!\n"); return -1; }
    }
    if (ANinfo.RTKProcMode == 1)if (FEkf == NULL) { printf("Cannot open the EkfFile to save!\n"); return -1; }
    if (ANinfo.RTKProcMode == 1)if (FRTK_EKF == NULL) { printf("Cannot open the EkfFile to save!\n"); return -1; }

    // (实时)获取串口数据流、选择读取时间
    SOCKET BaseCom, RoverCom;
    double realtime_minutes = 0, totalT = 0.0, epochnum = 0.0, t0 = 0.0;
    if (ANinfo.IsFileData == 0)
    {
        // 读取串口
        bool basecom = OpenSocket(BaseCom, ANinfo.BasNetIP, ANinfo.BasNetPort);
        bool rovercom = OpenSocket(RoverCom, ANinfo.RovNetIP, ANinfo.RovNetPort);
        if (basecom == false || rovercom == false)
        {
            printf("Cant open the Ip&Port.\n");
            return 0;
        }
        // 输入时间
        cout << "\nEnter the number of minutes for real-time data collection: ";
        cin >> realtime_minutes;
        if (realtime_minutes <= 0) { cout << "Invalid input for real-time collection time\n"; return -1; }
    }

    // 创建一个 RAWDAT 结构体实例
    RAWDAT rawdata;             // 卫星数据
    PPRESULT Bresult;           // 基站的SPP解算结果
    PPRESULT Rresult;           // 流动站的SSP解算结果
    FloatResult FloatResult;    // 浮动解的结果
    RTKEKF ekf;                 // 滤波数据
    int k = 0;                  // 用于输出到文件中

    while (1) {
        if (ANinfo.IsFileData == 0 && totalT >= realtime_minutes * 60.0)break;  // 实时的模式，判断时间是否超时

        // 1、时间同步
        int synflag = 0;
        if (ANinfo.IsFileData == 1)synflag = SynRTKObsPP(FBase, FRover, &rawdata);          // 文件模式
        else if (ANinfo.IsFileData == 0)synflag = SynRTKObsRT(BaseCom, RoverCom, &rawdata); // 实时模式
        if (synflag == 0)continue;      // 时间同步失败
        else if (synflag == -1)break;   // 文件结束

        // 2、基站的SPP解算，获得卫星位置
        DetectOutlier(&rawdata.BasEpk_cur);
        if (SPP(&rawdata.BasEpk_cur, &rawdata, &Bresult))SPV(&rawdata.BasEpk_cur, &Bresult);

        // 流动站的SPP解算，获得卫星位置
        DetectOutlier(&rawdata.RovEpk_cur);
        if (SPP(&rawdata.RovEpk_cur, &rawdata, &Rresult))SPV(&rawdata.RovEpk_cur, &Rresult);
        
        // 3、观测数据有效性标记
        MarkValid(&rawdata.BasEpk_0, &rawdata.BasEpk_cur);
        MarkValid(&rawdata.RovEpk_0, &rawdata.RovEpk_cur);

        // 4、计算单差观测值
        SDEphObs(&rawdata.RovEpk_cur, &rawdata.BasEpk_cur, &rawdata.SdObs);

        // 5、周跳探测
        DetectCycleSlip(&rawdata.SdObs);

        // 6、基准星选取
        DetRefSat(&rawdata.RovEpk_cur, &rawdata.BasEpk_cur, &rawdata.SdObs, &rawdata.DDObs);
        if (rawdata.DDObs.RefPrn[0] == -1 || rawdata.DDObs.RefPrn[1] == -1) // 基准星选取失败
        {
            if (Rresult.Position[0] == 0 && Rresult.Position[1] == 0 && Rresult.Position[2] == 0)continue;
            double dxyz[3] = { 0.0,0.0,0.0 };
            for (int i = 0; i < 3; i++)dxyz[i] = Rresult.Position[i] - Bresult.Position[i];
            if (ANinfo.IsFileData == 0)printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f PDOP: %0.3f  GPS: %d BDS: %d SigmaPos: %0.3f SPP\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                dxyz[0], dxyz[1], dxyz[2],
                Rresult.PDOP, Rresult.GPSSatNum, Rresult.BDSSatNum,
                Rresult.SigmaPos);

            continue; 
        }
        // 获取读取的总时间
        if (epochnum == 0) { t0 = rawdata.RovEpk_cur.Time.Week * 604800.0 + rawdata.RovEpk_cur.Time.SecOfWeek; epochnum++; }
        if (epochnum == 1) { totalT = rawdata.RovEpk_cur.Time.Week * 604800.0 + rawdata.RovEpk_cur.Time.SecOfWeek - t0; }

        switch (ANinfo.RTKProcMode) // 1EKF 2LSQ
        {
            // 最小二乘
        case 2: {
            // 1、浮点解计算
            bool Floatflag = RTKFloat(&rawdata, &Bresult, &Rresult, &FloatResult);
            if (!Floatflag) 
            {   // 浮点解失败则返回单点定位
                double dxyz[3] = { 0.0,0.0,0.0 };
                for (int i = 0; i < 3; i++)dxyz[i] = Rresult.Position[i] - Bresult.Position[i];
                if (ANinfo.IsFileData == 0)printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f PDOP: %0.3f  GPS: %d BDS: %d SigmaPos: %0.3f SPP\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                    dxyz[0], dxyz[1], dxyz[2],
                    Rresult.PDOP, Rresult.GPSSatNum, Rresult.BDSSatNum,
                    Rresult.SigmaPos);

                double SPP_BLH[3];  // 输出SPP到文件中
                XYZToBLH(Rresult.Position, SPP_BLH, R_WGS84, F_WGS84);
                fprintf(FRTK_LSQ, "%u %.f %0.18f %0.18f %0.18f 0 3\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                    SPP_BLH[0], SPP_BLH[1], SPP_BLH[2]);

                continue;
            } // 浮点解失败

            // 2、lambda固定
            if (lambda(rawdata.DDObs.Sats * 2, 2, FloatResult.N, FloatResult.Qn, rawdata.DDObs.FixedAmb, rawdata.DDObs.ResAmb) != 0)continue;   // lambda固定失败
            rawdata.DDObs.Ratio = rawdata.DDObs.ResAmb[1] / rawdata.DDObs.ResAmb[0];
            if (rawdata.DDObs.Ratio >= ANinfo.RatioThres)
            {
                if (rawdata.DDObs.Ratio >= 100)rawdata.DDObs.Ratio = 100;   // 大于100 赋值100
                rawdata.DDObs.bFixed = true;
            }
            else rawdata.DDObs.bFixed = false;

            // 3、固定解计算
            bool Fixedflag = RTKFix(&rawdata, &Bresult, &Rresult, &FloatResult);
            if (!Fixedflag) { printf("SecOfWeek: %d %f The calculation of fixedresult is fail!\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek); continue; } // 固定解失败

            if (k == 0) { fprintf(FSpp, "#Time X Y Z(m)\n"); }
            double SPPBasePos[3] = { 0.0,0.0,0.0 }, SPP_blh[3] = { 0.0,0.0,0.0 }, SPP_denu[3] = { 0.0,0.0,0.0 };
            BLHToXYZ(rawdata.BaseBest.Pos, SPPBasePos, R_WGS84, F_WGS84);   // 计算基站的XYZ坐标
            XYZToBLH(Rresult.Position, SPP_blh, R_WGS84, F_WGS84);          // 计算流动站BLH坐标
            GEOCOOR SPP_base;
            SPP_base.longitude = rawdata.BaseBest.Pos[0]; SPP_base.latitude = rawdata.BaseBest.Pos[1]; SPP_base.height = rawdata.BaseBest.Pos[2];
            CompEnudPos(SPPBasePos, Rresult.Position, &SPP_base, SPP_denu);
            ToFileSppResult(&rawdata, &Rresult, SPP_denu, FSpp);    // 打印SPP结果

            // 计算并保存浮点解denu
            double RoverFloatXYZ[3] = { 0.0,0.0,0.0 }, RoverFloatBLH[3] = { 0.0,0.0,0.0 }, BasePos[3] = { 0.0,0.0,0.0 }, dENU_float[3] = { 0.0,0.0,0.0 };
            BLHToXYZ(rawdata.BaseBest.Pos, BasePos, R_WGS84, F_WGS84);                              // 计算基站的XYZ坐标
            for (int j = 0; j < 3; j++) { RoverFloatXYZ[j] = FloatResult.dX[j] + BasePos[j]; }
            XYZToBLH(RoverFloatXYZ, RoverFloatBLH, R_WGS84, F_WGS84);
            GEOCOOR baseblh;
            baseblh.longitude = rawdata.BaseBest.Pos[0]; baseblh.latitude = rawdata.BaseBest.Pos[1]; baseblh.height = rawdata.BaseBest.Pos[2];
            CompEnudPos(BasePos, RoverFloatXYZ, &baseblh, dENU_float);

            if (k == 0) { fprintf(FFloat, "#Time X Y Z(m) dE dN dU(m) Sigma PDOP HDOP VDOP GPS BDS\n"); }
            ToFileFloatResult(&rawdata, RoverFloatXYZ, dENU_float, &FloatResult, FFloat);    // 输出浮点解结果

            // 计算并保存固定解denu
            double RoverFixedXYZ[3] = { 0.0,0.0,0.0 }, RoverFixedBLH[3] = { 0.0,0.0,0.0 }, dENU_fixed[3] = { 0.0,0.0,0.0 };
            for (int j = 0; j < 3; j++) { RoverFixedXYZ[j] = rawdata.DDObs.dPos[j] + BasePos[j]; } // 计算流动站XYZ坐标
            XYZToBLH(RoverFixedXYZ, RoverFixedBLH, R_WGS84, F_WGS84);                             // 计算流动站BLH坐标
            CompEnudPos(BasePos, RoverFixedXYZ, &baseblh, dENU_fixed);

            if (k == 0) { fprintf(FFixed, "#Time X Y Z(m) dE dN dU(m) RMS Ratio GPS BDS\n"); k++; }
            ToFileFixedResult(&rawdata, RoverFixedXYZ, dENU_fixed, FFixed);    // 输出固定解结果            

            // 计算并输出精确坐标dENU
            // 保存精准流动站的基线向量。
            double precisionBLH[3] = { PreciseL,PreciseB,PreciseH }, precisionXYZ[3] = { 0.0,0.0,0.0 }, dENU_precision[3] = { 0.0,0.0,0.0 };
            BLHToXYZ(precisionBLH, precisionXYZ, R_WGS84, F_WGS84);
            CompEnudPos(BasePos, precisionXYZ, &baseblh, dENU_precision);

            //printf("%0.12f %0.12f %0.12f\n", dENU_precision[0], dENU_precision[1], dENU_precision[2]);

            if (ANinfo.IsFileData == 0 && rawdata.DDObs.bFixed == true) printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f PDOP: %0.3f  GPS: %d BDS: %d ratio: %0.3f Fixed\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                rawdata.DDObs.dPos[0], rawdata.DDObs.dPos[1], rawdata.DDObs.dPos[2], FloatResult.PDOP,
                rawdata.DDObs.DDSatNum[0], rawdata.DDObs.DDSatNum[1], rawdata.DDObs.Ratio);
            if (ANinfo.IsFileData == 0 && rawdata.DDObs.bFixed == false) printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f PDOP: %0.3f GPS: %d BDS: %d ratio: %0.3f Float\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                FloatResult.dX[0], FloatResult.dX[1], FloatResult.dX[2], FloatResult.PDOP,
                rawdata.DDObs.DDSatNum[0], rawdata.DDObs.DDSatNum[1], rawdata.DDObs.Ratio);

            FigureRTKPlot(&rawdata, RoverFixedBLH, &ekf, RoverFixedBLH, FRTK_LSQ, 2);
            break;
        }
        case 1:
            // 1、将当前双差数据备份
            ekf.CurDDObs = rawdata.DDObs;   // 当前历元参考星

            // 2、初始化滤波
            if (!ekf.IsInit)
            {
                // 初始化后输出SPP结果
                double dxyz[3] = { 0.0,0.0,0.0 };
                for (int i = 0; i < 3; i++)dxyz[i] = Rresult.Position[i] - Bresult.Position[i];
                if (ANinfo.IsFileData == 0)printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f PDOP: %0.3f  GPS: %d BDS: %d SigmaPos: %0.3f SPP\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                    dxyz[0], dxyz[1], dxyz[2],
                    Rresult.PDOP, Rresult.GPSSatNum, Rresult.BDSSatNum,
                    Rresult.SigmaPos);

                //double SPP_BLH[3];  // 输出到SPP文件中
                //XYZToBLH(Rresult.Position, SPP_BLH, R_WGS84, F_WGS84);
                //fprintf(FRTK_EKF, "%u %.f %0.18f %0.18f %0.18f 0 3\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                //    SPP_BLH[0], SPP_BLH[1], SPP_BLH[2]);

                ekf.CurDDObs = rawdata.DDObs;   // 同步
                InitFilter(&rawdata, &Bresult, &Rresult, &ekf);
                ekf.SDObs = rawdata.SdObs;      // 备份单差顺序
                ekf.DDObs = rawdata.DDObs;      // 备份参考星信息
                continue;
            }

            // 时间预测
            if (!EkfPredict(&rawdata, &ekf))
            {   // 时间预测失败则返回SPP
                double dxyz[3] = { 0.0,0.0,0.0 };
                for (int i = 0; i < 3; i++)dxyz[i] = Rresult.Position[i] - Bresult.Position[i];
                if (ANinfo.IsFileData == 0)printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f PDOP: %0.3f  GPS: %d BDS: %d SigmaPos: %0.3f SPP\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                    dxyz[0], dxyz[1], dxyz[2],
                    Rresult.PDOP, Rresult.GPSSatNum, Rresult.BDSSatNum,
                    Rresult.SigmaPos);

                //double SPP_BLH[3];  // 输出SPP到文件中
                //XYZToBLH(Rresult.Position, SPP_BLH, R_WGS84, F_WGS84);
                //fprintf(FRTK_EKF, "%u %.f %0.18f %0.18f %0.18f 0 3\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                //    SPP_BLH[0], SPP_BLH[1], SPP_BLH[2]);

                continue;
            }

            // 测量更新
            if (!EkfMeasureUpdate(&rawdata, &ekf))
            {   // 测量更新失败则返回SPP
                double dxyz[3] = { 0.0,0.0,0.0 };
                for (int i = 0; i < 3; i++)dxyz[i] = Rresult.Position[i] - Bresult.Position[i];
                if (ANinfo.IsFileData == 0)printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f PDOP: %0.3f()  GPS: %d BDS: %d SigmaPos: %0.3f SPP\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                    dxyz[0], dxyz[1], dxyz[2],
                    Rresult.PDOP, Rresult.GPSSatNum, Rresult.BDSSatNum,
                    Rresult.SigmaPos);

                //double SPP_BLH[3];  // 输出到SPP文件中
                //XYZToBLH(Rresult.Position, SPP_BLH, R_WGS84, F_WGS84);
                //fprintf(FRTK_EKF, "%u %.f %0.18f %0.18f %0.18f 0 3\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                //    SPP_BLH[0], SPP_BLH[1], SPP_BLH[2]);

                continue;
            }

            // 拷贝当前单双差数据到滤波
            memcpy(&ekf.SDObs, &rawdata.SdObs, sizeof(ekf.SDObs));
            memcpy(&ekf.DDObs, &rawdata.DDObs, sizeof(ekf.DDObs));

            if (k == 0) { fprintf(FSpp, "#Time X Y Z(m)\n"); }
            double SPPBasePos[3] = { 0.0,0.0,0.0 }, SPP_blh[3] = { 0.0,0.0,0.0 }, SPP_denu[3] = { 0.0,0.0,0.0 };
            BLHToXYZ(rawdata.BaseBest.Pos, SPPBasePos, R_WGS84, F_WGS84);   // 计算基站的XYZ坐标
            XYZToBLH(Rresult.Position, SPP_blh, R_WGS84, F_WGS84);          // 计算流动站BLH坐标
            GEOCOOR SPP_base;
            SPP_base.longitude = rawdata.BaseBest.Pos[0]; SPP_base.latitude = rawdata.BaseBest.Pos[1]; SPP_base.height = rawdata.BaseBest.Pos[2];
            CompEnudPos(SPPBasePos, Rresult.Position, &SPP_base, SPP_denu);
            ToFileSppResult(&rawdata, &Rresult, SPP_denu, FSpp);    // 打印SPP结果

            // 保存定位结果
            double BasePos[3] = { 0.0,0.0,0.0 }, RoverFinal_blh[3] = { 0.0,0.0,0.0 }, dENU[3] = { 0.0,0.0,0.0 };
            BLHToXYZ(rawdata.BaseBest.Pos, BasePos, R_WGS84, F_WGS84);  // 计算基站的XYZ坐标
            XYZToBLH(ekf.X0, RoverFinal_blh, R_WGS84, F_WGS84);         // 计算流动站BLH坐标
            GEOCOOR baseblh;
            baseblh.longitude = rawdata.BaseBest.Pos[0]; baseblh.latitude = rawdata.BaseBest.Pos[1]; baseblh.height = rawdata.BaseBest.Pos[2];
            CompEnudPos(BasePos, ekf.X0, &baseblh, dENU);

            if (k == 0) { fprintf(FEkf, "#Time X Y Z(m) dE dN dU(m) Ratio GPS BDS\n"); k++; }
            ToFileEkfResult(&rawdata, &ekf, dENU, FEkf);    // 输出卡尔曼结果

            double dxyz[3] = { 0,0.0,0.0 };
            for (int i = 0; i < 3; i++)dxyz[i] = ekf.X0[i] - Bresult.Position[i];

            if (ANinfo.IsFileData == 0 && ekf.IsFixed == true && ekf.IsInit == true) printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f GPS: %d BDS: %d ratio: %0.3f Fixed\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                dxyz[0], dxyz[1], dxyz[2],
                ekf.CurDDObs.DDSatNum[0], ekf.CurDDObs.DDSatNum[1], ekf.ratio);
            if (ANinfo.IsFileData == 0 && ekf.IsFixed == false && ekf.IsInit == true) printf("Time %u %.f dX: %0.8f dY: %0.8f dZ: %0.8f GPS: %d BDS: %d ratio: %0.3f Float\n", rawdata.RovEpk_cur.Time.Week, rawdata.RovEpk_cur.Time.SecOfWeek,
                dxyz[0], dxyz[1], dxyz[2],
                ekf.CurDDObs.DDSatNum[0], ekf.CurDDObs.DDSatNum[1], ekf.ratio);

            FigureRTKPlot(&rawdata, RoverFinal_blh, &ekf, RoverFinal_blh, FRTK_EKF, 1);
            break;
        }

        //if (rawdata.RovEpk_cur.Time.SecOfWeek == 284255.0)break;
    }
    // 关闭文件
    if (ANinfo.IsFileData == 1)fclose(FBase);
    if (ANinfo.IsFileData == 1)fclose(FRover);
    fclose(FSpp);
    if (ANinfo.RTKProcMode == 2) fclose(FFloat);
    if (ANinfo.RTKProcMode == 2) fclose(FFixed);
    if (ANinfo.RTKProcMode == 1)fclose(FEkf);
    if (ANinfo.RTKProcMode == 1)fclose(FRTK_EKF);
    if (ANinfo.RTKProcMode == 2)fclose(FRTK_LSQ);
}