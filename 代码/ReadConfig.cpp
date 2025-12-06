#include"RTK_Structs.h"

/*-----------------------------------------------
	读取配置文件信息
------------------------------------------------*/
bool ReadRTKConfigInfo(const char FName[], ROVERCFGINFO* ANInfo)
{
    int i;
    FILE* Frin;
    char Line[256];
    const char CfgStr[CFGITEMS][32] = {
        "FROM FILE OR COM",             // 配置项1：指定是否为文件（FILE）或通信（COM）模式
        "RTK PROCESSING MODE",          // 配置项2：RTK处理模式
        "ROVER IP ADDRESS AND PORT",    // 配置项3：流动站的IP地址和端口
        "BASE IP ADDRESS AND PORT",     // 配置项4：基站的IP地址和端口
        "ROVER COM SETUP",              // 配置项5：流动站的串口设置
        "BASE OBSDATA SOURCE FILE",     // 配置项6：基站的观测数据源文件
        "ROVER OBSDATA SOURCE FILE",    // 配置项7：流动站的观测数据源文件
        "POSITION SPP FILE",            // 配置项8：SPP定位结果文件
        "POSITION FLOAT FILE",          // 配置项9：浮点解定位结果文件
        "POSITION FIXED FILE",          // 配置项10：固定解定位结果文件
        "POSITION EKF FILE",            // 配置项11：卡尔曼滤波定位结果文件
        "CODE AND CARRIER PHASE NOISE", // 配置项12：伪距噪声和载波相位噪声
        "THRESHOLD FOR ELEVATION MASK", // 配置项13：卫星高度角的阈值
        "RATIO FOR DD FIXED SOLUTION"   // 配置项14：差分固定解的比值阈值
    };

    // 打开配置文件
    if ((Frin = fopen(FName, "rt")) == NULL) {
        printf("ERROR: Can not Open SATODS configuration Info File: %s\n", FName);
        return false;
    }
    cout << "\nThe content of the configuration file is:\n";

    // 读取文件内容
    while (fgets(Line, sizeof(Line), Frin) != NULL) {
        Line[strcspn(Line, "\n")] = 0;  // 删除换行符
        char* comment_pos = strchr(Line, '!');
        if (comment_pos != NULL) *comment_pos = 0;   // 找到注释开始的地方并截断

        // 检查是否包含配置项
        for (i = 0; i < CFGITEMS; i++) {
            if (strstr(Line, CfgStr[i]) != NULL) {
                switch (i) {
                case 0:
                    // 使用 &ANInfo->IsFileData 来确保传递的是地址
                    if (sscanf(Line, "%*[^:]: %d", &ANInfo->IsFileData) != 1) {  // 读取文件或者通信模式
                        printf("ERROR: Invalid FILEDATA MODE configuration in file: %s\n", FName);
                        fclose(Frin);
                        return false;
                    }
                    printf("FILEDATA MODE: %d\n", ANInfo->IsFileData);  // 成功读取后输出
                    break;
                case 1:
                    if (sscanf(Line, "%*[^:]: %d", &ANInfo->RTKProcMode) != 1) {  // 读取RTK处理模式
                        printf("ERROR: Invalid RTK PROCESSING MODE configuration in file: %s\n", FName);
                        fclose(Frin);
                        return false;
                    }
                    printf("RTK PROCESSING MODE: %d\n", ANInfo->RTKProcMode);  // 成功读取后输出
                    break;
                    break;
                case 2:
                    if (ANInfo->IsFileData == 0) {
                        if (sscanf(Line, "%*[^:]: %s %d", ANInfo->RovNetIP, &ANInfo->RovNetPort) != 2) {   // 读取流动站的IP和端口
                            printf("ERROR: Invalid ROVER IP and PORT configuration in file: %s\n", FName);
                            fclose(Frin);
                            return false;
                        }
                        printf("ROVER IP: %s, PORT: %d\n", ANInfo->RovNetIP, ANInfo->RovNetPort);  // 成功读取后输出
                    }
                    break;
                case 3:
                    if (ANInfo->IsFileData == 0) {
                        if (sscanf(Line, "%*[^:]: %s %d", ANInfo->BasNetIP, &ANInfo->BasNetPort) != 2) {    // 读取基站的IP和端口
                            printf("ERROR: Invalid BASE IP and PORT configuration in file: %s\n", FName);
                            fclose(Frin);
                            return false;
                        }
                        printf("BASE IP: %s, PORT: %d\n", ANInfo->BasNetIP, ANInfo->BasNetPort);  // 成功读取后输出
                    }
                    break;
                case 4:
                    if (ANInfo->IsFileData == 0) {
                        if (sscanf(Line, "%*[^:]: %d %d", &ANInfo->RovPort, &ANInfo->RovBaud) != 2) {   // 读取流动站的串口设置
                            printf("ERROR: Invalid ROVER COM configuration in file: %s\n", FName);
                            fclose(Frin);
                            return false;
                        }
                        printf("ROVER COM PORT: %d, BAUD RATE: %d\n", ANInfo->RovPort, ANInfo->RovBaud);  // 成功读取后输出
                    }
                    break;
                case 5:
                    if (ANInfo->IsFileData == 1) {
                        if (sscanf(Line, "%*[^:]: %s", ANInfo->BasObsDatFile) != 1) {  // 基站观测数据文件
                            printf("ERROR: Invalid BASE OBSDATA configuration in file: %s\n", FName);
                            fclose(Frin);
                            return false;
                        }
                        printf("BASE OBS DATA FILE: %s\n", ANInfo->BasObsDatFile);  // 成功读取后输出
                    }
                    break;
                case 6:
                    if (ANInfo->IsFileData == 1) {
                        if (sscanf(Line, "%*[^:]: %s", ANInfo->RovObsDatFile) != 1) {  // 流动站观测数据文件
                            printf("ERROR: Invalid ROVER OBSDATA configuration in file: %s\n", FName);
                            fclose(Frin);
                            return false;
                        }
                        printf("ROVER OBS DATA FILE: %s\n", ANInfo->RovObsDatFile);  // 成功读取后输出
                    }
                    break;
                case 7:
                    if (sscanf(Line, "%*[^:]: %s", ANInfo->SppFile) != 1) {  // 单点定位结果数据文件
                        printf("ERROR: Invalid SPP FILE configuration in file: %s\n", FName);
                        fclose(Frin);
                        return false;
                    }
                    printf("SPP RESULT FILE: %s\n", ANInfo->SppFile);  // 成功读取后输出
                    break;
                case 8:
                    if (ANInfo->RTKProcMode == 2) {
                        if (sscanf(Line, "%*[^:]: %s", ANInfo->FloatFile) != 1) {  // 浮点解定位结果数据文件
                            printf("ERROR: Invalid FLOAT FILE configuration in file: %s\n", FName);
                            fclose(Frin);
                            return false;
                        }
                    }
                    printf("FLOAT RESULT FILE: %s\n", ANInfo->FloatFile);  // 成功读取后输出
                    break;
                case 9:
                    if (ANInfo->RTKProcMode == 2) {
                        if (sscanf(Line, "%*[^:]:%s", ANInfo->FixedFile) != 1) { // 固定解定位结果文件
                            printf("ERROR: Invalid FIXED FILE configuration in file: %s\n", FName);
                            fclose(Frin);
                            return false;
                        }
                    }
                    printf("FIXED RESULT FILE: %s\n", ANInfo->FixedFile);  // 成功读取后输出
                    break;
                    break;
                case 10:
                    if (ANInfo->RTKProcMode == 1) {
                        if (sscanf(Line, "%*[^:]: %s", ANInfo->EkfFile) != 1) {  // 浮点解定位结果数据文件
                            printf("ERROR: Invalid EKF FILE configuration in file: %s\n", FName);
                            fclose(Frin);
                            return false;
                        }
                    }
                    printf("EKF RESULT FILE: %s\n", ANInfo->FloatFile);  // 成功读取后输出
                    break;

                case 11:
                    if (sscanf(Line, "%*[^:]: %lf %lf", &ANInfo->CodeNoise, &ANInfo->CPNoise) != 2) {  // 伪距噪声和载波相位噪声
                        printf("ERROR: Invalid CODENOISE configuration in file: %s\n", FName);
                        fclose(Frin);
                        return false;
                    }
                    printf("CODE NOISE: %lf, CARRIER PHASE NOISE: %lf\n", ANInfo->CodeNoise, ANInfo->CPNoise);  // 成功读取后输出
                    break;
                case 12:
                    if (sscanf(Line, "%*[^:]: %lf", &ANInfo->ElevThreshold) != 1) {  // 高度角阈值
                        printf("ERROR: Invalid ELEVTHRESHOLD configuration in file: %s\n", FName);
                        fclose(Frin);
                        return false;
                    }
                    printf("ELEVATION THRESHOLD: %lf\n", ANInfo->ElevThreshold);  // 成功读取后输出
                    break;
                case 13:
                    if (sscanf(Line, "%*[^:]: %lf", &ANInfo->RatioThres) != 1) {  // Ratio检验阈值
                        printf("ERROR: Invalid RATIOTHRESHOLD configuration in file: %s\n", FName);
                        fclose(Frin);
                        return false;
                    }
                    printf("RATIO THRESHOLD: %lf\n", ANInfo->RatioThres);  // 成功读取后输出
                    break;
                default:
                    break;
                }
            }
        }
    }

    cout << "\nSuccessfully read the config file!\n";
    fclose(Frin);

    return true;
}
