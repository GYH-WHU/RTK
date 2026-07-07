# RTK 实时动态定位算法

本仓库整理了基于 C++ 的 RTK 卫星导航算法实验代码，支持 GPS/BDS 双系统，包含 NovAtel OEM7 数据解码、SPP/SPV、站间单差、双差观测、周跳探测、LSQ 浮点/固定解、EKF 滤波解、LAMBDA 整周模糊度固定，以及 MATLAB 结果分析脚本和实验报告。

仓库当前包含源码、配置模板、已生成的 EKF/LSQ 结果和报告；原始基准站/流动站二进制观测数据未随仓库上传。若要重新运行文件模式，需要自行准备 `Base.bin` 和 `Rover_5002.bin` 等观测数据，并将配置路径调整到本地目录。

## 主要功能

- NovAtel OEM7 二进制数据解码：RANGE、GPS/BDS 星历和伪距定位结果。
- SPP/SPV：基准站和流动站单点定位、单点测速。
- RTK 数据同步：文件模式和实时 Socket 模式下的基准站/流动站观测同步。
- 观测预处理：粗差探测、有效性标记、站间单差、周跳探测、参考星选择。
- LSQ RTK：双差浮点解、LAMBDA 模糊度固定、固定解输出。
- EKF RTK：状态初始化、时间更新、量测更新和滤波结果输出。
- 结果分析：输出 SPP、浮点解、固定解、EKF 解和 RTKPlot 格式结果，MATLAB 支持误差与 Ratio 分析。

## 项目结构

```text
.
├─ 代码/
│  ├─ main.cpp                    # 交互式入口，选择 LSQ/EKF 与文件/实时模式
│  ├─ RTK_Structs.h               # GNSS/RTK 常量、结构体和函数声明
│  ├─ DecodeNovOem7Dat.cpp        # NovAtel OEM7 解码
│  ├─ SPPSPV.cpp                  # SPP 单点定位与 SPV 单点测速
│  ├─ RTK.cpp                     # RTK 同步、单差/双差、浮点解、固定解、EKF
│  ├─ lambda.cpp                  # LAMBDA 整周模糊度搜索
│  ├─ ErrorCorrect.cpp            # 电离层、对流层和粗差处理
│  ├─ PV_Clock.cpp                # GPS/BDS 卫星位置、速度、钟差计算
│  ├─ CoorTrans.cpp.cpp           # BLH/XYZ/ENU 与高度角方位角计算
│  ├─ MatrixCalculate.cpp         # 矩阵乘法与求逆
│  ├─ ReadConfig.cpp              # RTK 配置文件读取
│  ├─ Socket.cpp                  # 实时网络数据接收
│  ├─ OutPutResult.cpp            # SPP/Float/Fixed/EKF/RTKPlot 输出
│  ├─ TimeChange.cpp              # 通用时、MJD、GPS 时转换
│  └─ 配置文件/
│     ├─ RtkConfig_LSQ_FILE.txt
│     ├─ RtkConfig_LSQ_COM.txt
│     ├─ RtkConfig_EKF_FILE.txt
│     └─ RtkConfig_EKF_COM.txt
├─ 结果/
│  ├─ LSQ/                        # 已生成 LSQ SPP/Float/Fixed/RTKPlot 结果
│  ├─ EKF/                        # 已生成 EKF/SPP/RTKPlot 结果
│  ├─ main.m
│  ├─ dENU_fig.m
│  ├─ dENU_fig2.m
│  ├─ Ratio_fig.m
│  └─ ReadFile.m
├─ 报告/
│  ├─ 实验报告.pdf
│  └─ 图例.pptx
└─ README.md
```

## 编译

Windows + MinGW 示例：

```powershell
cd .\代码
g++ -std=c++11 .\main.cpp .\CoorTrans.cpp.cpp .\DecodeNovOem7Dat.cpp .\ErrorCorrect.cpp .\lambda.cpp .\MatrixCalculate.cpp .\OutPutResult.cpp .\PV_Clock.cpp .\ReadConfig.cpp .\RTK.cpp .\Socket.cpp .\SPPSPV.cpp .\TimeChange.cpp -I <Eigen路径> -lws2_32 -o rtk_solver.exe
```

Visual Studio 也可直接创建控制台工程，加入 `代码/*.cpp` 和 Eigen include path，并链接 `ws2_32.lib`。

## 运行说明

程序启动后选择配置模式：

```text
0  LSQ_FILE
1  LSQ_COM
2  EKF_FILE
3  EKF_COM
```

源码中的 `main.cpp` 默认读取：

```text
Data\RtkConfig_LSQ_FILE.txt
Data\RtkConfig_LSQ_COM.txt
Data\RtkConfig_EKF_FILE.txt
Data\RtkConfig_EKF_COM.txt
```

而仓库中配置模板位于：

```text
代码\配置文件\
```

因此重新运行前需要二选一：

1. 在运行目录下创建 `Data/`，把 `代码/配置文件/*.txt` 复制进去。
2. 或者修改 `main.cpp` 中的配置文件路径，直接指向 `代码/配置文件/`。

文件模式还需要准备配置中指定的原始观测文件，例如：

```text
Data\short-baseline-data\Base.bin
Data\short-baseline-data\Rover_5002.bin
```

配置中的主要输出路径为：

```text
Result\LSQ\SPP_FILE.txt
Result\LSQ\FLOAT_LSQ_FILE.txt
Result\LSQ\FIXED_LSQ_FILE.txt
Result\EKF\SPP_FILE.txt
Result\EKF\EKF_FILE.txt
```

当前仓库已经包含一组生成结果，位于 `结果/LSQ/` 和 `结果/EKF/`。

## MATLAB 结果分析

```matlab
cd 结果
main
```

菜单：

```text
0  LSQ 浮点解/固定解对比
1  SPP/EKF/LSQ 对比
2  Ratio 分析
```

注意：`结果/main.m` 中的部分路径指向带日期的子目录，如需直接复用当前 `结果/LSQ/` 与 `结果/EKF/` 文件，可按本地结果文件名调整 MATLAB 脚本路径。

## 代码模块

| 文件 | 说明 |
| --- | --- |
| `DecodeNovOem7Dat.cpp` | OEM7 RANGE、GPS/BDS 星历和接收机位置解码 |
| `SPPSPV.cpp` | 卫星发射时刻 PVT、SPP、SPV |
| `RTK.cpp` | 数据同步、单差/双差、参考星选择、RTK Float/Fix、EKF |
| `lambda.cpp` | 模糊度整数最小二乘搜索 |
| `ErrorCorrect.cpp` | Klobuchar、Hopfield、粗差探测 |
| `ReadConfig.cpp` | 读取数据源、处理模式、观测噪声、截止角和 Ratio 阈值 |
| `OutPutResult.cpp` | 输出 SPP、Float、Fixed、EKF 和 RTKPlot 文件 |

## 环境要求

- Windows
- C++11 编译器
- Eigen3
- Windows Socket 库
- MATLAB，用于结果分析

## 报告

```text
报告/实验报告.pdf
```

## 作者

GYH-WHU
