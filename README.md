# RTK实时动态定位系统

## 项目简介

本项目是一个基于C++实现的RTK（实时动态定位）卫星导航算法系统，支持GPS和BDS双系统定位，提供单点定位（SPP）、浮点解和固定解等多种定位模式。系统支持文件模式和实时模式两种数据输入方式，并实现了最小二乘法（LSQ）和扩展卡尔曼滤波（EKF）两种RTK解算算法。

## 主要功能

### 1. 定位模式
- **单点定位（SPP）**：基于伪距观测值的单点定位
- **单点测速（SPV）**：基于多普勒观测值的速度解算
- **RTK浮点解**：基于载波相位观测值的相对定位浮点解
- **RTK固定解**：基于LAMBDA算法的整周模糊度固定解

### 2. 数据处理
- **数据解码**：支持NovAtel OEM7格式二进制数据解码
- **时间同步**：基准站与流动站观测数据的时间同步
- **周跳探测**：基于MW（Melbourne-Wübbena）和GF（Geometry-Free）组合的周跳探测
- **粗差探测**：观测值粗差检测与剔除
- **误差改正**：电离层延迟改正、对流层延迟改正

### 3. 算法实现
- **最小二乘法（LSQ）**：基于最小二乘的RTK浮点解和固定解
- **扩展卡尔曼滤波（EKF）**：基于卡尔曼滤波的RTK定位算法
- **LAMBDA算法**：整周模糊度快速解算算法

### 4. 坐标系统
- **坐标转换**：WGS84坐标系与大地坐标（BLH）之间的转换
- **ENU坐标**：站心地平坐标系（东-北-天）误差计算

## 项目结构

```
github/
├── 代码/                          # 源代码目录
│   ├── main.cpp                  # 主程序入口
│   ├── RTK_Structs.h             # 数据结构定义
│   ├── RTK.cpp                   # RTK算法实现
│   ├── SPPSPV.cpp                # 单点定位和测速
│   ├── DecodeNovOem7Dat.cpp      # NovAtel数据解码
│   ├── ErrorCorrect.cpp          # 误差改正
│   ├── CoorTrans.cpp.cpp         # 坐标转换
│   ├── TimeChange.cpp            # 时间转换
│   ├── MatrixCalculate.cpp       # 矩阵计算
│   ├── lambda.cpp                # LAMBDA算法
│   ├── Socket.cpp                # 网络通信
│   ├── ReadConfig.cpp            # 配置文件读取
│   ├── OutPutResult.cpp          # 结果输出
│   ├── PV_Clock.cpp              # 卫星位置、速度、钟差计算
│   └── 配置文件/                  # 配置文件目录
│       ├── RtkConfig_EKF_COM.txt # EKF实时模式配置
│       ├── RtkConfig_EKF_FILE.txt# EKF文件模式配置
│       ├── RtkConfig_LSQ_COM.txt # LSQ实时模式配置
│       └── RtkConfig_LSQ_FILE.txt# LSQ文件模式配置
├── 结果/                          # 结果输出目录
│   ├── EKF/                      # EKF算法结果
│   │   ├── EKF.txt              # EKF定位结果
│   │   ├── SPP.txt               # 单点定位结果
│   │   └── RTKPlot.pos           # RTKPlot绘图文件
│   ├── LSQ/                      # LSQ算法结果
│   │   ├── LSQ_FIXED.txt         # 固定解结果
│   │   ├── LSQ_FLOAT.txt         # 浮点解结果
│   │   ├── SPP.txt               # 单点定位结果
│   │   └── RTKPlot.pos           # RTKPlot绘图文件
│   └── *.m                       # MATLAB绘图脚本
├── 报告/                          # 实验报告
│   ├── 实验报告.pdf
│   └── 图例.pptx
└── README.md                      # 本文件
```

## 依赖环境

### 必需依赖
- **C++编译器**：支持C++11标准的编译器（如Visual Studio 2015+、GCC 4.8+）
- **Eigen库**：用于矩阵运算（版本3.3+）
- **Windows Socket**：用于网络通信（Windows平台）

### 可选依赖
- **MATLAB**：用于结果可视化和分析（可选）

## 编译方法

### Windows平台（Visual Studio）

1. 安装Visual Studio 2015或更高版本
2. 下载并配置Eigen库：
   - 从 [Eigen官网](https://eigen.tuxfamily.org/) 下载Eigen库
   - 将Eigen库解压到项目目录或系统包含路径

3. 创建Visual Studio项目：
   - 新建C++控制台应用程序项目
   - 将所有`.cpp`和`.h`文件添加到项目
   - 配置项目属性：
     - C/C++ → 常规 → 附加包含目录：添加Eigen库路径
     - C/C++ → 语言 → C++语言标准：选择C++11或更高
     - 链接器 → 输入 → 附加依赖项：添加`ws2_32.lib`（用于Socket）

4. 编译项目

### Linux平台（GCC）

```bash
# 安装Eigen库（Ubuntu/Debian）
sudo apt-get install libeigen3-dev

# 编译
g++ -std=c++11 -O2 -I/path/to/eigen \
    main.cpp RTK.cpp SPPSPV.cpp DecodeNovOem7Dat.cpp \
    ErrorCorrect.cpp CoorTrans.cpp.cpp TimeChange.cpp \
    MatrixCalculate.cpp lambda.cpp Socket.cpp ReadConfig.cpp \
    OutPutResult.cpp PV_Clock.cpp \
    -o rtk_solver -lws2_32  # Windows需要，Linux不需要
```

## 使用方法

### 1. 配置文件准备

系统提供四种配置模式，对应不同的配置文件：

- **LSQ_FILE**：最小二乘法 + 文件模式
- **LSQ_COM**：最小二乘法 + 实时模式
- **EKF_FILE**：扩展卡尔曼滤波 + 文件模式
- **EKF_COM**：扩展卡尔曼滤波 + 实时模式

配置文件格式示例（`RtkConfig_LSQ_FILE.txt`）：

```
FROM FILE OR COM:		1	! 0=COM; 1=FILE; 2=Net
RTK PROCESSING MODE:		2	! 1=EKF; 2=LSQ
ROVER IP ADDRESS AND PORT:
BASE IP ADDRESS AND PORT:
ROVER COM SETUP:
BASE OBSDATA SOURCE FILE:	Data\short-baseline-data\Base.bin
ROVER OBSDATA SOURCE FILE:	Data\short-baseline-data\Rover_5002.bin
POSITION SPP FILE:		Result\LSQ\SPP_FILE.txt
POSITION FLOAT FILE:		Result\LSQ\FLOAT_LSQ_FILE.txt
POSITION FIXED FILE:		Result\LSQ\FIXED_LSQ_FILE.txt
POSITION EKF FILE:
CODE AND CARRIER PHASE NOISE:	0.5 0.002
THRESHOLD FOR ELEVATION MASK:	1.0! 高度截止角（度）
RATIO FOR DD FIXED SOLUTION:	3.0
EOF
```

### 2. 运行程序

编译完成后，运行可执行文件：

```bash
./rtk_solver  # Linux
rtk_solver.exe  # Windows
```

程序启动后会提示选择配置模式：

```
Please enter the configuration file you want to read:
0:LSQ_FILE  1:LSQ_COM
2:EKF_FILE  3:EKF_COM
```

输入对应的数字（0-3）选择配置模式。

### 3. 实时模式

如果选择实时模式（LSQ_COM或EKF_COM），程序会提示输入实时数据采集时间（分钟）：

```
Enter the number of minutes for real-time data collection: 
```

输入采集时间后，程序将开始从网络接收数据并进行实时解算。

### 4. 结果输出

程序运行完成后，结果文件将保存在`结果/`目录下：

- **SPP结果**：单点定位结果，包含时间、XYZ坐标、ENU误差等
- **浮点解结果**：RTK浮点解结果，包含基线向量、模糊度、精度指标等
- **固定解结果**：RTK固定解结果，包含固定后的基线向量、Ratio值等
- **RTKPlot.pos**：用于RTKPlot软件绘图的定位结果文件

## 配置参数说明

| 参数 | 说明 | 单位 |
|------|------|------|
| FROM FILE OR COM | 数据源模式：0=串口，1=文件，2=网络 | - |
| RTK PROCESSING MODE | RTK处理模式：1=EKF，2=LSQ | - |
| ROVER IP ADDRESS AND PORT | 流动站IP地址和端口（实时模式） | - |
| BASE IP ADDRESS AND PORT | 基准站IP地址和端口（实时模式） | - |
| BASE OBSDATA SOURCE FILE | 基准站观测数据文件路径（文件模式） | - |
| ROVER OBSDATA SOURCE FILE | 流动站观测数据文件路径（文件模式） | - |
| CODE AND CARRIER PHASE NOISE | 伪距和载波相位观测值噪声 | m, 周 |
| THRESHOLD FOR ELEVATION MASK | 高度角截止阈值 | 度 |
| RATIO FOR DD FIXED SOLUTION | 双差固定解Ratio检验阈值 | - |

## 算法原理

### 1. 单点定位（SPP）

基于伪距观测值的非线性最小二乘解算，通过迭代求解接收机位置和钟差。

### 2. RTK浮点解

- 构建双差观测方程
- 使用最小二乘法或卡尔曼滤波估计基线向量和双差模糊度
- 考虑伪距和载波相位的权重分配

### 3. RTK固定解

- 使用LAMBDA算法对浮点模糊度进行整数估计
- 通过Ratio检验判断模糊度固定是否可靠
- 固定模糊度后重新解算基线向量

### 4. 扩展卡尔曼滤波（EKF）

- 状态向量：接收机位置（3维）+ 双差模糊度（2N维，N为卫星数）
- 时间更新：预测状态和协方差矩阵
- 测量更新：利用双差观测值更新状态估计
- 模糊度固定：在测量更新后进行LAMBDA固定

## 结果分析

### MATLAB可视化

项目提供了MATLAB脚本用于结果可视化：

- `main.m`：主绘图脚本
- `dENU_fig.m`：绘制ENU方向误差图
- `Ratio_fig.m`：绘制Ratio值变化图
- `ReadFile.m`：读取结果文件

使用方法：

```matlab
cd 结果
main  % 运行主脚本
```

## 注意事项

1. **数据格式**：输入数据必须为NovAtel OEM7格式的二进制文件
2. **坐标系**：系统使用WGS84坐标系
3. **时间系统**：使用GPS时间系统，BDS时间会自动转换为GPS时间
4. **网络模式**：实时模式需要确保网络连接正常，IP地址和端口配置正确
5. **结果目录**：确保`结果/`目录存在，否则程序可能无法保存结果

## 常见问题

### Q1: 编译时找不到Eigen库
**A**: 检查Eigen库路径是否正确配置在编译器的包含目录中。

### Q2: 实时模式无法连接
**A**: 检查网络配置、IP地址和端口是否正确，确保防火墙允许连接。

### Q3: 定位结果精度较差
**A**: 检查观测数据质量、卫星数量、高度角阈值设置等参数。

### Q4: 模糊度无法固定
**A**: 检查Ratio阈值设置、观测数据连续性、周跳探测是否正常。

## 作者信息

本项目为卫星导航算法与程序设计课程作业。

## 许可证

本项目仅供学习和研究使用。

## 更新日志

- **v1.0**：初始版本，实现基本的RTK定位功能
  - 支持GPS和BDS双系统
  - 实现LSQ和EKF两种算法
  - 支持文件和实时两种模式

---

如有问题或建议，欢迎反馈！

