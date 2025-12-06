#include"RTK_Structs.h"

extern ROVERCFGINFO ANinfo;

/*-----------------------------------------------
	数据时间同步函数（文件）
------------------------------------------------*/
int SynRTKObsPP(FILE* FBase, FILE* FRover, RAWDAT* Raw)
{
	/* 1. 获取流动站的观测数据 */
	// 保存前一个历元的数据
	memcpy(&Raw->BasEpk_0, &Raw->BasEpk_cur, sizeof(Raw->BasEpk_0));
	memcpy(&Raw->RovEpk_0, &Raw->RovEpk_cur, sizeof(Raw->RovEpk_0));

	// 解码流动站的数据
	static unsigned char buff_R[MAXRAWLEN];
	static unsigned char buff_D[MAXRAWLEN];
	static int lenD_R = 0;	static int lenD_B = 0;	// 有效数据长度
	int lenR_R = 0; int lenR_B = 0;					// 剩余数据长度
	double dt;				// 时间差

	while (!feof(FRover))
	{
		// 读取流动站数据
		lenR_R = fread(buff_R + lenD_R, sizeof(unsigned char), MAXRAWLEN - lenD_R, FRover);
		if (lenR_R < MAXRAWLEN - lenD_R)
			return -1;	// 文件读取到尾

		lenD_R += lenR_R;

		// 解码流动站数据
		if (DecodeNovOem7Dat(buff_R, lenD_R, &Raw->RovEpk_cur, Raw->GpsEph, Raw->BdsEph, &Raw->RoverBest) == 0)
			break;	// 解码成功
	}
	if (feof(FRover))
	{
		cout << "The file of Rover comes to end!\n";
		return -1;
	}

	/* 2. 比较观测时刻的差值 */
	// 计算流动站和基站的时间差
	dt = (Raw->RovEpk_cur.Time.Week - Raw->BasEpk_cur.Time.Week) * 604800.0
		+ Raw->RovEpk_cur.Time.SecOfWeek - Raw->BasEpk_cur.Time.SecOfWeek;

	// ① 若时间差在限差内，则同步成功
	if (fabs(dt) < 0.01) return 1;

	// ② 流动站数据滞后，无法解算
	else if (dt < 0) return 0;

	// ③ 基站数据滞后，循环读取基站数据
	while (!feof(FBase))
	{
		// 读取基站数据
		lenR_B = fread(buff_D + lenD_B, sizeof(unsigned char), MAXRAWLEN - lenD_B, FBase);
		if (lenR_B < MAXRAWLEN - lenD_B)
			return -1;	// 解码失败

		lenD_B += lenR_B;

		// 解码基站数据
		if (DecodeNovOem7Dat(buff_D, lenD_B, &Raw->BasEpk_cur, Raw->GpsEph, Raw->BdsEph, &Raw->BaseBest) == 0)
		{
			// 计算流动站和基站的时间差
			dt = (Raw->RovEpk_cur.Time.Week - Raw->BasEpk_cur.Time.Week) * 604800.0
				+ Raw->RovEpk_cur.Time.SecOfWeek - Raw->BasEpk_cur.Time.SecOfWeek;

			// 若时间差在限差内，则同步成功
			if (fabs(dt) < 0.01) return 1;

			// 流动站数据滞后，无法解算
			else if (dt < 0) return 0;

			// 否则继续解算基站数据
		}
	}
	if (feof(FBase))
	{
		cout << "The file of Base comes to end!\n";
		return -1;
	}

	return 0;	// 数据同步失败
}

/*-----------------------------------------------
	数据时间同步函数（实时）
------------------------------------------------*/
int SynRTKObsRT(SOCKET& BaseCom, SOCKET& RoverCom, RAWDAT* Raw)
{
	/* 1. 获取流动站的观测数据 */
	// 保存前一个历元的数据
	memcpy(&Raw->BasEpk_0, &Raw->BasEpk_cur, sizeof(Raw->BasEpk_0));
	memcpy(&Raw->RovEpk_0, &Raw->RovEpk_cur, sizeof(Raw->RovEpk_0));

	Sleep(980);

	// 解码流动站的数据
	static unsigned char buff_R[MAXRAWLEN * 2];
	static int RlenB = 0;							// RBuf的长度(字节数)
	unsigned char Rbuf[MAXRAWLEN];
	int RlenT = 0;
	if ((RlenT = recv(RoverCom, (char*)Rbuf, MAXRAWLEN, 0)) > 0)
	{
		if ((RlenB + RlenT) > MAXRAWLEN * 2)RlenB = 0;	// 如果本次读入的数据加入buff_R中会超过其最大长度，则先将原有的数据舍弃
		memcpy(buff_R + RlenB, Rbuf, RlenT);
		RlenB = RlenT + RlenB;
		if (DecodeNovOem7Dat(buff_R, RlenB, &Raw->RovEpk_cur, Raw->GpsEph, Raw->BdsEph, &Raw->RoverBest) != 0)
		{
			cout << "Failed to decode roverdata.\n";
			return 0;
		}
	}

	// 解码基站数据
	static unsigned char buff_B[MAXRAWLEN * 2];
	static int BlenB = 0;							// BBuf的长度(字节数)
	unsigned char Bbuf[MAXRAWLEN];
	int BlenT = 0;
	if ((BlenT = recv(BaseCom, (char*)Bbuf, MAXRAWLEN, 0)) > 0)
	{
		if ((BlenB + BlenT) > MAXRAWLEN * 2)BlenB = 0;	// 如果本次读入的数据加入buff_B中会超过其最大长度，则先将原有的数据舍弃
		memcpy(buff_B + BlenB, Bbuf, BlenT);
		BlenB = BlenT + BlenB;
		if (DecodeNovOem7Dat(buff_B, BlenB, &Raw->BasEpk_cur, Raw->GpsEph, Raw->BdsEph, &Raw->BaseBest) != 0)
		{
			cout << "Failed to decode basedata.\n";
			return 0;
		}
	}

	// 求两站时间的差
	double dt = (Raw->RovEpk_cur.Time.Week - Raw->BasEpk_cur.Time.Week) * 604800.0
		+ Raw->RovEpk_cur.Time.SecOfWeek - Raw->BasEpk_cur.Time.SecOfWeek;

	if (fabs(dt) < 5.0 && (Raw->RovEpk_cur.Time.SecOfWeek > Raw->RovEpk_0.Time.SecOfWeek))return 1;
	else
	{
		printf("Time %u %.f Time syn failed, dt is: %.1f\n", Raw->RovEpk_cur.Time.Week, Raw->RovEpk_cur.Time.SecOfWeek, dt);
		return 0;
	}
}

/*-----------------------------------------------
	观测数据有效性标记函数
------------------------------------------------*/
void MarkValid(const EPOCHOBS* EpkLast, EPOCHOBS* EpkCur)
{
	// 循环寻找本颗卫星在上一个历元中的数据
	for (int i = 0; i < EpkCur->SatNum; i++)
	{
		bool flag[2];	// 两个频率的有效性标志
		flag[0] = flag[1] = false;

		// 遍历上一个历元的数据
		for (int j = 0; j < EpkLast->SatNum; j++)
		{
			// 找到相同系统、相同卫星号的卫星数据
			if (EpkLast->SatObs[j].System == EpkCur->SatObs[i].System && EpkLast->SatObs[j].Prn == EpkCur->SatObs[i].Prn)
			{
				// 对两个频率的LockTime都进行检查
				for (int n = 0; n < 2; n++)
				{
					// 如果上一个历元缺失或者失锁，则本历元的连续跟踪时间需要大于等于6秒，才认为有效
					if (EpkLast->SatObs[j].LockTime[n] == 0.0)
					{
						if (EpkCur->SatObs[i].LockTime[n] >= 6.0)flag[n] = true;
					}

					// 如果上一个历元存在，则计算时间差，dt≥0则认为有效
					else
					{
						double dt = EpkCur->SatObs[i].LockTime[n] - EpkLast->SatObs[j].LockTime[n];
						if (dt >= 0)flag[n] = true;
					}
				}
				break;
			}
		}

		// 只有双频的flag都是有效时，才将该数据标记为true（前面已经使用周跳探测进行初步的valid标记）
		if (EpkCur->SatObs[i].Valid == true)
		{
			if (!(flag[0] && flag[1]))
			{
				EpkCur->SatObs[i].Valid = false;	// 需要同时为真
			}
		}
	}
}

/*-----------------------------------------------
	基站与流动站 站间单差函数
------------------------------------------------*/
void SDEphObs(const EPOCHOBS* EpkR, const EPOCHOBS* EpkB, SDEPOCHOBS* SDObs)
{
	// 初始化SDObs中的数据
	memset(SDObs->SdSatObs, 0, sizeof(SDObs->SdSatObs));

	// 单差观测数据的可用卫星数量
	int SatNum = 0;

	// 根据流动站卫星数进行循环，对于每颗流动站的卫星
	for (int i = 0; i < EpkR->SatNum; i++)
	{
		// 1. 检查卫星号和系统号是否正常，卫星观测值是否完整，如果不正常，continue；
		if (EpkR->SatObs[i].Valid == false || EpkR->SatPVT[i].Valid == false)continue;

		// 2. 在基站观测值中查找与该流动站相同的卫星，如果未找到，continue；
		for (int j = 0; j < EpkB->SatNum; j++)
		{
			// 若基站的观测数据有效性不达标则continue
			if (EpkB->SatObs[j].Valid == false || EpkB->SatPVT[j].Valid == false)continue;

			// 3. 对同类型和同频率的观测值求差并保存，保存索引号、卫星号和系统号等相关信息，累加单差观测值卫星数量
			else if (EpkB->SatObs[j].System == EpkR->SatObs[i].System && EpkB->SatObs[j].Prn == EpkR->SatObs[i].Prn)
			{
				SDObs->SdSatObs[SatNum].System = EpkR->SatObs[i].System;
				SDObs->SdSatObs[SatNum].Prn = EpkR->SatObs[i].Prn;
				SDObs->SdSatObs[SatNum].nRover = i;	// 流动站的索引号
				SDObs->SdSatObs[SatNum].nBase = j;	// 基站的索引号

				// 计算站间单差
				for (int k = 0; k < 2; k++)
				{
					// 伪距单差，双频都有伪距观测值才做单差
					if (fabs(EpkR->SatObs[i].P[k]) > 1e-8 && fabs(EpkB->SatObs[j].P[k]) > 1e-8)
						SDObs->SdSatObs[SatNum].dP[k] = EpkR->SatObs[i].P[k] - EpkB->SatObs[j].P[k];
					else SDObs->SdSatObs[SatNum].dP[k] = 0;

					// 载波相位单差，双频都有载波相位观测值才做单差
					if (fabs(EpkR->SatObs[i].L[k]) > 1e-8 && fabs(EpkB->SatObs[j].L[k]) > 1e-8)
						SDObs->SdSatObs[SatNum].dL[k] = EpkR->SatObs[i].L[k] - EpkB->SatObs[j].L[k];
					else SDObs->SdSatObs[SatNum].dL[k] = 0;

					// 半周标记
					if (EpkR->SatObs[i].half[k] && EpkB->SatObs[j].half[k] == 1)SDObs->SdSatObs[SatNum].half[k] = 1;	// 无半周
					else SDObs->SdSatObs[SatNum].half[k] = 0;	// 存在半周
				}
				// 累加单差观测值卫星数量
				SatNum++;
				break;
			}
			else continue;
		}
	}
	// 4. 循环结束后，对单差观测值的观测时刻和卫星数量赋值。
	SDObs->Time = EpkR->Time;
	SDObs->SatNum = SatNum;
}

/*-----------------------------------------------
	单差观测值的周跳探测函数
------------------------------------------------*/
void DetectCycleSlip(SDEPOCHOBS* Obs)
{
	// 存放当前历元的单差组合观测值
	MWGF SDComObs[MAXCHANNUM];
	memset(SDComObs, 0, sizeof(SDComObs));
	for (int i = 0; i < Obs->SatNum; i++)
	{
		// 默认无效，只有通过检测才有效
		Obs->SdSatObs[i].Valid = 0;

		// 1、检查该卫星的单差数据是否有效和完整，若不全或为0，将Valid标记为false，continue
		if (fabs(Obs->SdSatObs[i].dL[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dL[1]) < 1e-8 || fabs(Obs->SdSatObs[i].dP[0]) < 1e-8 || fabs(Obs->SdSatObs[i].dP[1]) < 1e-8)
		{
			Obs->SdSatObs[i].Valid = 0;
			continue;
		}
		// 记录系统与卫星号
		SDComObs[i].Sys = Obs->SdSatObs[i].System;
		SDComObs[i].Prn = Obs->SdSatObs[i].Prn;

		// 2、计算当前历元该卫星的GF和MW组合值
		SDComObs[i].GF = Obs->SdSatObs[i].dL[0] - Obs->SdSatObs[i].dL[1];
		if (SDComObs[i].Sys == GPS)	// 如果是GPS系统
		{
			SDComObs[i].MW = (FG1_GPS * Obs->SdSatObs[i].dL[0] - FG2_GPS * Obs->SdSatObs[i].dL[1]) / (FG1_GPS - FG2_GPS)
				- (FG1_GPS * Obs->SdSatObs[i].dP[0] + FG2_GPS * Obs->SdSatObs[i].dP[1]) / (FG1_GPS + FG2_GPS);
		}
		else if (SDComObs[i].Sys == BDS)	// 如果是BDS系统
		{
			SDComObs[i].MW = (FG1_BDS * Obs->SdSatObs[i].dL[0] - FG3_BDS * Obs->SdSatObs[i].dL[1]) / (FG1_BDS - FG3_BDS)
				- (FG1_BDS * Obs->SdSatObs[i].dP[0] + FG3_BDS * Obs->SdSatObs[i].dP[1]) / (FG1_BDS + FG3_BDS);
		}
		else
		{
			Obs->SdSatObs[i].Valid = 0;	// 不是GPS或者BDS系统则单差观测值无效
			continue;
		}

		SDComObs[i].n = 1;
		// 3、从上个历元的MWGF数据中查找该卫星的GF和MW组合值
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			if (Obs->SdCObs[j].Sys == SDComObs[i].Sys && Obs->SdCObs[j].Prn == SDComObs[i].Prn)
			{
				// 4、计算当前历元该卫星GF与上一历元对应GF的差值dGF，计算当前历元该卫星MW与上一历元对应MW平滑值的差值dMW
				double dGF = fabs(SDComObs[i].GF - Obs->SdCObs[j].GF);
				double dMW = fabs(SDComObs[i].MW - Obs->SdCObs[j].MW);

				// 5、检查dGF和dMW是否超限，限差阈值为5cm和3m。若超限，标记为粗差，将Valid标记为false，若不超限，标记为可用将Valid标记为true
				if (dGF < GFThres && dMW < MWThres)
				{
					Obs->SdSatObs[i].Valid = 1;
					// 计算该卫星的MW平滑值
					SDComObs[i].MW = (Obs->SdCObs[j].MW * Obs->SdCObs[j].n + SDComObs[i].MW) / (Obs->SdCObs[j].n + 1);
					SDComObs[i].n = Obs->SdCObs[j].n + 1;
				}
				// 超限则标记为false
				else
				{
					Obs->SdSatObs[i].Valid = 0;
					cout << "Second Of Week: " << Obs->Time.SecOfWeek << " Satllite: " << ((SDComObs[i].Sys == 1) ? 'G' : (SDComObs[i].Sys == 2) ? 'B' : '?') << SDComObs[i].Prn << " GF or MW is Error" << "\n";
				}
				break;
			}
		}
	}
	// 6、将缓冲区组合观测值的拷贝到obs里
	memcpy(Obs->SdCObs, SDComObs, sizeof(SDComObs));
}

/*-----------------------------------------------
	选取基准卫星函数
------------------------------------------------*/
void DetRefSat(const EPOCHOBS* EpkRover, const EPOCHOBS* EpkBase, SDEPOCHOBS* SDObs, DDCOBS* DDObs)
{
	if (SDObs->SatNum == 0)return;
	static int LastRefPrn[2] = { -1,-1 };	// 保存上一个历元的参考星PRN
	int N[2] = { 0,0 };				// 每个历元中GPS和BDS的卫星计数
	int Id[2][MAXCHANNUM];			// 两个系统的卫星索引
	double Elev[2][MAXCHANNUM];		// 两个系统的高度角
	double CN_avg[2][MAXCHANNUM];	// 两个系统的平均信噪比
	int RankE[2][MAXCHANNUM];		// 两个系统的高度角排名
	int RankC[2][MAXCHANNUM];		// 两个系统的信噪比排名
	// 初始化
	memset(Id, 0, sizeof(Id));
	memset(Elev, 0, sizeof(Elev));
	memset(CN_avg, 0, sizeof(CN_avg));
	memset(RankE, 0, sizeof(RankE));
	memset(RankC, 0, sizeof(RankC));

	// 1、收集候选卫星
	for (int i = 0; i < SDObs->SatNum; i++)
	{
		// 1、判断卫星是否通过周跳检测、粗差、半周检测
		if (SDObs->SdSatObs[i].Valid == false || SDObs->SdSatObs[i].half[0] == 0 || SDObs->SdSatObs[i].half[1] == 0)
			continue;

		int iR = SDObs->SdSatObs[i].nRover;
		int iB = SDObs->SdSatObs[i].nBase;
		// 2、判断卫星星历（单差数据保存与原始观测数据保存形式不同）（其中包含判断连续观测时间是否大于6秒）
		if (EpkRover->SatPVT[iR].Valid == false || EpkBase->SatPVT[iB].Valid == false)
			continue;
		if (EpkRover->SatPVT[iR].Elevation == 0 || EpkRover->SatObs[iR].cn0[0] == 0 || EpkRover->SatObs[iR].cn0[1] == 0)continue;

		// 存储不同系统的卫星号、高度角、平均信噪比
		short Sys = -1;	// 卫星的系统 0为GPS 1为BDS
		if (SDObs->SdSatObs[i].System == GPS)Sys = 0;
		else if (SDObs->SdSatObs[i].System == BDS)Sys = 1;
		else continue;

		if (N[Sys] < MAXCHANNUM)
		{
			Id[Sys][N[Sys]] = i;	// 索引
			Elev[Sys][N[Sys]] = EpkRover->SatPVT[iR].Elevation;	// 高度角
			CN_avg[Sys][N[Sys]] = (EpkRover->SatObs[iR].cn0[0] + EpkRover->SatObs[iR].cn0[1]) / 2.0;	// 平均信噪比
			N[Sys]++;	// 卫星计数+1
			if (N[Sys] >= MAXCHANNUM)break;
		}
	}

	// 为每个系统选择参考星
	for (int k = 0; k < 2; k++) // 0=GPS, 1=BDS
	{
		// 若没有存储卫星则继续寻找
		if (N[k] == 0)
		{
			DDObs->RefPrn[k] = -1; // 无候选卫星
			continue;
		}
		// 3、高度角排名
		for (int a = 0; a < N[k]; a++)
		{
			int r = 1;
			for (int b = 0; b < N[k]; b++)
			{
				if (Elev[k][b] > Elev[k][a])r++;
			}
			RankE[k][a] = r;
		}

		// 4、信噪比排名
		for (int a = 0; a < N[k]; a++)
		{
			int r = 1;
			for (int b = 0; b < N[k]; b++)
			{
				if (CN_avg[k][b] > CN_avg[k][a])r++;
			}
			RankC[k][a] = r;
		}

		// 5、选择名次最小；并列时优先排CN0
		int best = 0;
		int continuity = 0;
		// 如果该参考星和上一个是同一个，那么给连续性加分（减排名）
		if (SDObs->SdSatObs[Id[k][0]].Prn == LastRefPrn[k] && SDObs->SdSatObs[Id[k][0]].Prn != -1 && LastRefPrn[k] != -1)continuity = -1;
		int BestSum = RankE[k][0] + RankC[k][0] + continuity;

		for (int a = 1; a < N[k]; a++)
		{
			if (SDObs->SdSatObs[Id[k][a]].Prn == LastRefPrn[k] && SDObs->SdSatObs[Id[k][a]].Prn != -1 && LastRefPrn[k] != -1)continuity = -1;
			int sum = RankE[k][a] + RankC[k][a] + continuity;	// 求下一个卫星的高度角、信噪比排名的和
			if (sum < BestSum)	// 若和更低则更新
			{
				best = a;
				BestSum = sum;
			}
			else if (sum == BestSum)	// 若和一样则比较信噪比
			{
				if (CN_avg[k][a] >= CN_avg[k][best])best = a;
			}
		}

		// 6、记录该系统的参考星的卫星号
		int sdIdx = Id[k][best];
		int iR = SDObs->SdSatObs[sdIdx].nRover;
		int iB = SDObs->SdSatObs[sdIdx].nBase;

		double quality = EpkRover->SatPVT[iR].Elevation + EpkRover->SatObs[iR].cn0[0] + EpkRover->SatObs[iR].cn0[1] +
			EpkBase->SatObs[iB].cn0[0] + EpkBase->SatObs[iB].cn0[1];
		if (quality > 200.0 && EpkRover->SatPVT[iR].Elevation > 40.0)
		{
			DDObs->RefPrn[k] = SDObs->SdSatObs[Id[k][best]].Prn;	// 记录卫星号
			DDObs->RefPos[k] = Id[k][best];							// 记录在单差数组中的位置
			LastRefPrn[k] = DDObs->RefPrn[k];
		}
		else
		{
			DDObs->RefPrn[k] = -1;
			DDObs->RefPos[k] = -1;	// 无参考星
		}
	}
}

/*************************************************************************/
/********************** 基于最小二乘法的相对定位 *************************/
/*************************************************************************/
/*-----------------------------------------------
	相对定位浮点解函数
------------------------------------------------*/
bool RTKFloat(RAWDAT* Rawdata, PPRESULT* Base, PPRESULT* Rover, FloatResult* Fres)
{
	// 1、设置基站和流动站位置的初值
	double BasePos[3];
	BLHToXYZ(Rawdata->BaseBest.Pos, BasePos, R_WGS84, F_WGS84);	// 获取基站坐标
	double RoverPos[3] = { Rover->Position[0],Rover->Position[1], Rover->Position[2] };	// 获取流动站初值坐标（SPP解算的结果）

	// 2、计算GPS和BDS双差卫星数量（各个系统每个频率的双差卫星数量）,计算双差并初始化双差模糊度
	int DDSatNum[2] = { 0,0 };	// 双差卫星数 0为GPS 1为BDS
	vector<double> DDN;	// 双差模糊度
	vector<int>ValidSatId;	// 有效卫星索引

	// 遍历所有单差卫星
	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)
	{
		// 确定卫星系统
		int Sys = -1;
		if (Rawdata->SdObs.SdSatObs[i].System == GPS)Sys = 0;
		else if (Rawdata->SdObs.SdSatObs[i].System == BDS)Sys = 1;
		else continue;

		// 参考星和无效卫星不计入双差卫星数
		if (i == Rawdata->DDObs.RefPos[Sys])continue;
		else if (!(Rawdata->SdObs.SdSatObs[i].Valid == 1 &&
			Rawdata->SdObs.SdSatObs[i].half[0] == 1 &&
			Rawdata->SdObs.SdSatObs[i].half[1] == 1))continue;

		// 计算双差观测值
		vector<double>ddp(2, 0.0);
		vector<double>ddl(2, 0.0);
		for (int j = 0; j < 2; j++)
		{
			ddp[j] = Rawdata->SdObs.SdSatObs[i].dP[j] - Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]].dP[j];
			ddl[j] = Rawdata->SdObs.SdSatObs[i].dL[j] - Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]].dL[j];
		}

		// 计算并初始化双差模糊度
		if (Sys == 0)	// GPS
		{
			DDN.push_back((ddl[0] - ddp[0]) / WL1_GPS);
			DDN.push_back((ddl[1] - ddp[1]) / WL2_GPS);
		}
		else if (Sys == 1)	// BDS
		{
			DDN.push_back((ddl[0] - ddp[0]) / WL1_BDS);
			DDN.push_back((ddl[1] - ddp[1]) / WL3_BDS);
		}
		ValidSatId.push_back(i);
		DDSatNum[Sys]++;	// 统计卫星数量
	}

	if (DDSatNum[0] + DDSatNum[1] < 5)
	{
		cout << "SecOfWeek: " << Rawdata->BasEpk_cur.Time.SecOfWeek << " The number of observation satellites is less than five!\n";
		return false;	// 观测卫星数量不足
	}

	// 3、计算基站坐标到所有卫星的几何距离
	vector<double>BaseToSats(Rawdata->BasEpk_cur.SatNum, 0.0);
	for (int i = 0; i < Rawdata->BasEpk_cur.SatNum; i++)
	{
		vector<double>dxyz(3, 0.0);
		for (int j = 0; j < 3; j++) { dxyz[j] = BasePos[j] - Rawdata->BasEpk_cur.SatPVT[i].SatPos[j]; }
		BaseToSats[i] = sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);
	}

	// 提取参考星在单差数据中的位置
	int RefSatRover[2] = { -1, -1 };
	int RefSatBase[2] = { -1, -1 };
	for (int sysIdx = 0; sysIdx < 2; sysIdx++) {
		if (Rawdata->DDObs.RefPos[sysIdx] >= 0) {
			RefSatRover[sysIdx] = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[sysIdx]].nRover;
			RefSatBase[sysIdx] = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[sysIdx]].nBase;
		}
	}
	/* 进入最小二乘 */
	MatrixXd B, P, BT, BTP, N;
	VectorXd w, L, X, V;
	int nObs = 0, nParam = 0;	// 观测个数、参数个数
	bool flag = false;	// 收敛标志，位置改正数小于阈值时为true
	int count = 0;		// 迭代次数
	int MaxCount = 50;	// 最大迭代次数
	double Threshold = 1e-8;	// 收敛阈值
	while (!flag && count < MaxCount)
	{
		// 4、计算流动站到参考星的几何距离
		vector<double>RoverToRefSat(2, 0.0);
		for (int Sys = 0; Sys < 2; Sys++)
		{
			if (DDSatNum[Sys] == 0) continue;
			if (RefSatRover[Sys] < 0) continue; // 无参考星则跳过
			vector<double>dxyz(3, 0.0);
			for (int j = 0; j < 3; j++) { dxyz[j] = RoverPos[j] - Rawdata->RovEpk_cur.SatPVT[RefSatRover[Sys]].SatPos[j]; }
			RoverToRefSat[Sys] = sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);
		}
		// 5、对单差观测值进行循环
		// 构建观测方程矩阵
		nObs = (DDSatNum[0] + DDSatNum[1]) * 4;	// 每个卫星4个观测值（两种观测值 * 两个频率）
		nParam = 3 + (DDSatNum[0] + DDSatNum[1]) * 2;	// 3个位置参数 + 双差模糊度参数个数（双频）

		B = MatrixXd::Zero(nObs, nParam);	// 设计矩阵
		w = VectorXd::Zero(nObs);	// 观测值向量
		P = MatrixXd::Zero(nObs, nObs);	// 权矩阵

		int obsRow = 0;	// 观测方程从第1行1列开始填充
		int paramCol = 3;	// 模糊度参数从第四列开始填充
		for (int SatId = 0; SatId < ValidSatId.size(); SatId++)
		{
			// 5.1 参考星不参与计算
			int i = ValidSatId[SatId];
			int Sys = 0;
			if (Rawdata->SdObs.SdSatObs[i].System == GPS)Sys = 0;
			else if (Rawdata->SdObs.SdSatObs[i].System == BDS)Sys = 1;
			else continue;
			if (Rawdata->DDObs.RefPos[Sys] < 0) continue; // 无参考星时跳过该系统卫星

			// 保存当前卫星信息
			int roverId = Rawdata->SdObs.SdSatObs[i].nRover;
			int baseId = Rawdata->SdObs.SdSatObs[i].nBase;

			// 5.2 线性化双差观测方程

			vector<double>dxyz(3, 0.0);	// 计算流动站与该卫星的dx、dy、dz
			for (int j = 0; j < 3; j++) { dxyz[j] = RoverPos[j] - Rawdata->RovEpk_cur.SatPVT[roverId].SatPos[j]; }
			double RoverToSat = sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);

			vector<double>lmn(3, 0.0);	// 该卫星的方向余弦	
			for (int j = 0; j < 3; j++) { lmn[j] = dxyz[j] / RoverToSat; }

			int refRoverId = RefSatRover[Sys];
			vector<double>dxyz_ref(3, 0.0);	// 参考星的dx、dy、dz
			for (int j = 0; j < 3; j++) { dxyz_ref[j] = RoverPos[j] - Rawdata->RovEpk_cur.SatPVT[refRoverId].SatPos[j]; }

			vector<double>lmn_ref(3, 0.0);	// 参考星的方向余弦
			for (int j = 0; j < 3; j++) { lmn_ref[j] = dxyz_ref[j] / RoverToRefSat[Sys]; }

			vector<double>dd_lmn(3, 0.0);	// 双差的方向余弦
			for (int j = 0; j < 3; j++) { dd_lmn[j] = lmn[j] - lmn_ref[j]; }

			// 计算双差观测值
			vector<double>ddp(2, 0.0);
			vector<double>ddl(2, 0.0);
			for (int j = 0; j < 2; j++)
			{
				ddp[j] = Rawdata->SdObs.SdSatObs[i].dP[j] - Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]].dP[j];
				ddl[j] = Rawdata->SdObs.SdSatObs[i].dL[j] - Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]].dL[j];
			}
			double ddRou = (RoverToSat - RoverToRefSat[Sys]) + (BaseToSats[RefSatBase[Sys]] - BaseToSats[baseId]);	// 双差几何距离

			// 5.3 填充 伪距观测系数矩阵B 和 观测值向量w的前两行
			vector<double>wl(2, 0.0);	// 波长
			if (Sys == 0) { wl[0] = WL1_GPS; wl[1] = WL2_GPS; }
			else if (Sys == 1) { wl[0] = WL1_BDS; wl[1] = WL3_BDS; }

			for (int j = 0; j < 2; j++)	// 先填充伪距观测方程系数
			{
				for (int k = 0; k < 3; k++) { B(obsRow, k) = dd_lmn[k]; }
				w(obsRow) = ddp[j] - ddRou;

				obsRow++;	// 行数+1
			}
			// 填充 相位 观测系数矩阵B 和 观测值向量w的后两行
			for (int j = 0; j < 2; j++)
			{
				for (int k = 0; k < 3; k++) { B(obsRow, k) = dd_lmn[k]; }
				B(obsRow, paramCol) = wl[j];	// 模糊度系数
				w(obsRow) = ddl[j] - ddRou - wl[j] * DDN[SatId * 2 + j];

				obsRow++;	// 行数+1
				paramCol++;	// 列数+1
			}
		}
		// 6、构建权矩阵P
		for (int Sys = 0; Sys < 2; Sys++)
		{
			if (DDSatNum[Sys] == 0)continue;	//容错处理

			double N = DDSatNum[Sys] / (DDSatNum[Sys] + 1.0);
			double oN = -1.0 / (DDSatNum[Sys] + 1.0);
			vector<double>BG_P(4, 0.0);
			BG_P[0] = N; BG_P[1] = 1000 * N;
			BG_P[2] = oN; BG_P[3] = 1000 * oN;

			int startRow = 0, startCol = 0;	// 确定GPS和BDS系统的起始位置
			if (Sys == 0) { startRow = 0; startCol = 0; }
			else { startRow = DDSatNum[0] * 4; startCol = DDSatNum[0] * 4; }

			// 填充权矩阵P
			for (int a = 0; a < DDSatNum[Sys] * 4; a++)
			{
				for (int b = 0; b < DDSatNum[Sys] * 4; b++)
				{
					int row = startRow + a, col = startCol + b;	// 获得当前行和列
					if (a == b)	// 对角线上的元素
					{
						// 每次填充的前两行
						if (a % 4 < 2) { P(row, col) = BG_P[0]; }
						else { P(row, col) = BG_P[1]; }	// 后两行
					}
					else if (a % 4 == b % 4)	// 非对角线但是相关的元素
					{
						// 每次填充的前两行
						if (a % 4 < 2) { P(row, col) = BG_P[2]; }
						else { P(row, col) = BG_P[3]; }	// 后两行
					}
				}
			}
		}
		// 7、最小二乘解算
		BT = B.transpose();	// B矩阵转置
		BTP = BT * P;
		N = BTP * B;
		L = BTP * w;

		// 求解 X = N^(-1) * L
		X = N.llt().solve(L);

		// 更新参数
		for (int j = 0; j < 3; j++) { RoverPos[j] += X[j]; }	// 更新位置
		for (int e = 0; e < (DDSatNum[0] + DDSatNum[1]) * 2; e++) { DDN[e] += X(3 + e); }

		// 8、收敛判断
		if (fabs(X(0)) < Threshold && fabs(X(1)) < Threshold && fabs(X(2)) < Threshold)
		{
			//cout << "周内秒：" << Rawdata->BasEpk_cur.Time.SecOfWeek << " 浮点解最小二乘解算收敛成功！\n";
			flag = true;
		}

		count++;	// 迭代次数+1
	}
	if (!flag)return flag;

	// 10、存储结果
	MatrixXd N_inv = N.inverse();
	// 存储双差卫星数量
	for (int j = 0; j < 2; j++) { Rawdata->DDObs.DDSatNum[j] = Fres->stanum[j] = DDSatNum[j]; }
	Rawdata->DDObs.Sats = Fres->totalsatnum = DDSatNum[0] + DDSatNum[1];	// 存储卫星总数量
	// 存储浮点解的基线向量
	for (int j = 0; j < 3; j++) { Fres->dX[j] = RoverPos[j] - BasePos[j]; }
	// 存储基线向量的DOP
	for (int j = 0; j < 3; j++) { Fres->DOP[j] = sqrt(N_inv(j, j)); }
	Fres->PDOP = sqrt(N_inv(0, 0) + N_inv(1, 1) + N_inv(2, 2));
	Fres->HDOP = sqrt(N_inv(0, 0) + N_inv(1, 1));
	Fres->VDOP = sqrt(N_inv(2, 2));
	// 存储双差模糊度（浮点解）
	for (int j = 0; j < MAXCHANNUM * 2; j++) { Fres->N[j] = 0.0; }	// 置零
	for (int j = 0; j < Fres->totalsatnum * 2; j++) { Fres->N[j] = DDN[j]; }
	// 存储模糊度的协因数矩阵Qn
	for (int i = 0; i < MAXCHANNUM * 2; i++) { for (int j = 0; j < MAXCHANNUM * 2; j++) { Fres->Qn[i * MAXCHANNUM * 2 + j] = 0.0; } }	// 置零
	GetQnn(Fres->totalsatnum, N_inv, Fres->Qn);
	// 存储验后单位权中误差
	V = B * X - w;	// 残差向量
	double VTPV = V.transpose() * P * V;	// 残差加权平方和
	int df = nObs - nParam;	// 自由度
	double SIGMA = sqrt(VTPV / df);	// 单位权中误差
	Fres->sigma = SIGMA;

	return flag;
}

// Qn的赋值代码
void GetQnn(const int satnum, const MatrixXd& N_inv, double* Q)
{
	int N_inv_size = (3 + 2 * satnum) * (3 + 2 * satnum); // N_inv中的有效数据个数
	int skip_count = 3 * (3 + 2 * satnum); // 需要跳过的元素个数
	int index_N_inv = skip_count; // 当前N_inv的索引
	int index_Q = 0; // 当前Q的索引

	while (index_N_inv + 2 * satnum <= N_inv_size)
	{
		// 跳过3个位置参数
		index_N_inv += 3;

		// 提取2*satnum个模糊度参数的协因数
		for (int i = 0; i < 2 * satnum; i++)
		{
			// 将一维索引转换为二维索引
			int row = index_N_inv / (3 + 2 * satnum);
			int col = index_N_inv % (3 + 2 * satnum);
			Q[index_Q] = N_inv(row, col);
			index_Q++;
			index_N_inv++;
		}
	}
}

/*-----------------------------------------------
	相对定位固定解函数
------------------------------------------------*/
bool RTKFix(RAWDAT* Rawdata, PPRESULT* Base, PPRESULT* Rover, FloatResult* Fres)
{
	//if (Rawdata->DDObs.bFixed == false)return false;
	// 1. 获得初始位置
	double BasePos[3];
	BLHToXYZ(Rawdata->BaseBest.Pos, BasePos, R_WGS84, F_WGS84);	// 基站坐标
	double RoverPos[3];
	for (int i = 0; i < 3; i++)RoverPos[i] = Rover->Position[i];	// 流动站坐标

	// 2. 提取参考星索引
	int RefSatRover[2] = { -1,-1 }, RefSatBase[2] = { -1,-1 };
	for (int Sys = 0; Sys < 2; Sys++)
	{
		if (Rawdata->DDObs.RefPos[Sys] >= 0)
		{
			RefSatRover[Sys] = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]].nRover;
			RefSatBase[Sys] = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]].nBase;
		}
	}
	// 计算基站到各个卫星的几何距离、并构建有效卫星列表
	vector<int>ValidSatId;
	int DDSatNum[2] = { 0,0 };
	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)
	{
		// 确定卫星系统
		int Sys = -1;
		if (Rawdata->SdObs.SdSatObs[i].System == GPS)Sys = 0;
		else if (Rawdata->SdObs.SdSatObs[i].System == BDS)Sys = 1;
		else continue;

		// 参考星和无效卫星不计入有效卫星列表
		if (i == Rawdata->DDObs.RefPos[Sys])continue;
		else if (!(Rawdata->SdObs.SdSatObs[i].Valid == 1 &&
			Rawdata->SdObs.SdSatObs[i].half[0] == 1 &&
			Rawdata->SdObs.SdSatObs[i].half[1] == 1))continue;

		// 构建有效卫星列表
		ValidSatId.push_back(i);
		DDSatNum[Sys]++;
	}
	if (DDSatNum[0] + DDSatNum[1] < 2)  cout << "SecOfWeek: " << Rawdata->BasEpk_cur.Time.SecOfWeek << " The number of satellites is not enough to set the fixed sulution!\n";
	vector<double>BaseToSats(Rawdata->BasEpk_cur.SatNum, 0.0);
	for (int i = 0; i < Rawdata->BasEpk_cur.SatNum; i++)
	{
		// 几何距离
		vector<double>dxyz(3, 0.0);
		for (int j = 0; j < 3; j++)dxyz[j] = BasePos[j] - Rawdata->BasEpk_cur.SatPVT[i].SatPos[j];
		BaseToSats[i] = sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);
	}

	// 3. 读取已固定的双差整周模糊度
	vector<double>DDN(Rawdata->DDObs.FixedAmb, Rawdata->DDObs.FixedAmb + ValidSatId.size() * 2);

	// 4. 迭代最小二乘
	MatrixXd B, P, BT, BTP, N;
	VectorXd w, L, X, V;
	bool flag = false;
	int count = 0, MaxCount = 50, nObs = 0, nParam = 0;
	double Threshold = 1e-8;
	while (!flag && count < MaxCount)
	{
		// 4.1 计算流动站到参考星的几何距离
		vector<double>RoverToRefSat(2, 0.0);
		for (int Sys = 0; Sys < 2; Sys++)
		{
			if (DDSatNum[Sys] == 0) continue;
			if (RefSatRover[Sys] < 0) continue; // 无参考星则跳过
			vector<double>dxyz(3, 0.0);
			for (int j = 0; j < 3; j++) { dxyz[j] = RoverPos[j] - Rawdata->RovEpk_cur.SatPVT[RefSatRover[Sys]].SatPos[j]; }
			RoverToRefSat[Sys] = sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);
		}

		// 4.2 填充B和w矩阵
		nObs = ValidSatId.size() * 2;	// 每个卫星2个观测值（两个载波相位观测）
		nParam = 3;						// 3个位置参数
		B = MatrixXd::Zero(nObs, nParam);	// 设计矩阵
		w = VectorXd::Zero(nObs);	// 观测向量
		P = MatrixXd::Zero(nObs, nObs);	// 权矩阵

		for (int SatId = 0; SatId < ValidSatId.size(); SatId++)
		{
			// 跳过参考星
			int i = ValidSatId[SatId];
			int Sys = 0;
			if (Rawdata->SdObs.SdSatObs[i].System == GPS)Sys = 0;
			else if (Rawdata->SdObs.SdSatObs[i].System == BDS)Sys = 1;
			else continue;
			if (Rawdata->DDObs.RefPos[Sys] < 0) continue; // 无参考星时跳过该系统卫星

			// 保存当前卫星信息
			int roverId = Rawdata->SdObs.SdSatObs[i].nRover;
			int baseId = Rawdata->SdObs.SdSatObs[i].nBase;

			// 线性化双差观测方程
			vector<double>dxyz(3, 0.0);	// 计算流动站与该卫星的dx、dy、dz
			for (int j = 0; j < 3; j++) { dxyz[j] = RoverPos[j] - Rawdata->RovEpk_cur.SatPVT[roverId].SatPos[j]; }
			double RoverToSat = sqrt(dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2]);

			vector<double>lmn(3, 0.0);	// 该卫星的方向余弦	
			for (int j = 0; j < 3; j++) { lmn[j] = dxyz[j] / RoverToSat; }

			int refRoverId = RefSatRover[Sys];
			vector<double>dxyz_ref(3, 0.0);	// 参考星的dx、dy、dz
			for (int j = 0; j < 3; j++) { dxyz_ref[j] = RoverPos[j] - Rawdata->RovEpk_cur.SatPVT[refRoverId].SatPos[j]; }

			vector<double>lmn_ref(3, 0.0);	// 参考星的方向余弦
			for (int j = 0; j < 3; j++) { lmn_ref[j] = dxyz_ref[j] / RoverToRefSat[Sys]; }

			vector<double>dd_lmn(3, 0.0);	// 双差的方向余弦
			for (int j = 0; j < 3; j++) { dd_lmn[j] = lmn[j] - lmn_ref[j]; }

			vector<double>ddl(2, 0.0);	// 相位双差观测值
			for (int j = 0; j < 2; j++) { ddl[j] = Rawdata->SdObs.SdSatObs[i].dL[j] - Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]].dL[j]; }
			double ddRou = (RoverToSat - RoverToRefSat[Sys]) + (BaseToSats[RefSatBase[Sys]] - BaseToSats[baseId]);	// 双差几何距离

			vector<double>wl(2, 0.0);	// 波长
			if (Sys == 0) { wl[0] = WL1_GPS; wl[1] = WL2_GPS; }
			else if (Sys == 1) { wl[0] = WL1_BDS; wl[1] = WL3_BDS; }

			// 填写矩阵
			int row[2] = { 2 * SatId,2 * SatId + 1 };
			for (int a = 0; a < 2; a++)
			{
				for (int b = 0; b < 3; b++)B(row[a], b) = dd_lmn[b];
			}	// 设计矩阵
			for (int a = 0; a < 2; a++) { w(row[a]) = ddl[a] - ddRou - wl[a] * DDN[2 * SatId + a]; }	// 观测向量
		}
		// 4.3 填写P权矩阵
		for (int Sys = 0; Sys < 2; Sys++)
		{
			if (DDSatNum[Sys] == 0)continue;	//容错处理

			int startRow = 0, startCol = 0;	// 确定GPS和BDS系统的起始位置
			if (Sys == 0) { startRow = 0; startCol = 0; }
			else { startRow = DDSatNum[0] * 2; startCol = DDSatNum[0] * 2; }

			double N = DDSatNum[Sys] / (DDSatNum[Sys] + 1.0);	// 同一个卫星自相关
			double oN = -1.0 / (DDSatNum[Sys] + 1.0);			// 不同卫星间相关

			for (int a = 0; a < 2 * DDSatNum[Sys]; a++)
			{
				for (int b = 0; b < 2 * DDSatNum[Sys]; b++)
				{
					int row = startRow + a, col = startCol + b;
					if (a == b)P(row, col) = N;
					else if (a % 2 == b % 2)P(row, col) = oN;
				}
			}
		}
		// 4.4 最小二乘解
		BT = B.transpose();
		BTP = BT * P;
		N = BTP * B;
		L = BTP * w;
		X = N.llt().solve(L);
		V = B * X - w;	// 残差向量

		// 4.5 更新位置
		for (int j = 0; j < 3; j++)RoverPos[j] += X(j);

		// 4.6 收敛判断
		if (fabs(X(0)) < Threshold && fabs(X(1)) < Threshold && fabs(X(2)) < Threshold)flag = true;
		count++;
	}

	if (!flag)return false;

	// 5. 存储结果
	// 计算基线向量
	for (int j = 0; j < 3; j++) { Rawdata->DDObs.dPos[j] = RoverPos[j] - BasePos[j]; }
	// 存储卫星数量
	for (int j = 0; j < 2; j++) { Fres->stanum[j] = DDSatNum[j]; }
	Rawdata->DDObs.Sats = Fres->stanum[0] + Fres->stanum[1];

	return flag;
}

/*************************************************************************/
/********************** 基于卡尔曼滤波的相对定位 *************************/
/*************************************************************************/
/*-----------------------------------------------
	滤波初始化函数
------------------------------------------------*/
void InitFilter(RAWDAT* Rawdata, PPRESULT* Base, PPRESULT* Rover, RTKEKF* ekf)
{
	// 初始化时间
	ekf->Time = Rawdata->RovEpk_cur.Time;

	// 统计本历元双差模糊度卫星数
	int SatNum = 0;
	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)
	{
		if (i == Rawdata->DDObs.RefPos[0] || i == Rawdata->DDObs.RefPos[1])continue;	// 不含参考星
		auto& sdobs = Rawdata->SdObs.SdSatObs[i];
		if (sdobs.Valid == 1 && sdobs.half[0] == 1 && sdobs.half[1] == 1)SatNum++;	// 观测值有效则卫星数+1
	}
	int n = 3 + 2 * SatNum;	// 状态纬度（3坐标 + 每颗卫星双差模糊度）

	// 创建状态量X0和方差协方差P0矩阵
	// 初始化状态量X0
	for (int j = 0; j < 3; j++) { ekf->X0[j] = Rover->Position[j]; }
	int k = 3;	// 前k个状态
	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)
	{
		if (i == Rawdata->DDObs.RefPos[0] || i == Rawdata->DDObs.RefPos[1]) { ekf->Index[i] = -1; continue; }	// 参考星不计入
		auto& sdobs = Rawdata->SdObs.SdSatObs[i];
		if (sdobs.Valid != 1 || sdobs.half[0] != 1 || sdobs.half[1] != 1) { ekf->Index[i] = -1; continue; }	// 无效观测不计入

		// 若有效，构建双差观测
		ekf->Index[i] = k;
		int Sys = (sdobs.System == GPS ? 0 : 1);	// GPS为0 BDS为1
		auto& refpos = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]];
		double ddp[2] = { 0.0,0.0 }, ddl[2] = { 0.0,0.0 };
		for (int j = 0; j < 2; j++)
		{
			ddp[j] = sdobs.dP[j] - refpos.dP[j];
			ddl[j] = sdobs.dL[j] - refpos.dL[j];
		}
		ekf->X0[k + 0] = (Sys == 0) ? ((ddl[0] - ddp[0]) / WL1_GPS) : ((ddl[0] - ddp[0]) / WL1_BDS);
		ekf->X0[k + 1] = (Sys == 0) ? ((ddl[1] - ddp[1]) / WL2_GPS) : ((ddl[1] - ddp[1]) / WL3_BDS);
		k += 2;
	}
	// 初始化协方差矩阵P0
	for (int i = 0; i < n; i++)
	{
		ekf->P0[i * n + i] = (i < 3) ? 100.0 : 500.0;
	}

	// 滤波初始化完成
	ekf->IsInit = true;
}

/*-----------------------------------------------
	时间更新函数
------------------------------------------------*/
bool EkfPredict(RAWDAT* Rawdata, RTKEKF* ekf)
{
	// 统计当前和上一个历元的双差卫星数量
	int DSats_cur = 0, DSats_prv = 0;
	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)	// 当前历元
	{
		if (i == ekf->CurDDObs.RefPos[0] || i == ekf->CurDDObs.RefPos[1])continue;	// 参考卫星不计入
		auto& sdobs = Rawdata->SdObs.SdSatObs[i];
		if (sdobs.Valid == 1 && sdobs.half[0] == 1 && sdobs.half[1] == 1)DSats_cur++;	// 有效卫星计数
		if (sdobs.System == GPS)ekf->CurDDObs.DDSatNum[0] += 1;
		else if (sdobs.System == BDS) ekf->CurDDObs.DDSatNum[1] += 1;
	}
	for (int i = 0; i < ekf->SDObs.SatNum; i++)	// 上一历元
	{
		if (i == ekf->DDObs.RefPos[0] || i == ekf->DDObs.RefPos[1])continue;	// 参考卫星不计入
		auto& sdobs = ekf->SDObs.SdSatObs[i];
		if (sdobs.Valid == 1 && sdobs.half[0] == 1 && sdobs.half[1] == 1)DSats_prv++;	// 有效卫星计入
	}
	int n_cur = 3 + 2 * DSats_cur;
	int n_prv = 3 + 2 * DSats_prv;
	//if (ekf->nSats == 0) n_prv = 3 + 2 * DSats_prv;	// 矩阵维数
	//else n_prv = 3 + 2 * ekf->nSats;
	
	// 上一个历元的X0和P0
	VectorXd X0 = VectorXd::Zero(n_prv); for (int i = 0; i < n_prv; i++) { X0(i) = ekf->X0[i]; }
	MatrixXd P0 = MatrixXd::Zero(n_prv, n_prv);
	for (int i = 0; i < n_prv; i++)
	{
		for (int j = 0; j < n_prv; j++)
		{
			P0(i, j) = ekf->P0[i * n_prv + j];
		}
	}

	// 构建Φ和Q矩阵
	MatrixXd Phi = MatrixXd::Zero(n_cur, n_prv);
	MatrixXd Q = MatrixXd::Zero(n_cur, n_cur);

	// 填充Φ和Q矩阵
	for (int i = 0; i < 3; i++)
	{
		Phi(i, i) = 1.0;	// 位置就是上一个历元的位置
		Q(i, i) = 1e-8;	// 由于是静态，预测误差较小
	}

	// 记录参考星有无变化
	bool RefChanged[2] = { false,false };
	for (int i = 0; i < 2; i++) { if (ekf->CurDDObs.RefPrn[i] != ekf->DDObs.RefPrn[i])RefChanged[i] = true; }

	// 记录本历元的参考星在上一个历元中是否可用
	bool IsValidPrev[2] = { false,false };
	int RefIndexPrev[2] = { -1,-1 };	// 本历元参考星在上一个历元单差数据中的对应存储位置
	for (int i = 0; i < ekf->SDObs.SatNum; i++)
	{
		auto& sdobs = ekf->SDObs.SdSatObs[i];
		if (sdobs.Valid != 1 || sdobs.half[0] != 1 || sdobs.half[1] != 1)continue;
		if (sdobs.System == GPS && sdobs.Prn == ekf->CurDDObs.RefPrn[0]) { IsValidPrev[0] = true; RefIndexPrev[0] = i; }
		if (sdobs.System == BDS && sdobs.Prn == ekf->CurDDObs.RefPrn[1]) { IsValidPrev[1] = true; RefIndexPrev[1] = i; }
	}

	// 遍历当前历元，建立模糊度传递关系
	int row = 3;	// 模糊度起始行
	vector<int>NeedInit; NeedInit.reserve(DSats_cur);
	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)
	{
		if (i == Rawdata->DDObs.RefPos[0] || i == Rawdata->DDObs.RefPos[1])continue;	// 跳过参考星
		auto& sdobs = Rawdata->SdObs.SdSatObs[i];
		if (sdobs.Valid != 1 || sdobs.half[0] != 1 || sdobs.half[1] != 1)continue;	// 跳过无效观测

		bool FoundPrev = false;	// 用于记录当前历元的该颗卫星对应的双差模糊度在上一个历元中能否找到
		for (int j = 0; j < ekf->SDObs.SatNum; j++)
		{
			auto& prevsdobs = ekf->SDObs.SdSatObs[j];
			if (prevsdobs.System == sdobs.System && prevsdobs.Prn == sdobs.Prn)
			{
				FoundPrev = true;
				int Sys = (prevsdobs.System == GPS ? 0 : 1);	// GPS为0 BDS为1

				// 上一个历元该卫星可用且有索引
				if (prevsdobs.Valid == 1 && prevsdobs.half[0] == 1 && prevsdobs.half[1] == 1 && ekf->Index[j] != -1)
				{
					// 情况1：参考星不变，则直接继承N
					if (!RefChanged[Sys])
					{
						int col = ekf->Index[j];	// 上一个历元该卫星N的列
						Phi(row, col) = 1.0;
						Phi(row + 1, col + 1) = 1.0;
						Q(row, row) = 1e-10;
						Q(row + 1, row + 1) = 1e-10;
					}

					// 情况2：参考星变了，且本历元参考星在上一个历元也可以用
					else if (IsValidPrev[Sys] && ekf->Index[RefIndexPrev[Sys]] != -1)
					{
						int colSat = ekf->Index[j];
						int colRef = ekf->Index[RefIndexPrev[Sys]];
						Phi(row, colSat) = 1.0; Phi(row, colRef) = -1.0;
						Phi(row + 1, colSat + 1) = 1.0, Phi(row + 1, colRef + 1) = -1.0;
						Q(row, row) += 1e-10;
						Q(row + 1, row + 1) += 1e-10;
					}
					// 情况3：参考星变了，且本历元参考星在上一个历元没有用
					else
					{
						Q(row, row) = 100.0;
						Q(row + 1, row + 1) = 500.0;
						NeedInit.push_back(i);
					}
				}
				// 上一个历元这颗卫星有问题，则初始化
				else
				{
					Q(row, row) = 100.0;
					Q(row + 1, row + 1) = 500.0;
					NeedInit.push_back(i);
				}
				break;
			}
		}
		// 上一个历元没有这颗卫星，则初始化
		if (!FoundPrev)
		{
			Q(row, row) = 100.0;
			Q(row + 1, row + 1) = 500.0;
			NeedInit.push_back(i);
		}
		row += 2;
	}

	// 预测X、P
	VectorXd X_pred = Phi * X0;
	MatrixXd P_pred = Phi * P0 * Phi.transpose() + Q;

	// 对本历元需要初始化的 模糊度 进行近似初始化
	int RefIndexRover[2] = { -1,-1 };
	if (Rawdata->DDObs.RefPrn[0] != -1)RefIndexRover[0] = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[0]].nRover;
	if (Rawdata->DDObs.RefPrn[1] != -1)RefIndexRover[1] = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[1]].nRover;
	double RefSatPosGPS[3] = { 0.0,0.0,0.0 }, RefSatPosBDS[3] = { 0.0,0.0,0.0 }, RoverPos[3] = { 0.0,0.0,0.0 };
	for (int i = 0; i < 3; i++)
	{
		if (RefIndexRover[0] != -1)RefSatPosGPS[i] = Rawdata->RovEpk_cur.SatPVT[RefIndexRover[0]].SatPos[i];
		if (RefIndexRover[1] != -1)RefSatPosBDS[i] = Rawdata->RovEpk_cur.SatPVT[RefIndexRover[1]].SatPos[i];
		RoverPos[i] = X_pred(i);
	}

	int row2 = 3, idxNeed = 0;
	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)
	{
		if (i == Rawdata->DDObs.RefPos[0] || i == Rawdata->DDObs.RefPos[1])continue;	// 跳过参考星
		auto& sdobs = Rawdata->SdObs.SdSatObs[i];
		if (sdobs.Valid != 1 || sdobs.half[0] != 1 || sdobs.half[1] != 1)continue;	// 跳过无效卫星

		bool ThisInit = (idxNeed < (int)NeedInit.size() && i == NeedInit[idxNeed]);	// 检查当前卫星是否需要初始化模糊度
		if (ThisInit)
		{
			int Sys = (sdobs.System == GPS ? 0 : 1);	// GPS为0 BDS为1
			int SatIdxR = sdobs.nRover;
			double SatPos[3] = { 0.0,0.0,0.0 };
			for (int j = 0; j < 3; j++) { SatPos[j] = Rawdata->RovEpk_cur.SatPVT[SatIdxR].SatPos[j]; }
			double* RefPos = (Sys == 0) ? RefSatPosGPS : RefSatPosBDS;

			// 计算观测双差
			int RefPosIdx = Rawdata->DDObs.RefPos[Sys];
			auto& refsdobs = Rawdata->SdObs.SdSatObs[RefPosIdx];
			double ddp[2] = { 0.0,0.0 }, ddl[2] = { 0.0,0.0 };
			for (int j = 0; j < 2; j++)
			{
				ddp[j] = sdobs.dP[j] - refsdobs.dP[j];
				ddl[j] = sdobs.dL[j] - refsdobs.dL[j];
			}
			X_pred(row2) = (Sys == 0) ? ((ddl[0] - ddp[0]) / WL1_GPS) : ((ddl[1] - ddp[1]) / WL1_BDS);
			X_pred(row2 + 1) = (Sys == 0) ? ((ddl[0] - ddp[0]) / WL2_GPS) : ((ddl[1] - ddp[1]) / WL3_BDS);
			idxNeed++;
		}
		row2 += 2;	// 一颗卫星双频
	}
	// 写回X和P
	for (int i = 0; i < n_cur; i++) { ekf->X[i] = X_pred(i); }
	for (int i = 0; i < n_cur; i++)
	{
		for (int j = 0; j < n_cur; j++)
		{
			ekf->P[i * n_cur + j] = P_pred(i, j);
		}
	}

	// 更新索引
	int k = 3;	// 存储索引
	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)
	{
		if (i == Rawdata->DDObs.RefPos[0] || i == Rawdata->DDObs.RefPos[1]) { ekf->Index[i] = -1; continue; }	// 参考星不计入可用双差卫星数
		auto& sdobs = Rawdata->SdObs.SdSatObs[i];
		if (sdobs.Valid == 1 && sdobs.half[0] == 1 && sdobs.half[1] == 1) { ekf->Index[i] = k; k += 2; }	// 本颗卫星对应站间单差数据产生的双频双差模糊度存放在状态X中的索引
		else ekf->Index[i] = -1;
	}

	ekf->nSats = DSats_cur;
	ekf->Time = Rawdata->RovEpk_cur.Time;

	return true;
}

/*-----------------------------------------------
	计算方向余弦函数
------------------------------------------------*/
inline Vector3d LineOfSightDD(const double Rover[3], const double Sat[3], const double Ref[3])
{
	Vector3d rRover(Rover[0], Rover[1], Rover[2]);
	Vector3d rSat(Sat[0], Sat[1], Sat[2]);
	Vector3d rRef(Ref[0], Ref[1], Ref[2]);
	double RhoSat = (rRover - rSat).norm();
	double RhoRef = (rRover - rRef).norm();
	Vector3d eS = (rRover - rSat) / RhoSat;
	Vector3d eRef = (rRover - rRef) / RhoRef;

	return(eS - eRef);	// H的前三列系数
}

/*-----------------------------------------------
	测量更新函数
------------------------------------------------*/
bool EkfMeasureUpdate(RAWDAT* Rawdata, RTKEKF* ekf)
{
	int TotalSat = ekf->nSats;
	if (TotalSat <= 0)return false;

	int n = 3 + 2 * TotalSat;	// 状态维数
	int m = 4 * TotalSat;	// 观测维数（每个卫星双频双观测）

	VectorXd X = VectorXd::Zero(n);
	for (int i = 0; i < n; i++) { X(i) = ekf->X[i]; }
	MatrixXd P = MatrixXd::Zero(n, n);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			P(i, j) = ekf->P[i * n + j];
		}
	}

	// 提取参考星索引
	int RefIdxRover[2] = { -1,-1 }, RefIdxBase[2] = { -1,-1 };
	double RefSatPosGPS_R[3] = { -1,-1,-1 }, RefSatPosBDS_R[3] = { -1,-1,-1 };
	for (int i = 0; i < 2; i++)
	{
		if (Rawdata->DDObs.RefPos[i] != -1)RefIdxRover[i] = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[i]].nRover;
		if (Rawdata->DDObs.RefPos[i] != -1)RefIdxBase[i] = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[i]].nBase;
	}
	for (int i = 0; i < 3; i++)
	{
		if (RefIdxRover[0] != -1)RefSatPosGPS_R[i] = Rawdata->RovEpk_cur.SatPVT[RefIdxRover[0]].SatPos[i];
		if (RefIdxRover[1] != -1)RefSatPosBDS_R[i] = Rawdata->RovEpk_cur.SatPVT[RefIdxRover[1]].SatPos[i];
	}

	// 基站XYZ
	double BaseXYZ[3] = { 0.0,0.0,0.0 };
	BLHToXYZ(Rawdata->BaseBest.Pos, BaseXYZ, R_WGS84, F_WGS84);
	// 计算基站到各个卫星之间的几何距离
	vector<double>RhoBase(Rawdata->BasEpk_cur.SatNum, 0.0);
	for (int i = 0; i < Rawdata->BasEpk_cur.SatNum; i++)
	{
		Vector3d Base(BaseXYZ[0], BaseXYZ[1], BaseXYZ[2]);
		Vector3d Sats(Rawdata->BasEpk_cur.SatPVT[i].SatPos[0], Rawdata->BasEpk_cur.SatPVT[i].SatPos[1], Rawdata->BasEpk_cur.SatPVT[i].SatPos[2]);
		RhoBase[i] = (Base - Sats).norm();
	}

	// 构建H、v、R
	MatrixXd H = MatrixXd::Zero(m, n);
	VectorXd v = VectorXd::Zero(m);
	MatrixXd R = MatrixXd::Zero(m, m);

	int DDSatNum[2] = { 0,0 };	// GPS BDS双差卫星数量
	int row = 0;
	double Rover[3] = { X(0),X(1),X(2) };

	for (int i = 0; i < Rawdata->SdObs.SatNum; i++)
	{
		if (i == Rawdata->DDObs.RefPos[0] || i == Rawdata->DDObs.RefPos[1])continue;	// 跳过参考星
		auto& sdobs = Rawdata->SdObs.SdSatObs[i];
		if (sdobs.Valid != 1 || sdobs.half[0] != 1 || sdobs.half[1] != 1)continue;	// 跳过无效观测

		int Sys = (sdobs.System == GPS ? 0 : 1);	// GPS为0 BDS为1
		double* RefPos = (Sys == 0) ? RefSatPosGPS_R : RefSatPosBDS_R;	// 参考星位置

		// 获取当前卫星坐标
		int SatIdxR = sdobs.nRover;
		double SatPosR[3] = { 0.0,0.0,0.0 };
		for (int j = 0; j < 3; j++) { SatPosR[j] = Rawdata->RovEpk_cur.SatPVT[SatIdxR].SatPos[j]; }

		// 计算方向余弦
		Vector3d lmnd = LineOfSightDD(Rover, SatPosR, RefPos);

		// 填充H系数矩阵
		for (int j = 0; j < 4; j++) { H.block<1, 3>(row + j, 0) = lmnd.transpose(); }	// 前四行的前三列是lmn
		int AmbCol = 3 + 2 * (DDSatNum[0] + DDSatNum[1]);
		H(row + 2, AmbCol) = (Sys == 0) ? WL1_GPS : WL1_BDS;	// 双频载波相位对应的系数
		H(row + 3, AmbCol + 1) = (Sys == 0) ? WL2_GPS : WL3_BDS;

		// 填充v观测残差矩阵
		auto& refsdobs = Rawdata->SdObs.SdSatObs[Rawdata->DDObs.RefPos[Sys]];
		double ddp[2] = { 0.0,0.0 }, ddl[2] = { 0.0,0.0 };
		for (int j = 0; j < 2; j++)
		{
			ddp[j] = sdobs.dP[j] - refsdobs.dP[j];
			ddl[j] = sdobs.dL[j] - refsdobs.dL[j];
		}

		// 填充R几何差矩阵
		Vector3d rRover(Rover[0], Rover[1], Rover[2]);
		Vector3d rSat(SatPosR[0], SatPosR[1], SatPosR[2]);
		Vector3d rRef(RefPos[0], RefPos[1], RefPos[2]);
		double DDRho_R = (rRover - rSat).norm() - (rRover - rRef).norm();

		// Base端几何差
		double DDRho_B = RhoBase[RefIdxBase[Sys]] - RhoBase[sdobs.nBase];
		double DD_Rou = DDRho_R + DDRho_B;

		// 填充v残差矩阵
		for (int j = 0; j < 2; j++)
		{
			v(row + j) = ddp[j] - DD_Rou;
			v(row + 2 + j) = ddl[j] - DD_Rou - H(row + 2 + j, AmbCol + j) * X(AmbCol + j);
		}

		// 更新可用卫星数量
		DDSatNum[Sys]++;
		row += 4;
	}

	// 填充R矩阵
	double GP[4] = { 4.0 * ANinfo.CodeNoise * ANinfo.CodeNoise,4.0 * ANinfo.CPNoise * ANinfo.CPNoise,2.0 * ANinfo.CodeNoise * ANinfo.CodeNoise,2.0 * ANinfo.CPNoise * ANinfo.CPNoise };
	double BP[4] = { 4.0 * ANinfo.CodeNoise * ANinfo.CodeNoise,4.0 * ANinfo.CPNoise * ANinfo.CPNoise,2.0 * ANinfo.CodeNoise * ANinfo.CodeNoise,2.0 * ANinfo.CPNoise * ANinfo.CPNoise };
	for (int i = 0; i < 4 * DDSatNum[0]; i++)	// 填充GPS部分
	{
		for (int j = 0; j < 4 * DDSatNum[0]; j++)
		{
			// 填充对角线元素
			if (i == j)R(0 + i, 0 + j) = ((i % 4 <= 1) ? GP[0] : GP[1]);
			// 填充非对角元素
			else if ((i % 4) == (j % 4))R(0 + i, 0 + j) = ((i % 4 <= 1) ? GP[2] : GP[3]);
		}
	}
	for (int i = 0; i < 4 * DDSatNum[1]; i++)	// 填充BDS部分
	{
		for (int j = 0; j < 4 * DDSatNum[1]; j++)
		{
			// 填充对角线元素
			if (i == j)R(4 * DDSatNum[0] + i, 4 * DDSatNum[0] + j) = ((i % 4 <= 1) ? BP[0] : BP[1]);
			// 填充非对角线元素
			else if ((i % 4) == (j % 4))R(4 * DDSatNum[0] + i, 4 * DDSatNum[0] + j) = ((i % 4 <= 1) ? BP[2] : BP[3]);
		}
	}

	/* 一次更新 */
	MatrixXd S = H * P * H.transpose() + R;
	MatrixXd K_k = P * H.transpose() * S.inverse();
	VectorXd X_k = X + K_k * v;
	MatrixXd I = MatrixXd::Identity(n, n);
	MatrixXd P_k = (I - K_k * H) * P * (I - K_k * H).transpose() + K_k * R * K_k.transpose();

	X = X_k, P = P_k;

	/* LAMBDA固定 */
	VectorXd Nfloat = X.segment(3, 2 * TotalSat);
	MatrixXd Pn = P.block(3, 3, 2 * TotalSat, 2 * TotalSat);

	vector<double>FixedAmb(4 * TotalSat, 0.0);
	double ResAmb[2] = { 0.0,0.0 };
	int lamflag = lambda(2 * TotalSat, 2, Nfloat.data(), Pn.data(), FixedAmb.data(), ResAmb);

	// LAMBDA固定失败
	if (lamflag != 0)
	{
		// 备份X、P作为下一个历元的先验
		for (int i = 0; i < n; i++) { ekf->X0[i] = X(i); }
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				ekf->P0[i * n + j] = P(i, j);
			}
		}
		return false;
	}

	ekf->IsFixed = false;
	double ratio = ResAmb[1] / max(1e-16, ResAmb[0]);
	ekf->ratio = ratio;

	// 比率通过阈值，做二次更新
	if (ratio >= ANinfo.RatioThres)
	{
		if (ratio >= 100)ekf->ratio = 100;	// 大于100时，赋值100
		/* 二次更新 */
		VectorXd Nint(2 * TotalSat);
		for (int i = 0; i < 2 * TotalSat; i++) { Nint(i) = FixedAmb[i]; }	// 取第一组整数解

		MatrixXd H1 = MatrixXd::Zero(2 * TotalSat, n);
		for (int i = 0; i < 2 * TotalSat; i++) { H1(i, i + 3) = 1.0; }

		VectorXd v1 = Nint - X.segment(3, 2 * TotalSat);
		MatrixXd R1 = MatrixXd::Identity(2 * TotalSat, 2 * TotalSat) * 1e-8;
		MatrixXd S1 = H1 * P * H1.transpose() + R1;
		MatrixXd K1 = P * H1.transpose() * S1.inverse();

		X = X + K1 * v1;
		P = (I - K1 * H1) * P * (I - K1 * H1).transpose() + K1 * R1 * K1.transpose();

		ekf->IsFixed = true;
	}
	else ekf->IsInit = false;	// 否则重新初始化

	// 备份到X0、P0作为下一个历元的先验
	for (int i = 0; i < n; i++) { ekf->X0[i] = X(i); }
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			ekf->P0[i * n + j] = P(i, j);
		}
	}

	X.setZero();
	P.setZero();

	return true;
}