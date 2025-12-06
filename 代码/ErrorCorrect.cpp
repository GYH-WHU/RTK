#include"RTK_Structs.h"

/*-----------------------------------------------
	对流层误差改正
------------------------------------------------*/
double Hopfield(const double H, const double Elev)
{
	// 容错处理：如果测站高于对流层顶，返回0
	if (abs(H) > Htrop) {
		return 0.0;
	}

	// 计算准备参数
	double RH = RelHum * exp(-0.0006396 * (H - SeaLevel));
	double p = AtmosPre * pow(1.0 - 0.0000226 * (H - SeaLevel), 5.225);

	double T = TempT0 - 0.0065 * (H - SeaLevel);
	double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
	double h_w = 11000.0;
	double h_d = 40136.0 + 148.72 * (TempT0 - 273.16);
	double K_w = 155.2e-7 * 4810.0 * e * (h_w - H) / pow(T, 2);
	double K_d = 155.2e-7 * p * (h_d - H) / T;

	// 计算delta
	double delta_d, delta_w;
	double Sin_1 = sin(sqrt(pow(Elev, 2) + 6.25) * Rad);
	double Sin_2 = sin(sqrt(pow(Elev, 2) + 2.25) * Rad);
	delta_d = K_d / Sin_1;
	delta_w = K_w / Sin_2;

	// 返回对流层改正值
	return delta_d + delta_w;
}


/*-----------------------------------------------
	GPS电离层误差改正
------------------------------------------------*/
// 输入仰角、方位角、经度、纬度（单位为度）
// GPS（L1）电离层改正
double Klobutchar_GPS(const GPSTIME* Time, double Elev, double Azim, double RcvPos[3], const IONOPARA* Ion)
{
	// 容错处理
	if (!Ion || RcvPos[0] == 0.0 || RcvPos[1] == 0.0 || RcvPos[2] > 1e6 || Elev <= 0.0)
		return 0.0;

	// === 单位转换：用户指定为角度（度）和弧度 ===
	double phi_u = RcvPos[1] * Rad;  // 纬度，弧度
	double lam_u = RcvPos[0] * Rad;  // 经度，弧度
	double E = Elev;                 // 高度角，度
	double A = Azim;                 // 方位角，度

	// === 步骤1：地心张角 ψ，单位 semicircle ===
	double psi = 0.0137 / (E / 180.0 + 0.11) - 0.022;	// 1 semicircle = 180°

	// === 步骤2：穿刺点纬度（单位：semicircle）===
	double phi_i = phi_u * Deg / 180.0 + psi * cos(A * Rad);
	if (phi_i > 0.416) phi_i = 0.416;
	if (phi_i < -0.416) phi_i = -0.416;

	// === 步骤3：穿刺点经度 ===
	double lam_i = lam_u * Deg / 180.0 + psi * sin(A * Rad) / cos(phi_i * PAI);

	// === 步骤4：地磁纬度 φ_m ===
	double phi_m = phi_i + 0.064 * cos((lam_i - 1.617) * PAI);

	// === 步骤5：本地时间 t（秒）===
	double t = 43200.0 * lam_i + Time->SecOfWeek;
	while (t >= 86400.0) t -= 86400.0;
	while (t < 0.0) t += 86400.0;

	// === 步骤6：振幅（秒） ===
	double AMP = Ion->alpha[0] * 1 + Ion->alpha[1] * phi_m + Ion->alpha[2] * phi_m * phi_m + Ion->alpha[3] * phi_m * phi_m * phi_m;
	if (AMP < 0.0) AMP = 0.0;

	// === 步骤7：周期（秒） ===
	double PER = Ion->beta[0] * 1 + Ion->beta[1] * phi_m + Ion->beta[2] * phi_m * phi_m + Ion->beta[3] * phi_m * phi_m * phi_m;
	if (PER < 72000.0) PER = 72000.0;

	// === 步骤8：相位 X（弧度）===
	double X = 2.0 * PAI * (t - 50400.0) / PER;

	// === 步骤9：斜率因子 F（无量纲）===
	double F = 1.0 + 16.0 * pow(0.53 - E / 180.0, 3.0);

	// === 步骤10：延迟（秒）===
	double I_sec;
	if (fabs(X) < PAI / 2.0)
		I_sec = 5e-9 + AMP * (1.0 - X * X / 2.0 + pow(X, 4) / 24.0);
	else
		I_sec = 5e-9;

	return I_sec * F * C_Light;  // 返回单位：米
}

/*-----------------------------------------------
	BDS电离层误差改正
------------------------------------------------*/
// 输入仰角、方位角、经度、纬度（单位为度）
// BDS（L1）电离层改正
double Klobutchar_BDS(const GPSTIME* Time, double Elev, double Azim, double RcvPos[3], const IONOPARA* Ion)
{
	// 容错处理
	if (!Ion || RcvPos[0] == 0.0 || RcvPos[1] == 0.0 || RcvPos[2] > 1e6 || Elev <= 0.0)
		return 0.0;

	// === 单位转换 ===
	double lam_u = RcvPos[0] * Rad;     // 经度（弧度）
	double phi_u = RcvPos[1] * Rad;     // 纬度（弧度）
	double E = Elev * Rad;              // 高度角（弧度）
	double A = Azim * Rad;              // 方位角（弧度）

	// === 地心张角 ψ（弧度）===
	double Re = 6378000.0;	// 地球半径
	double h = 375000.0;	// 电离层高度
	double psi = PAI / 2.0 - E - asin((Re / (Re + h)) * cos(E));

	// === 电离层穿刺点地理坐标 ===
	double sin_phi_m = sin(phi_u) * cos(psi) + cos(phi_u) * sin(psi) * cos(A);
	double phi_m = asin(sin_phi_m);  // 电离层点纬度（弧度）

	double sin_dlam = sin(psi) * sin(A) / cos(phi_m);
	double lam_m = lam_u + asin(sin_dlam);  // 电离层点经度（弧度）

	// === 周内本地时间（单位：秒）===
	double t = Time->SecOfWeek + lam_m * 43200.0 / PAI;
	while (t >= 86400.0) t -= 86400.0;
	while (t < 0.0) t += 86400.0;

	// === φm/π 转为用于α/β插值（无单位）===
	double phi_norm = phi_m / PAI;

	// === A2: 幅度 ===
	double A2 = Ion->alpha[0] + Ion->alpha[1] * phi_norm + Ion->alpha[2] * pow(phi_norm, 2) + Ion->alpha[3] * pow(phi_norm, 3);
	if (A2 < 0.0) A2 = 0.0;

	// === A4: 周期 ===
	double A4 = Ion->beta[0] + Ion->beta[1] * phi_norm + Ion->beta[2] * pow(phi_norm, 2) + Ion->beta[3] * pow(phi_norm, 3);
	if (A4 < 72000.0) A4 = 72000.0;
	if (A4 > 172800.0) A4 = 172800.0;

	// === I'z(t)：垂直延迟（秒）===
	double x = 2.0 * PAI * (t - 50400.0) / A4;
	double Iz;
	if (fabs(t - 50400.0) < A4 / 4.0)
		Iz = 5e-9 + A2 * cos(x);
	else
		Iz = 5e-9;

	// === I_B1I(t)：路径电离层延迟（秒）===
	double cosE = cos(E);
	double F = 1.0 / sqrt(1.0 - pow((Re * cosE / (Re + h)), 2));
	double IonoDelay_sec = F * Iz;

	// === 转换为距离（米）===
	return IonoDelay_sec * C_Light;
}

/*-----------------------------------------------
	粗差探测
------------------------------------------------*/
void DetectOutlier(EPOCHOBS* Obs)
{
	// 当前历元GF和MW临时结算结果
	MWGF CurComObs[MAXCHANNUM];
	memset(CurComObs, 0, sizeof(CurComObs));

	for (int i = 0; i < Obs->SatNum; i++)
	{
		Obs->SatObs[i].Valid = false;

		// 1、检查该卫星的双频伪距和相位数据是否有效和完整，若不全或为0，将Valid标记为false，continue
		if (fabs(Obs->SatObs[i].P[0]) < 1e-5 || fabs(Obs->SatObs[i].P[1]) < 1e-5 || fabs(Obs->SatObs[i].L[0]) < 1e-5 || fabs(Obs->SatObs[i].L[1]) < 1e-5)
		{
			continue;
		}

		// 2、计算当前历元该卫星的GF和MW组合值
		double Lgf, Lmw;

		if (Obs->SatObs[i].System == GPS)	// 如果是GPS系统
		{
			Lgf = Obs->SatObs[i].L[0] - Obs->SatObs[i].L[1];
			Lmw = (FG1_GPS * Obs->SatObs[i].L[0] - FG2_GPS * Obs->SatObs[i].L[1]) / (FG1_GPS - FG2_GPS)
				- (FG1_GPS * Obs->SatObs[i].P[0] + FG2_GPS * Obs->SatObs[i].P[1]) / (FG1_GPS + FG2_GPS);

		}
		else if (Obs->SatObs[i].System == BDS)	// 如果是BDS系统
		{
			Lgf = Obs->SatObs[i].L[0] - Obs->SatObs[i].L[1];
			Lmw = (FG1_BDS * Obs->SatObs[i].L[0] - FG3_BDS * Obs->SatObs[i].L[1]) / (FG1_BDS - FG3_BDS)
				- (FG1_BDS * Obs->SatObs[i].P[0] + FG3_BDS * Obs->SatObs[i].P[1]) / (FG1_BDS + FG3_BDS);
		}
		else
		{
			Obs->SatObs[i].Valid = false;
			continue;
		}

		// 存储当前MWGF计算值
		CurComObs[i].Prn = Obs->SatObs[i].Prn;
		CurComObs[i].Sys = Obs->SatObs[i].System;
		CurComObs[i].GF = Lgf;
		CurComObs[i].MW = Lmw;
		CurComObs[i].n = 1;


		// 3、对于可用的观测数据，计算伪距的IF组合观测值，用于SPP
		if (CurComObs[i].Sys == GPS)
		{
			CurComObs[i].PIF = (FG1_GPS * FG1_GPS * Obs->SatObs[i].P[0] - FG2_GPS * FG2_GPS * Obs->SatObs[i].P[1]) / (FG1_GPS * FG1_GPS - FG2_GPS * FG2_GPS);
		}
		else if (CurComObs[i].Sys == BDS)
		{
			CurComObs[i].PIF = (FG1_BDS * FG1_BDS * Obs->SatObs[i].P[0] - FG3_BDS * FG3_BDS * Obs->SatObs[i].P[1]) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS);
		}

		CurComObs[i].n = 1;	// 作为新历史的初始计数
		for (int j = 0; j < MAXCHANNUM; j++)
		{
			// 4、从上个历元的MWGF数据中查找该卫星的GF和MW组合值
			if (Obs->ComObs[j].Prn == CurComObs[i].Prn && Obs->ComObs[j].Sys == CurComObs[i].Sys)
			{
				// 5、6、计算当前历元该卫星GF与上一历元对应GF的差值dGF，计算当前历元该卫星MW与上一历元对应MW平滑值的差值dMW
				double dGF = fabs(CurComObs[i].GF - Obs->ComObs[j].GF);
				double dMW = fabs(CurComObs[i].MW - Obs->ComObs[j].MW);

				// 7、检查dGF和dMW是否超限，限差阈值建议为5cm和3m。若超限，标记为粗差，将Valid标记为false ，若不超限，标记为可用将Valid标记为true，并计算该卫星的MW平滑值
				if (dGF < GFThres && dMW < MWThres)
				{
					Obs->SatObs[i].Valid = true;
					CurComObs[i].MW = (Obs->ComObs[j].MW * Obs->ComObs[j].n + CurComObs[i].MW) / (Obs->ComObs[j].n + 1);
					CurComObs[i].n = Obs->ComObs[j].n + 1;
				}
				// 本身初始化的时候就是false，不需要处理
				break;
			}
		}
	}
	// 8、将缓冲区组合观测值的拷贝到obs里
	memcpy(Obs->ComObs, CurComObs, sizeof(CurComObs));
}