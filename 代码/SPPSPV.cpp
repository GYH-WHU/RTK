#include"RTK_Structs.h"

/*-----------------------------------------------
	信号发射时刻卫星位置的计算
------------------------------------------------*/
void ComputeSatPVTAtSignalTrans(EPOCHOBS* Epk, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, double UserPos[3])
{
	// 将时间取出
	GPSTIME T = Epk->Time;

	// 循环对每个星历求卫星位置
	for (int i = 0; i < Epk->SatNum; i++)
	{
		GPSTIME t_AtSig;			// 定义卫星发射时刻
		double T_Threshold = 0.0;	// 设置迭代计算卫星钟差的阈值
		Matrix3d Rz;				// 定义旋转矩阵

		if (!CompSatClkOff(Epk->SatObs[i].Prn, Epk->SatObs[i].System, &Epk->Time, GPSEph, BDSEph, Epk->SatPVT + i))
		{
			Epk->SatPVT[i].Valid = false;
			continue;	// 星历过期则跳过计算，并标记为false
		}

		// 当系统是GPS系统时
		if (Epk->SatObs[i].System == GPS)
		{
			GPSEPHREC* eph = GPSEph + Epk->SatObs[i].Prn - 1;	// 取本组观测值对应的卫星的星历

			// 初始钟差设置为零
			Epk->SatPVT[i].SatClkOft = 0.0;

			// 迭代计算卫星钟差
			do
			{
				// 计算卫星发射时刻
				t_AtSig.Week = T.Week;
				t_AtSig.SecOfWeek = T.SecOfWeek - Epk->SatObs[i].P[0] / C_Light - Epk->SatPVT[i].SatClkOft;

				// 计算卫星钟差
				double t_toc = (t_AtSig.Week - eph->TOC.Week) * 604800.0 + t_AtSig.SecOfWeek - eph->TOC.SecOfWeek;	// t-toc
				double dt = eph->ClkBias
					+ eph->ClkDrift * t_toc
					+ eph->ClkDriftRate * t_toc * t_toc;

				// 更新迭代判断值
				T_Threshold = fabs(dt - Epk->SatPVT[i].SatClkOft);

				// 更新卫星钟差
				Epk->SatPVT[i].SatClkOft = dt;

			} while (T_Threshold > 1e-12);	// 迭代停止

			// 计算卫星位置、速度、钟差和钟速（含相对论效应）
			CompGPSSatPVT(Epk->SatObs[i].Prn, &t_AtSig, eph, Epk->SatPVT + i);

			// 计算信号传播时间
			double Dx = Epk->SatPVT[i].SatPos[0] - UserPos[0];
			double Dy = Epk->SatPVT[i].SatPos[1] - UserPos[1];
			double Dz = Epk->SatPVT[i].SatPos[2] - UserPos[2];
			double t_trans = sqrt(Dx * Dx + Dy * Dy + Dz * Dz) / C_Light;

			// 计算地球自转改正
			double alpha = W_WGS84 * t_trans;	// 地球自转角度(弧度)
			Rz << cos(alpha), sin(alpha), 0.0,	// 构造绕Z轴旋转的矩阵
				-sin(alpha), cos(alpha), 0.0,
				0.0, 0.0, 1.0;
			Vector3d pos(Epk->SatPVT[i].SatPos[0],	// 构造原始卫星位置向量
				Epk->SatPVT[i].SatPos[1],
				Epk->SatPVT[i].SatPos[2]);
			Vector3d vel(Epk->SatPVT[i].SatVel[0],	// 构造原始卫星速度向量
				Epk->SatPVT[i].SatVel[1],
				Epk->SatPVT[i].SatVel[2]);
			Vector3d pos_new = Rz * pos;		// 地球自转位置变换
			Vector3d vel_new = Rz * vel;		// 地球自转速度变换
			for (int j = 0; j < 3; j++)			// 更新地球自转改正后的位置与速度
			{
				Epk->SatPVT[i].SatPos[j] = pos_new(j);
				Epk->SatPVT[i].SatVel[j] = vel_new(j);
			}

			// 计算卫星的高度角与方位角
			GEOCOOR blh;
			XYZToBLH(UserPos, blh.Blh, R_WGS84, F_WGS84);
			CompSatElAz(UserPos, Epk->SatPVT[i].SatPos, &blh, &Epk->SatPVT[i].Elevation, &Epk->SatPVT[i].Azimuth);

			// 计算对流层改正
			Epk->SatPVT[i].TropCorr = Hopfield(blh.Blh[2], Epk->SatPVT[i].Elevation);
		}
		// 当系统是BDS系统时
		else if (Epk->SatObs[i].System == BDS)
		{
			GPSEPHREC* eph = BDSEph + Epk->SatObs[i].Prn - 1;	// 取本组观测值对应的卫星的星历
			GPSTIME t_BDS;										// BDS时间

			// 初始钟差设置为零
			Epk->SatPVT[i].SatClkOft = 0.0;

			// 迭代计算卫星钟差
			do
			{
				// 计算卫星发射时刻
				t_AtSig.Week = T.Week;
				t_AtSig.SecOfWeek = T.SecOfWeek - Epk->SatObs[i].P[0] / C_Light - Epk->SatPVT[i].SatClkOft;

				t_BDS.Week = t_AtSig.Week - 1356;			// 北斗时间系统减去1356周
				t_BDS.SecOfWeek = t_AtSig.SecOfWeek - 14.0;	// 北斗时间系统减去14秒
				// 计算卫星钟差
				double t_toc = (t_BDS.Week - eph->TOC.Week) * 604800.0 + t_BDS.SecOfWeek - eph->TOC.SecOfWeek;	// t-toc
				double dt = eph->ClkBias
					+ eph->ClkDrift * t_toc
					+ eph->ClkDriftRate * t_toc * t_toc;

				// 更新迭代判断值
				T_Threshold = fabs(dt - Epk->SatPVT[i].SatClkOft);

				// 更新卫星钟差
				Epk->SatPVT[i].SatClkOft = dt;

			} while (T_Threshold > 1e-12);	// 迭代停止

			// 计算卫星位置、速度、钟差和钟速（含相对论效应）
			CompBDSSatPVT(Epk->SatObs[i].Prn, &t_AtSig, eph, Epk->SatPVT + i);

			// 计算信号传播时间
			double Dx = Epk->SatPVT[i].SatPos[0] - UserPos[0];
			double Dy = Epk->SatPVT[i].SatPos[1] - UserPos[1];
			double Dz = Epk->SatPVT[i].SatPos[2] - UserPos[2];
			double t_trans = sqrt(Dx * Dx + Dy * Dy + Dz * Dz) / C_Light;

			// 计算地球自转改正
			double alpha = W_CGCS2000 * t_trans;// 地球自转角度(弧度)
			Rz << cos(alpha), sin(alpha), 0.0,	// 构造绕Z轴旋转的矩阵
				-sin(alpha), cos(alpha), 0.0,
				0.0, 0.0, 1.0;
			Vector3d pos(Epk->SatPVT[i].SatPos[0],	// 构造原始卫星位置向量
				Epk->SatPVT[i].SatPos[1],
				Epk->SatPVT[i].SatPos[2]);
			Vector3d vel(Epk->SatPVT[i].SatVel[0],	// 构造原始卫星速度向量
				Epk->SatPVT[i].SatVel[1],
				Epk->SatPVT[i].SatVel[2]);
			Vector3d pos_new = Rz * pos;		// 地球自转位置变换
			Vector3d vel_new = Rz * vel;		// 地球自转速度变换
			for (int j = 0; j < 3; j++)			// 更新地球自转改正后的位置与速度
			{
				Epk->SatPVT[i].SatPos[j] = pos_new(j);
				Epk->SatPVT[i].SatVel[j] = vel_new(j);
			}

			// 计算卫星的高度角与方位角
			GEOCOOR blh;
			XYZToBLH(UserPos, blh.Blh, R_CGCS2000, F_CGCS2000);
			CompSatElAz(UserPos, Epk->SatPVT[i].SatPos, &blh, &Epk->SatPVT[i].Elevation, &Epk->SatPVT[i].Azimuth);

			// 计算对流层改正
			Epk->SatPVT[i].TropCorr = Hopfield(blh.Blh[2], Epk->SatPVT[i].Elevation);
		}
	}
}

/*-----------------------------------------------
	单点定位SPP计算
------------------------------------------------*/
bool SPP(EPOCHOBS* Epoch, RAWDAT* Raw, PPRESULT* Result)
{
	// 1、记录当前历元时间
	Result->Time = Epoch->Time;

	// 2、初始化状态参数
	Vector3d X_R0(Result->Position[0], Result->Position[1], Result->Position[2]); // 上一历元收敛位置作为初值
	Vector2d dt_R0(0.0, 0.0);									// [GPS钟差, BDS钟差]，本历元初始设为0
	bool flag = true;											// 迭代标志
	int count = 0;												// 迭代计数

	do {
		// 3、计算每颗卫星信号发射时刻的卫星位置、钟差等，更新Epoch->SatPVT
		ComputeSatPVTAtSignalTrans(Epoch, Raw->GpsEph, Raw->BdsEph, X_R0.data());

		// 4、遍历每颗卫星，构造观测矩阵B、残差向量w、权阵主对角元素
		vector<RowVectorXd> B_rows;								// 每一颗有效卫星的观测方程系数
		vector<double> w_vals;									// 每一颗有效卫星的观测残差
		//vector<double> P_diag;									// 权阵主对角元素（等权为1）

		//double w_out[MAXCHANNUM];								// 用于检查输出
		//double Rou_out[MAXCHANNUM];							// 用于检查输出

		int satnum[2] = { 0, 0 };								// GPS与BDS系统参与解算卫星数

		for (int i = 0; i < Epoch->SatNum; i++) {
			// 4.1、无效卫星（粗差或位置计算失败）直接跳过
			if (!Epoch->SatObs[i].Valid || !Epoch->SatPVT[i].Valid) continue;

			// 4.2、计算接收机到卫星的空间距离Rou
			double Dx = Epoch->SatPVT[i].SatPos[0] - X_R0[0];
			double Dy = Epoch->SatPVT[i].SatPos[1] - X_R0[1];
			double Dz = Epoch->SatPVT[i].SatPos[2] - X_R0[2];
			double Rou = sqrt(Dx * Dx + Dy * Dy + Dz * Dz);

			// 4.3、构造观测矩阵当前行（B_row）：前三个元素为空间方向余弦，后两位为系统钟差分量
			RowVectorXd B_row = RowVectorXd::Zero(5);
			B_row.head<3>() = -Vector3d(Dx, Dy, Dz) / Rou;		// dX/dY/dZ

			double w_i; // 当前卫星残差
			if (Epoch->SatObs[i].System == GPS) {
				// 4.4、GPS卫星：第4列（索引3）为1，其余为0
				B_row[3] = 1.0;
				satnum[0]++;
				// 观测方程残差
				w_i = Epoch->ComObs[i].PIF - (Rou + dt_R0[0] - C_Light * Epoch->SatPVT[i].SatClkOft + Epoch->SatPVT[i].TropCorr);
			}
			else if (Epoch->SatObs[i].System == BDS) {
				// 4.6、BDS卫星：第5列（索引4）为1，其余为0
				B_row[4] = 1.0;
				satnum[1]++;
				// 观测方程残差（含BDS群延迟Tgd1改正项）
				w_i = Epoch->ComObs[i].PIF - (Rou + dt_R0[1] - C_Light * Epoch->SatPVT[i].SatClkOft + Epoch->SatPVT[i].TropCorr
					+ (FG1_BDS * FG1_BDS * C_Light * Epoch->SatPVT[i].Tgd1) / (FG1_BDS * FG1_BDS - FG3_BDS * FG3_BDS));
			}
			else continue;

			// 4.8、观测矩阵、残差向量与权阵主对角同步压入
			B_rows.push_back(B_row);
			w_vals.push_back(w_i);
			//P_diag.push_back(1.0);  // 单位权

			//w_out[i] = w_i;
			//Rou_out[i] = Rou;
		}

		// 5、统计有效卫星总数、解算参数数
		int validSat = (int)B_rows.size();					// 参与解算的卫星数
		int Para = 3 + (satnum[0] > 0) + (satnum[1] > 0);	// 参数个数：坐标XYZ+每个系统的接收机钟差
		if (validSat < Para) return false;					// 有效卫星数不足，直接退出

		// 6、构造观测矩阵、残差向量
		MatrixXd B(validSat, 5);
		VectorXd w(validSat);
		for (int i = 0; i < validSat; i++) {
			B.row(i) = B_rows[i];
			w(i) = w_vals[i];
		}

		// 7、构造权阵（等权）
		MatrixXd P = MatrixXd::Zero(validSat, validSat);
		for (int i = 0; i < validSat; ++i) {
			P(i, i) = 1.0;
		}

		// 8、根据参与卫星系统数确定解算参数维数
		VectorXd x;											// 最小二乘参数增量
		MatrixXd N_cut;										// 修改后的N矩阵
		MatrixXd PDOP_N_inv;								// 用于计算PDOP

		// 仅使用 GPS 卫星
		if (satnum[0] > 0 && satnum[1] == 0) {
			B.col(4).setZero();  // 清除 BDS钟差列
			N_cut = B.transpose() * P * B;
			VectorXd W_cut = B.transpose() * P * w;

			N_cut.conservativeResize(4, 4);	// 只保留前4列4行
			W_cut.conservativeResize(4);	// 只保留前4列

			x = N_cut.ldlt().solve(W_cut);
			PDOP_N_inv = N_cut.inverse();

			// 还原为5维
			x.conservativeResize(5);
			x[4] = 0;
		}
		// 仅使用 BDS 卫星
		else if (satnum[1] > 0 && satnum[0] == 0) {
			// 构造仅含 XYZ + BDS钟差的观测矩阵
			MatrixXd B_BDS(validSat, 4);
			for (int i = 0; i < validSat; ++i) {
				B_BDS.row(i).head<3>() = B.row(i).head<3>();
				B_BDS(i, 3) = B(i, 4);  // 第4列为 BDS钟差列
			}

			MatrixXd N_bds = B_BDS.transpose() * P * B_BDS;
			VectorXd W_bds = B_BDS.transpose() * P * w;

			x = N_bds.ldlt().solve(W_bds);
			PDOP_N_inv = N_bds.inverse();

			// 还原为5维
			x.conservativeResize(5);
			x.tail<2>() = Vector2d(0, x[3]);	// 将最后两个元素替换
		}
		// GPS+BDS联合解，参数数为5，直接解
		else {
			MatrixXd N = B.transpose() * P * B;					// 法方程系数矩阵
			VectorXd W = B.transpose() * P * w;					// 法方程右端项

			x = N.ldlt().solve(W);
			PDOP_N_inv = N.inverse();
		}

		// 9、判断收敛条件（参数修正量足够小）
		if (x.norm() < 1e-7) flag = false;

		// 10、更新参数估计：坐标+系统钟差
		X_R0 += x.head<3>();	// 将三维坐标改正量加到坐标上
		dt_R0[0] += x[3];		// 将钟差修正量加到钟差上
		dt_R0[1] += x[4];

		// 11、计算定位精度（PDOP，取位置参数对应的协方差）
		Result->PDOP = sqrt(PDOP_N_inv(0, 0) + PDOP_N_inv(1, 1) + PDOP_N_inv(2, 2));

		// 12、计算验后单位权中误差
		VectorXd v = B * x - w;
		Result->SigmaPos = sqrt((v.transpose() * P * v)(0, 0) / (validSat - Para));

		// 13、迭代计数，超限保护
		count++;
		Result->BDSSatNum = satnum[1];
		Result->GPSSatNum = satnum[0];
		Result->AllSatNum = validSat;
		if (count > 15) flag = false;

		//if (!flag)
		//{
		//	for (int i = 0; i < Epoch->SatNum; i++)
		//	{
		//		// 4.1、无效卫星（粗差或位置计算失败）直接跳过
		//		if (!Epoch->SatObs[i].Valid || !Epoch->SatPVT[i].Valid) continue;

		//		if (Epoch->SatObs[i].System == GPS)
		//		{
		//			// 4.5、调试输出每颗卫星的观测残差组成
		//			printf("[Epoch %d %6.0f] G%02d | PIF = %9.3f m | Rou = %9.3f m | dt_R0 = %+.9f s | ClkOff = %+.9f s | Trop = %6.3f m | w = %+.3f m\n",
		//				Epoch->Time.Week,
		//				Epoch->Time.SecOfWeek,
		//				Epoch->SatObs[i].Prn,
		//				Epoch->ComObs[i].PIF,
		//				Rou_out[i],
		//				dt_R0[0],
		//				Epoch->SatPVT[i].SatClkOft,
		//				Epoch->SatPVT[i].TropCorr,
		//				w_out[i]);

		//		}
		//		else if (Epoch->SatObs[i].System == BDS)
		//		{

		//			// 4.7、调试输出
		//			printf("[Epoch %d %6.0f] C%02d | PIF = %9.3f m | Rou = %9.3f m | dt_R0 = %+.9f s | ClkOff = %+.9f s | Trop = %6.3f m | w = %+.3f m\n",
		//				Epoch->Time.Week,
		//				Epoch->Time.SecOfWeek,
		//				Epoch->SatObs[i].Prn,
		//				Epoch->ComObs[i].PIF,
		//				Rou_out[i],
		//				dt_R0[1],
		//				Epoch->SatPVT[i].SatClkOft,
		//				Epoch->SatPVT[i].TropCorr,
		//				w_out[i]);
		//		}
		//	}
		//}
	} while (flag); // 14、参数未收敛时继续迭代

	// 15、输出最终收敛位置结果

	Result->RcvClkOft[0] = dt_R0[0];
	Result->RcvClkOft[1] = dt_R0[1];

	Result->Position[0] = X_R0[0];
	Result->Position[1] = X_R0[1];
	Result->Position[2] = X_R0[2];

	return true;
}

/*-----------------------------------------------
	单点测速SPV计算
------------------------------------------------*/
void SPV(EPOCHOBS* Epoch, PPRESULT* Result)
{
	Result->Time = Epoch->Time;		// 记录当前历元时间

	// 定义观测矩阵 B，使用Eigen的MatrixXd，大小由卫星数决定
	MatrixXd B(Epoch->SatNum, 4);
	B.setZero();					// 观测矩阵置零

	// 定义残差矩阵 w
	VectorXd w(Epoch->SatNum);
	w.setZero();					// 残差矩阵置零

	// 有效卫星个数
	int valid_satnum = 0;

	// 遍历所有卫星，计算每颗卫星的观测矩阵和残差
	for (int i = 0; i < Epoch->SatNum; i++)
	{
		// 观测数据不完整或者有粗差，或者卫星位置计算失败，不进行定位解算
		if (!Epoch->SatObs[i].Valid || !Epoch->SatPVT[i].Valid) continue;

		// 计算接收机到卫星的空间距离 Rou
		double xsr = Result->Position[0] - Epoch->SatPVT[i].SatPos[0];
		double ysr = Result->Position[1] - Epoch->SatPVT[i].SatPos[1];
		double zsr = Result->Position[2] - Epoch->SatPVT[i].SatPos[2];
		double rou = sqrt(xsr * xsr + ysr * ysr + zsr * zsr);

		// 计算卫星的相对速度 (Rou_dot)
		double roudot = -(xsr * Epoch->SatPVT[i].SatVel[0] + ysr * Epoch->SatPVT[i].SatVel[1] + zsr * Epoch->SatPVT[i].SatVel[2]) / rou;

		//if (Epoch->SatObs[i].System == GPS)
		//{
		//printf("[Epoch %d %6.0f] G%02d | xsr = %9.3f m | ysr = %9.3f m | zsr = %+.9f m | rou = %+.9f m | SatVelX = %+.9f m/s | SatVelY = %+.9f m/s | SatVelZ = %+.9f m/s\n",
		//	Epoch->Time.Week,
		//	Epoch->Time.SecOfWeek,
		//	Epoch->SatObs[i].Prn,
		//	xsr,
		//	ysr,
		//	zsr,
		//	rou,
		//	Epoch->SatPVT[i].SatVel[0],
		//	Epoch->SatPVT[i].SatVel[1],
		//	Epoch->SatPVT[i].SatVel[2]);
		//}

		//if (Epoch->SatObs[i].System == BDS)
		//{
		//	printf("[Epoch %d %6.0f] C%02d | xsr = %9.3f m | ysr = %9.3f m | zsr = %+.9f m | rou = %+.9f m | SatVelX = %+.9f m/s | SatVelY = %+.9f m/s | SatVelZ = %+.9f m/s\n",
		//		Epoch->Time.Week,
		//		Epoch->Time.SecOfWeek,
		//		Epoch->SatObs[i].Prn,
		//		xsr,
		//		ysr,
		//		zsr,
		//		rou,
		//		Epoch->SatPVT[i].SatVel[0],
		//		Epoch->SatPVT[i].SatVel[1],
		//		Epoch->SatPVT[i].SatVel[2]);
		//}


		// 构造观测矩阵 B 的当前行
		B(i, 0) = xsr / rou;
		B(i, 1) = ysr / rou;
		B(i, 2) = zsr / rou;
		B(i, 3) = 1;

		// 计算残差 w
		w(i) = Epoch->SatObs[i].D[0] - (roudot - C_Light * Epoch->SatPVT[i].SatClkSft);

		valid_satnum++;
	}

	// 如果有效卫星数不足 4 个，则退出
	if (valid_satnum < 4) return;

	// 定义法方程的系数矩阵 N 和右端项 W
	MatrixXd N(4, 4);
	VectorXd W(4);

	// 计算法方程矩阵 N = B^T * B
	MatrixXd BT = B.transpose();		// B矩阵的转置
	N = BT * B;							// 矩阵相乘得到 N

	// 计算 W = B^T * w
	W = BT * w;							// 矩阵相乘得到 W

	// 进行最小二乘求解
	MatrixXd N_inv(4, 4);				// 定义 N 矩阵的逆

	// 使用行列式判断矩阵是否可逆
	if (N.determinant() != 0) {
		N_inv = N.inverse();			// 求 N 矩阵的逆
	}
	else {
		return;							// 矩阵不可逆，直接退出
	}

	VectorXd X = N_inv * W;				// 求解最小二乘解

	// 将结果保存到 Result 中
	Result->Velocity[0] = X(0);
	Result->Velocity[1] = X(1);
	Result->Velocity[2] = X(2);

	// 计算验后单位权中误差
	VectorXd v(Epoch->SatNum);			// 残差向量
	VectorXd vT(Epoch->SatNum);			// 残差向量的转置
	double vTv;							// 残差平方和

	// 计算观测值 B * X
	VectorXd Bx = B * X;

	// 计算残差 v = Bx - w
	v = Bx - w;

	// 计算残差的转置 vT = v^T
	vT = v.transpose();

	// 计算残差平方和 vTv
	vTv = vT.dot(v);

	// 计算并保存验后单位权中误差
	Result->SigmaVel = sqrt(vTv / (valid_satnum - 4));  // 单位权中误差
}

