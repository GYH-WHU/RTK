#include"RTK_Structs.h"

/*-----------------------------------------------
    判断星历是否过期
------------------------------------------------*/
bool CompSatClkOff(const int Prn, const GNSSSys Sys, const GPSTIME* t, GPSEPHREC* GPSEph, GPSEPHREC* BDSEph, SATPVT* Mid)
{
    double dt, LT = 7500.0; // GPS时间有效性阈值（秒）
    GPSTIME T;
    GPSEPHREC* EPH;

    T = *t;

    // 根据卫星系统类型选择对应星历数据
    if (Sys == GPS)
    {
        EPH = GPSEph + Prn - 1; // GPS星历数组索引
    }
    else if (Sys == BDS)
    {
        EPH = BDSEph + Prn - 1; // BDS星历数组索引
        T.Week -= 1356;
        T.SecOfWeek -= 14;
        LT = 3900.0;            // BDS时间有效性阈值
    }
    else return false;          // 非GPS或BDS

    // 计算钟差
    dt = (T.Week - EPH->TOC.Week) * 604800.0 + T.SecOfWeek - EPH->TOC.SecOfWeek;
    if (fabs(dt) > LT || EPH->SVHealth != 0)
    {
        return false;           // 星历过期，即不健康
    }
    //计算卫星钟差
    Mid->SatClkOft = EPH->ClkBias + EPH->ClkDrift * dt + EPH->ClkDriftRate * pow(dt, 2);
    return true;
}

/*-----------------------------------------------
    GPS卫星位置、速度、钟差与钟速计算
------------------------------------------------*/
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATPVT* Mid)
{
    // 星历是否过期或健康判断
    if (Eph->SVHealth == 1 || (fabs((t->Week - Eph->TOE.Week) * 7 * 24 * 60 * 60 + t->SecOfWeek - Eph->TOE.SecOfWeek)) > 7500)
    {
        Mid->Valid = false;
        return 1;
    }
    Mid->Valid = true;

    /*  一、计算卫星位置  */
    double A, n0, deltaT, n, Mk, Ek, Et, vk, phi_k, delta_u, delta_r, delta_i,
        u_k, r_k, i_k, X_k, Y_k, Omega_k, X, Y, Z;

    // 1、计算轨道长半轴
    A = Eph->SqrtA * Eph->SqrtA;

    // 2、计算平均角速度
    n0 = sqrt(GM_WGS84 / (A * A * A));

    // 3、计算相对于星历参考历元的时间
    deltaT = (t->Week - Eph->TOE.Week) * 604800.0 + t->SecOfWeek - Eph->TOE.SecOfWeek;

    // 4、对平均运动角速度进行改正
    n = n0 + Eph->DeltaN;

    // 5、计算平近点角
    Mk = Eph->M0 + n * deltaT;

    // 6、计算偏近点角
    Ek = 0;
    Et = Mk;    // 迭代器
    while (fabs(Et - Ek) > 1e-12)
    {
        Et = Ek;
        Ek = Mk + Eph->e * sin(Ek);
    }

    // 7、计算真近点角
    double sin_vk = sqrt(1.0 - Eph->e * Eph->e) * sin(Ek) / (1.0 - Eph->e * cos(Ek));
    double cos_vk = (cos(Ek) - Eph->e) / (1.0 - Eph->e * cos(Ek));
    vk = atan2(sin_vk, cos_vk);

    // 8、计算升交角距
    phi_k = vk + Eph->omega;

    // 9、计算二阶调和改正数
    delta_u = Eph->Cuc * cos(2.0 * phi_k) + Eph->Cus * sin(2.0 * phi_k);
    delta_r = Eph->Crc * cos(2.0 * phi_k) + Eph->Crs * sin(2.0 * phi_k);
    delta_i = Eph->Cic * cos(2.0 * phi_k) + Eph->Cis * sin(2.0 * phi_k);

    // 10-12、 计算改正后的升交角距、轨道半径和轨道倾角
    u_k = phi_k + delta_u;
    r_k = A * (1.0 - Eph->e * cos(Ek)) + delta_r;
    i_k = Eph->i0 + Eph->iDot * deltaT + delta_i;

    // 13、计算卫星在轨道上面的位置
    X_k = r_k * cos(u_k);
    Y_k = r_k * sin(u_k);

    // 14、计算改正后的升交点经度
    Omega_k = Eph->OMEGA + (Eph->OMEGADot - W_WGS84) * deltaT - W_WGS84 * Eph->TOE.SecOfWeek;

    // 15、计算在地固坐标系下的位置
    X = X_k * cos(Omega_k) - Y_k * cos(i_k) * sin(Omega_k);
    Y = X_k * sin(Omega_k) + Y_k * cos(i_k) * cos(Omega_k);
    Z = Y_k * sin(i_k);

    /*  二、计算卫星速度  */
    double  phi_k_dot, u_k_dot, r_k_dot, i_k_dot, omega_k_dot, E_k_dot, Vx, Vy, Vz,
        xk_prime, yk_prime, xk_prime_dot, yk_prime_dot;

    E_k_dot = n / (1.0 - Eph->e * cos(Ek));
    phi_k_dot = E_k_dot * sqrt(1.0 - Eph->e * Eph->e) / (1.0 - Eph->e * cos(Ek));

    u_k_dot = phi_k_dot + 2.0 * (Eph->Cus * cos(2.0 * phi_k) - Eph->Cuc * sin(2.0 * phi_k)) * phi_k_dot;
    r_k_dot = A * Eph->e * sin(Ek) * E_k_dot + 2.0 * (Eph->Crs * cos(2.0 * phi_k) - Eph->Crc * sin(2.0 * phi_k)) * phi_k_dot;
    i_k_dot = Eph->iDot + 2.0 * (Eph->Cis * cos(2.0 * phi_k) - Eph->Cic * sin(2.0 * phi_k)) * phi_k_dot;

    omega_k_dot = Eph->OMEGADot - W_WGS84;

    xk_prime = r_k * cos(u_k);
    yk_prime = r_k * sin(u_k);
    xk_prime_dot = r_k_dot * cos(u_k) - r_k * u_k_dot * sin(u_k);
    yk_prime_dot = r_k_dot * sin(u_k) + r_k * u_k_dot * cos(u_k);

    Matrix<double, 3, 4> R_dot;
    R_dot <<
        cos(Omega_k), -sin(Omega_k) * cos(i_k), -(xk_prime * sin(Omega_k) + yk_prime * cos(Omega_k) * cos(i_k)), yk_prime* sin(Omega_k)* sin(i_k),
        sin(Omega_k), cos(Omega_k)* cos(i_k), (xk_prime * cos(Omega_k) - yk_prime * sin(Omega_k) * cos(i_k)), -yk_prime * cos(Omega_k) * sin(i_k),
        0, sin(i_k), 0, yk_prime* cos(i_k);

    Vector4d right_side;
    right_side <<
        xk_prime_dot,
        yk_prime_dot,
        omega_k_dot,
        i_k_dot;

    Vector3d V_k;
    V_k = R_dot * right_side;

    // 输出结果
    Vx = V_k(0);
    Vy = V_k(1);
    Vz = V_k(2);


    /*  三、计算卫星钟差与钟速  */
       // 计算卫星钟差
    double T_sv, t_rel, delta_T_sv, delta_t_rel, F;
    //F = -2.0 * sqrt(GM_WGS84) / (C_Light * C_Light);
    F = -4.442807633e-10;

    t_rel = F * Eph->e * Eph->SqrtA * sin(Ek);
    T_sv = Eph->ClkBias + Eph->ClkDrift * deltaT + Eph->ClkDriftRate * deltaT * deltaT + t_rel;

    // 计算卫星钟速
    delta_t_rel = F * Eph->e * Eph->SqrtA * cos(Ek) * E_k_dot;
    delta_T_sv = Eph->ClkDrift + 2.0 * Eph->ClkDriftRate * deltaT + delta_t_rel;

    /*  四、保存结果  */
    Mid->SatPos[0] = X;
    Mid->SatPos[1] = Y;
    Mid->SatPos[2] = Z;

    Mid->SatVel[0] = Vx;
    Mid->SatVel[1] = Vy;
    Mid->SatVel[2] = Vz;

    Mid->SatClkOft = T_sv;
    Mid->SatClkSft = delta_T_sv;

    Mid->Tgd1 = Eph->TGD1;
    Mid->Tgd2 = Eph->TGD2;

    return 0;
}

/*-----------------------------------------------
    BDS卫星位置、速度、钟差与钟速计算
------------------------------------------------*/
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPHREC* Eph, SATPVT* Mid)
{
    // 星历是否过期或健康判断
    if (Eph->SVHealth == 1 || (fabs((t->Week - 1356.0 - Eph->TOE.Week) * 7 * 24 * 60 * 60 + t->SecOfWeek - 14.0 - Eph->TOE.SecOfWeek)) > 7500)
    {
        Mid->Valid = false;
        return 1;
    }
    Mid->Valid = true;

    /*  一、计算卫星位置  */
    double A, n0, T, deltaT, Toe, n, Mk, Ek, Et, vk, phi_k, delta_u, delta_r, delta_i,
        u_k, r_k, i_k, X_k, Y_k, Omega_k, X, Y, Z;

    // 1、计算轨道长半轴
    A = Eph->SqrtA * Eph->SqrtA;

    // 2、计算平均角速度
    n0 = sqrt(GM_CGCS2000 / (A * A * A));

    // 3、计算相对于星历参考历元的时间
    deltaT = (t->Week - 1356 - Eph->TOE.Week) * 604800.0 + t->SecOfWeek - 14.0 - Eph->TOE.SecOfWeek;

    if (deltaT > 302400.0) {
        deltaT -= 604800.0;
    }
    else if (deltaT < -302400.0) {
        deltaT += 604800.0;
    }

    // 4、对平均运动角速度进行改正
    n = n0 + Eph->DeltaN;

    // 5、计算平近点角
    Mk = Eph->M0 + n * deltaT;

    // 6、计算偏近点角
    Ek = 0;
    Et = Mk;    // 迭代器
    while (fabs(Et - Ek) > 1e-12)
    {
        Et = Ek;
        Ek = Mk + Eph->e * sin(Ek);
    }

    // 7、计算真近点角
    double sin_vk = sqrt(1.0 - Eph->e * Eph->e) * sin(Ek) / (1.0 - Eph->e * cos(Ek));
    double cos_vk = (cos(Ek) - Eph->e) / (1.0 - Eph->e * cos(Ek));
    vk = atan2(sin_vk, cos_vk);

    // 8、计算升交角距
    phi_k = vk + Eph->omega;

    // 9、计算二阶调和改正数
    delta_u = Eph->Cuc * cos(2.0 * phi_k) + Eph->Cus * sin(2.0 * phi_k);
    delta_r = Eph->Crc * cos(2.0 * phi_k) + Eph->Crs * sin(2.0 * phi_k);
    delta_i = Eph->Cic * cos(2.0 * phi_k) + Eph->Cis * sin(2.0 * phi_k);

    // 10-12、 计算改正后的升交角距、轨道半径和轨道倾角
    u_k = phi_k + delta_u;
    r_k = A * (1.0 - Eph->e * cos(Ek)) + delta_r;
    i_k = Eph->i0 + Eph->iDot * deltaT + delta_i;

    // 13、计算卫星在轨道上面的位置
    X_k = r_k * cos(u_k);
    Y_k = r_k * sin(u_k);

    double Vx, Vy, Vz, E_k_dot;
    // 判断是否为BDS系统的GEO卫星（i0 < 30度）
    bool isBDS_GEO = (Eph->System == BDS) && (Eph->i0 * 180.0 / PAI < 30.0); // 假设i0为角度单位

    // GEO卫星
    if (isBDS_GEO)
    {
        // 14、计算改正后的升交点经度
        Omega_k = Eph->OMEGA + Eph->OMEGADot * deltaT - W_CGCS2000 * Eph->TOE.SecOfWeek;

        double X_gk = X_k * cos(Omega_k) - Y_k * cos(i_k) * sin(Omega_k);
        double Y_gk = X_k * sin(Omega_k) + Y_k * cos(i_k) * cos(Omega_k);
        double Z_gk = Y_k * sin(i_k);

        double f = -5.0 * Rad;
        double p = deltaT * W_CGCS2000;

        Matrix3d R_x, R_z, R_z_dot;

        R_x << 1, 0, 0,
            0, cos(f), sin(f),
            0, -sin(f), cos(f);

        R_z << cos(p), sin(p), 0,
            -sin(p), cos(p), 0,
            0, 0, 1;

        // 15、计算在地固坐标系下的位置
        Vector3d pos_gk(X_gk, Y_gk, Z_gk);
        Vector3d pos_ecef = R_z * R_x * pos_gk;

        X = pos_ecef(0);
        Y = pos_ecef(1);
        Z = pos_ecef(2);

        /*  二、计算GEO卫星速度  */
        double Omega_k_dot, phi_k_dot, u_k_dot, r_k_dot, i_k_dot, xk_dot, yk_dot,
            X_dot_gk, Y_dot_gk, Z_dot_gk;

        Omega_k_dot = Eph->OMEGADot;
        E_k_dot = n / (1.0 - Eph->e * cos(Ek));
        phi_k_dot = E_k_dot * sqrt(1.0 - Eph->e * Eph->e) / (1.0 - Eph->e * cos(Ek));

        u_k_dot = phi_k_dot + 2.0 * (Eph->Cus * cos(2.0 * phi_k) - Eph->Cuc * sin(2.0 * phi_k)) * phi_k_dot;
        r_k_dot = A * Eph->e * sin(Ek) * E_k_dot + 2.0 * (Eph->Crs * cos(2.0 * phi_k) - Eph->Crc * sin(2.0 * phi_k)) * phi_k_dot;
        i_k_dot = Eph->iDot + 2.0 * (Eph->Cis * cos(2.0 * phi_k) - Eph->Cic * sin(2.0 * phi_k)) * phi_k_dot;

        // Omega_k_dot、i_k_dot 已知，X_k 和 Y_k 的导数如下：
        xk_dot = r_k_dot * cos(u_k) - r_k * u_k_dot * sin(u_k);
        yk_dot = r_k_dot * sin(u_k) + r_k * u_k_dot * cos(u_k);

        // X_gk velocity
        X_dot_gk = xk_dot * cos(Omega_k) - X_k * sin(Omega_k) * Omega_k_dot
            - yk_dot * cos(i_k) * sin(Omega_k)
            - Y_k * cos(Omega_k) * cos(i_k) * Omega_k_dot
            + Y_k * sin(Omega_k) * sin(i_k) * i_k_dot;

        // Y_gk velocity
        Y_dot_gk = xk_dot * sin(Omega_k) + X_k * cos(Omega_k) * Omega_k_dot
            + yk_dot * cos(i_k) * cos(Omega_k)
            - Y_k * sin(Omega_k) * cos(i_k) * Omega_k_dot
            - Y_k * cos(Omega_k) * sin(i_k) * i_k_dot;

        // Z_gk velocity
        Z_dot_gk = yk_dot * sin(i_k) + Y_k * cos(i_k) * i_k_dot;

        Vector3d vel_gk(X_dot_gk, Y_dot_gk, Z_dot_gk);

        R_z_dot << -W_CGCS2000 * sin(p), W_CGCS2000* cos(p), 0,
            -W_CGCS2000 * cos(p), -W_CGCS2000 * sin(p), 0,
            0, 0, 0;

        Vector3d vel_ecef = R_z * R_x * vel_gk + R_z_dot * R_x * pos_gk;

        Vx = vel_ecef(0);
        Vy = vel_ecef(1);
        Vz = vel_ecef(2);

    }

    // 非GEO卫星
    else
    {
        // 14、计算改正后的升交点经度
        Omega_k = Eph->OMEGA + (Eph->OMEGADot - W_CGCS2000) * deltaT - W_CGCS2000 * Eph->TOE.SecOfWeek;

        // 15、计算在地固坐标系下的位置
        X = X_k * cos(Omega_k) - Y_k * cos(i_k) * sin(Omega_k);
        Y = X_k * sin(Omega_k) + Y_k * cos(i_k) * cos(Omega_k);
        Z = Y_k * sin(i_k);


        /*  二、计算卫星速度  */
        double  phi_k_dot, u_k_dot, r_k_dot, i_k_dot, omega_k_dot,
            xk_prime, yk_prime, xk_prime_dot, yk_prime_dot;

        E_k_dot = n / (1.0 - Eph->e * cos(Ek));
        phi_k_dot = E_k_dot * sqrt(1.0 - Eph->e * Eph->e) / (1.0 - Eph->e * cos(Ek));

        u_k_dot = phi_k_dot + 2.0 * (Eph->Cus * cos(2.0 * phi_k) - Eph->Cuc * sin(2.0 * phi_k)) * phi_k_dot;
        r_k_dot = A * Eph->e * sin(Ek) * E_k_dot + 2.0 * (Eph->Crs * cos(2.0 * phi_k) - Eph->Crc * sin(2.0 * phi_k)) * phi_k_dot;
        i_k_dot = Eph->iDot + 2.0 * (Eph->Cis * cos(2.0 * phi_k) - Eph->Cic * sin(2.0 * phi_k)) * phi_k_dot;

        omega_k_dot = Eph->OMEGADot - W_CGCS2000;

        xk_prime = r_k * cos(u_k);
        yk_prime = r_k * sin(u_k);
        xk_prime_dot = r_k_dot * cos(u_k) - r_k * u_k_dot * sin(u_k);
        yk_prime_dot = r_k_dot * sin(u_k) + r_k * u_k_dot * cos(u_k);

        Matrix<double, 3, 4> R_dot;
        R_dot <<
            cos(Omega_k), -sin(Omega_k) * cos(i_k), -(xk_prime * sin(Omega_k) + yk_prime * cos(Omega_k) * cos(i_k)), yk_prime* sin(Omega_k)* sin(i_k),
            sin(Omega_k), cos(Omega_k)* cos(i_k), (xk_prime * cos(Omega_k) - yk_prime * sin(Omega_k) * cos(i_k)), -yk_prime * cos(Omega_k) * sin(i_k),
            0, sin(i_k), 0, yk_prime* cos(i_k);

        Vector4d right_side;
        right_side <<
            xk_prime_dot,
            yk_prime_dot,
            omega_k_dot,
            i_k_dot;

        Vector3d V_k;
        V_k = R_dot * right_side;

        // 输出结果
        Vx = V_k(0);
        Vy = V_k(1);
        Vz = V_k(2);
    }

    /*  三、计算卫星钟差与钟速  */
       // 计算卫星钟差
    double T_sv, t_rel, delta_T_sv, delta_t_rel, F;
    //F = -2.0 * sqrt(GM_WGS84) / (C_Light * C_Light);
    F = -4.442807633e-10;

    t_rel = F * Eph->e * Eph->SqrtA * sin(Ek);
    T_sv = Eph->ClkBias + Eph->ClkDrift * deltaT + Eph->ClkDriftRate * deltaT * deltaT + t_rel;

    // 计算卫星钟速
    delta_t_rel = F * Eph->e * Eph->SqrtA * cos(Ek) * E_k_dot;
    delta_T_sv = Eph->ClkDrift + 2.0 * Eph->ClkDriftRate * deltaT + delta_t_rel;


    /*  四、保存结果  */
    Mid->SatPos[0] = X;
    Mid->SatPos[1] = Y;
    Mid->SatPos[2] = Z;

    Mid->SatVel[0] = Vx;
    Mid->SatVel[1] = Vy;
    Mid->SatVel[2] = Vz;

    Mid->SatClkOft = T_sv;
    Mid->SatClkSft = delta_T_sv;

    Mid->Tgd1 = Eph->TGD1;
    Mid->Tgd2 = Eph->TGD2;

    return 0;
}
