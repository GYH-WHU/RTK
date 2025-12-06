#include "RTK_Structs.h"

/*-----------------------------------------------
    大地坐标 -> 笛卡尔坐标 (BLH -> XYZ)
------------------------------------------------*/
void BLHToXYZ(const double BLH[3], double XYZ[3], const double R, const double F)
{
    double a = R;                           // 长半轴
    double f = F;                           // 扁率
    double b = a * (1.0 - f);               // 短半轴
    double e2 = (a * a - b * b) / (a * a);  // 第一偏心率平方

    double L = BLH[0] * Rad;                // 经度 (弧度)
    double B = BLH[1] * Rad;                // 纬度 (弧度)
    double H = BLH[2];                      // 高程 (米)

    double sinB = sin(B);
    double cosB = cos(B);
    double sinL = sin(L);
    double cosL = cos(L);

    double N = a / sqrt(1.0 - e2 * sinB * sinB); // 卯酉圈半径

    XYZ[0] = (N + H) * cosB * cosL;
    XYZ[1] = (N + H) * cosB * sinL;
    XYZ[2] = (N * (1.0 - e2) + H) * sinB;
}

/*-----------------------------------------------
    笛卡尔坐标 -> 大地坐标 (XYZ -> BLH)
------------------------------------------------*/
void XYZToBLH(const double XYZ[3], double BLH[3], const double R, const double F)
{
    double a = R;
    double b = a * (1 - F);
    double e2 = (a * a - b * b) / (a * a);

    double x = XYZ[0];
    double y = XYZ[1];
    double z = XYZ[2];
    double p = sqrt(x * x + y * y);

    double B = atan2(z, p * (1 - e2)); // 初始纬度 (弧度)
    double H = 0.0;

    int MaxCount = 1000;
    double Threshold = 1e-12;

    int count = 0;
    double delta = 0.0;

    do {
        double sinB = sin(B);
        double N = a / sqrt(1.0 - e2 * sinB * sinB);
        H = p / cos(B) - N;


        double B_new = atan2(z + e2 * (a / sqrt(1.0 - e2 * sinB * sinB)) * sinB, p);
        delta = B - B_new;
        B = B_new;
        count++;
    } while (count < MaxCount && std::fabs(delta) > Threshold);

    if (count >= MaxCount) {
        std::cerr << "警告: 未在" << MaxCount
            << "次迭代内收敛（误差：" << std::fabs(delta)
            << " 弧度）" << std::endl;
    }

    BLH[0] = B * Deg;           // 纬度 (度)
    BLH[1] = atan2(y, x) * Deg; // 经度 (度)
    BLH[2] = H;                 // 高程 (米)
}

/*-----------------------------------------------
    生成 BLH -> NEU 的旋转矩阵
------------------------------------------------*/
void BlhToNeuMat(const GEOCOOR* Blh, Matrix3d& Mat)
{
    double B = Blh->Blh[0] * Rad;
    double L = Blh->Blh[1] * Rad;

    double sinB = sin(B), cosB = cos(B);
    double sinL = sin(L), cosL = cos(L);

    Mat << -sinL, cosL, 0.0,
        -sinB * cosL, -sinB * sinL, cosB,
        cosB* cosL, cosB* sinL, sinB;
}

/*-----------------------------------------------
    计算 ENU 定位误差
------------------------------------------------*/
void CompEnudPos(const double Xs[], const double Xr[], const GEOCOOR* Blh, double dENU[])
{
    Vector3d Delta;
    Delta << Xs[0] - Xr[0], Xs[1] - Xr[1], Xs[2] - Xr[2];

    Matrix3d R;
    BlhToNeuMat(Blh, R);  // E/N/U方向旋转矩阵

    Vector3d d_neu = R * Delta;

    dENU[0] = d_neu(0); // E
    dENU[1] = d_neu(1); // N
    dENU[2] = d_neu(2); // U
}

/*-----------------------------------------------
    计算卫星高度角与方位角
------------------------------------------------*/
void CompSatElAz(const double Xr[], const double Xs[], const GEOCOOR* Blh, double* Elev, double* Azim)
{
    double dENU[3] = { 0.0, 0.0, 0.0 };
    CompEnudPos(Xs, Xr, Blh, dENU);

    double EN = sqrt(dENU[0] * dENU[0] + dENU[1] * dENU[1]);

    *Elev = atan2(dENU[2], EN) * Deg;
    *Azim = atan2(dENU[0], dENU[1]) * Deg;  // E在前、N在后

    if (*Azim < 0.0) *Azim += 360.0;
}