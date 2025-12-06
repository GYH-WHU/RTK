#include "RTK_Structs.h"

/*-----------------------------------------------
    通用时到简化儒略日的转换算法
------------------------------------------------*/
void CommonTimeToMjdTime(const COMMONTIME* ct, MJDTIME* mjd)
{
    // 处理月份小于3的情况,小于3则月+12，年-1
    double JD = int(365.25 * ((ct->Month < 2) ? (ct->Year - 1) : ct->Year))
        + int(30.6001 * (((ct->Month < 2) ? (ct->Month + 12) : ct->Month) + 1))
        + ct->Day
        + ((ct->Hour * 3600 + ct->Minute * 60 + ct->Second) / 86400.0)
        + 1720981.5;

    double MJD = JD - 2400000.5;

    mjd->Days = static_cast<int>(MJD);
    mjd->FracDay = ((ct->Hour * 3600.0 + ct->Minute * 60.0 + ct->Second) / 86400.0);    // 直接用通用时的 时分秒计算小数天
}

/*-----------------------------------------------
    通用时转化为GPS时间
------------------------------------------------*/
void CommonTimeToGpsTime(const COMMONTIME* ct, GPSTIME* gps)
{
    MJDTIME mjd1;
    CommonTimeToMjdTime(ct, &mjd1);
    MjdTimeToGpsTime(&mjd1, gps);
}

/*-----------------------------------------------
    简化儒略日时间到通用时的转换算法
------------------------------------------------*/
void MjdTimeToCommonTime(const MJDTIME* mjd, COMMONTIME* ct)
{
    // 简化儒略日转儒略日
    double JD = mjd->Days + mjd->FracDay + 2400000.5;

    int a = int(JD + 0.5);
    int b = a + 1537;
    int c = int((b - 122.1) / 365.25);
    int d = int(365.25 * c);
    int  e = int((b - d) / 30.6001);

    // 计算年月日
    ct->Day = b - d - int(30.6001 * e) + (JD + 0.5 - int(JD + 0.5));
    ct->Month = e - 1 - 12 * int(e / 14);
    ct->Year = c - 4715 - int((7 + ct->Month) / 10);

    // 计算时分秒
    double seconds_of_day = mjd->FracDay * 86400.0;
    ct->Hour = int(seconds_of_day / 3600);
    ct->Minute = int((seconds_of_day - ct->Hour * 3600) / 60);
    ct->Second = seconds_of_day - ct->Hour * 3600 - ct->Minute * 60;
}

/*-----------------------------------------------
    简化儒略日时间到GPS时间的转换算法
------------------------------------------------*/
void MjdTimeToGpsTime(const MJDTIME* mjd, GPSTIME* gps)
{
    gps->Week = int((mjd->Days - 44244.0) / 7.0 + mjd->FracDay / 7.0);
    gps->SecOfWeek = (mjd->Days + mjd->FracDay - 44244.0 - gps->Week * 7.0) * 86400.0;
}

/*-----------------------------------------------
    GPS时间转通用时间
------------------------------------------------*/
void GpsTimeToCommonTime(const GPSTIME* gps, COMMONTIME* ct)
{
    MJDTIME mjd1;
    GpsTimeToMjdTime(gps, &mjd1);
    MjdTimeToCommonTime(&mjd1, ct);
}

/*-----------------------------------------------
    GPS时间到简化儒略日时间的转换算法
------------------------------------------------*/
void GpsTimeToMjdTime(const GPSTIME* gps, MJDTIME* mjd)
{
    double MJD = 44244.0 + gps->Week * 7 + gps->SecOfWeek / 86400.0;
    mjd->Days = int(MJD);
    mjd->FracDay = (gps->SecOfWeek - int(gps->SecOfWeek / 86400.0) * 86400.0) / 86400.0;
}
