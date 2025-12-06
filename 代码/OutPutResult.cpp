#include "RTK_Structs.h"
void ToFileSppResult(RAWDAT* rawdata, PPRESULT* Fres, double enu[3], FILE* OutputFile )
{
	fprintf(OutputFile, "%u %.f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %d %d\n", rawdata->RovEpk_cur.Time.Week, rawdata->RovEpk_cur.Time.SecOfWeek,
		Fres->Position[0], Fres->Position[1], Fres->Position[2],
		enu[0], enu[1], enu[2],
		Fres->PDOP, Fres->SigmaPos,
		Fres->BDSSatNum, Fres->GPSSatNum);
}

void ToFileFloatResult(RAWDAT* rawdata, double XYZ[3], double denu[3], FloatResult* Fres, FILE* OutputFile )
{
	fprintf(OutputFile, "%u %.f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.8f %0.3f %0.3f %0.3f %d %d\n", rawdata->RovEpk_cur.Time.Week, rawdata->RovEpk_cur.Time.SecOfWeek,
		XYZ[0], XYZ[1], XYZ[2],
		denu[0], denu[1], denu[2],
		Fres->sigma, Fres->PDOP, Fres->HDOP, Fres->VDOP,
		rawdata->DDObs.DDSatNum[0], rawdata->DDObs.DDSatNum[1]);
}

void ToFileFixedResult(RAWDAT* rawdata, double XYZ[3], double denu[3], FILE* OutputFile )
{
	fprintf(OutputFile, "%u %.f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.3f %d %d\n", rawdata->RovEpk_cur.Time.Week, rawdata->RovEpk_cur.Time.SecOfWeek,
		XYZ[0], XYZ[1], XYZ[2],
		denu[0], denu[1], denu[2],
		rawdata->DDObs.Ratio,
		rawdata->DDObs.DDSatNum[0], rawdata->DDObs.DDSatNum[1]);
}

void ToFileEkfResult(RAWDAT* rawdata, RTKEKF* ekf, double denu[3], FILE* OutputFile )
{
	fprintf(OutputFile, "%u %.f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.3f %d %d\n", rawdata->RovEpk_cur.Time.Week, rawdata->RovEpk_cur.Time.SecOfWeek,
		ekf->X0[0], ekf->X0[1], ekf->X0[2],
		denu[0], denu[1], denu[2],
		ekf->ratio, ekf->CurDDObs.DDSatNum[0], ekf->CurDDObs.DDSatNum[1]);
}

// mode=1ÊÇEKF£¬mode=2ÊÇLSQ
void FigureRTKPlot(RAWDAT* rawdata, double FixedBlh[3], RTKEKF* ekf, double EkfBlh[3], FILE* OutputFile, int mode)
{
	if (mode == 2)
	{
		int State = -1;
		if (rawdata->DDObs.bFixed == true)State = 1;
		if (rawdata->DDObs.bFixed == false)State = 2;
		fprintf(OutputFile, "%u %.f %0.18f %0.18f %0.18f %0.3f %d\n", rawdata->RovEpk_cur.Time.Week, rawdata->RovEpk_cur.Time.SecOfWeek,
			FixedBlh[0], FixedBlh[1], FixedBlh[2],
			rawdata->DDObs.Ratio, State);
	}
	if (mode == 1)
	{
		int State = 1;
		//if (ekf->IsFixed == true)
		//{
		//	State = 1;
		//}
		//else if (ekf->IsFixed == false)State = 2;

		fprintf(OutputFile, "%u %.f %0.18f %0.18f %0.18f %0.3f %d\n", rawdata->RovEpk_cur.Time.Week, rawdata->RovEpk_cur.Time.SecOfWeek,
			EkfBlh[0], EkfBlh[1], EkfBlh[2],
			ekf->ratio, State);
	}
}