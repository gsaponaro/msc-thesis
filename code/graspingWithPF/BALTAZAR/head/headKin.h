/*****************************************************************************************
Description:	Class with Baltazar's Kinematics
Author:			Samuel Ferreira
Date:			28.05.2007
*****************************************************************************************/

/* half of the baseline and focal distande in meters -> B, F */
/* Parameters of the transformation of world's coordenates to the neck ones -> tW=[tXw, tYw, tZw] */
/* Parameters of the transformation of pan's coordenates to the head ones -> t=[tY, tZ] */
/* Discomfort Weights: Wd=[wvL, wvR, wp, wt] */
/* Energie Weights -> We=[wvL, wvR, wp, wt] */
/* Limit Weights -> Wl=[wvLi, wvLs, wvRi, wvRs, wpi, wps, wti, wts] */
/* Total Weights -as W=[wd, we, wl] */
/* image size -> W, H */
/* jointsLimits=[vLi, vLs, vRi, vRs, pi, ps, ti, ts] i:inferior; s:superior */

#include "../myMatrix.h"

//#include <winsock2.h>
//#include <time.h>
//#include <io.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <fcntl.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <string.h>
//#include <winbase.h>
//#include <conio.h>
//#include <ctime>
//#include <windows.h> 
//#include <new.h>

#include <yarp/os/Time.h>

class headkin{

private:

	//Input Parameters
	double jointsLimits[8];
	double tW[3];
	double t[2];
	double B;
	double Wd[4];
	double We[4];
	double Wl[8];
	double W[3];
	double PanStep;

	//auxiliar variables
	double lastJoints[4]; //{vL vR p t}
	double Po[3], Pl[3], Pr[3];

	//Jacobians
	myMatrix<double> dFdPo, dPdqo, dPdpo;

	//Time variables
	yarp::os::Time Tm;

	double Ktt, Kti, Ktf;

public:
	headkin()
	{

		// initialises all class's variables with zero
		for(int i=0; i<8; i++)
			jointsLimits[i]= 0;

		for(int i=0; i<3; i++)
			tW[i]= 0;

		for(int i=0; i<2; i++)
			t[i]= 0;

		B = 0;

		for(int i=0; i<4; i++)
			Wd[i]= 0;

		for(int i=0; i<4; i++)
			We[i]= 0;

		for(int i=0; i<8; i++)
			Wl[i]= 0;

		for(int i=0; i<3; i++)
			W[i]= 0;

		PanStep=0;

		for(int i=0; i<4; i++)
			lastJoints[i]=0;

		for(int i=0; i<3; i++){
			Po[i]=0;
			Pl[i]=0;
			Pr[i]=0;
		}

		myMatrix<double> _dFdPo(3,6), _dPdqo(6,4), _dPdpo(6,3);

		dFdPo = _dFdPo;
		dPdqo = _dPdqo;
		dPdpo = _dPdpo;

	};

	~headkin(){};

	void initHEADKIN(double _jointsLimits[8], double _tW[3], double _t[2], double _B, double _Wd[4], double _We[4], double _Wl[8], double _W[3], double _PanStep);

	void baltaJacobians(double actHJ[4], myMatrix<double> *Jq=NULL, myMatrix<double> *Jp=NULL);
	double *genVect(double DownLim, double UpLim, double step, int &vectSize);
	int CalcMin(double *J, int sizeJ);
	void testLimits(double joint[4], bool LJ);
	void applyLimits(double joint[4]);
	void headDirKin(double Joints[4], double pont3d[3]);
	bool headInvKin(double actHeadJoints[4], double pont3d[3], double jointAng[4], double &Ktime);

};
