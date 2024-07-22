/* KFApp c++ hearder file KFApp.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2017-04-29
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _KFAPP_H
#define _KFAPP_H

#include "psins.h"
#include "./../../../main/main.h"

// #define FRQ	FRQ100
// #define TS	(1.0/FRQ)

#define FRQ SAMPLE_RATE
#define TS	(1.0/FRQ)

extern float my_v[3];

class CKFApp:public CSINSGNSS
{
public:

	CKFApp(double ts);
	virtual void Init(const CSINS &sins0, int grade=-1);
};

void AVPUartOut(const CKFApp &kf);
void AVPUartOut(const CVect3 &att, const CVect3 &vn=O31, const CVect3 &pos=O31);

#endif

