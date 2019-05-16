#pragma once
/*
Define the class of registration
*/

#ifndef SCAN_REGISTRATION_H
#define SCAN_REGISTRATION_H

#include <LOAM_useful_functions.h>

class ScanRegistration {

public:
	ScanRegistration();

	ScanRegistrationBack ScanRegistrationHandle(const PointCloud& laserCloudIn1);

private:

	static const double scanPeriod /*= 0.1*/;

	bool systemInited;		//system init flag

	static const int N_SCANS = 16;		//laser lines

	float cloudCurvature[40000];	//c�������ʣ�����һ���ܴ�����飬40000�� Ӳ����֧��vector
	int cloudSortInd[40000];	//��ÿ���ߵ����������õ�������
	int cloudNeighborPicked[40000];	//���ĳ����Ϊ1����ô��������Ӧ�ĵ㲻���ٱ�ѡΪ�������ˡ�
	int cloudNeighborPickedForCorner[40000];
	int cloudLabel[40000];	//�������:2-�������ʺܴ�1-�������ʱȽϴ�,-1-�������ʺ�С��0-���ʱȽ�С(����1������2,0������1,0��1�����˵���ȫ���ĵ�)

	int imuPointerFront; //lastʵ����������¼����imuhandler�����msgʱ����Ѿ�������
	int imuPointerLast;//front������¼laserscanhandler����ĵ���ʱ�䵽����
	static const int imuQueLength = 200;	//imuѭ�����г���

											//�������ݿ�ʼ��һ�����λ��/�ٶ�/ŷ����
	float imuRollStart, imuPitchStart, imuYawStart;
	float imuRollCur, imuPitchCur, imuYawCur;

	float imuVeloXStart, imuVeloYStart, imuVeloZStart;
	float imuShiftXStart, imuShiftYStart, imuShiftZStart;

	//��ǰ����ٶȣ�λ����Ϣ
	float imuVeloXCur, imuVeloYCur, imuVeloZCur;
	float imuShiftXCur, imuShiftYCur, imuShiftZCur;

	//ÿ�ε������ݵ�ǰ������ڿ�ʼ��һ����Ļ���λ�ƣ��ٶ�
	float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
	float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

	//IMU��Ϣ
	double imuTime[imuQueLength];
	float imuRoll[imuQueLength];
	float imuPitch[imuQueLength];
	float imuYaw[imuQueLength];

	float imuAccX[imuQueLength];
	float imuAccY[imuQueLength];
	float imuAccZ[imuQueLength];

	float imuVeloX[imuQueLength];
	float imuVeloY[imuQueLength];
	float imuVeloZ[imuQueLength];

	float imuShiftX[imuQueLength];
	float imuShiftY[imuQueLength];
	float imuShiftZ[imuQueLength];

};
#endif