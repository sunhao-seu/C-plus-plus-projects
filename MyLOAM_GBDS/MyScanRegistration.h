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

	float cloudCurvature[40000];	//c。。曲率，设了一个很大的数组，40000； 硬件不支持vector
	int cloudSortInd[40000];	//对每条线的曲率排序后得到的序列
	int cloudNeighborPicked[40000];	//如果某索引为1，那么该索引对应的点不能再被选为特征点了。
	int cloudNeighborPickedForCorner[40000];
	int cloudLabel[40000];	//点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了1,0和1构成了点云全部的点)

	int imuPointerFront; //last实际上用来记录的是imuhandler处理的msg时间戳已经到哪了
	int imuPointerLast;//front用来记录laserscanhandler处理的点云时间到哪了
	static const int imuQueLength = 200;	//imu循环队列长度

											//点云数据开始第一个点的位移/速度/欧拉角
	float imuRollStart, imuPitchStart, imuYawStart;
	float imuRollCur, imuPitchCur, imuYawCur;

	float imuVeloXStart, imuVeloYStart, imuVeloZStart;
	float imuShiftXStart, imuShiftYStart, imuShiftZStart;

	//当前点的速度，位移信息
	float imuVeloXCur, imuVeloYCur, imuVeloZCur;
	float imuShiftXCur, imuShiftYCur, imuShiftZCur;

	//每次点云数据当前点相对于开始第一个点的畸变位移，速度
	float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
	float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

	//IMU信息
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