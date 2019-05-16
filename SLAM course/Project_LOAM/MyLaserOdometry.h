#pragma once

#ifndef LASER_ODOMETRY_H
#define LASER_ODOMETRY_H

#include <LOAM_useful_functions.h>
#include <cmath>
#include <loam_velodyne/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <math_utils.h>

#include <fstream>


class LaserOdometry {
public:
	LaserOdometry();
	LaserOdometryBack MyLaserOdometryHandler(const ScanRegistrationBack& ScanValueBack);


private:

	static const float scanPeriod /*= 0.1*/;

	static const int skipFrameNum = 0;	//����������˼����⣬Ӧ����������֡������������֡����һ��
	bool systemInited;					//ϵͳ��ʼ����־λ

	bool newCornerPointsSharp;		//һ�ѱ�־λ���ж��������Ƿ��Ѿ������ˡ�
	bool newCornerPointsLessSharp;
	bool newSurfPointsFlat;
	bool newSurfPointsLessFlat;
	bool newLaserCloudFullRes;
	bool newImuTrans;

	pcl::PointCloud<PointType>::Ptr cornerPointsSharp;		//��Registrationvalue���ƹ�����sharp point//������ƣ�sharp  һ�ѵ�������ָ�룻 ��ǰ֡�Ľǵ����
	pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;	//��Registrationvalue���ƹ�����lesssharp point
	pcl::PointCloud<PointType>::Ptr surfPointsFlat;
	pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;	//�ǵ���ƣ� corner,  ��һ֡��
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
	pcl::PointCloud<PointType>::Ptr laserCloudOri;			//�洢��ǰ֡����������һ֡ƥ���ϵ������㣻����Ҳ����ûƥ���ϾͲ��������
	pcl::PointCloud<PointType>::Ptr coeffSel;				//�洢de,dh��ƫ����Ϣ������֮�󹹽�jacobian���󲢽��㡣�� Ori�Ƕ�Ӧ�ĵ㼯��
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;		//lasercloud������Registration���水�߷ֺõĵ㼯
	pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;		//�洢��һ֡�Ľǵ���Ϣ��kdtree
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;		//�洢��һ֡��ƽ�����Ϣ��kdtree

	int laserCloudCornerLastNum;
	int laserCloudSurfLastNum;

	int pointSelCornerInd[40000];
	float pointSearchCornerInd1[40000];
	float pointSearchCornerInd2[40000];

	int pointSelSurfInd[40000];
	float pointSearchSurfInd1[40000];
	float pointSearchSurfInd2[40000];
	float pointSearchSurfInd3[40000];

	//imu
	Twist transform;		////��ǰ֡�����һ֡��״̬ת������in the local frame
	Twist transformSum;		//�ܵ�ת�� ��ǰ֡����ڵ�һ֡��״̬ת������in the global frame
	Angle imuRollStart, imuPitchStart, imuYawStart;
	Angle imuRollLast, imuPitchLast, imuYawLast;
	Vector3 imuShiftFromStart;
	Vector3 imuVeloFromStart;

	void TransformToStart(PointType const * const pi, PointType * const po);	//����ǰ֡�ĵ���ʱ�����ת�Ƶ���ʼʱ��start
	void AccumulateRotation(Angle cx, Angle cy, Angle cz, Angle lx, Angle ly, Angle lz, Angle &ox, Angle &oy, Angle &oz);
	void PluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
		const Angle& blx, const Angle& bly, const Angle& blz,
		const Angle& alx, const Angle& aly, const Angle& alz,
		Angle &acx, Angle &acy, Angle &acz);
	void TransformToEnd(PointType const * const pi, PointType * const po);

	void laserCloudSharpHandler(const pcl::PointCloud<PointType>& cornerPointsSharp2);
	void laserCloudLessSharpHandler(const pcl::PointCloud<PointType>& cornerPointsLessSharp2);
	void laserCloudFlatHandler(const pcl::PointCloud<PointType>& surfPointsFlat2);
	void laserCloudLessFlatHandler(const pcl::PointCloud<PointType>& surfPointsLessFlat2);
	void laserCloudFullResHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudFullRes2);
	void imuTransHandler(const pcl::PointCloud<pcl::PointXYZ>& imuTrans2);

};
#endif