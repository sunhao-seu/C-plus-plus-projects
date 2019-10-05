#pragma once

#ifndef LASER_ODOMETRY_H
#define LASER_ODOMETRY_H

#include <LOAM_useful_functions.h>
#include <cmath>
//#include <loam_velodyne/common.h>
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

	static const int skipFrameNum = 0;	//仅从字面意思上理解，应该是跳过的帧数；；隔多少帧处理一次
	bool systemInited;					//系统初始化标志位

	bool newCornerPointsSharp;		//一堆标志位，判断新数据是否已经进来了。
	bool newCornerPointsLessSharp;
	bool newSurfPointsFlat;
	bool newSurfPointsLessFlat;
	bool newLaserCloudFullRes;
	bool newImuTrans;

	pcl::PointCloud<PointType>::Ptr cornerPointsSharp;		//从Registrationvalue复制过来的sharp point//尖锐点云；sharp  一堆点云类型指针； 当前帧的角点点云
	pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;	//从Registrationvalue复制过来的lesssharp point
	pcl::PointCloud<PointType>::Ptr surfPointsFlat;
	pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;	//角点点云， corner,  上一帧的
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
	pcl::PointCloud<PointType>::Ptr laserCloudOri;			//存储当前帧特征点与上一帧匹配上的特征点；；；也就是没匹配上就不存入这里。
	pcl::PointCloud<PointType>::Ptr coeffSel;				//存储de,dh的偏导信息，用于之后构建jacobian矩阵并解算。； Ori是对应的点集。
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;		//lasercloud，就是Registration里面按线分好的点集
	pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;		//存储上一帧的角点信息的kdtree
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;		//存储上一帧的平面点信息的kdtree

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
	Twist transform;		////当前帧相对上一帧的状态转移量，in the local frame
	Twist transformSum;		//总的转换 当前帧相对于第一帧的状态转移量，in the global frame
	Angle imuRollStart, imuPitchStart, imuYawStart;
	Angle imuRollLast, imuPitchLast, imuYawLast;
	Vector3 imuShiftFromStart;
	Vector3 imuVeloFromStart;

	void TransformToStart(PointType const * const pi, PointType * const po);	//将当前帧的点云时间戳都转移到初始时刻start
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

	int biggest_Last = 0;
	int biggest_Current = 0;

	int OdometryCornerStoreLast[1000000];
	int OdometryCornerStore[1000000];
	int OdometrySurfStoreLast[1000000];
	int OdometrySurfStore[1000000];
	int OdometryFrame_count = 0;		//for module1 Find the biggest number of feature points of each frame. 
	int Odometrybiggest_Last = 0;
	int Odometrybiggest_Current = 0;


	//my knn parameters
	struct ThreeDimPoint corner_last_data_set[k_data_set_size];
	int corner_last_useful_data_set_size = 0;
	struct MaxMin corner_last_data_max_min;
	struct SplitArraySize corner_last_split_array_size;
	type_point corner_last_x_split_array[k_axis_spaces_max], corner_last_y_split_array[k_axis_spaces_max], corner_last_z_split_array[k_axis_spaces_max];
	int corner_last_sub_sets[k_sub_region_max][k_sub_space_array_size];
	int corner_last_sub_sets_size[k_sub_region_max] = { 0 };		//important

	struct ThreeDimPoint surf_last_data_set[k_data_set_size];
	int surf_last_useful_data_set_size = 0;
	struct MaxMin surf_last_data_max_min;
	struct SplitArraySize surf_last_split_array_size;
	type_point surf_last_x_split_array[k_axis_spaces_max], surf_last_y_split_array[k_axis_spaces_max], surf_last_z_split_array[k_axis_spaces_max];
	int surf_last_sub_sets[k_sub_region_max][k_sub_space_array_size];
	int surf_last_sub_sets_size[k_sub_region_max] = { 0 };		//important

	clock_t time1, time2, time3, time4;
	double time_last1, time_last2, time_last3;
};
#endif