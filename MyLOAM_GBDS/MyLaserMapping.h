#pragma once
#ifndef LASER_MAPPING_H
#define LASER_MAPPING_H

#include <LOAM_useful_functions.h>
#include <math.h>

#include <nanoflann_pcl.h>
//#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <math_utils.h>

#include <fstream>

#include <ctime>

class LaserMapping {

public:
	LaserMapping();

	LaserMappingBack LaserMappingHandler(const LaserOdometryBack& odometryValueBack);


private:

	//用于输入groundtruth的变量
	//double **groundtruth;

	//
	//用于存储当前RANSAC的平面法向量和前一帧的平面法向量
	//Eigen::VectorXf planeParBef;
	//Eigen::VectorXf planeParTobe;

	int num_corner;
	int num_surf;

	//std::ofstream outfile;
	//std::ofstream outfile2;
	//int num_id;
	//int num_id2;
	bool systemInit;
	static const float scanPeriod/*=0.1*/;

	static const int stackFrameNum = 1;		//常值 //控制接收到的点云数据，每隔几帧处理一次
	static const int mapFrameNum = 1;		//常值 //控制处理得到的点云map，每隔几次publich给rviz显示

	//时间戳
	//double timeLaserCloudCornerLast;
	//double timeLaserCloudSurfLast;
	//double timeLaserCloudFullRes;
	//double timeLaserOdometry;

	bool newLaserCloudCornerLast;
	bool newLaserCloudSurfLast;
	bool newLaserCloudFullRes;
	bool newLaserOdometry;

	int laserCloudCenWidth;			 //cube中心点对应的索引，会随着位置的移动变化
	int laserCloudCenHeight;		 //cube中心点对应的索引，会随着位置的移动变化
	int laserCloudCenDepth;			 //cube中心点对应的索引，会随着位置的移动变化
	static const int laserCloudWidth = 21; // 子cube沿宽方向的分割个数 //cube的数量，不变的
	static const int laserCloudHeight = 11;  // 高方向个数
	static const int laserCloudDepth = 21;  // 深度方向个数
	static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;  // 子cube总数；也就是用了4851个cube来描述雷达的数据？但是雷达数据怎么跟cube对接的？

	int laserCloudValidInd[125];//lidar视域范围内(FOV)的点云集索引
	int laserCloudSurroundInd[125];//lidar周围的点云集索引

	int frameCount;
	int mapFrameCount;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;		//odometry返回值。。//最新接收到的边沿点
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;			//odometry返回值。。//最新接收到的平面点
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;		//laserCloudCornerStack2 经过voxel filter后的点集//存放当前收到的下采样之后的边沿点(in the local frame)
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;		//laserCloudSurfStack2 经过voxel filter后的点集//存放当前收到的下采样之后的平面点(in the local frame)
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2;		//转换到地图坐标系下的当前帧的角点 //存放当前收到的边沿点，作为下采样的数据源
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2;		//转换到地图坐标系下的当前帧的平面点//存放当前收到的平面点，作为下采样的数据源
	pcl::PointCloud<PointType>::Ptr laserCloudOri;			//原始点云坐标
	pcl::PointCloud<PointType>::Ptr coeffSel;
	pcl::PointCloud<PointType>::Ptr laserCloudSurround;//匹配使用的特征点（下采样之后的）
	pcl::PointCloud<PointType>::Ptr laserCloudSurround2;//匹配使用的特征点（下采样之前的）
	pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;//map中提取的匹配使用的边沿点
	pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;//map中提取的匹配使用的平面点
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;					//odometry返回值。。点云数据//点云全部点
	//cube角点数组;width*height*depth   //array都是以50米为单位的立方体地图，运行过程中会一直保存(有需要的话可考虑优化，只保存近邻的，或者直接数组开小一点)
	//这个array是一个尺寸为laserCloudNum的数组，laserCloudCornerArray[i]存放着一个大cube,一个大cube 包含很多点？
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];  //存放边沿点的cube。。其实就是将一个范围内的点voxel化了。将cube理解成点就可以了，一个cube对应一个点
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];//存放平面点的cube； 
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];//中间变量，存放下采样过的边沿点
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];//中间变量，存放下采样过的平面点

	/*************高频转换量**************/
	//odometry计算得到的到世界坐标系下的转移矩阵
	Twist transformSum;					//odometry返回值。。上一帧的转换矩阵，全局坐标系中
	//转移增量，只使用了后三个平移增量
	Twist transformIncre;				//前一帧雷达位姿减累计位姿，得到当前帧雷达在全局地图中的位姿；【x,y,z】
	
	/*************低频转换量*************/
	//2以起始位置为原点的世界坐标系下的转换矩阵（猜测与调整的对象）
	Twist transformTobeMapped;			//当前帧在已有地图中的绝对位姿【x,y,z,rot_x,rot_y,rot_z】,也就是可以直接拿来map了
	//2存放mapping之前的Odometry计算的世界坐标系的转换矩阵（注：低频量，不一定与transformSum一样）
	Twist transformBefMapped;			//transformBefMapped：前一帧或者说上一次地图中雷达的位姿；；此处相减，得到当前帧雷达在全局地图中的位姿
	//2存放mapping之后的经过mapping微调之后的转换矩阵
	Twist transformAftMapped;			


	void laserCloudCornerLastHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudCornerLast2);
	void laserCloudSurfLastHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudSurfLast2);
	void laserCloudFullResHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudFullRes2);
	void laserOdometryHandler(const float* transformSum2);

	void transformAssociateToMap();
	void pointAssociateToMap(PointType const * const pi, PointType * const po);
	void pointAssociateTobeMapped(PointType const * const pi, PointType * const po);
	void transformUpdate();


	int CornerStore[1000000];
	int MapCornerStore[1000000];
	int SurfStore[1000000];
	int MapSurfStore[1000000];
	int Frame_count = 0;
	int biggest_Map = 0;
	int biggest_Local = 0;

	clock_t time1, time2, time3, time4;
	double time_last1,time_last2,time_last3;
};
#endif