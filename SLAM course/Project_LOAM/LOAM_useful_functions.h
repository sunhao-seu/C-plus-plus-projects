#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <cmath>

#include <pcl/point_types.h>

#include <vector>
#include <iostream>

#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <sstream>
#include <fstream>
#include <Eigen/Core>

#include <iostream>

#include <string>

#include <boost/thread/thread.hpp>

#include <loam_velodyne/common.h>

//#define MY_VTK_MAP_VIEW_ON
#define MY_VTK_TRAJACTORY_VIEW_ON

#include <pcl/io/vtk_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>


#define MY_READ_PCAP 

#ifdef MY_READ_PCAP
//#include<My_read_pcap.h>
#include<pcap_read.h>
#else
#include "driver_port.h"
#endif

using namespace std;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointCloud<PointXYZRGB> PointCloudXYZRGB;

/* Return Struct of Registration*/
struct ScanRegistrationBack {

	pcl::PointCloud<PointT>::Ptr laserCloud;		//按线整理之后的点集
	pcl::PointCloud<PointT> cornerPointsSharp;		//更尖锐点。。也就是该段内尖锐点的个数超过4时识别到的尖锐点存入这个数组
	pcl::PointCloud<PointT> cornerPointsLessSharp;	//一般尖锐点
	pcl::PointCloud<PointT> surfPointsFlat;		//surfPointsLessFlat 包含 surfPointsFlat。。。
	pcl::PointCloud<PointT> surfPointsLessFlat;
	pcl::PointCloud<pcl::PointXYZ> imuTrans;

};

// registration中的sharp在Odometry中是corner; flat在这里是surf
struct LaserOdometryBack {

	float transformSum[6];
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

};

struct LaserMappingBack {

	float transformAftMapped[6];

	pcl::PointCloud<PointType>::Ptr laserCloudSurround;
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

};

struct MaintenanceBack {

	float transformMapped[6];
};


/*
Change pcap data to PCL pointCloud Data
*/
PointCloud::Ptr pcap2PointCloud(std::vector<_array> &Points);
PointCloudXYZ::Ptr PointT2PointXYZ(const PointCloud::Ptr & PointTCloud);	//XYZI转换成XYZ，显示的点云由彩色变成黑白的==
PointCloudXYZRGB PointT2PointXYZRGB(const PointCloud & PointTCloud);	//XYZI转换成XYZRGB，显示的点云由彩色变成黑白的==输入不是指针，而是数组
PointCloudXYZRGB::Ptr Show_Registration(ScanRegistrationBack & ScanBackValue2);	//显示registration的结果，当时不知道怎么调颜色，，重新生成了RGB点云
void Lidar_Trajactory(const float LidarPose[6], PointCloud::Ptr & Trajactory);	//from transform matrix to pointcloud datatype to show the trajactory in the map
void Read_PCD_colored_Show(string & filename);		//读pcd文件，并彩色显示该点云
void Read_PCD_Show_trajactory(string Mapname, string Trajactoryname);	//读地图文件与轨迹文件，在地图里显示轨迹