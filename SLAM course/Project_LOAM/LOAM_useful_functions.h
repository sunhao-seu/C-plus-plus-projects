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

	pcl::PointCloud<PointT>::Ptr laserCloud;		//��������֮��ĵ㼯
	pcl::PointCloud<PointT> cornerPointsSharp;		//������㡣��Ҳ���Ǹö��ڼ����ĸ�������4ʱʶ�𵽵ļ��������������
	pcl::PointCloud<PointT> cornerPointsLessSharp;	//һ������
	pcl::PointCloud<PointT> surfPointsFlat;		//surfPointsLessFlat ���� surfPointsFlat������
	pcl::PointCloud<PointT> surfPointsLessFlat;
	pcl::PointCloud<pcl::PointXYZ> imuTrans;

};

// registration�е�sharp��Odometry����corner; flat��������surf
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
PointCloudXYZ::Ptr PointT2PointXYZ(const PointCloud::Ptr & PointTCloud);	//XYZIת����XYZ����ʾ�ĵ����ɲ�ɫ��ɺڰ׵�==
PointCloudXYZRGB PointT2PointXYZRGB(const PointCloud & PointTCloud);	//XYZIת����XYZRGB����ʾ�ĵ����ɲ�ɫ��ɺڰ׵�==���벻��ָ�룬��������
PointCloudXYZRGB::Ptr Show_Registration(ScanRegistrationBack & ScanBackValue2);	//��ʾregistration�Ľ������ʱ��֪����ô����ɫ��������������RGB����
void Lidar_Trajactory(const float LidarPose[6], PointCloud::Ptr & Trajactory);	//from transform matrix to pointcloud datatype to show the trajactory in the map
void Read_PCD_colored_Show(string & filename);		//��pcd�ļ�������ɫ��ʾ�õ���
void Read_PCD_Show_trajactory(string Mapname, string Trajactoryname);	//����ͼ�ļ���켣�ļ����ڵ�ͼ����ʾ�켣