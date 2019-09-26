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

	//��������groundtruth�ı���
	//double **groundtruth;

	//
	//���ڴ洢��ǰRANSAC��ƽ�淨������ǰһ֡��ƽ�淨����
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

	static const int stackFrameNum = 1;		//��ֵ //���ƽ��յ��ĵ������ݣ�ÿ����֡����һ��
	static const int mapFrameNum = 1;		//��ֵ //���ƴ���õ��ĵ���map��ÿ������publich��rviz��ʾ

	//ʱ���
	//double timeLaserCloudCornerLast;
	//double timeLaserCloudSurfLast;
	//double timeLaserCloudFullRes;
	//double timeLaserOdometry;

	bool newLaserCloudCornerLast;
	bool newLaserCloudSurfLast;
	bool newLaserCloudFullRes;
	bool newLaserOdometry;

	int laserCloudCenWidth;			 //cube���ĵ��Ӧ��������������λ�õ��ƶ��仯
	int laserCloudCenHeight;		 //cube���ĵ��Ӧ��������������λ�õ��ƶ��仯
	int laserCloudCenDepth;			 //cube���ĵ��Ӧ��������������λ�õ��ƶ��仯
	static const int laserCloudWidth = 21; // ��cube�ؿ���ķָ���� //cube�������������
	static const int laserCloudHeight = 11;  // �߷������
	static const int laserCloudDepth = 21;  // ��ȷ������
	static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;  // ��cube������Ҳ��������4851��cube�������״�����ݣ������״�������ô��cube�Խӵģ�

	int laserCloudValidInd[125];//lidar����Χ��(FOV)�ĵ��Ƽ�����
	int laserCloudSurroundInd[125];//lidar��Χ�ĵ��Ƽ�����

	int frameCount;
	int mapFrameCount;
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;		//odometry����ֵ����//���½��յ��ı��ص�
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;			//odometry����ֵ����//���½��յ���ƽ���
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;		//laserCloudCornerStack2 ����voxel filter��ĵ㼯//��ŵ�ǰ�յ����²���֮��ı��ص�(in the local frame)
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;		//laserCloudSurfStack2 ����voxel filter��ĵ㼯//��ŵ�ǰ�յ����²���֮���ƽ���(in the local frame)
	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack2;		//ת������ͼ����ϵ�µĵ�ǰ֡�Ľǵ� //��ŵ�ǰ�յ��ı��ص㣬��Ϊ�²���������Դ
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack2;		//ת������ͼ����ϵ�µĵ�ǰ֡��ƽ���//��ŵ�ǰ�յ���ƽ��㣬��Ϊ�²���������Դ
	pcl::PointCloud<PointType>::Ptr laserCloudOri;			//ԭʼ��������
	pcl::PointCloud<PointType>::Ptr coeffSel;
	pcl::PointCloud<PointType>::Ptr laserCloudSurround;//ƥ��ʹ�õ������㣨�²���֮��ģ�
	pcl::PointCloud<PointType>::Ptr laserCloudSurround2;//ƥ��ʹ�õ������㣨�²���֮ǰ�ģ�
	pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;//map����ȡ��ƥ��ʹ�õı��ص�
	pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;//map����ȡ��ƥ��ʹ�õ�ƽ���
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;					//odometry����ֵ������������//����ȫ����
	//cube�ǵ�����;width*height*depth   //array������50��Ϊ��λ���������ͼ�����й����л�һֱ����(����Ҫ�Ļ��ɿ����Ż���ֻ������ڵģ�����ֱ�����鿪Сһ��)
	//���array��һ���ߴ�ΪlaserCloudNum�����飬laserCloudCornerArray[i]�����һ����cube,һ����cube �����ܶ�㣿
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];  //��ű��ص��cube������ʵ���ǽ�һ����Χ�ڵĵ�voxel���ˡ���cube���ɵ�Ϳ����ˣ�һ��cube��Ӧһ����
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];//���ƽ����cube�� 
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];//�м����������²������ı��ص�
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];//�м����������²�������ƽ���

	/*************��Ƶת����**************/
	//odometry����õ��ĵ���������ϵ�µ�ת�ƾ���
	Twist transformSum;					//odometry����ֵ������һ֡��ת������ȫ������ϵ��
	//ת��������ֻʹ���˺�����ƽ������
	Twist transformIncre;				//ǰһ֡�״�λ�˼��ۼ�λ�ˣ��õ���ǰ֡�״���ȫ�ֵ�ͼ�е�λ�ˣ���x,y,z��
	
	/*************��Ƶת����*************/
	//2����ʼλ��Ϊԭ�����������ϵ�µ�ת�����󣨲²�������Ķ���
	Twist transformTobeMapped;			//��ǰ֡�����е�ͼ�еľ���λ�ˡ�x,y,z,rot_x,rot_y,rot_z��,Ҳ���ǿ���ֱ������map��
	//2���mapping֮ǰ��Odometry�������������ϵ��ת������ע����Ƶ������һ����transformSumһ����
	Twist transformBefMapped;			//transformBefMapped��ǰһ֡����˵��һ�ε�ͼ���״��λ�ˣ����˴�������õ���ǰ֡�״���ȫ�ֵ�ͼ�е�λ��
	//2���mapping֮��ľ���mapping΢��֮���ת������
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