#include <LOAM_useful_functions.h>
#include "MyScanRegistration.h"
#include "MyLaserOdometry.h"
#include "MyLaserMapping.h"


#ifdef MY_VTK_TRAJACTORY_VIEW_ON
#undef MY_VTK_MAP_VIEW_ON
#endif // MY_VTK_TRAJACTORY_VIEW_ON


int main(int argc, char* argv[])
{
#ifdef MY_VTK_TRAJACTORY_VIEW_ON
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	std::string iterations_cnt, iterations_cnt2, iterations_cnt3;
#endif

#ifdef MY_VTK_MAP_VIEW_ON
	pcl::visualization::CloudViewer MapViewer("mapping viewer");
#endif

//test pcd read
//	Read_PCD_Show_trajactory("outside_Map_pcd.pcd", "Trajactory_pcd.pcd");
	


	ScanRegistrationBack ScanValueBack;
	ScanRegistration Scanner;

	LaserOdometryBack OdometryValueBack;
	LaserOdometry Odometrier;

	LaserMappingBack MappingBackValue;
	LaserMapping Mapper;

	//cloud point file name and path
	const std::string fileNamePcap = "outside.pcap";
	const std::string calibrationPath = "VLP-16.xml";
	//get the handle of processing pcap data
	MyHdlGrabber _myHdlGrabber(fileNamePcap, calibrationPath);
	int a = _myHdlGrabber.GetNumberOfFrames();
	std::cout << "Totally " << a << " Frames of lidar data!" << std::endl;

	pcl::VoxelGrid<PointT> downSizeFilterMap;
	downSizeFilterMap.setLeafSize(0.2, 0.2, 0.2);		// set the grid spacing; unit grid is 0.2
	PointCloud::Ptr pFinalMap(new PointCloud());
	PointCloud::Ptr LidarTrajactory(new PointCloud());
	PointCloud::Ptr LidarPose(new PointCloud());
	PointCloud::Ptr OdometryTrajactory(new PointCloud());

	int count = 1000;	//Lidar frames count
	while (true)
	{
		if (count % 100 == 0)
			std::cout << "Frame " << count << std::endl;
		if (_myHdlGrabber.GetFrame(count))
		{
			std::cout << "No Frame" << std::endl;
			break;
		}
		else if (_myHdlGrabber.Points.size() == 0)
		{
			std::cout << "error, no data!" << std::endl;
			std::cout << "break!" << std::endl;
			break;
		}
		else
		{
			//change pcap data to vector array; or to points cloud data type
			PointCloud::Ptr newCloud = pcap2PointCloud(_myHdlGrabber.Points);

			//obtain the scan point cloud data； pre-process cloud points and extract the feature points; correspond to paper 5.1
			ScanValueBack = Scanner.ScanRegistrationHandle(*newCloud);	//传入的是一个指针

			// Show the result of registration
			//PointCloudXYZRGB::Ptr Verify_Registration(new PointCloudXYZRGB);
			//Verify_Registration =  Show_Registration(ScanValueBack);

			//process odometry and return the value to odometryValueBack 
			OdometryValueBack = Odometrier.MyLaserOdometryHandler(ScanValueBack);

			// process laserMapping and return the value to mappingBackValue
			MappingBackValue = Mapper.LaserMappingHandler(OdometryValueBack);

			//将所有数据放入总地图
			*pFinalMap += *(MappingBackValue.laserCloudFullRes);

			Lidar_Trajactory(MappingBackValue.transformAftMapped, LidarTrajactory);
			Lidar_Trajactory(OdometryValueBack.transformSum, OdometryTrajactory);
			LidarPose->points.push_back(LidarTrajactory->points.back());

			//pFinalMapVoxel->clear();
			downSizeFilterMap.setInputCloud(pFinalMap);
			downSizeFilterMap.filter(*pFinalMap);

#ifdef MY_VTK_TRAJACTORY_VIEW_ON
			std::stringstream ss;
			ss << count;
			if (count % 500 == 0)
			{
				viewer->removeShape(iterations_cnt);
				iterations_cnt = "Map cloud sweep = " + ss.str();
				viewer->addPointCloud<PointT>(pFinalMap, iterations_cnt);
			}
			pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(LidarPose, 0, 255, 0);
			iterations_cnt2 = "Map cloud trajactory = " + ss.str();
			viewer->addPointCloud<PointT>(LidarPose, colorHandler, iterations_cnt2);

			if (count % 100 == 0)
			{
				viewer->removeShape(iterations_cnt3);
				pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler2(OdometryTrajactory, 255, 0, 0);
				iterations_cnt3 = "Odometry Trajactory = " + ss.str();
				viewer->addPointCloud<PointT>(OdometryTrajactory, colorHandler2, iterations_cnt3);
			}

			// Add 3D colored axes to help see the transformation.
			viewer->addCoordinateSystem(1.0, "reference", 0);

			viewer->spinOnce(4);
#endif

#ifdef MY_VTK_MAP_VIEW_ON
			if (count % 500 == 0)
			{
				MapViewer.showCloud(pFinalMap);
			}
#endif

		}
		count += 2;

	}

	//sTORE THE POINTcLOUD 
	//PointCloud& Mapcloud = *pFinalMap;    //点云
	////初始化点云数据PCD文件头
	//Mapcloud.width = pFinalMap->points.size();
	//Mapcloud.height = 1;
	//Mapcloud.is_dense = false;
	//pcl::io::savePCDFileASCII("outside_Map_pcd.pcd", Mapcloud);
	//std::cerr << "Saved " << Mapcloud.points.size() << " data points to outside_Map_pcd.pcd." << std::endl;

	pcl::io::savePCDFileASCII("outside_Map_pcd.pcd", *pFinalMap);
	pcl::io::savePCDFileASCII("Trajactory_pcd.pcd", *LidarTrajactory);
	pcl::io::savePCDFileASCII("OdometryTrajactory_pcd.pcd", *OdometryTrajactory);
	std::cerr << "Saved " << pFinalMap->points.size() << " data points to outside_Map_pcd.pcd." << std::endl;
	std::cerr << "Saved " << LidarTrajactory->points.size() << " data points to Trajactory_pcd.pcd." << std::endl;

#ifdef MY_VTK_TRAJACTORY_VIEW_ON
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
#endif

	getchar();
	return 0;
}
