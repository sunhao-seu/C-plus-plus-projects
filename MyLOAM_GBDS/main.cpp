#include <LOAM_useful_functions.h>
#include "MyScanRegistration.h"
#include "MyLaserOdometry.h"
#include "MyLaserMapping.h"

#ifdef MY_VTK_TRAJACTORY_VIEW_ON
#undef MY_VTK_MAP_VIEW_ON
#endif // MY_VTK_TRAJACTORY_VIEW_ON


int main(int argc, char* argv[])
{
#ifdef MY_VTK_MAP_VIEW_ON
	pcl::visualization::CloudViewer MapViewer("mapping viewer");
#endif

#ifdef USE_GBDS_ON
	remove("gbds_time_store.txt");
#endif
#ifdef USE_KDTREE_ON
	remove("kdtree_time_store.txt");
#endif

#ifdef BOTH_KDTREE_AND_GBDS
	remove("kdtree_time_store.txt");
	remove("dataset_size.txt");
#endif
	


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

	int count = 1000;	//Lidar frames count

	while (true)
	{
		//if (count % 100 == 0)
			std::cout << "Frame " << count << std::endl;
			if (count > 1300)
			{
				cout << "haha0";
			}
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

			OdometryValueBack = Odometrier.MyLaserOdometryHandler(ScanValueBack);

			// process laserMapping and return the value to mappingBackValue
			MappingBackValue = Mapper.LaserMappingHandler(OdometryValueBack);

			//将所有数据放入总地图
			*pFinalMap += *(MappingBackValue.laserCloudFullRes);

			//pFinalMapVoxel->clear();
			downSizeFilterMap.setInputCloud(pFinalMap);
			downSizeFilterMap.filter(*pFinalMap);


#ifdef MY_VTK_MAP_VIEW_ON
			//if (count % 500 == 0)
			//{
			MapViewer.showCloud(pFinalMap);
			//}
#endif

		}
		count += 2;

	}

	getchar();
	return 0;
}
