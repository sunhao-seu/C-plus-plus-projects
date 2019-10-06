#include <LOAM_useful_functions.h>

PointCloud::Ptr pcap2PointCloud(std::vector<_array> &Points)
{
	PointCloud::Ptr cloud(new PointCloud);
	int cnt = 0;
	for (int i = 0; i < Points.size(); i++)
	{
		//if (Points[i].LaserId == 1)
		{
			PointT p;
			cnt++;
			p.x = Points[i].xyz[0];
			p.y = Points[i].xyz[1];
			p.z = Points[i].xyz[2];

			p.intensity = Points[i].xyz[2];
			cloud->points.push_back(p);		//push points information to cloud
		}

	}
	// set the height,width and isdense
	cloud->height = 1;		//unorganized dataset
	cloud->width = cnt;		//total number of points in dataset
	cloud->is_dense = false;	//whether it might contain Inf/NaN values*(false).

	return cloud;
}

PointCloudXYZ::Ptr PointT2PointXYZ(const PointCloud::Ptr & PointTCloud)
{
	PointCloudXYZ::Ptr PointXYZCloud(new PointCloudXYZ);
	int cnt = 0;
	for (int i = 0; i < PointTCloud->size(); i++)
	{
		//if (Points[i].LaserId == 1)
		{
			PointXYZ p;
			cnt++;
			p.x = PointTCloud->points[i].x;
			p.y = PointTCloud->points[i].y;
			p.z = PointTCloud->points[i].z;

			PointXYZCloud->points.push_back(p);		//push points information to cloud
		}

	}
	// set the height,width and isdense
	PointXYZCloud->height = 1;		//unorganized dataset
	PointXYZCloud->width = cnt;		//total number of points in dataset
	PointXYZCloud->is_dense = false;	//whether it might contain Inf/NaN values*(false).

	return PointXYZCloud;
}

PointCloudXYZRGB PointT2PointXYZRGB(const PointCloud & PointTCloud)
{
	PointCloudXYZRGB PointXYZRGBCloud;
	int cnt = 0;
	for (int i = 0; i < PointTCloud.size(); i++)
	{
		//if (Points[i].LaserId == 1)
		{
			PointXYZRGB p;
			cnt++;
			p.x = PointTCloud.points[i].x;
			p.y = PointTCloud.points[i].y;
			p.z = PointTCloud.points[i].z;
			//默认白色
			p.r = 255;
			p.g = 255;
			p.b = 255;

			PointXYZRGBCloud.points.push_back(p);		//push points information to cloud
		}

	}
	// set the height,width and isdense
	PointXYZRGBCloud.height = 1;		//unorganized dataset
	PointXYZRGBCloud.width = cnt;		//total number of points in dataset
	PointXYZRGBCloud.is_dense = false;	//whether it might contain Inf/NaN values*(false).

	return PointXYZRGBCloud;
}

PointCloudXYZRGB::Ptr Show_Registration(ScanRegistrationBack & ScanBackValue2)
{
	PointCloudXYZRGB::Ptr Show_sweep(new PointCloudXYZRGB);
	PointCloudXYZRGB Flat_a = PointT2PointXYZRGB(ScanBackValue2.surfPointsFlat);
	PointCloudXYZRGB LessFlat_a = PointT2PointXYZRGB(ScanBackValue2.surfPointsLessFlat);
	PointCloudXYZRGB LessSharp_a = PointT2PointXYZRGB(ScanBackValue2.cornerPointsLessSharp);
	PointCloudXYZRGB Sharp_a = PointT2PointXYZRGB(ScanBackValue2.cornerPointsSharp);
	for (int i = 0; i < Flat_a.size(); i++)
	{
		Flat_a.points[i].g = 0;
		Flat_a.points[i].b = 0;
	}
	for (int i = 0; i < LessFlat_a.size(); i++)
	{
		LessFlat_a.points[i].g = 0;
		LessFlat_a.points[i].b = 0;
	}
	for (int i = 0; i < LessSharp_a.size(); i++)
	{
		LessSharp_a.points[i].r = 0;
		LessSharp_a.points[i].b = 0;
	}
	for (int i = 0; i < Sharp_a.size(); i++)
	{
		Sharp_a.points[i].r = 0;
		Sharp_a.points[i].g = 0;
	}
	*Show_sweep += Flat_a;
	//*Show_sweep += LessFlat_a;
	//*Show_sweep += LessSharp_a;
	*Show_sweep += Sharp_a;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(Show_sweep);
	viewer->addPointCloud<pcl::PointXYZRGB>(Show_sweep, rgb, "sample cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(0);
	}

	return Show_sweep;
}

void Lidar_Trajactory(const float LidarPose[6], PointCloud::Ptr & Trajactory)
{
	PointT p;
	p.x = LidarPose[3];
	p.y = LidarPose[4];
	p.z = LidarPose[5];
	p.intensity = 0;
	Trajactory->points.push_back(p);

	// set the height,width and isdense
	Trajactory->height = 1;		//unorganized dataset
	Trajactory->width += 1;		//total number of points in dataset
	Trajactory->is_dense = false;	//whether it might contain Inf/NaN values*(false).
}

void Read_PCD_colored_Show(string & filename)
{
	/*Show colored point cloud*/
	PointCloud::Ptr Mapcloud(new PointCloud);
	pcl::io::loadPCDFile(filename, *Mapcloud);
	//VOCEL FILTER
	//pcl::VoxelGrid<PointT> TESTFilterMap;
	//TESTFilterMap.setLeafSize(0.4, 0.4, 0.4);		// set the grid spacing; unit grid is 0.2
	//TESTFilterMap.setInputCloud(Mapcloud);
	//TESTFilterMap.filter(*Mapcloud);
	pcl::visualization::CloudViewer TestViewer("file viewer");
	TestViewer.showCloud(Mapcloud);
	while (!TestViewer.wasStopped())
	{}
}

void Read_PCD_Show_trajactory(string Mapname, string Trajactoryname)
{
	/*Show colored point cloud*/
	PointCloud::Ptr Mapcloud(new PointCloud);
	pcl::io::loadPCDFile(Mapname, *Mapcloud);
	PointCloud::Ptr Trajactorycloud(new PointCloud);
	pcl::io::loadPCDFile(Trajactoryname, *Trajactorycloud);

	//VOCEL FILTER
	pcl::VoxelGrid<PointT> TESTFilterMap;
	TESTFilterMap.setLeafSize(0.5, 0.5, 0.5);		// set the grid spacing; unit grid is 0.2
	TESTFilterMap.setInputCloud(Mapcloud);
	TESTFilterMap.filter(*Mapcloud);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> Testviewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	std::string test1, test2;

	std::stringstream ss;
	ss << 1;

	test1 = "Map cloud" + ss.str();
	Testviewer->addPointCloud<PointT>(Mapcloud, test1);

	pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(Trajactorycloud, 255, 0, 0);
	test2 = "Trajactory " + ss.str();
	Testviewer->addPointCloud<PointT>(Trajactorycloud, colorHandler, test2);

	// Add 3D colored axes to help see the transformation.
	Testviewer->addCoordinateSystem(1.0, "reference", 0);
	while (!Testviewer->wasStopped())
	{
		Testviewer->spinOnce();
	}
}