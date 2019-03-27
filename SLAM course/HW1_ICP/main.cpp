//#include<opencv2\opencv.hpp>
//#include<iostream>
//#include<fstream>
//#include<string>
//#include<sstream>
//#include <Eigen/Eigen>
#include<read_files.h>
#include "Eigen\Dense"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>	//pointcloud transform
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>	//FILTER
#include <pcl/registration/icp.h>	//ICP
#include <pcl/kdtree/kdtree_flann.h> //kd_tree
#include <ICP_function.h>
#include <cstdio>	//remove file

//#define MY_VTK_VIEW_ON

#ifdef MY_VTK_VIEW_ON
#include <pcl/io/vtk_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif


int main(int argc, char** argv)
{
	std::string depth_file = "depth.txt";
	std::vector<std::string> depth_name;	//depth file name array
	read_picture_txt(depth_file, depth_name);	//get depth files path and name

	PointCloud::Ptr SourcePointCloud(new PointCloud);	//SourcePointCloud and TargetPointCloud
	PointCloud::Ptr TargetPointCloud(new PointCloud);
	PointCloud::Ptr pointCloud_icp(new PointCloud);
	PointCloud::Ptr SourcePointCloud_new(new PointCloud);
	PointCloud::Ptr SourcePointCloud_corres(new PointCloud);	//nearest search correspondance of cloud1
	PointCloud::Ptr TargetPointCloud_corres(new PointCloud);	//nearest search correspondance of cloud2
	PointT p;	//store the global pose for a moment
	Eigen::Matrix4d ICP_transformation;		//transformation matrix
	Eigen::Matrix4d ICP_transformation_all = Eigen::Matrix4d::Identity();		//transformation matrix
	std::vector<Eigen::Matrix4d> ICP_transformation_store;		//store transformation matrix between pictures
	std::vector<double> E_store;

	bool process = 1; // while condition
	int picture_ind = 0;	//read depth picture indice
	get_cloud(depth_name, picture_ind, SourcePointCloud_new);
	while (process)
	{
		// update source points cloud
		*SourcePointCloud = *SourcePointCloud_new;
		TargetPointCloud->clear();
		
		picture_ind = picture_ind + 20;
		if (picture_ind >= depth_name.size())
		{break;}

		std::cout << "compare with picture " << (picture_ind-20) << " and " << picture_ind << std::endl;
		// read new point cloud
		get_cloud(depth_name, picture_ind, TargetPointCloud);

		bool ICP_continue = 1;
		double E_icp_threshold = 0.00001;
		int iteration_count = 1;
		double pre_error = 0.0;
		while (ICP_continue)
		{


			//ICP algorithm by my code;
			//preprocessing;  find correspondance; remove the outliers.
			Correspondance_search(SourcePointCloud, TargetPointCloud, SourcePointCloud_corres, TargetPointCloud_corres);

			// icp iteration
			ICP_transformation = ICP_iterate_p2p(SourcePointCloud_corres, TargetPointCloud_corres);

			//std::cout << "ICP_transformation= " << ICP_transformation << std::endl;
			ICP_transformation_all *= ICP_transformation;
			std::cout << "ICP_transformation_all= " << ICP_transformation_all << std::endl;
			
			// transformation of original cloud to icp cloud; PCL library
			pcl::transformPointCloud(*SourcePointCloud_corres, *pointCloud_icp, ICP_transformation);

			// calculate relative error or absolute error
			double E_icp_absolute = 0.0;
			double E_icp_realative = 0.0;
			//double E_icp1 = 0.0;
			for (int i = 0; i < pointCloud_icp->size(); i++)
			{
				//E_icp1 += ((Points_rmcen1[i].x)*(Points_rmcen1[i].x) + (Points_rmcen1[i].y)*(Points_rmcen1[i].y) + (Points_rmcen1[i].z)*(Points_rmcen1[i].z));
				E_icp_realative += ((SourcePointCloud_corres->points[i].x - pointCloud_icp->points[i].x)*(SourcePointCloud_corres->points[i].x - pointCloud_icp->points[i].x)
					+ (SourcePointCloud_corres->points[i].y - pointCloud_icp->points[i].y)*(SourcePointCloud_corres->points[i].y - pointCloud_icp->points[i].y)
					+ (SourcePointCloud_corres->points[i].z - pointCloud_icp->points[i].z)*(SourcePointCloud_corres->points[i].z - pointCloud_icp->points[i].z));
			}
			//int N_SMALL = (N1 >= N2) ? N2 : N1;
			for (int i = 0; i < pointCloud_icp->size(); i++)
			{
				//absolute error
				E_icp_absolute += ((TargetPointCloud_corres->points[i].x - pointCloud_icp->points[i].x)*(TargetPointCloud_corres->points[i].x - pointCloud_icp->points[i].x)
					+ (TargetPointCloud_corres->points[i].y - pointCloud_icp->points[i].y)*(TargetPointCloud_corres->points[i].y - pointCloud_icp->points[i].y)
					+ (TargetPointCloud_corres->points[i].z - pointCloud_icp->points[i].z)*(TargetPointCloud_corres->points[i].z - pointCloud_icp->points[i].z));
			}
			//E_icp1 = E_icp1 - 2 * (sigma[0] + sigma[1] + sigma[2]);
			E_icp_realative = E_icp_realative/ pointCloud_icp->size();
			std::cout << "E_icp= " << E_icp_realative << std::endl;
			std::cout << "E_icp_absolute= " << E_icp_absolute << std::endl;

			if (E_icp_realative > E_icp_threshold)
			{
				ICP_continue = 1;
				//*SourcePointCloud = *pointCloud_icp;
				pcl::transformPointCloud(*SourcePointCloud, *SourcePointCloud, ICP_transformation);
				iteration_count++;
				pre_error = E_icp_realative;
			}
			else 
			{ 
				ICP_continue = 0; 
				//store the transformation matrix
				ICP_transformation_store.push_back(ICP_transformation_all);
				//store the error of last result
				E_store.push_back(E_icp_realative);
				E_store.push_back(E_icp_absolute);
				pcl::transformPointCloud(*SourcePointCloud, *SourcePointCloud_new, ICP_transformation);
			}

			//important 
			SourcePointCloud_corres->clear();
			TargetPointCloud_corres->clear();

		}

		
		//************ICP algorithm by PCL library...*****************
		//pcl::IterativeClosestPoint<PointT, PointT> icp;
		////icp.setInputCloud(SourcePointCloud);
		//icp.setInputSource(SourcePointCloud);
		//icp.setInputTarget(TargetPointCloud);
		//pcl::PointCloud<PointT> Final;
		//icp.align(Final);
		//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		//	icp.getFitnessScore() << std::endl;
		//std::cout << icp.getFinalTransformation() << std::endl;

		/*//*******transformation of original cloud to next cloud; for test*******************
		Eigen::Matrix4f icp_transformation = Eigen::Matrix4f::Identity();
		icp_transformation << 0.999723, 0.0157258, 0.0174915, 0.0216797,
			-0.0160207, 0.999731, 0.0168878, 0.0273432,
			-0.0172207, -0.017164, 0.999705, 0.0396911,
			0, 0, 0, 1;
			my:ICP_transformation_all= 0.999803  0.00371073   0.0269258   0.0216221
			-0.00376861    0.999991  0.00206204  0.00739735
			-0.0022712 -0.00226537    0.999867   0.0280159
			0           0           0           1
		PointCloud::Ptr pointCloud_icp(new PointCloud);
		pcl::transformPointCloud(*SourcePointCloud, *pointCloud_icp, icp_transformation);*/


#ifdef MY_VTK_VIEW_ON
		// Visualize both the original and the next cloud with white and green points.
		pcl::visualization::PCLVisualizer viewer("cloud viewer");
		std::stringstream ss;
		ss << picture_ind-20;
		std::string iterations_cnt = "original cloud iteration = " + ss.str();
		viewer.addPointCloud(SourcePointCloud_store, iterations_cnt);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler1(TargetPointCloud, 0, 255, 0);
		std::stringstream ss2;
		ss2 << picture_ind;
		std::string iterations_cnt2 = "next cloud iteration = " + ss.str();
		viewer.addPointCloud(TargetPointCloud, colorHandler1, iterations_cnt2);
		// The icp result points will be red in color.
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(pointCloud_icp, 255, 0, 0);
		std::stringstream ss3;
		ss3 << picture_ind;
		std::string iterations_cnt3 = "icp calculated original to next = " + ss.str();
		viewer.addPointCloud(pointCloud_icp, colorHandler, iterations_cnt3);
		// Add 3D colored axes to help see the transformation.
		viewer.addCoordinateSystem(1.0, "reference", 0);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}
		viewer.removeShape(iterations_cnt);
		viewer.removeShape(iterations_cnt2);
		viewer.removeShape(iterations_cnt3);

#endif

		//system("pause");
	}

	std::cout << "ICP done, results are in ICP_transformation_store vector and E_store vector" << std::endl;
	std::cout << "press any key to exit! " << std::endl;

	// write the ICP_T matrix to a .txt file
	remove("ICP_transformation_store.txt");
	std::ofstream fout;
	fout.open("ICP_transformation_store.txt", std::ios::app);//在文件末尾追加写入
	for (int i = 0; i < ICP_transformation_store.size(); i++)
	{
		fout << ICP_transformation_store[i] << std::endl;//每次写完一个矩阵以后换行
	}
	fout.close();

	//read from txt file
	//std::vector<Matrix4f> T0;//创建储存多个矩阵的vector对象
	//std::ifstream fin;
	//fin.open("mat.txt");
	////下面for的终止条件中的6表示txt文件中矩阵的个数
	//for (int n = 0; n < 6; n++) {
	//	Matrix4f tempT;
	//	for (int i = 0; i < 4; i++) {
	//		for (int j = 0; j < 4; j++) {
	//			fin >> tempT(i, j);
	//		}
	//	}//for循环遍历依次读取
	//	T0.push_back(tempMat);
	//}
	//fin.close();

	getchar();
	return 0;

}