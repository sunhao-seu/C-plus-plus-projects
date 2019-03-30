#pragma warning(disable: 4996)
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

//#define MY_VTK_VIEW_ON		//Control the points cloud view
//#define ICP_P2L					// Control the P2P algorithm or P2L algorithm;

#ifdef MY_VTK_VIEW_ON
#include <pcl/io/vtk_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif


int main(int argc, char** argv)
{
	std::string depth_file = "depth.txt";
	std::string truth_file_name = "groundtruth.txt";
	std::vector<std::string> depth_name;	//depth file name array
	std::vector<double> file_time_stamp;	//depth file name array
	std::vector<double> groundtruth_time_stamp;	//depth file name array
	read_picture_txt(depth_file, depth_name, file_time_stamp);	//get depth files path and name
	std::vector<Eigen::Matrix4d> ground_truth_RT;
	GetGroundTruth(truth_file_name, groundtruth_time_stamp, ground_truth_RT);	//get groundtruth homogenous matrix

	PointCloud::Ptr SourcePointCloud(new PointCloud);	//SourcePointCloud and TargetPointCloud
	PointCloud::Ptr TargetPointCloud(new PointCloud);
	PointCloud::Ptr pointCloud_icp(new PointCloud);
	PointCloud::Ptr SourcePointCloud_store(new PointCloud);
	PointCloud::Ptr SourcePointCloud_corres(new PointCloud);	//nearest search correspondance of cloud1
	PointCloud::Ptr TargetPointCloud_corres(new PointCloud);	//nearest search correspondance of cloud2
	PointT p;	//store the global pose for a moment

	Eigen::Matrix4d ICP_transformation;		//transformation matrix
	Eigen::Matrix4d ICP_transformation_all;		//transformation matrix
	Eigen::Matrix4d Ground_truth_transformation;		//transformation matrix

	std::vector<Eigen::Matrix4d> ICP_transformation_store;		//store transformation matrix between pictures
	std::vector<Eigen::Matrix4f> ICP_Library_store;		//store transformation matrix between pictures
	std::vector<Eigen::Matrix4d> Ground_truth_transformation_store;		//store transformation matrix between pictures
	std::vector<double> E_store;
	// Output Normals of points surface
	pcl::PointCloud<pcl::Normal>::Ptr Normals(new pcl::PointCloud<pcl::Normal>);

	bool process = 1; // while condition
	int picture_ind = 0;	//read depth picture indice
	Ground_truth_transformation = Get_time_stamp_Matrix(file_time_stamp[picture_ind], groundtruth_time_stamp, ground_truth_RT);
	Ground_truth_transformation_store.push_back(Ground_truth_transformation);
	while (process)
	{
		// update source points cloud
		ICP_transformation_all = Eigen::Matrix4d::Identity();
		TargetPointCloud->clear();
		SourcePointCloud->clear();
		get_cloud(depth_name, picture_ind, SourcePointCloud);

		*SourcePointCloud_store = *SourcePointCloud;

		picture_ind = picture_ind + 20;
		if (picture_ind >= depth_name.size())
		{break;}

		Ground_truth_transformation = Get_time_stamp_Matrix(file_time_stamp[picture_ind], groundtruth_time_stamp, ground_truth_RT);
		Ground_truth_transformation_store.push_back(Ground_truth_transformation);

		std::cout << "compare with picture " << (picture_ind-20) << " and " << picture_ind << std::endl;
		std::cout << "*************************" << std::endl;
		std::cout << "*************************" << std::endl;
		// read new point cloud
		get_cloud(depth_name, picture_ind, TargetPointCloud);

		bool ICP_continue = 1;
		double E_icp_threshold = 0.0001;
		int iteration_count = 1;
		double pre_error = 0.0;
		while (ICP_continue)
		{
			//ICP algorithm by my code;
						//preprocessing;  find correspondance; remove the outliers.
						Correspondance_search(SourcePointCloud, TargetPointCloud, SourcePointCloud_corres, TargetPointCloud_corres);
			
			#ifdef ICP_P2L
						//ICP P2L iteration
						// calculate normals
						Normals->clear();
						Get_Cloud_Normal(TargetPointCloud_corres, Normals);
						ICP_transformation = ICP_iterate_P2L(SourcePointCloud_corres, TargetPointCloud_corres, Normals);
			#else
						// icp  P2P iteration
						ICP_transformation = ICP_iterate_p2p(SourcePointCloud_corres, TargetPointCloud_corres);
			
			#endif
						//std::cout << "ICP_transformation= " << ICP_transformation << std::endl;
						ICP_transformation_all *= ICP_transformation;
						std::cout << "ICP_transformation_all= " << ICP_transformation_all << std::endl;
						
						// transformation of original cloud to icp cloud; PCL library
						pcl::transformPointCloud(*SourcePointCloud_corres, *pointCloud_icp, ICP_transformation);
			
						double E_P2P_realative = 0.0;
						E_P2P_realative = Get_P2P_Relative_error(SourcePointCloud_corres, pointCloud_icp);
						std::cout << "E_P2P_realative= " << E_P2P_realative << std::endl;
			
			#ifdef ICP_P2L			
						double E_P2L_absolute = 0.0;
						E_P2L_absolute = Get_P2L_Absolute_error(pointCloud_icp, TargetPointCloud_corres, Normals);
						std::cout << "E_P2L_absolute= " << E_P2L_absolute << std::endl;
						
			#else	
						// calculate relative error or absolute error
						double E_P2P_absolute = 0.0;
						E_P2P_absolute = Get_P2P_Absolute_error(pointCloud_icp, TargetPointCloud_corres);
						std::cout << "E_icp_absolute= " << E_P2P_absolute << std::endl;
			#endif
			
						if (E_P2P_realative > E_icp_threshold)
						{
							ICP_continue = 1;
							//*SourcePointCloud = *pointCloud_icp;
							pcl::transformPointCloud(*SourcePointCloud, *SourcePointCloud, ICP_transformation);
							iteration_count++;
							pre_error = E_P2P_realative;
						}
						else 
						{ 
							ICP_continue = 0; 
							//store the transformation matrix
							ICP_transformation_store.push_back(ICP_transformation_all);
							//store the error of last result
							E_store.push_back(E_P2P_realative);
			#ifdef ICP_P2L	
							E_store.push_back(E_P2L_absolute);
			#else
							E_store.push_back(E_P2P_absolute);
			#endif
						}
			
						//important 
						SourcePointCloud_corres->clear();
						TargetPointCloud_corres->clear();
			
					}


					// ************ICP algorithm by PCL library...*****************
			//pcl::IterativeClosestPoint<PointT, PointT> icp;
			////icp.setInputCloud(SourcePointCloud);
			//icp.setInputSource(SourcePointCloud);
			//icp.setInputTarget(TargetPointCloud);
			//pcl::PointCloud<PointT> Final;
			//icp.align(Final);
			//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			//	icp.getFitnessScore() << std::endl;
			//std::cout << icp.getFinalTransformation() << std::endl;
			//ICP_Library_store.push_back(icp.getFinalTransformation());

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

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler1(TargetPointCloud, 255, 0, 0);
		std::stringstream ss2;
		ss2 << picture_ind;
		std::string iterations_cnt2 = "next cloud iteration = " + ss.str();
		viewer.addPointCloud(TargetPointCloud, colorHandler1, iterations_cnt2);

		// The icp result points will be red in color.
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(pointCloud_icp, 0, 255, 0);
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
#ifdef ICP_P2L
	remove("ICP_P2L_transformation_store.txt");
	std::ofstream fout;
	fout.open("ICP_P2L_transformation_store.txt", std::ios::app);//在文件末尾追加写入
#else
	remove("ICP_P2P_transformation_store.txt");
	std::ofstream fout;
	fout.open("ICP_P2P_transformation_store.txt", std::ios::app);//在文件末尾追加写入
#endif
	for (int i = 0; i < ICP_transformation_store.size(); i++)
	{
		fout << ICP_transformation_store[i] << std::endl;//每次写完一个矩阵以后换行
	}
	fout.close();

//store the groundtruth RT matrix
	remove("Ground_truth_transformation_store.txt");
	std::ofstream fout2;
	fout2.open("Ground_truth_transformation_store.txt", std::ios::app);//在文件末尾追加写入
	for (int i = 0; i < Ground_truth_transformation_store.size(); i++)
	{
		fout2 << Ground_truth_transformation_store[i] << std::endl;//每次写完一个矩阵以后换行
	}
	fout2.close();

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