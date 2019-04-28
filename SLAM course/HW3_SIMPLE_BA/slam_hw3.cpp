#pragma warning( disable : 4996)
#include <Bundle_Adjustment.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"		//line

#include <opencv2/opencv.hpp>	//ransac

#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

#include <eigen3/Eigen/SVD>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>


int main()
{
	clock_t time_start, time_finish;
	double  time_duration;
	time_start = clock();

	std::string rgb_file = "rgb.txt";
	std::string depth_file = "depth.txt";
	std::string truth_file_name = "groundtruth.txt";
	std::vector<std::string> rgb_name, depth_name;	//depth file name array
	std::vector<double> rgb_file_time_stamp, depth_file_time_stamp;	//depth file name array
	std::vector<double> groundtruth_time_stamp;	//depth file name array
	read_picture_txt(rgb_file, rgb_name, rgb_file_time_stamp);	//get rgb files path and name
	read_picture_txt(depth_file, depth_name, depth_file_time_stamp);	//get depth files path and name
	std::vector<Eigen::Matrix4d> ground_truth_RT;
	GetGroundTruth(truth_file_name, groundtruth_time_stamp, ground_truth_RT);	//get groundtruth homogenous matrix

	int picture_ind = FirstIndice;
	Eigen::Matrix4d Ground_truth_pose;		//transformation matrix
	std::vector<Eigen::Matrix4d> Ground_truth_store, Camera_Trajactory2;		//store 4*4 matrix of groundtruth pose
	Ground_truth_pose = Get_time_stamp_Matrix(rgb_file_time_stamp[picture_ind], groundtruth_time_stamp, ground_truth_RT);
	Ground_truth_store.push_back(Ground_truth_pose);

	std::vector<cv::Mat> Camera_Trajactory_Transform;	//存储相邻位姿之间的转换矩阵
	cv::Mat Camera_Last_pose = (cv::Mat_<double>(4, 4) <<
		Ground_truth_pose(0, 0), Ground_truth_pose(0, 1), Ground_truth_pose(0, 2), Ground_truth_pose(0, 3),
		Ground_truth_pose(1, 0), Ground_truth_pose(1, 1), Ground_truth_pose(1, 2), Ground_truth_pose(1, 3),
		Ground_truth_pose(2, 0), Ground_truth_pose(2, 1), Ground_truth_pose(2, 2), Ground_truth_pose(2, 3),
		0, 0, 0, 1);
	std::vector<cv::Mat> Camera_Trajactory;	//存储相机位姿
	Camera_Trajactory.push_back(Camera_Last_pose);	//	存入相机初始位姿

	int depth_corres_indice;		//rgb图时间戳对应的深度图的索引
	std::vector<ObservPoints> PointChain;	//特征点链

	bool Calculate_Trajactory = 1;
	while (Calculate_Trajactory)
	{
#ifdef SHOW_INFORMATION
		Calculate_Trajactory = 0;
#endif
		//read adjacent two rgb pictures***********************************************************
		cv::Mat img_1 = cv::imread(rgb_name[picture_ind]);
		depth_corres_indice = Get_depth_corres_indice(rgb_file_time_stamp[picture_ind], depth_file_time_stamp);
		cv::Mat img_d1 = cv::imread(depth_name[depth_corres_indice], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
		picture_ind += 20;
		std::cout << "*****************************************************Compare image " << picture_ind - 20 << " with image " << picture_ind << std::endl;
		if (picture_ind > LastIndice || depth_corres_indice == -1)//rgb_name.size()
		{
			break;
		}
		//得到真实位姿
		Ground_truth_pose = Get_time_stamp_Matrix(rgb_file_time_stamp[picture_ind], groundtruth_time_stamp, ground_truth_RT);
		Ground_truth_store.push_back(Ground_truth_pose);

		cv::Mat img_2 = cv::imread(rgb_name[picture_ind]);
		depth_corres_indice = Get_depth_corres_indice(rgb_file_time_stamp[picture_ind], depth_file_time_stamp);
		cv::Mat img_d2 = cv::imread(depth_name[depth_corres_indice], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
		cv::Mat img_2_copy = img_2.clone();
		cv::Mat img_1_copy = img_1.clone();

		// find the correspondance points ORB and matched.
		std::vector<cv::KeyPoint> keypoints_1;
		std::vector<cv::KeyPoint> keypoints_2;
		std::vector< cv::DMatch > good_matches;
		Feature_Extract_Match(img_1, img_2, keypoints_1, keypoints_2, good_matches);
		if (good_matches.size() < 10)
		{
			std::cout << "Don't find enough match points" << std::endl;
			break;
		}

		//**************** Relative pose estimation *****************************
		//RANSAC匹配过程  Using opencv library;
		std::vector <cv::KeyPoint> Ransaced_keypoints_1, Ransaced_keypoints_2;
		std::vector <cv::DMatch> Ransaced_Matches;

		Ransac_Inliers(good_matches, keypoints_1, keypoints_2, Ransaced_Matches, Ransaced_keypoints_1, Ransaced_keypoints_2);

#ifdef SHOW_INFORMATION
		std::cout << "RANSAC后匹配点数" << Ransaced_Matches.size() << std::endl;
		cv::Mat img_Ransaced_Matches;
		cv::drawMatches(img_1_copy, Ransaced_keypoints_1, img_2_copy, Ransaced_keypoints_2, Ransaced_Matches, img_Ransaced_Matches);
#endif
#ifdef SHOW_IMAGE
		cv::imshow("After RANSAC", img_Ransaced_Matches);
		cv::waitKey(0);
#endif

		// Get R,t from matched points after ransac
		cv::Mat Transform_R, Transform_T;
		Get_RT(Ransaced_keypoints_1, Ransaced_keypoints_2, Ransaced_Matches, Transform_R, Transform_T);

		//验证R,T
		//Verify_R_T(keypoints_1, keypoints_2, Ransaced_Matches, Transform_R, Transform_T);

		//三角化并验证
		std::vector< cv::Point3d > ThreeDpoints;
		triangulation(Ransaced_keypoints_1, Ransaced_keypoints_2, Ransaced_Matches, Transform_R, Transform_T, ThreeDpoints);
		//Verify_Triangulation(keypoints_1, keypoints_2, Ransaced_Matches, Transform_R, Transform_T, points);

		//直接从groundtruth 的depth.png里读出深度
		std::vector<cv::Point3f> pts_3d;
		std::vector<cv::Point2f> pts_2d;
		for (cv::DMatch m : Ransaced_Matches)
		{
			ushort d = img_d1.ptr<unsigned short>(int(Ransaced_keypoints_1[m.queryIdx].pt.y))[int(Ransaced_keypoints_1[m.queryIdx].pt.x)];
			if (d == 0)   // bad depth
				continue;
			float dd = d / 5000.0;		//depth深度到真实深度
			cv::Point2d p1 = pixel2cam(Ransaced_keypoints_1[m.queryIdx].pt);
			pts_3d.push_back(cv::Point3f(p1.x*dd, p1.y*dd, dd));
			pts_2d.push_back(Ransaced_keypoints_2[m.trainIdx].pt);
		}

		std::cout << "3d-2d pairs: " << pts_3d.size() << std::endl;

		//第一幅图是3D点，第二幅图是2D点。直接通过opencv函数来求解RT
		cv::Mat solvepnp_r, solvepnp_T;
		solvePnP(pts_3d, pts_2d, Camera_K, cv::Mat(), solvepnp_r, solvepnp_T, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法
		cv::Mat solvepnp_R;
		cv::Rodrigues(solvepnp_r, solvepnp_R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
#ifdef SHOW_INFORMATION
		std::cout << "solvepnp_R=" << std::endl << solvepnp_R << std::endl;
		std::cout << "solvepnp_T=" << std::endl << solvepnp_T << std::endl;
#endif

		//prepare for BA
#ifdef SHOW_INFORMATION
		std::cout << "Camera_Last_pose=" << std::endl << Camera_Last_pose << std::endl;
#endif
		//处理第一帧情况
		if (picture_ind < 30)
		{
			for (int i = 0; i < pts_3d.size(); i++)
			{
				ObservPoints point_store;
				cv::Point3f World_point;
				point_store.CameraId.push_back((picture_ind - 20) / 20);	//上一帧ID
				point_store.CameraId_UV_Point.push_back(cv::Point2f(Ransaced_keypoints_1[Ransaced_Matches[i].queryIdx].pt.x,
					Ransaced_keypoints_1[Ransaced_Matches[i].queryIdx].pt.y) );	//存入该特征点在上一帧中的UV像素坐标
				point_store.CameraId.push_back((picture_ind) / 20);		//当前帧ID
				point_store.CameraId_UV_Point.push_back(cv::Point2f(Ransaced_keypoints_2[Ransaced_Matches[i].queryIdx].pt.x,
					Ransaced_keypoints_2[Ransaced_Matches[i].queryIdx].pt.y)); //存入该特征点在这一帧中的UV像素坐标

				Local3D_to_World3D(pts_3d[i], Camera_Last_pose, World_point);
				point_store.WorldPoint = World_point;
				//point_store.OB_KeyPoint1 = Ransaced_keypoints_1[Ransaced_Matches[i].queryIdx];	//	应该存入第一幅图的关键点，以便与后面的匹配
				point_store.OB_KeyPoint = Ransaced_keypoints_2[Ransaced_Matches[i].trainIdx];	//	存入第二幅图的关键点
				point_store.Is_in_Chain = true;
				PointChain.push_back(point_store);
			}
		}
		else
		{
			//需要判断之前的特征点是否存储过，旧的更新CameraID信息，新的直接push; 旧的应该还需要更新keypoint信息
			//对于新来的每一个点，判断已存在在Chain里面
			for (int i = 0; i < pts_3d.size(); i++)
			{
				bool KPexisted = 0;
				int KPexisted_indice;
				for (int j = 0; j < PointChain.size(); j++)
				{
					if ( (PointChain[j].Is_in_Chain == true) && (PointChain[j].OB_KeyPoint.pt == Ransaced_keypoints_1[Ransaced_Matches[i].queryIdx].pt) )
					{
						KPexisted = 1;
						KPexisted_indice = j;
						break;
					}
					PointChain[j].Is_in_Chain = false;
				}

				if (KPexisted)
				{
					//如果已经存在，存入当前cameraID,更新keypoint,设Is_in_chain 为true.
					PointChain[KPexisted_indice].CameraId.push_back((picture_ind) / 20); //存入当前ID
					PointChain[KPexisted_indice].CameraId_UV_Point.push_back(cv::Point2f(
						Ransaced_keypoints_2[Ransaced_Matches[i].queryIdx].pt.x,
						Ransaced_keypoints_2[Ransaced_Matches[i].queryIdx].pt.y)); //存入该特征点在这一帧中的UV像素坐标
					PointChain[KPexisted_indice].OB_KeyPoint = Ransaced_keypoints_2[Ransaced_Matches[i].trainIdx];	//更新特征点
					PointChain[KPexisted_indice].Is_in_Chain = true;	//设Is_in_chain 为true.
				}
				else
				{
					ObservPoints point_store;
					cv::Point3f World_point;
					point_store.CameraId.push_back((picture_ind - 20) / 20);	//上一帧ID
					point_store.CameraId_UV_Point.push_back(cv::Point2f(Ransaced_keypoints_1[Ransaced_Matches[i].queryIdx].pt.x,
						Ransaced_keypoints_1[Ransaced_Matches[i].queryIdx].pt.y));	//存入该特征点在上一帧中的UV像素坐标
					point_store.CameraId.push_back((picture_ind) / 20);		//当前帧ID
					point_store.CameraId_UV_Point.push_back(cv::Point2f(Ransaced_keypoints_2[Ransaced_Matches[i].queryIdx].pt.x,
						Ransaced_keypoints_2[Ransaced_Matches[i].queryIdx].pt.y)); //存入该特征点在这一帧中的UV像素坐标

					Local3D_to_World3D(pts_3d[i], Camera_Last_pose, World_point);
					point_store.WorldPoint = World_point;
					//point_store.OB_KeyPoint1 = Ransaced_keypoints_1[Ransaced_Matches[i].queryIdx];	//	应该存入第一幅图的关键点，以便与后面的匹配
					point_store.OB_KeyPoint = Ransaced_keypoints_2[Ransaced_Matches[i].trainIdx];	//	存入第二幅图的关键点
					point_store.Is_in_Chain = true;
					PointChain.push_back(point_store);
				}
			}
		}

		std::cout << "PointChain size=" << std::endl << PointChain.size() << std::endl;

		cv::Mat Transform_Relative = (cv::Mat_<double>(4, 4) <<
			solvepnp_R.at<double>(0, 0), solvepnp_R.at<double>(0, 1), solvepnp_R.at<double>(0, 2), solvepnp_T.at<double>(0, 0),
			solvepnp_R.at<double>(1, 0), solvepnp_R.at<double>(1, 1), solvepnp_R.at<double>(1, 2), solvepnp_T.at<double>(1, 0),
			solvepnp_R.at<double>(2, 0), solvepnp_R.at<double>(2, 1), solvepnp_R.at<double>(2, 2), solvepnp_T.at<double>(2, 0),
			0, 0, 0, 1
			);

		Camera_Trajactory_Transform.push_back(Transform_Relative);
		//Tw2 = Tw1 * (T21.inv())
		//Camera_Last_pose = Camera_Last_pose * Transform_Relative;
		cv::Mat Last_Ground_Truth = (cv::Mat_<double>(4, 4) <<
			Ground_truth_store[(picture_ind / 20) - 1](0, 0), Ground_truth_store[(picture_ind / 20) - 1](0, 1), Ground_truth_store[(picture_ind / 20) - 1](0, 2), Ground_truth_store[(picture_ind / 20) - 1](0, 3),
			Ground_truth_store[(picture_ind / 20) - 1](1, 0), Ground_truth_store[(picture_ind / 20) - 1](1, 1), Ground_truth_store[(picture_ind / 20) - 1](1, 2), Ground_truth_store[(picture_ind / 20) - 1](1, 3),
			Ground_truth_store[(picture_ind / 20) - 1](2, 0), Ground_truth_store[(picture_ind / 20) - 1](2, 1), Ground_truth_store[(picture_ind / 20) - 1](2, 2), Ground_truth_store[(picture_ind / 20) - 1](2, 3),
			Ground_truth_store[(picture_ind / 20) - 1](3, 0), Ground_truth_store[(picture_ind / 20) - 1](3, 1), Ground_truth_store[(picture_ind / 20) - 1](3, 2), Ground_truth_store[(picture_ind / 20) - 1](3, 3)
			);
		//Camera_Last_pose = Last_Ground_Truth * (Transform_Relative.inv());		//Tw2 = Tw1 * (T21.inv())
		Camera_Last_pose = Camera_Last_pose * (Transform_Relative.inv());		//Tw2 = Tw1 * (T21.inv())
		Camera_Trajactory.push_back(Camera_Last_pose);	//这种是把存放数据的地址放进去了。。我曹**********************


#ifdef SHOW_INFORMATION
		std::cout << "Transform_Relative " << Transform_Relative << std::endl;
#endif

	}

	//****************end the project, store the pose**********************************
	std::cout << "Calculated done, results are in ICP_transformation_store vector and E_store vector" << std::endl;
	std::cout << "press any key to exit! " << std::endl;

	//store the groundtruth RT matrix
	remove("Ground_truth.txt");
	std::ofstream fout1;
	fout1.open("Ground_truth.txt", std::ios::app);//在文件末尾追加写入
	for (int i = 0; i < Ground_truth_store.size(); i++)
	{
		fout1 << Ground_truth_store[i] << std::endl;//每次写完一个矩阵以后换行
	}
	fout1.close();

	remove("Camera_Trajactory_Transform_store.txt");
	std::ofstream fout2;
	fout2.open("Camera_Trajactory_Transform_store.txt", std::ios::app);//在文件末尾追加写入
	for (int i = 0; i < Camera_Trajactory_Transform.size(); i++)
	{
		Eigen::Matrix4d Transform_store;
		Transform_store << Camera_Trajactory_Transform[i].at<double>(0, 0), Camera_Trajactory_Transform[i].at<double>(0, 1), Camera_Trajactory_Transform[i].at<double>(0, 2), Camera_Trajactory_Transform[i].at<double>(0, 3),
			Camera_Trajactory_Transform[i].at<double>(1, 0), Camera_Trajactory_Transform[i].at<double>(1, 1), Camera_Trajactory_Transform[i].at<double>(1, 2), Camera_Trajactory_Transform[i].at<double>(1, 3),
			Camera_Trajactory_Transform[i].at<double>(2, 0), Camera_Trajactory_Transform[i].at<double>(2, 1), Camera_Trajactory_Transform[i].at<double>(2, 2), Camera_Trajactory_Transform[i].at<double>(2, 3),
			0, 0, 0, 1;
		fout2 << Transform_store << std::endl;//每次写完一个矩阵以后换行
	}
	fout2.close();

	//trransform RT to quartanier,
	remove("Web_compare_MY_TRUTH.txt");
	std::ofstream fout3;
	fout3.open("Web_compare_MY_TRUTH.txt", std::ios::app);//在文件末尾追加写入
	Eigen::Matrix<double, 1, 8> my_truth_vector;
	Eigen::Matrix3d Rotation_matrix;
	Eigen::Matrix4d TransformMatrix;
	Eigen::Matrix4d ICP_pose = Ground_truth_store[0];
	Camera_Trajactory2.push_back(ICP_pose);
	for (int i = 0; i < Camera_Trajactory_Transform.size(); i++)
	{
		Rotation_matrix = ICP_pose.block<3, 3>(0, 0);
		Eigen::Quaterniond q = Eigen::Quaterniond(Rotation_matrix);
		my_truth_vector[0] = double(rgb_file_time_stamp[20 * i]);
		my_truth_vector[1] = (ICP_pose(0, 3));	//tx
		my_truth_vector[2] = (ICP_pose(1, 3));	//ty
		my_truth_vector[3] = (ICP_pose(2, 3));	//tz
		my_truth_vector[4] = (q.x());	//qx
		my_truth_vector[5] = (q.y());	//tx
		my_truth_vector[6] = (q.z());	//ty
		my_truth_vector[7] = (q.w());	//tz
		fout3 << std::fixed << std::setprecision(4) << my_truth_vector << std::endl;
		//TransformMatrix << Camera_Trajactory_Transform[i];
		TransformMatrix << Camera_Trajactory_Transform[i].at<double>(0, 0), Camera_Trajactory_Transform[i].at<double>(0, 1), Camera_Trajactory_Transform[i].at<double>(0, 2), ((Camera_Trajactory_Transform[i].at<double>(0, 3))),
			Camera_Trajactory_Transform[i].at<double>(1, 0), Camera_Trajactory_Transform[i].at<double>(1, 1), Camera_Trajactory_Transform[i].at<double>(1, 2), ((Camera_Trajactory_Transform[i].at<double>(1, 3))),
			Camera_Trajactory_Transform[i].at<double>(2, 0), Camera_Trajactory_Transform[i].at<double>(2, 1), Camera_Trajactory_Transform[i].at<double>(2, 2), ((Camera_Trajactory_Transform[i].at<double>(2, 3))),
			0, 0, 0, 1;
		//ICP_pose =  Ground_truth_store[i] * (TransformMatrix.inverse());	//相对轨迹
		ICP_pose = ICP_pose * (TransformMatrix.inverse());	//绝对轨迹
		Camera_Trajactory2.push_back(ICP_pose);
	}
	fout3.close();

	int a = Camera_Trajactory.size();
	int b = Camera_Trajactory2.size();
	Camera_Trajactory.clear();
	for (int i = 0; i < Camera_Trajactory2.size(); i++)
	{
		cv::Mat Camera_traactory_store = (cv::Mat_<double>(4, 4) <<
			Camera_Trajactory2[i](0, 0), Camera_Trajactory2[i](0, 1), Camera_Trajactory2[i](0, 2), Camera_Trajactory2[i](0, 3),
			Camera_Trajactory2[i](1, 0), Camera_Trajactory2[i](1, 1), Camera_Trajactory2[i](1, 2), Camera_Trajactory2[i](1, 3),
			Camera_Trajactory2[i](2, 0), Camera_Trajactory2[i](2, 1), Camera_Trajactory2[i](2, 2), Camera_Trajactory2[i](2, 3),
			Camera_Trajactory2[i](3, 0), Camera_Trajactory2[i](3, 1), Camera_Trajactory2[i](3, 2), Camera_Trajactory2[i](3, 3));
#ifdef Using_Camera_RT
		Camera_Trajactory.push_back(Camera_traactory_store.inv());	//绝对位姿Twc取逆得到相机的外参Tcw。。
#else
		Camera_Trajactory.push_back(Camera_traactory_store);
#endif
	}

	remove("MATLAB_MY_TRUTH.txt");
	std::ofstream fout4;
	fout4.open("MATLAB_MY_TRUTH.txt", std::ios::app);//在文件末尾追加写入
	for (int i = 0; i < Camera_Trajactory2.size(); i++)
	{
		fout4 << std::fixed << std::setprecision(4) << Camera_Trajactory2[i] << std::endl; //每次写完一个矩阵以后换行
	}
	fout4.close();

	//remove("MATLAB_MY_TRUTH.txt");
	//std::ofstream fout4;
	//fout4.open("MATLAB_MY_TRUTH.txt", std::ios::app);//在文件末尾追加写入
	//for (int i = 0; i < Camera_Trajactory.size(); i++)
	//{
	//	Eigen::Matrix4d Matlab_pose;
	//	Matlab_pose << Camera_Trajactory[i].at<double>(0, 0), Camera_Trajactory[i].at<double>(0, 1), Camera_Trajactory[i].at<double>(0, 2), Camera_Trajactory[i].at<double>(0, 3),
	//		Camera_Trajactory[i].at<double>(1, 0), Camera_Trajactory[i].at<double>(1, 1), Camera_Trajactory[i].at<double>(1, 2), Camera_Trajactory[i].at<double>(1, 3),
	//		Camera_Trajactory[i].at<double>(2, 0), Camera_Trajactory[i].at<double>(2, 1), Camera_Trajactory[i].at<double>(2, 2), Camera_Trajactory[i].at<double>(2, 3),
	//		Camera_Trajactory[i].at<double>(3, 0), Camera_Trajactory[i].at<double>(3, 1), Camera_Trajactory[i].at<double>(3, 2), Camera_Trajactory[i].at<double>(3, 3);
	//	fout4 << std::fixed << std::setprecision(4) << Matlab_pose << std::endl; //每次写完一个矩阵以后换行
	//}
	//fout4.close();

	time_finish = clock();
	time_duration = (double)(time_finish - time_start) / CLOCKS_PER_SEC;
	std::cout << "Simple VO running time: " << time_duration << std::endl;
	

//**************************Bundle Adjustment********************************************************
	std::cout << "Run Bundle Adjustment: " << time_duration << std::endl;
	// 初始化g2o
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> > Block;  // pose 维度为 6, landmark 维度为 3
	Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
	Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));      // 矩阵块求解器
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));	//levenberg algorithm非线性优化算法
	g2o::SparseOptimizer optimizer;		//稀疏性矩阵优化器
	optimizer.setAlgorithm(solver);

	/* setToOriginImpl设定被优化变量的原始值，
	oplusImpl比较重要，我们根据增量方程计算出增量之后，
	就是通过这个函数对估计值进行调整的，
	因此这个函数的内容一定要写对，
	否则会造成一直优化却得不到好的优化结果的现象。*/

	// vertex
	//camera Camera_Pose
	for (int i = 0; i < Camera_Trajactory.size(); i++)
	{
		g2o::VertexSE3Expmap* Camera_Pose = new g2o::VertexSE3Expmap(); // camera Camera_Pose
		Eigen::Matrix3d R_mat;
		R_mat <<
			Camera_Trajactory[i].at<double>(0, 0), Camera_Trajactory[i].at<double>(0, 1), Camera_Trajactory[i].at<double>(0, 2),
			Camera_Trajactory[i].at<double>(1, 0), Camera_Trajactory[i].at<double>(1, 1), Camera_Trajactory[i].at<double>(1, 2),
			Camera_Trajactory[i].at<double>(2, 0), Camera_Trajactory[i].at<double>(2, 1), Camera_Trajactory[i].at<double>(2, 2);
		Eigen::Vector3d T_mat;
		T_mat << Camera_Trajactory[i].at<double>(0, 3), Camera_Trajactory[i].at<double>(1, 3), Camera_Trajactory[i].at<double>(2, 3);
		Camera_Pose->setId(i);
		if (i == 0)
			Camera_Pose->setFixed(true);
		Camera_Pose->setEstimate( g2o::SE3Quat(R_mat,T_mat) );
		optimizer.addVertex(Camera_Pose);
	}
	
	//Observation Point 
	for (int i = 0; i < PointChain.size(); i++)
	{
		//ConstVectorRef temVecPoint(raw_points + point_block_size * i, point_block_size);
		g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
		point->setId(i + Camera_Trajactory.size());
		point->setEstimate(Eigen::Vector3d(PointChain[i].WorldPoint.x, PointChain[i].WorldPoint.y, PointChain[i].WorldPoint.z));
		point->setMarginalized(true); // g2o 中必须设置 marg 参见第十讲内容g2o 中必须设置边缘化。设定为true时，将会被边缘化加速求解。见式(10.56)至式(10.59)。
		optimizer.addVertex(point);
	}

	// parameter: camera intrinsics
	g2o::CameraParameters* camera = new g2o::CameraParameters(
		Camera_K.at<double>(0, 0), Eigen::Vector2d(Camera_K.at<double>(0, 2), Camera_K.at<double>(1, 2)), 0
	);
	camera->setId(0);
	optimizer.addParameter(camera);

	// edges
	std::vector<g2o::EdgeProjectXYZ2UV*> edges;
	//int edge_index = 0;
	for (int i = 0; i < PointChain.size(); i++)
	{
		for (int j = 0; j < PointChain[i].CameraId.size(); j++)
		{
			const int edge_camera_id = PointChain[i].CameraId[j]; // get id for the camera; 
			const int edge_point_id = i + Camera_Trajactory.size(); // get id for the point 

			cv::Point2f Pixel_point;
			//World3D_to_CameraUV(PointChain[i].WorldPoint, Camera_Trajactory[edge_camera_id], Pixel_point);
			Pixel_point = PointChain[i].CameraId_UV_Point[j];

			g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();

#ifdef BA_Robustify
			g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
			rk->setDelta(1.0);
			edge->setRobustKernel(rk);

#endif
			//edge->setId(edge_point_id);
			//其实是在设置一条边的两个节点，0,1应该是固定的，因为一条边只有两个节点
			//index为3D特征点的节点ID，得到的就是这个3D点的信息
			edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(edge_point_id)));
			edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(edge_camera_id)));//看到这个点的相机的ID的信息
			//应该有两条边，对应两个观测信息
			edge->setMeasurement(Eigen::Vector2d(Pixel_point.x, Pixel_point.y));
			edge->setParameterId(0, 0);	//不知道作用
			Eigen::Matrix<double, 2, 2> information;
			information = Eigen::MatrixXd::Identity(2,2);
			edge->setInformation(information);
			optimizer.addEdge(edge);
			edges.push_back(edge);
		}
	}
	optimizer.save("before_g2o.g2o");
	

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	optimizer.setVerbose(true); // 打开调试输出
	optimizer.initializeOptimization();
	optimizer.optimize(50);
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> (t2 - t1);
	std::cout << "optimization costs time: " << time_used.count() << " seconds." << std::endl;

	optimizer.save("after_g2o.g2o");
	std::cout << std::endl << "after optimization:" << std::endl;

	//我们比较关心两帧之间的变换矩阵
	//for (int i = 0; i < Camera_Trajactory.size(); i++)
	//{
	//	g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
	//	Eigen::Isometry3d pose = v->estimate();
	//	std::cout << "Optimized Pose " << i << std::endl << pose.matrix() << std::endl;
	//}

	//// 估计inlier的个数
	//int inliers = 0;
	//for (auto e : edges)
	//{
	//	e->computeError();
	//	// chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
	//	if (e->chi2() > 1)
	//	{
	//		std::cout << "error = " << e->chi2() << std::endl;
	//	}
	//	else
	//	{
	//		inliers++;
	//	}
	//}

	//std::cout << "inliers in total points: " << inliers << "/" << PointChain.size() << std::endl;

	
	//存储BA后两帧之间的转换矩阵
	remove("MY_BA_TRUTH.txt");
	std::ofstream fout5;
	fout5.open("MY_BA_TRUTH.txt", std::ios::app);//在文件末尾追加写入
	for (int i = 0; i < Camera_Trajactory.size(); i++)
	{
		g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
		Eigen::Isometry3d pose = v->estimate();
		fout5 << std::fixed << std::setprecision(4) << pose.matrix() << std::endl; //每次写完一个矩阵以后换行
	}
	fout5.close();

	//存储BA后相机绝对位姿，与groundtruth对应
	std::vector<Eigen::Matrix4d> BA_Trajactory;
	remove("MY_BA_Trajactory_TRUTH.txt");
	std::ofstream fout6;
	fout6.open("MY_BA_Trajactory_TRUTH.txt", std::ios::app);//在文件末尾追加写入
	for (int i = 0; i < Camera_Trajactory.size(); i++)
	{
		g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
		Eigen::Isometry3d pose = v->estimate();
		//std::cout << "Camera pose BA " << ((pose.matrix()).inverse()) << std::endl;

#ifdef Using_Camera_RT
		fout6 << std::fixed << std::setprecision(4) << (( pose.matrix()).inverse() ) << std::endl; //每次写完一个矩阵以后换行
		BA_Trajactory.push_back(((pose.matrix()).inverse()));
#else
		fout6 << std::fixed << std::setprecision(4) << (( pose.matrix()).inverse() ) << std::endl; //每次写完一个矩阵以后换行
		BA_Trajactory.push_back(((pose.matrix()).inverse()));
#endif
	}
	fout6.close();

	//trransform RT to quartanier,
	remove("Web_compare_MY_BA_TRUTH.txt");
	std::ofstream fout7;
	fout7.open("Web_compare_MY_BA_TRUTH.txt", std::ios::app);//在文件末尾追加写入
	Eigen::Matrix<double, 1, 8> my_BA_truth_vector;
	Eigen::Matrix3d BA_Rotation_matrix;
	for (int i = 0; i < BA_Trajactory.size(); i++)
	{
		BA_Rotation_matrix = BA_Trajactory[i].block<3, 3>(0, 0);
		Eigen::Quaterniond q = Eigen::Quaterniond(BA_Rotation_matrix);
		my_BA_truth_vector[0] = double(rgb_file_time_stamp[20 * i]);
		my_BA_truth_vector[1] = (BA_Trajactory[i](0, 3));	//tx
		my_BA_truth_vector[2] = (BA_Trajactory[i](1, 3));	//ty
		my_BA_truth_vector[3] = (BA_Trajactory[i](2, 3));	//tz
		my_BA_truth_vector[4] = (q.x());	//qx
		my_BA_truth_vector[5] = (q.y());	//tx
		my_BA_truth_vector[6] = (q.z());	//ty
		my_BA_truth_vector[7] = (q.w());	//tz
		fout7 << std::fixed << std::setprecision(4) << my_BA_truth_vector << std::endl;
	}
	fout7.close();

	// 以及所有特征点的位置
	//for (size_t i = 0; i<PointChain.size(); i++)
	//{
	//	g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i + Camera_Trajactory.size()));
	//	std::cout << "vertex id " << i + Camera_Trajactory.size() << ", pos = ";
	//	Eigen::Vector3d pos = v->estimate();
	//	std::cout << pos(0) << "," << pos(1) << "," << pos(2) << std::endl;
	//}

	std::cout << "Finished! " << std::endl;

	getchar();

	return 0;
}
