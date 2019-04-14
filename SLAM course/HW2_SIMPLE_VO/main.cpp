#pragma warning( disable : 4996)
#include <read_files.h>
#include <Basic_VO_function.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"		//line

#include <opencv2/opencv.hpp>	//ransac
const int iteration_times = 1000;

int main()
{
	std::string rgb_file = "rgb.txt";
	std::string truth_file_name = "groundtruth.txt";
	std::vector<std::string> rgb_name;	//depth file name array
	std::vector<double> file_time_stamp;	//depth file name array
	std::vector<double> groundtruth_time_stamp;	//depth file name array
	read_picture_txt(rgb_file, rgb_name, file_time_stamp);	//get depth files path and name
	int picture_ind = 0;
	//cv::Mat Camera_K = (cv::Mat_<double>(3, 3) << 525, 0, 319.5, 0, 525, 239.5, 0, 0, 1);
	//int focal_length = Camera_K.at<double>(1, 1);						//相机焦距, TUM dataset标定值
	//cv::Point2d principal_point(Camera_K.at<double>(0, 2), Camera_K.at<double>(1, 2));				//相机主点, TUM dataset标定值

	std::vector<cv::Mat> Camera_Trajactory_Transform;
	std::vector<cv::Point3d> Pre3DPoints, Current3DPoints;
	cv::Mat T_Pose_Pre;	//pose of camera1;
	T_Pose_Pre = (cv::Mat_<double>(4, 4) << 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 );	//last camera's pose
	std::vector<cv::Point> inliers_PreImg;
	bool Calculate_Trajactory = 1;
	while (Calculate_Trajactory)
	{
		//Calculate_Trajactory = 0;
//read adjacent two rgb pictures***********************************************************
		cv::Mat img_1 = cv::imread(rgb_name[picture_ind]);
		picture_ind += 20;
		if (picture_ind > 100)
		{
			break;
		}
		cv::Mat img_2 = cv::imread(rgb_name[picture_ind]);
		cv::Mat img_2_copy = img_2.clone();
		cv::Mat img_1_copy = img_1.clone();

		std::vector<cv::Point> good_match_img1, good_match_img2;
		Feature_Extract_Match(img_1, img_2, good_match_img1, good_match_img2);


// 7-points with 1000 iteration to find the inliers.***********************************************************
		cv::Mat Fundmental_try, Fundmental_most, Essential_try, Essential_most;		//possible Fundmental
		int inliers_count = 0;		//count number of inliers
		double TotalError = 0.0;		//total error of inliers
		int inliners_most = 0;		//biggest number of inliers
		double TotalError_most = TotalError_default;		//biggesttotal error of inliers
		std::vector<Eigen::Matrix3d> Fundmental_Matrix;		//seven points possible fundamental matrix
		std::vector<cv::Point> inliers_img1, inliers_img2;	//store the inliers of imag1 and imag2
		std::vector<cv::Point2f> Normalized_inliers_img1, Normalized_inliers_img2;	//store the inliers of imag1 and imag2
		cv::Mat Fundamental_EightPoint, Essential_EightPoint;
		cv::Mat R_EightPoint, T_EightPoint;
		//std::vector<cv::Mat> Fundamental_EightPoint_store;
		srand((unsigned int)time(0));
		for (int RANSAC_iter = 0; RANSAC_iter < iteration_times; RANSAC_iter++)
		{
			Fundmental_Matrix.clear();		//清空lameda
			GetFundamentalLameda(good_match_img1, good_match_img2, Fundmental_Matrix);
			// calculate the number of inliners;
			cv::Mat R, t;
			for (size_t i = 0; i < Fundmental_Matrix.size(); i++)
			{
				//从F解算出R,T，先得到E，再解算得到R,T。
				Fundmental_try = (cv::Mat_<double>(3, 3) << Fundmental_Matrix[i](0, 0), Fundmental_Matrix[i](0, 1), Fundmental_Matrix[i](0, 2),
					Fundmental_Matrix[i](1, 0), Fundmental_Matrix[i](1, 1), Fundmental_Matrix[i](1, 2),
					Fundmental_Matrix[i](2, 0), Fundmental_Matrix[i](2, 1), Fundmental_Matrix[i](2, 2));
				Essential_try = (Camera_K.t())*Fundmental_try*Camera_K;
				cv::recoverPose(Essential_try, good_match_img1, good_match_img2, Camera_K, R, t);
				//std::cout << "R is " << std::endl << R << std::endl;
				//std::cout << "t is " << std::endl << t << std::endl;

				/********解算出R,T自己写的代码******************/
				//********************MY deconpose E into R,T
				//Eigen::JacobiSVD<Eigen::MatrixXd> svd_E(E_try_test, Eigen::ComputeFullU | Eigen::ComputeFullV);
				//Eigen::Matrix3d U = svd_E.matrixU();
				//Eigen::Matrix3d V = svd_E.matrixV();
				//Eigen::Matrix3d Vt = svd_E.matrixV().transpose();
				//Eigen::Matrix3d Ut = svd_E.matrixU().transpose();
				//Eigen::Matrix3d W,Z,R1,R2;
				//W << 0, -1, 0, 1, 0, 0, 0, 0, 1;
				//Z << 0, 1, 0, -1, 0, 0, 0, 0, 0;
				//R1 = U * W * Vt;
				//R2 = U * W.transpose() * Vt;
				//Eigen::Matrix3d My_T;
				//My_T = U * Z * Ut;
				//std::cout << "MY R1 is " << std::endl << R1 << std::endl;
				//std::cout << "MY R2 is " << std::endl << R2 << std::endl;
				//std::cout << "MY T is " << std::endl << My_T << std::endl;

				//三角化获得对应的三维点
				std::vector<cv::Point3d> points;
				triangulation(good_match_img1, good_match_img2, R, t, points);

				//计算重投影误差，都投射到归一化平面上
				inliers_count = 0;
				TotalError = 0.0;
				CalculateInlierNumbers(good_match_img1, good_match_img2, points, R, t, inliers_count, TotalError);

				if ((inliers_count > inliners_most) || ((TotalError < TotalError_most) && (inliers_count == inliners_most) && inliers_count != 0)) // && inliners_count != good_match_img1.size()
				{
					inliners_most = inliers_count;
					TotalError_most = TotalError;
					Fundmental_most = Fundmental_try;
				}
			}
		}

		std::cout << "RANSAC后的内点个数:  " << inliners_most << std::endl;
		std::cout << "Fundmental_most:  " << Fundmental_most << std::endl;
		
		if (inliners_most < 8)
		{
			std::cout << "No enough feature points when comparing picture" << picture_ind-20 << " and "<< picture_ind << std::endl;
			getchar();
			return -1;
		}

		//Get inliers set and draw it in image.
		GetInliers(good_match_img1, good_match_img2, Fundmental_most, inliers_img1, inliers_img2);
		DrawFeaturePoints(img_2_copy, inliers_img2);
		DrawMovingLines(img_2_copy, inliers_img2, inliers_img1);
		cv::imshow("RANSAC后img2的特征点", img_2_copy);

//Eight points algorithm***********************************************************
		//归一化特征点，归一化和矩阵分别存入T1，T2中
		cv::Mat Normanized_T1, Normanized_T2;
		NormanizePoints(inliers_img1, Normalized_inliers_img1, Normanized_T1);
		NormanizePoints(inliers_img2, Normalized_inliers_img2, Normanized_T2);

		Fundamental_EightPoint = Get_Eight_Points_Fundamental(Normalized_inliers_img1, Normalized_inliers_img2, Normanized_T1, Normanized_T2);
		std::cout << "Fundamental_EightPoint " << Fundamental_EightPoint << std::endl;
		//GetInliers(good_match_img1, good_match_img2, Fundamental_EightPoint, inliers_img1, inliers_img2);
		DrawEpiLines(img_1_copy, img_2_copy, inliers_img1, inliers_img2, Fundamental_EightPoint);
		//从F解算出R,T，先得到E，再解算得到R,T。存入总的轨迹转化矩阵中
		Essential_EightPoint = (Camera_K.t())*Fundamental_EightPoint*Camera_K;
		cv::recoverPose(Essential_EightPoint, inliers_img1, inliers_img2, Camera_K, R_EightPoint, T_EightPoint);
		//cv::recoverPose(Essential_EightPoint, inliers_img1, inliers_img2, Camera_K, R_EightPoint, T_EightPoint);


//Scale propagation***********************************************************
		//std::vector<cv::Point3d> Pre3DPoints, Current3DPoints;
		//这里存储的应该是RANSAC后的特征点，避免误差较大的点影响。。
		//然后计算当前帧的特征点的3D点，
		std::vector<double> Scale_d;
		if (picture_ind > 20)
		{
			Scale_d.clear();
			ScaleTriangulation(inliers_img1, inliers_img2, R_EightPoint, T_EightPoint, Current3DPoints, T_Pose_Pre);
			for (size_t i = 0; i < inliers_img1.size(); i++)
			{
				int Current_x = inliers_img1[i].x;
				int Current_y = inliers_img1[i].y;
				for (size_t j = 0; j < inliers_PreImg.size(); j++)
				{
					//如果在前面找到了跟当前帧一样的特征点，scale propagation
					if (inliers_PreImg[j].x == Current_x && inliers_PreImg[j].y == Current_y)
					{
						cv::Mat Point3DCamera1 = (R_EightPoint.t())* ( (cv::Mat_<double>(3, 1) << Current3DPoints[j].x, Current3DPoints[j].y, Current3DPoints[j].z) - T_EightPoint);
						double DepthCurrent = sqrt((Point3DCamera1.at<double>(0, 0) * Point3DCamera1.at<double>(0, 0)) +
							(Point3DCamera1.at<double>(1, 0) * Point3DCamera1.at<double>(1, 0)) +
							(Point3DCamera1.at<double>(2, 0) * Point3DCamera1.at<double>(2, 0))
						);
						double DepthPre = sqrt((Pre3DPoints[j].x * Pre3DPoints[j].x) +
							(Pre3DPoints[j].y * Pre3DPoints[j].y) +
							(Pre3DPoints[j].z * Pre3DPoints[j].z)
						);
						Scale_d.push_back(DepthPre/ DepthCurrent);
						break;
					}
				}
			}
			//找到所有的d之后，取d得中值d_median，然后T_EightPoint = d_median.*T_EightPoint
			sort(Scale_d.begin(), Scale_d.end());
			double Scale_d_median = Scale_d[int(Scale_d.size()/2)];
			T_EightPoint = Scale_d_median * T_EightPoint;
		}


//Prepare for next frame***********************************************************
		ScaleTriangulation(inliers_img1, inliers_img2, R_EightPoint, T_EightPoint, Pre3DPoints, T_Pose_Pre);
		inliers_PreImg.assign(inliers_img2.begin(), inliers_img2.end());//将v2赋值给v1

		cv::Mat Transform_EightPoint = (cv::Mat_<double>(4, 4) <<
			R_EightPoint.at<double>(0, 0), R_EightPoint.at<double>(0, 1), R_EightPoint.at<double>(0, 2), T_EightPoint.at<double>(0, 0),
			R_EightPoint.at<double>(1, 0), R_EightPoint.at<double>(1, 1), R_EightPoint.at<double>(1, 2), T_EightPoint.at<double>(1, 0),
			R_EightPoint.at<double>(2, 0), R_EightPoint.at<double>(2, 1), R_EightPoint.at<double>(2, 2), T_EightPoint.at<double>(2, 0),
			0, 0, 0, 1
			);
		// Update T_Pose_Pre
		T_Pose_Pre = Transform_EightPoint * T_Pose_Pre;

		Camera_Trajactory_Transform.push_back(Transform_EightPoint);
		std::cout << "Transform_EightPoint " << Transform_EightPoint << std::endl;
		
	}
	std::cout << "Calculated done, results are in ICP_transformation_store vector and E_store vector" << std::endl;
	std::cout << "press any key to exit! " << std::endl;

	remove("Camera_Trajactory_Transform_store.txt");
	std::ofstream fout2;
	fout2.open("Camera_Trajactory_Transform_store.txt", std::ios::app);//在文件末尾追加写入
	for (int i = 0; i < Camera_Trajactory_Transform.size(); i++)
	{
		fout2 << Camera_Trajactory_Transform[i] << std::endl;//每次写完一个矩阵以后换行
	}
	fout2.close();

	cv::waitKey(0);
	return 0;
}


