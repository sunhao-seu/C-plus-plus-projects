#include <read_files.h>
#include <Basic_VO_function.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"		//line

#include <opencv2/opencv.hpp>	//ransac

const int iteration_times = 100;

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

	//read adjacent two rgb pictures
	cv::Mat img_1 = cv::imread(rgb_name[picture_ind]);
	picture_ind += 20;
	cv::Mat img_2 = cv::imread(rgb_name[picture_ind]);

	std::vector<cv::Point> good_match_img1, good_match_img2;
	Feature_Extract_Match(img_1, img_2, good_match_img1, good_match_img2);


	////**************** Relative pose estimation *****************************
	//////RANSAC匹配过程  Using opencv library;
	////std::vector<cv::DMatch> m_Matches;
	////m_Matches = good_matches;
	////int ptCount = good_matches.size();
	////if (ptCount < 10)
	////{
	////	std::cout << "Don't find enough match points" << std::endl;
	////	return 0;
	////}
	//////坐标转换为float类型
	////std::vector <cv::KeyPoint> RAN_KP1, RAN_KP2;
	//////size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
	////for (size_t i = 0; i < m_Matches.size(); i++)
	////{
	////	RAN_KP1.push_back(keypoints_1[good_matches[i].queryIdx]);
	////	RAN_KP2.push_back(keypoints_2[good_matches[i].trainIdx]);
	////	//RAN_KP1是要存储img01中能与img02匹配的点
	////	//goodMatches存储了这些匹配点对的img01和img02的索引值
	////}
	//////坐标变换
	////std::vector <cv::Point2f> p01, p02;
	////for (size_t i = 0; i < m_Matches.size(); i++)
	////{
	////	p01.push_back(RAN_KP1[i].pt);
	////	p02.push_back(RAN_KP2[i].pt);
	////}
	///*vector <Point2f> img1_corners(4);
	//img1_corners[0] = Point(0,0);
	//img1_corners[1] = Point(img_1.cols,0);
	//img1_corners[2] = Point(img_1.cols, img_1.rows);
	//img1_corners[3] = Point(0, img_1.rows);
	//vector <Point2f> img2_corners(4);*/
	//////求转换矩阵
	////Mat m_homography;
	////vector<uchar> m;
	////m_homography = findHomography(p01, p02, RANSAC);//寻找匹配图像
	////求基础矩阵 Fundamental,3*3的基础矩阵
	///*std::vector<uchar> RansacStatus;
	//cv::Mat Fundamental = cv::findFundamentalMat(p01, p02, RansacStatus, cv::FM_RANSAC);
	//std::cout << "Fundamental "<< Fundamental <<std::endl;*/
	//////重新定义关键点RR_KP和RR_matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
	////std::vector <cv::KeyPoint> RR_KP1, RR_KP2;
	////std::vector <cv::DMatch> RR_matches;
	////int index = 0;
	////for (size_t i = 0; i < m_Matches.size(); i++)
	////{
	////	if (RansacStatus[i] != 0)
	////	{
	////		RR_KP1.push_back(RAN_KP1[i]);
	////		RR_KP2.push_back(RAN_KP2[i]);
	////		m_Matches[i].queryIdx = index;
	////		m_Matches[i].trainIdx = index;
	////		RR_matches.push_back(m_Matches[i]);
	////		index++;
	////	}
	////}
	////std::cout << "RANSAC后匹配点数" << RR_matches.size() << std::endl;
	////cv::Mat img_RR_matches;
	////cv::drawMatches(img_1, RR_KP1, img_2, RR_KP2, RR_matches, img_RR_matches);
	////cv::imshow("After RANSAC", img_RR_matches);

/*******transform pixel points to 3*1 matrix********************/
	//transform pixel points to 3*1 matrix
	//std::vector<Eigen::Matrix<double, 3, 1>> MatchedPointsMat_img1, MatchedPointsMat_img2;
	////如果使用归一化坐标，那么求出来的就是E；；
	//for (size_t i = 0; i < Match_points_num; i++)
	//{
	//	Eigen::Matrix<double, 3, 1> temp;
	//	temp << good_match_img1[i].x, good_match_img1[i].y, 1;
	//	//temp = Camera_K.inverse()*temp;
	//	MatchedPointsMat_img1.push_back(temp);
	//	temp << good_match_img2[i].x, good_match_img2[i].y, 1;
	//	//temp = Camera_K.inverse()*temp;
	//	MatchedPointsMat_img2.push_back(temp);
	//}
	
	// 7-points with 1000 iteration to find the inliers.
	int Match_points_num = good_match_img1.size();
	cv::Mat Fundmental_try, Fundmental_most, Essential_try;		//possible Fundmental
	int inliers_count = 0;		//count number of inliers
	double TotalError = 0.0;		//total error of inliers
	int inliners_most = 0;		//biggest number of inliers
	double TotalError_most = TotalError_default;		//biggesttotal error of inliers
	std::vector<Eigen::Matrix3d> Fundmental_Matrix;
	srand((unsigned int)time(0));
	for (int RANSAC_iter = 0; RANSAC_iter < iteration_times; RANSAC_iter++)
	{
		Fundmental_Matrix.clear();		//清空lameda
		GetFundamentalLameda(good_match_img1, good_match_img2, Fundmental_Matrix);
		// calculate the number of inliners;
		cv::Mat R,t;
		for (size_t i = 0; i < Fundmental_Matrix.size(); i++)
		{
			//从F解算出R,T，先得到E，再解算得到R,T。
			Fundmental_try = (cv::Mat_<double>(3, 3) << Fundmental_Matrix[i](0, 0), Fundmental_Matrix[i](0, 1), Fundmental_Matrix[i](0, 2),
				Fundmental_Matrix[i](1, 0), Fundmental_Matrix[i](1, 1), Fundmental_Matrix[i](1, 2),
				Fundmental_Matrix[i](2, 0), Fundmental_Matrix[i](2, 1), Fundmental_Matrix[i](2, 2));
			Essential_try = (Camera_K.t())*Fundmental_try*Camera_K;
			cv::recoverPose(Essential_try, good_match_img1, good_match_img2, R, t, focal_length, principal_point);
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

			if ( (inliers_count > inliners_most) || ((TotalError < TotalError_most)&&(inliers_count == inliners_most)&& inliers_count != 0) ) // && inliners_count != good_match_img1.size()
			{
				inliners_most = inliers_count;
				TotalError_most = TotalError;
				Fundmental_most = Fundmental_try;
			}
		}
	}

	std::cout << "Fundmental_most: " << Fundmental_most << std::endl;
	//std::vector<cv::Vec3f> linesl;
	//computeCorrespondEpilines(good_match_img1, 1, Fundmental_most, linesl);

	//for (auto it = linesl.begin(); it != linesl.end(); it++)
	//{
	//	line(img1, Point(0, -(*it)[2] / (*it)[1]), Point(img1.cols, -((*it)[2] + (*it)[0] * img1.cols) / (*it)[1]), Scalar(255, 255, 255));
	//}
	//imshow("第一幅图像的对极线", img1);

	//eight points algorithm
	Eigen::Matrix3f Normanized_T;
	Normanized_T << (2 / img_1.cols), 0, -1, 0, (2 / img_1.rows), -1, 0, 0, 1;
	//good_match_img1 = Normanized_T* good_match_img1





	//Scale propagation

	cv::waitKey(0);
	return 0;
}


