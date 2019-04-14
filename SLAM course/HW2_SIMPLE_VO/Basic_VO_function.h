#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"		//line
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>	//ransac


const cv::Mat Camera_K = (cv::Mat_<double>(3, 3) << 525, 0, 319.5, 0, 525, 239.5, 0, 0, 1);
const int focal_length = Camera_K.at<double>(1, 1);						//相机焦距, TUM dataset标定值
const cv::Point2d principal_point(Camera_K.at<double>(0, 2), Camera_K.at<double>(1, 2));				//相机主点, TUM dataset标定值
const int TotalError_default = 1000000;

#define inlier_threshold 2

/*
feature extraction and match
Input: imag1; img2
Output: std::vector<cv::Point> good_match_img1, good_match_img2;
*/
void Feature_Extract_Match(const cv::Mat img_1, const cv::Mat img_2, std::vector<cv::Point> & good_match_img1, std::vector<cv::Point> & good_match_img2);


/*
随机取7个点，求取基础矩阵F的lameda； F1+lameda*F2
GetLameda(); In 7-points algorithm
Input: good_match_img1,good_match_img2;
Output: lameda
*/
void GetFundamentalLameda(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, std::vector<Eigen::Matrix3d> & Fundmental_Matrix);


/*
CalculateInlierNumbers()
Input: two imag matched points, Fundamental Matrix
Output: number of inlier points, 
*/
void CalculateInlierNumbers(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, const std::vector< cv::Point3d > points, const cv::Mat& R, const cv::Mat& t, int &inliers_count, double &TotalError);


/*
GetInliers()
Input:two image feature points, fundamental_most;
Output:two inliers points
*/
void GetInliers(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, const cv::Mat Fundmental_most, std::vector<cv::Point> & inliers_img1, std::vector<cv::Point> & inliers_img2);


/*
Draw feature points
DrawFeaturePoints()
Inpput:source image, feature points;
*/
void DrawFeaturePoints(const cv::Mat source_img, const std::vector<cv::Point> Feature_Points);

/*
Draw moving line
DrawMovingLines()
Inpput:source image, feature points1, feature points2;
*/
void DrawMovingLines(const cv::Mat source_img, const std::vector<cv::Point> Feature_Points1, const std::vector<cv::Point> Feature_Points2);


/*
	Opencv from pixel coordinate to camera coordinate;
	(像素平面到归一化平面)
	Input: cv::Point, Camera instrics K;
	Output: cv::Point2f
*/
cv::Point2f pixel2cam(const cv::Point& p, const cv::Mat& K);

//三维坐标x,y,z 到像素平面坐标 u,v
cv::Point cam3d2pixel(const cv::Point3d& p);

/*
Triangulation(三角化，输入两个图片的对应特征点，以及转换矩阵； 输出三角化后的空间三维点坐标)
Input:feature points in img1 and img2, Rotation and translation matrix from img1 to img2;
Output: Triangulate points set
*/
void triangulation(
	const std::vector<cv::Point> keypoint_1,
	const std::vector<cv::Point> keypoint_2,
	const cv::Mat& R, const cv::Mat& t,
	std::vector< cv::Point3d >& points);


/*
Get distance of two points in one image.
*/
double getDistance(cv::Point2f pointO, cv::Point2f pointA);


/*
Normalized the points<vector>
Input:  std::vector<cv::Point> Feature_Points
Output: Normalized points, transform matrix T
*/
void NormanizePoints(const std::vector<cv::Point> Source_Points, std::vector<cv::Point2f> & Normalized_Points, cv::Mat & T);


/*
EIght points calculate the FUndaental matrix
Get_Eight_Points_Fundamental();
Input:Normalized_inliers_img1,Normalized_inliers_img2,T1,T2
Output:Fundamental matrix
*/
cv::Mat Get_Eight_Points_Fundamental(const std::vector<cv::Point2f> Normalized_Points1, const std::vector<cv::Point2f> Normalized_Points2, const cv::Mat T1, const cv::Mat T2);


/*
GetEssentialInliers
*/
void GetEssentialInliers(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, const cv::Mat Essential_Matrix, std::vector<cv::Point> & inliers_img1, std::vector<cv::Point> & inliers_img2);


/*
DrawEpipolarLine();
Input: F, inlier points, image
Output: No
*/
void DrawEpipolarLine(const cv::Mat source_img, const std::vector<cv::Point> Inlier_Points, const cv::Mat Fundamental_Matrix);

void DrawEpiLines(const cv::Mat& img_1, const cv::Mat& img_2, std::vector<cv::Point>points1, std::vector<cv::Point>points2, const cv::Mat Fundamental_Matrix);

/*

*/
void ScaleTriangulation(
	const std::vector<cv::Point> keypoint_1,
	const std::vector<cv::Point> keypoint_2,
	const cv::Mat& R, const cv::Mat& t,
	std::vector< cv::Point3d >& points, const cv::Mat & T_Pose_Pre);