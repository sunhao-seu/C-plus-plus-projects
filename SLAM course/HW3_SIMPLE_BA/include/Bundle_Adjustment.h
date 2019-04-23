#pragma once
#include <png.h>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Eigen>
#include<sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"		//line
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>	//ransac

//#define SHOW_INFORMATION
//#define SHOW_IMAGE
//#define BA_Robustify

//Eigen::Isometry3d T = Eigen::cvMat2Eigen(result.rvec, result.tvec);
//cout << "T=" << T.matrix() << endl;

const cv::Mat Camera_K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
const cv::Mat Camera_K_INV = Camera_K.inv();
const double focal_length = Camera_K.at<double>(1, 1);						//相机焦距, TUM dataset标定值
const cv::Point2d principal_point(Camera_K.at<double>(0, 2), Camera_K.at<double>(1, 2));				//相机主点, TUM dataset标定值
const int TotalError_default = 1000000;

#define FirstIndice 0
#define inlier_threshold 0.8

struct ObservPoints
{
	cv::Point3f WorldPoint;
	std::vector<int> CameraId;
	cv::KeyPoint OB_KeyPoint1;
	cv::KeyPoint OB_KeyPoint2;
};


/*
read txt file.
get the order and the name of png files
store png file path and name in png_name string vector sequentially
*/
void read_picture_txt(std::string & file_name, std::vector<std::string> &picture_name, std::vector<double> & picture_time_stamp);


/*
read png files and store them with Eigen::ArrayXXf
*/
void read_depth_file(std::string & file_name, Eigen::ArrayXXf & depthArray);

/*
Find the groundtruth and transform it to RT
*/
void GetGroundTruth(std::string & truth_file_name, std::vector<double> & picture_time_stamp, std::vector<Eigen::Matrix4d> & ground_truth_RT);


/*
return groundtruth of specify timestamp
input: timestamp, goundtruth timestamp[vector], groundtruth homogenous matrix[vector]
output:specify homegenous matrix
*/
Eigen::Matrix4d Get_time_stamp_Matrix(double time_stamp, std::vector<double> & picture_time_stamp, std::vector<Eigen::Matrix4d> & ground_truth_RT);

/*
return the correspondance depth indice.
input: rgb_timestamp, depth_timestamp_vector
output:correspondance septh indice to rgb_timestamp
*/
int Get_depth_corres_indice(const double rgb_time_stamp, const std::vector<double> & depth_time_stamp);

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
cv::Point2f pixel2cam(const cv::Point& p);

/*
Extract keypoints of two image and return their match
*/
void Feature_Extract_Match(const cv::Mat & img_1, 
	const cv::Mat & img_2, std::vector<cv::KeyPoint> & keypoints_1, 
	std::vector<cv::KeyPoint> & keypoints_2, 
	std::vector< cv::DMatch > & good_matches);

/*
Use Ransac to select inliers. 
*/
void Ransac_Inliers(const std::vector< cv::DMatch > & good_matches,
	const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	std::vector <cv::DMatch> & Ransaced_Matches,
	std::vector <cv::KeyPoint> & Ransaced_keypoints_1,
	std::vector <cv::KeyPoint> & Ransaced_keypoints_2);

/*
Calculate Fundamental Matrix and decompose the R,T
*/
void Get_RT(const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	const std::vector< cv::DMatch > & Ransaced_Matches,
	cv::Mat & R, cv::Mat & t);


/*
Verify the R and T
*/
void Verify_R_T(const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	const std::vector< cv::DMatch > & Ransaced_Matches,
	const cv::Mat & R, const cv::Mat & t);

/*
Triangulation(三角化，输入两个图片的对应特征点，以及转换矩阵； 输出三角化后的空间三维点坐标)
Input:feature points in img1 and img2, Rotation and translation matrix from img1 to img2;
Output: Triangulate points set
*/
void triangulation(
	const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	const std::vector< cv::DMatch > & matches,
	const cv::Mat& R, const cv::Mat& t,
	std::vector< cv::Point3d >& points);

void Verify_Triangulation(const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	const std::vector< cv::DMatch > & matches,
	const cv::Mat& R, const cv::Mat& t,
	const std::vector< cv::Point3d >& points);

/*
Transform local 3D points to World 3D points
*/
void Local3D_to_World3D(const cv::Point3f & Local_point, const cv::Mat Camera_pose, cv::Point3f & World_point);

void World3D_to_CameraUV(const cv::Point3f & World_point, const cv::Mat Camera_pose, cv::Point2f & Local_point);

cv::Point2f cam2pixel(const cv::Point2f& p);