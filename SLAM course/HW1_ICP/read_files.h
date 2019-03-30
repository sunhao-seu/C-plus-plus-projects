#pragma once
#include <png.h>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <Eigen/Eigen>
#include<sstream>

/*
read png files and store them with Eigen::ArrayXXf
*/
void read_depth_file(std::string & file_name, Eigen::ArrayXXf & depthArray);

/*
read txt file. 
get the order and the name of png files
store png file path and name in png_name string vector sequentially
*/
void read_picture_txt(std::string & file_name, std::vector<std::string> &picture_name, std::vector<double> & picture_time_stamp);

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