#pragma once
#include <stdio.h>
#include <iostream>
#include <vector>

// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

#include <LOAM_useful_functions.h>

const int k_data_set_size = 1200000;		//size of dataset  64000*20 
//const int k_query_set_size = 5000;		//size of queryset; usually donot need this const value
const int k_sub_space_data_size = 4;		//the average size of data in a sub-space
///TODO: default error: the actual size of this array may larger than this array_size..
const int k_sub_space_array_size = 100;		//the average size of data in a sub-space
const float k_error_tare = 0.05;			//ANN error rate; 
const int k_axis_spaces_max = 40;	//the biggest sizeber of subspaces
const int k_sub_region_max = 64000;	//40*40*40
const int k_search_near_regions_max = 1000;	//
const int k_nearest_number_max = 10;	//10
const int k_query_set_size = 10;
const int k_search_times = 5;

//template<typename type_point>
typedef double type_point;

struct ThreeDimPoint {
	type_point x;
	type_point y;
	type_point z;
};

//template<typename type_point>
struct MaxMin {
	type_point xmin;
	type_point xmax;
	type_point ymin;
	type_point ymax;
	type_point zmin;
	type_point zmax;
};

//template<typename type_point>
struct SplitArraySize {
	int x_array_size;
	int y_array_size;
	int z_array_size;
};

/** @brief Remove the null points in pointcloud and Transform data from PointCloud to struct array
*
*  @param point_cloud_data_set: Original pointcloud data.
*  @param data_set: store the useful dataset in a struct array.
*  @param data_set_size: the array size of struct array. the useful data
*  @return Void.
*/
//template<int DATA_SIZE>
void MyPointCloudToArray(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_data_set, struct ThreeDimPoint data_set[], int & data_set_size);

void MyPointXYZIToThreeDimPoint(pcl::PointXYZI point, struct ThreeDimPoint & point_array);

/** @brief Get the min and max of x,y,z axis.
*  @param data_set: store the useful dataset in a struct array.
*  @param data_max_min: a structure store the x_min,x_max,y_min,y_max,z_min.z_max
*  @param data_set_size: the array size of struct array. the useful data
*  @return Void.
*/
//template<int DATA_SIZE>
void GetMaxMin(struct ThreeDimPoint data_set[], struct MaxMin & data_max_min, const int data_set_size);

/** @brief Get the Split array of x,y,z axis.  [evenly distrubute]
*  @param data_set_size: the size of the dataset
*  @param data_max_min: a structure store the x_min,x_max,y_min,y_max,z_min.z_max
*  @param x_split_array: split array of x axis. from x_min to x_max
*  @param split_array_size: the size of split array..contains x_split_size, y_split_size, z_split_size
*  @return Void.
*/
//template<int X_SPLIT_SIZE, int Y_SPLIT_SIZE, int Z_SPLIT_SIZE>
void SplitSubSpace(const int data_set_size, const struct MaxMin data_max_min, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize & split_array_size);

/** @brief Classify the dataset into different sub-spaces
*  @param data_set:  store the useful dataset in a struct array.
*  @param data_set_size: the array size of struct array.(the useful data)
*  @param x_split_array: split array of x axis. from x_min to x_max
*  @param split_array_size: the size of split array..contains x_split_size, y_split_size, z_split_size
*  @param sub_sets: the sub-spaces set contains the index of dataset.   [是否需要重新存储，以连续读取数据而不是随机读取]
*  @param sub_sets_size: the size of the second dimension of sub_sets..
*  @return Void.
*/
//template<int DATA_SIZE, int SUBSETS_NUM, int SUBSPACE_NUM>
void DataClassify(struct ThreeDimPoint data_set[], int data_set_size, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size, int sub_sets[][k_sub_space_array_size], int sub_sets_size[]);

/** @brief Calculate the index in the sub_sets of a point
*  @param point_data: the point data's sub_sets index need to be calculated
*  @param x_split_array: split array of x axis. from x_min to x_max
*  @param split_array_size: the size of split array..contains x_split_size, y_split_size, z_split_size
*  @return index.
*/
//template<typename T_INDEX>
int PCLCalculateIndex(struct ThreeDimPoint point_data, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size);

/** @brief Search K nearest neighbors of point_data in dataset.
*  @param point_data: find the nearest neighbors of this point
*  @param data_set: store the useful dataset in a struct array.
*  @param sub_sets: the sub-spaces set contains the index of dataset.
*  @param sub_sets_size: the size of the second dimension of sub_sets..
*  @param x_split_array: split array of x axis. from x_min to x_max
*  @param split_array_size: the size of split array..contains x_split_size, y_split_size, z_split_size
*  @param nearest_index: the index of nearest neighbors in the data_set
*  @param nearest_distance: the distance between the point and nearest neighbor of the same index of nearest_index
*  @return void
*/
//template<int DATA_SIZE, int SUBSETS_NUM, int SUBSPACE_NUM>
void SearchKNearestNeighbors(int K, struct ThreeDimPoint query_data, struct ThreeDimPoint data_set[], int sub_sets[][k_sub_space_array_size], int sub_sets_size[], type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size, int nearest_index[], type_point nearest_distance[]);

/** @brief Find the near region index of current_index with the distance of search_distance
*  @param current_index:  current index of search sub-spaces
*  @param search_distance: the distance of near region
*  @param near_regions[]:  store the index of search result
*  @return index.
*/
void Find_Near_Regions(int current_index, int search_distance, int near_regions[], int & near_region_count, struct SplitArraySize split_array_size);

/** @brief Calculate the euclidean distance between two vector3 data
*
*  @param data1,data2
*  @return the euclidean distance
*/
type_point PCLEucDist(struct ThreeDimPoint data1, struct ThreeDimPoint data2);