#pragma once

#undef __ARM_NEON__
#undef __ARM_NEON

#include <stdio.h>
#include <iostream>
#include <vector>
#include <time.h>

#ifdef USE_SDX
#include "sds_lib.h"
#endif

#include <cmath>
#include <stdlib.h>
#include <stdint.h>

#define __ARM_NEON__
#define __ARM_NEON

#define k_data_set_size 100000		//max size of dataset, Actual data size is k_data_set_size*3
//const int k_nearest_number_max  5000		//size of queryset usually donot need this const value
#define  k_sub_space_data_size 4		//the average size of data in a sub-space for splitting
#define  k_sub_set_space_size  20		//the max size of data in a sub-space
#define  k_error_tare  0.05			//ANN error rate

#define  k_axis_spaces_max  30	//the biggest size for splitting the axis
#define  k_sub_region_max  12500	//1000000/4/2 2 for margin
#define  k_sub_region_max_cbrt  23.208	//cbrt(k_sub_region_max)  cbrt(12500)
#define  k_axis_sub_region_max  1000	// max split size of a axis

#define  k_search_near_regions_max  1000	//search nn in the near regions
#define  k_nearest_number_max  10	//K < 10
#define  k_search_times  3		//when points in searching sub-spaces are bigger than k_search_times*k, then do search.
//#define  k_split_precise  1		//the split precise, unit: m     11/15 maybe this parameter need be defined by user..

#define  k_search_distance_store_number  100	//search nn in the near regions

#define  k_select_split_precise_threshold  10000	//search nn in the near regions

//new method: cannot reduce any data


//template<typename type_point>
typedef float type_point;

typedef struct ThreeDimPoint {
	type_point x;
	type_point y;
	type_point z;
} My_Points;

//template<typename type_point>
typedef struct MaxMin {
	type_point xmin;
	type_point xmax;
	type_point ymin;
	type_point ymax;
	type_point zmin;
	type_point zmax;
} My_MaxMin;

//template<typename type_point>
typedef struct SplitArraySize {
	int x_array_size;
	int y_array_size;
	int z_array_size;
} My_SplitArraySize;

void GetMaxMin(struct ThreeDimPoint data_set[], struct MaxMin & data_max_min, int data_set_size);
void SplitSubSpace(int data_set_size, struct MaxMin data_max_min, float split_precise, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize & split_array_size);
void SplitSubSpacePrecise(int data_set_size, struct MaxMin data_max_min, float split_precise, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize & split_array_size);
void DataClassify(struct ThreeDimPoint data_set[], int data_set_size, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size, int sub_sets[][k_sub_set_space_size], int sub_sets_size[], unsigned int my_rand_number_seed);
int PCLCalculateIndex(struct ThreeDimPoint point_data, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size);
void SearchKNearestNeighbors(int K, struct ThreeDimPoint query_data, struct ThreeDimPoint data_set[], int sub_sets[][k_sub_set_space_size], int sub_sets_size[], type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size, int nearest_index[], type_point nearest_distance[]);
void Find_Near_Regions(int current_index, int search_distance, int near_regions[], int & near_region_count, struct SplitArraySize split_array_size);
type_point PCLEucDist(struct ThreeDimPoint data1, struct ThreeDimPoint data2);
//产生随机数的主要函数
void my_rand_generate(unsigned int *value);
float my_cbrt(float value);
//void BuildGBDS(struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int sub_sets[k_sub_region_max][k_sub_set_space_size], int sub_sets_size[k_sub_region_max],type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize split_array_size);
void BuildGBDS(struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, float split_precise);
//void SearchKNNGBDS(int K, struct ThreeDimPoint query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max]);
void SearchKNNGBDS();

//void GBDSIPCore(bool select_build_GBDS, struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, struct ThreeDimPoint query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max]);
void GBDSIPCore(bool select_build_GBDS, My_Points data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, My_Points query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max], float split_precise);



void GetMaxMin_sw(struct ThreeDimPoint data_set[], struct MaxMin & data_max_min, int data_set_size);
void SplitSubSpace_sw(int data_set_size, struct MaxMin data_max_min, float split_precise, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize & split_array_size);
void SplitSubSpacePrecise_sw(int data_set_size, struct MaxMin data_max_min, float split_precise, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize & split_array_size);
void DataClassify_sw(struct ThreeDimPoint data_set[], int data_set_size, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size, int sub_sets[][k_sub_set_space_size], int sub_sets_size[], unsigned int my_rand_number_seed);
int PCLCalculateIndex_sw(struct ThreeDimPoint point_data, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size);
void SearchKNearestNeighbors_sw(int K, struct ThreeDimPoint query_data, struct ThreeDimPoint data_set[], int sub_sets[][k_sub_set_space_size], int sub_sets_size[], type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size, int nearest_index[], type_point nearest_distance[]);
void Find_Near_Regions_sw(int current_index, int search_distance, int near_regions[], int & near_region_count, struct SplitArraySize split_array_size);
type_point PCLEucDist_sw(struct ThreeDimPoint data1, struct ThreeDimPoint data2);
//产生随机数的主要函数
void my_rand_generate_sw(unsigned int *value);
//void BuildGBDS(struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int sub_sets[k_sub_region_max][k_sub_set_space_size], int sub_sets_size[k_sub_region_max],type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize split_array_size);
void BuildGBDS_sw(struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, float split_precise);
//void SearchKNNGBDS(int K, struct ThreeDimPoint query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max]);
void SearchKNNGBDS_sw();

//void GBDSIPCore(bool select_build_GBDS, struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, struct ThreeDimPoint query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max]);
void GBDSIPCore_sw(bool select_build_GBDS, My_Points data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, My_Points query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max], float split_precise);
