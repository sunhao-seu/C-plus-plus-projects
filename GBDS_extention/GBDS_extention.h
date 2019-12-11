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



#define k_data_set_size 100000		//max size of dataset, Actual data size is k_data_set_size*3
#define k_axis_cell_max  50	//the biggest size for splitting the axis  50*50*50 = 125000... need to be restrained
#define  k_ideal_cell_size 4
#define k_cells_number_max 25000
#define k_search_near_regions_max 100
#define k_search_points_times 3
#define k_nearest_number_max 10

void ExGetMaxMin_sw(struct ThreeDimPoint data_set[]);
void ExSplitSubCell_sw();
void ExDataClassify_sw();
int ExCalculateHash_sw(struct ThreeDimPoint point_data);

void ExSearchKNNGBDS_sw();
type_point ExEucDist_sw(struct ThreeDimPoint data1, struct ThreeDimPoint data2);

void ExBuildGBDS_sw(struct ThreeDimPoint data_set[k_data_set_size]);
void ExGBDSIPCore_sw(bool select_build_GBDS, My_Points data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, My_Points query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max], float split_precise);


void ExSplitSubSpacePrecise_sw();



void ExGetMaxMin_hw(struct ThreeDimPoint data_set[]);
void ExSplitSubCell_hw();
void ExDataClassify_hw();
int ExCalculateHash_hw(struct ThreeDimPoint point_data);

void ExSearchKNNGBDS_hw();
type_point ExEucDist_hw(struct ThreeDimPoint data1, struct ThreeDimPoint data2);

void ExBuildGBDS_hw(struct ThreeDimPoint data_set[k_data_set_size]);
void ExGBDSIPCore_hw(bool select_build_GBDS, My_Points data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, My_Points query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max], float split_precise);


void ExSplitSubSpacePrecise_hw();