#pragma once

#undef __ARM_NEON__
#undef __ARM_NEON

#include <stdio.h>
#include <iostream>
#include <vector>
#include <time.h>
#include <cmath>
#include <stdlib.h>
#include <stdint.h>

#define __ARM_NEON__
#define __ARM_NEON

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


#define k_data_set_size 500000		//max size of dataset, Actual data size is k_data_set_size*3
#define k_query_set_size 20000
#define k_axis_cell_max  100			//the biggest size for splitting the axis  50*50*50 = 125000... need to be restrained
#define  k_ideal_cell_size 4		//ideal size in a grid
#define k_cells_number_max 60000	//max cell numebrs; =k_data_set_size/k_ideal_cell_size
#define k_search_near_regions_max 1000	//max nearest search regions..
#define k_search_points_times 3		//total search points >= k_search_points_times*K
#define k_nearest_number_max 10		//K_MAX = 10
#define k_parallel_search_num  5

#define k_data_max_value_abs 500

/* Init the dataset and query set randomly*/
void InitData(My_Points data_set[k_data_set_size], const int input_dataset_size,
	My_Points query_set[k_query_set_size], const int input_query_size);

/* Do GBDS TEST*/
void GBDS_CPU_Test(My_Points data_set[k_data_set_size], const int input_dataset_size,
	My_Points query_set[k_query_set_size], const int input_query_size,
	const int K, int knn_index[k_query_set_size][k_nearest_number_max], 
	type_point knn_distance[k_query_set_size][k_nearest_number_max],
	int user_define_grid_number);

/* Calculate the AABB(minimum value and maximum value of each axis) of a dataset*/
My_MaxMin GetMaxMin(My_Points data_set[k_data_set_size], const int input_dataset_size);

/* Calculate the hash value of each data point*/
int CalculateHash(struct ThreeDimPoint point_data, My_MaxMin data_set_max_min, const int split_grids_s);

/* Classify the data to cell; and then reorder the dataset by hash value */
void DataClassify(My_Points data_set[k_data_set_size], const int input_dataset_size,
	const My_MaxMin data_set_max_min, const int real_split_grid, int cell_first_index[k_cells_number_max],
	int original_data_index[k_data_set_size]);

/* Search KNN of a query in dataset. 
return the NN index and distance */
void SearchKNNGBDS(My_Points data_set[k_data_set_size], const int input_dataset_size,
	const My_Points current_query, const int K,
	const My_MaxMin data_set_max_min, const int real_split_grid,
	int cell_first_index[k_cells_number_max], int original_data_index[k_data_set_size],
	int current_knn_index[k_nearest_number_max], type_point  current_knn_distance[k_nearest_number_max]);