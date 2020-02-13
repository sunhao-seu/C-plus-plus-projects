/*
GBDS CPU VERSION
BASELINE
Simplified....

WorkFlow:
0: init the dataset and queryset
1: Calculate the minimum value and maximum value of each axis
2: Classify each data in dataset to different cell; reorder the dataset by hash value
3: for each query point, search the KNN in ordered dataset
	3.1: calculate the cell of query point
	3.2: find the KNN in near cells

tip: #define COMPARE_GT in GBDS_BASELINE.cpp determine whether compare with groudtruth derived from brute force method.
*/

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <cstdlib>// Header file needed to use rand
#include <ctime> // Header file needed to use time
#include<fstream>

#include "GBDS_BASELINE.h"

int main(int argc, char* argv[]) {

	//some parameter predefined
	int user_define_grid_number = 30;
	int input_dataset_size = 20000;
	int K = 5;
	int input_query_size = 200;

	//dataset, qyery set, resut set
	My_Points data_set[k_data_set_size];
	My_Points query_set[k_query_set_size];
	type_point knn_distance[k_query_set_size][k_nearest_number_max];
	int knn_index[k_query_set_size][k_nearest_number_max];

	//init the dataset and queryset by random data
	InitData(data_set, input_dataset_size, query_set, input_query_size);

	//Do GBDS test
	GBDS_CPU_Test(data_set, input_dataset_size, query_set, input_query_size, K, knn_index, knn_distance, user_define_grid_number);

	getchar();
	return 0;
}

