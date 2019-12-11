

#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <cstdlib>// Header file needed to use rand
#include <ctime> // Header file needed to use time

#ifdef USE_SDX
#include "sds_lib.h"
#endif

//#include "gbds_pcl.h"
#include "GBDS_extention.h"

#define query_set_size 100
#define map_corner_useful_data_set_size 100000


int main(int argc, char* argv[]){

#ifdef USE_SDX
     My_Points* map_corner_data_set = (My_Points*)sds_alloc(k_data_set_size*3*sizeof(type_point));
     type_point* map_corner_nearest_distance = (type_point*)sds_alloc(k_nearest_number_max*sizeof(type_point));
     int* map_corner_nearest_index = (int*)sds_alloc(k_nearest_number_max*sizeof(int));
#else
	My_Points map_corner_data_set[k_data_set_size];
	type_point map_corner_nearest_distance[k_nearest_number_max];
	int map_corner_nearest_index[k_nearest_number_max];
#endif

     My_MaxMin data_max_min;

     unsigned seed = time(0);
     srand(seed);
     unsigned int rand_seed = (unsigned)(time(NULL));
     My_Points my_point_sel;

     My_Points query_set[query_set_size];
     float split_precise = 0;


     //get the simulate input data
     my_point_sel.x = rand() % 100;
     my_point_sel.y = rand() % 100;
     my_point_sel.z = rand() % 100;
     for(int i = 0; i < map_corner_useful_data_set_size; i ++)
     {
    	 map_corner_data_set[i].x = rand() % 100;
    	 map_corner_data_set[i].y = rand() % 100;
    	 map_corner_data_set[i].z = rand() % 100;
     }

     std::cout << "map_corner_data_set[50].z: " << map_corner_data_set[50].z << std::endl;

    std::cout << "Before hardware GBDSIPCore1: " << std::endl;
    int trandform_data_size = map_corner_useful_data_set_size;
	ExGBDSIPCore_sw(1, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);
    //GBDSIPCore_test(0, map_corner_data_set, map_corner_useful_data_set_size, &data_max_min);
    std::cout << "After hardware GBDSIPCore1: " << std::endl;
//	std::cout << "map_corner_data_max_min.xmax is: " << data_max_min.xmax << std::endl;
//	std::cout << "map_corner_data_max_min.xmin is: " << data_max_min.xmin << std::endl;

	std::cout << "Before hardware GBDSIPCore0: " << std::endl;
	trandform_data_size = 0;
	ExGBDSIPCore_sw(0, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);
	//GBDSIPCore_test(1, map_corner_data_set, map_corner_useful_data_set_size, &data_max_min);
	std::cout << "After hardware GBDSIPCore0: " << std::endl;


	std::cout << "query point: " << my_point_sel.x << " "<< my_point_sel.y << " "<< my_point_sel.z << std::endl;
	std::cout << "kdtree nearest neighbor index: " << map_corner_nearest_index[0] << std::endl;
	std::cout << "nearest point: " << map_corner_data_set[map_corner_nearest_index[0]].x << " "<< map_corner_data_set[map_corner_nearest_index[0]].y << " "<< map_corner_data_set[map_corner_nearest_index[0]].z << std::endl;
	std::cout << "kdtree nearest neighbor distance: " << map_corner_nearest_distance[0] << std::endl;

	for(int i = 0; i < query_set_size; i ++)
	{
		query_set[i].x = rand() % 1000;
		query_set[i].y = rand() % 1000;
		query_set[i].z = rand() % 1000;
	}

	int result_index_sw[query_set_size];
	int result_index_hw[query_set_size];
	//test software time
	clock_t before_knn_sw = clock();
	rand_seed = (unsigned)(time(NULL));
	trandform_data_size = map_corner_useful_data_set_size;
	ExGBDSIPCore_sw(1, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);
	clock_t build_gbds_sw = clock();
	for(int i = 0; i < query_set_size; i ++)
	{
		my_point_sel = query_set[i];

		trandform_data_size = 0;
		ExGBDSIPCore_sw(0, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);

		result_index_sw[i] = map_corner_nearest_index[0];
	}
	clock_t after_knn_sw = clock();
	double time_build_gbds_sw = (double)(build_gbds_sw - before_knn_sw) / CLOCKS_PER_SEC * 1000;
	double time_last_sw = (double)(after_knn_sw - before_knn_sw) / CLOCKS_PER_SEC * 1000;
	std::cout <<  " build gbds with " << map_corner_useful_data_set_size << " dataset build gbds time is :  "<<  time_build_gbds_sw  << " ms!"<< std::endl;
	std::cout << query_set_size << " queries in " << map_corner_useful_data_set_size << " dataset gbds_sw time is :  "<<  time_last_sw << " ms!" << std::endl;



	//test hardware time
	clock_t before_knn = clock();
	rand_seed = (unsigned)(time(NULL));
	trandform_data_size = map_corner_useful_data_set_size;
	ExGBDSIPCore_sw(1, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);
	clock_t build_gbds = clock();
	for(int i = 0; i < query_set_size; i ++)
	{
		my_point_sel = query_set[i];
		trandform_data_size = 0;
		ExGBDSIPCore_sw(0, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);

		result_index_hw[i] = map_corner_nearest_index[0];
	}
	clock_t after_knn = clock();
	double time_last = (double)(after_knn - before_knn) / CLOCKS_PER_SEC * 1000;
	double time_build_gbds = (double)(build_gbds - before_knn) / CLOCKS_PER_SEC * 1000;
	std::cout <<  " build gbds with " << map_corner_useful_data_set_size << " dataset build gbds time is :  "<<  time_build_gbds << " ms!" << std::endl;
	std::cout << query_set_size << " queries in " << map_corner_useful_data_set_size << " dataset gbds_hw time is :  "<<  time_last << " ms!" << std::endl;

	getchar();

	for(int i = 0; i < query_set_size; i ++)
	{
		if(result_index_hw[i] != result_index_sw[i])
		{
			std::cout << "eerror, result not equal at " << i  << "-th query"  << std::endl;
			return -1;
		}
	}
	std::cout << "ccorrect hw result!" << std::endl;

	
	return 0;
}

