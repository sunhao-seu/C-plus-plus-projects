//class perf_counter
//{
//public:
//	uint64_t tot, cnt, calls;
//	perf_counter() : tot(0), cnt(0), calls(0) {};
//	inline void reset() { tot = cnt = calls = 0; }
//	inline void start() { cnt = sds_clock_counter(); calls++; };
//	inline void stop() { tot += (sds_clock_counter() - cnt); };
//	inline uint64_t avg_cpu_cycles() { return ((tot + (calls >> 1)) / calls); };
//};




#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <cstdlib>// Header file needed to use rand
#include <ctime> // Header file needed to use time

//#define USE_SDX
//#define TEST_POINT_CLOUD

#ifdef USE_SDX
#include "sds_lib.h"

#else
#include <pcl/io/vtk_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/polygonmesh.h>
#endif

//#include "gbds_pcl.h"
#include "GBDS_extention.h"

//read dataset
#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>



#define query_set_size 100
#define map_corner_useful_data_set_size 1000000
#define data_max_value 100000


int main(int argc, char* argv[]) {

	std::string input_file_path;
	if (argc == 2)
	{
		input_file_path = argv[1];
		std::cout << "the input path is: " << input_file_path << std::endl;
}
	else
	{
		std::cout << "error number of arguments,,,," << (argc - 1) << "You ought to give only 1 arguments indicating the search path!" << std::endl << "press any key to exit..." << std::endl;
		getchar();
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPLYFile<pcl::PointXYZ>(input_file_path, *cloud) == -1) //* load the file 
	{
		PCL_ERROR("read file fail£¡ \n");
		system("PAUSE");
		return (-1);
}
	//pcl::StatisticalOutlierRemoval::applyFileter()
	pcl::visualization::CloudViewer viewer("Viewer");
	viewer.showCloud(cloud);
	system("PAUSE");


#ifdef USE_SDX
	My_Points* map_corner_data_set = (My_Points*)sds_alloc(k_data_set_size * 3 * sizeof(type_point));
	type_point* map_corner_nearest_distance = (type_point*)sds_alloc(k_nearest_number_max * sizeof(type_point));
	int* map_corner_nearest_index = (int*)sds_alloc(k_nearest_number_max * sizeof(int));
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

	//for (int i = 0; i < map_corner_useful_data_set_size; i++)
	//{
	//	map_corner_data_set[i].x = rand() % 1000;
	//	map_corner_data_set[i].y = rand() % 1000;
	//	map_corner_data_set[i].z = rand() % 1000;
	//}
unsigned int cloud_points_size = 0;
	
	//transform point cloud to array; GBDS input format
	//remove nul points and overflow points
	for (unsigned int i = 0; i < cloud->points.size(); i++)
	{
		if (abs(cloud->points[i].x) < data_max_value && abs(cloud->points[i].y) < data_max_value && abs(cloud->points[i].z) < data_max_value)
		{	
			map_corner_data_set[cloud_points_size].x = cloud->points[i].x;
			map_corner_data_set[cloud_points_size].y = cloud->points[i].y;
			map_corner_data_set[cloud_points_size].z = cloud->points[i].z;
			cloud_points_size++;
			if (cloud_points_size >= k_data_set_size)
			{
				std::cout << "cloud->points.size(): " << cloud->points.size() << std::endl << "k_data_set_size: " << k_data_set_size << std::endl;
				std::cout << "error, dataset size is bigger than array size. " << std::endl << "press any key to exit..." << std::endl;
				break;
				/*getchar();
				return -1;*/
			}
		}
	}
	
	std::cout << "cloud->points.size(): " << cloud_points_size << std::endl;
	std::cout << "over range data number: " << cloud->points.size() - cloud_points_size << std::endl;

	//get the simulate input data
	my_point_sel.x = map_corner_data_set[rand() % cloud_points_size].x;
	my_point_sel.y = map_corner_data_set[rand() % cloud_points_size].y;
	my_point_sel.z = map_corner_data_set[rand() % cloud_points_size].z;

	std::cout << "map_corner_data_set[10].z: " << map_corner_data_set[10].z << std::endl;

	std::cout << "Before hardware GBDSIPCore1: " << std::endl;
	//int trandform_data_size = map_corner_useful_data_set_size;
	unsigned int trandform_data_size = cloud_points_size;
	ExGBDSIPCore_sw(1, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);
	std::cout << "After hardware GBDSIPCore1: " << std::endl;


	std::cout << "Before hardware GBDSIPCore0: " << std::endl;
	trandform_data_size = 0;
	ExGBDSIPCore_sw(0, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);
	//GBDSIPCore_test(1, map_corner_data_set, map_corner_useful_data_set_size, &data_max_min);
	std::cout << "After hardware GBDSIPCore0: " << std::endl;


	std::cout << "query point: " << my_point_sel.x << " " << my_point_sel.y << " " << my_point_sel.z << std::endl;
	std::cout << "kdtree nearest neighbor index: " << map_corner_nearest_index[0] << std::endl;
	std::cout << "nearest point: " << map_corner_data_set[map_corner_nearest_index[0]].x << " " << map_corner_data_set[map_corner_nearest_index[0]].y << " " << map_corner_data_set[map_corner_nearest_index[0]].z << std::endl;
	std::cout << "kdtree nearest neighbor distance: " << map_corner_nearest_distance[0] << std::endl;
	std::cout << "2 distance: " << map_corner_nearest_distance[1] << std::endl;
	std::cout << "3 distance: " << map_corner_nearest_distance[2] << std::endl;
	std::cout << "4 distance: " << map_corner_nearest_distance[3] << std::endl;
	std::cout << "5 distance: " << map_corner_nearest_distance[4] << std::endl;

	for (int i = 0; i < query_set_size; i++)
	{
		query_set[i].x = rand() % 1000;
		query_set[i].y = rand() % 1000;
		query_set[i].z = rand() % 1000;
	}
	for (int i = 0; i < query_set_size; i++)
	{
		query_set[i].x = map_corner_data_set[rand() % cloud_points_size].x;
		query_set[i].y = map_corner_data_set[rand() % cloud_points_size].y;
		query_set[i].z = map_corner_data_set[rand() % cloud_points_size].z;
	}

	int result_index_sw[query_set_size];
	int result_index_hw[query_set_size];
	float result_distance_hw[query_set_size];
	float result_distance_sw[query_set_size];


	//test software time
	clock_t before_knn_sw = clock();
	rand_seed = (unsigned)(time(NULL));
	trandform_data_size = cloud_points_size;
	ExGBDSIPCore_sw(1, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);
	clock_t build_gbds_sw = clock();
	for (int i = 0; i < query_set_size; i++)
	{
		my_point_sel = query_set[i];

		trandform_data_size = 0;
		ExGBDSIPCore_sw(0, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);

		result_index_sw[i] = map_corner_nearest_index[0];
		result_distance_sw[i] = map_corner_nearest_distance[0];
	}
	clock_t after_knn_sw = clock();
	double time_build_gbds_sw = (double)(build_gbds_sw - before_knn_sw) / CLOCKS_PER_SEC * 1000;
	double time_last_sw = (double)(after_knn_sw - before_knn_sw) / CLOCKS_PER_SEC * 1000;
	std::cout << "software build gbds with " << map_corner_useful_data_set_size << " dataset build gbds time is :  " << time_build_gbds_sw << " ms!" << std::endl;
	std::cout << query_set_size << " queries in " << map_corner_useful_data_set_size << " dataset gbds_sw time is :  " << time_last_sw << " ms!" << std::endl;


	//test hardware time
	clock_t before_knn = clock();
	rand_seed = (unsigned)(time(NULL));
	trandform_data_size = cloud_points_size;
	ExGBDSIPCore_hw(1, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);
	clock_t build_gbds = clock();
	for (int i = 0; i < query_set_size; i++)
	{
		my_point_sel = query_set[i];
		trandform_data_size = 0;
		ExGBDSIPCore_hw(0, map_corner_data_set, trandform_data_size, rand_seed, 5, my_point_sel, map_corner_nearest_index, map_corner_nearest_distance, split_precise);

		result_index_hw[i] = map_corner_nearest_index[0];
		result_distance_hw[i] = map_corner_nearest_distance[0];
	}
	clock_t after_knn = clock();
	double time_last = (double)(after_knn - before_knn) / CLOCKS_PER_SEC * 1000;
	double time_build_gbds = (double)(build_gbds - before_knn) / CLOCKS_PER_SEC * 1000;
	std::cout << "hardware build gbds with " << map_corner_useful_data_set_size << " dataset build gbds time is :  " << time_build_gbds << " ms!" << std::endl;
	std::cout << query_set_size << " queries in " << map_corner_useful_data_set_size << " dataset gbds_hw time is :  " << time_last << " ms!" << std::endl;




	//getchar();
	int error_count = 0;
	int error_index = 0;
	for (int i = 0; i < query_set_size; i++)
	{
		if (abs(result_distance_hw[i] - result_distance_sw[i]) > 1)
		{
			error_index = i;
			error_count = error_count + 1;
		}
	}

	if (error_count > 0)
	{
		std::cout << "error number: " << error_count << std::endl;
		std::cout << "error, result not equal at " << error_index << "-th query" << std::endl;
		std::cout << "query is " << query_set[error_index].x << " " << query_set[error_index].y << " " << query_set[error_index].z << std::endl;
		std::cout << "gbds hardware nearest neighbor index: " << result_index_hw[error_index] << "  nearest distance: " << result_distance_hw[error_index] << std::endl;
		std::cout << "nearest point: " << map_corner_data_set[result_index_hw[error_index]].x << " " << map_corner_data_set[result_index_hw[error_index]].y << " " << map_corner_data_set[result_index_hw[error_index]].z << std::endl;
		std::cout << "gbds software nearest neighbor index: " << result_index_sw[error_index] << "  nearest distance: " << result_distance_sw[error_index] << std::endl;
		std::cout << "nearest point: " << map_corner_data_set[result_index_sw[error_index]].x << " " << map_corner_data_set[result_index_sw[error_index]].y << " " << map_corner_data_set[result_index_sw[error_index]].z << std::endl;
	}
	else
		std::cout << "ccorrect hw result!" << std::endl;

	std::cout << "print anything to exit.." << std::endl;
	getchar();

	return 0;
}

