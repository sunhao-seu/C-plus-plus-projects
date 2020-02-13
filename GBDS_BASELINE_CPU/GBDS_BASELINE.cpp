#include "GBDS_BASELINE.h"

#define COMPARE_GT		//groundtruth with brute force method

void InitData(My_Points data_set[k_data_set_size], const int input_dataset_size,
	My_Points query_set[k_query_set_size], const int input_query_size)
{
	for (int i = 0; i < input_dataset_size; i++)
	{
		data_set[i].x = ( rand() % (k_data_max_value_abs * 2) - k_data_max_value_abs);
		data_set[i].y = (rand() % (k_data_max_value_abs * 2) - k_data_max_value_abs);
		data_set[i].z = (rand() % (k_data_max_value_abs * 2) - k_data_max_value_abs);
	}
	for (int i = 0; i < input_query_size; i++)
	{
		query_set[i].x = (rand() % (k_data_max_value_abs * 2) - k_data_max_value_abs);
		query_set[i].y = (rand() % (k_data_max_value_abs * 2) - k_data_max_value_abs);
		query_set[i].z = (rand() % (k_data_max_value_abs * 2) - k_data_max_value_abs);
	}
}

void GBDS_CPU_Test(My_Points data_set[k_data_set_size], const int input_dataset_size,
	My_Points query_set[k_query_set_size], const int input_query_size,
	const int K, int knn_index[k_query_set_size][k_nearest_number_max],
	type_point knn_distance[k_query_set_size][k_nearest_number_max],
	int user_define_grid_number)
{
	My_Points data_set_local[k_data_set_size];
	//cache the dataset
	for (int i = 0; i < input_dataset_size; i++)
	{
		data_set_local[i].x = data_set[i].x;
		data_set_local[i].y = data_set[i].y;
		data_set_local[i].z = data_set[i].z;
	}

	//calculate the cell size from input argument user_define_grid_number
	int total_calculated_cell_size = user_define_grid_number * user_define_grid_number * user_define_grid_number;
	int real_split_grid = user_define_grid_number;	//real split grid
	if (total_calculated_cell_size >= k_cells_number_max)
	{
		total_calculated_cell_size = k_cells_number_max;
		real_split_grid = cbrt(total_calculated_cell_size);
	}

	My_MaxMin data_set_max_min;
	data_set_max_min = GetMaxMin(data_set_local, input_dataset_size);

	int cell_first_index[k_cells_number_max];		//the first index of each cell when merge them in ordered hash array.
	int original_data_index[k_data_set_size];

	DataClassify(data_set_local, input_dataset_size, data_set_max_min, real_split_grid, cell_first_index, original_data_index);

	for (int i = 0; i < input_query_size; i++)
	{
		My_Points current_query = query_set[i];
		int current_knn_index[k_nearest_number_max];
		type_point  current_knn_distance[k_nearest_number_max];
		for (int j = 0; j < k_nearest_number_max; j++)
		{
			current_knn_index[j] = 0;
			current_knn_distance[j] = 100000;
		}

		
		SearchKNNGBDS(data_set_local, input_dataset_size, current_query, K, data_set_max_min, real_split_grid, cell_first_index, original_data_index, current_knn_index, current_knn_distance);
		for (int j = 0; j < K; j++)
		{
			knn_index[i][j] = current_knn_index[j];
			knn_distance[i][j] = current_knn_distance[j];
		}
		std::cout << "nearest distance of " << i << " is " << knn_distance[i][0] << std::endl;

#ifdef COMPARE_GT
		type_point current_nn_gt = 100000;
		for (int j = 0; j < input_dataset_size; j++)
		{
			My_Points point_in_dataset = data_set[j];
			type_point distance = ((current_query.x - point_in_dataset.x) * (current_query.x - point_in_dataset.x)
				+ (current_query.y - point_in_dataset.y) * (current_query.y - point_in_dataset.y)
				+ (current_query.z - point_in_dataset.z) * (current_query.z - point_in_dataset.z)
				);	//data_set_PL[index]
			if (distance < current_nn_gt)
				current_nn_gt = distance;
		}
		std::cout << "Groundt distance of " << i << " is " << current_nn_gt << std::endl;
#endif

	}
}


My_MaxMin GetMaxMin(My_Points data_set[k_data_set_size], int input_dataset_size)
{
	type_point x_max, x_min, y_max, y_min, z_max, z_min;

	x_max = y_max = z_max = -k_data_max_value_abs;
	x_min = y_min = z_min = k_data_max_value_abs;

loop_cache_and_min_max:
	for (int i = 0; i < input_dataset_size; i++)
	{
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=1 max=100000
		type_point data_x = data_set[i].x;
		type_point data_y = data_set[i].y;
		type_point data_z = data_set[i].z;

		//compare x
		if (x_max < data_x)
			x_max = data_x;
		if (x_min > data_x)
			x_min = data_x;


		//compare y
		if (y_max < data_y)
			y_max = data_y;
		if (y_min > data_y)
			y_min = data_y;


		//compare z
		if (z_max < data_z)
			z_max = data_z;
		if (z_min > data_z)
			z_min = data_z;

	}
	//store these min max in a structure for convenient usage
	My_MaxMin data_set_max_min;
	data_set_max_min.xmin = x_min;
	data_set_max_min.xmax = x_max;
	data_set_max_min.ymin = y_min;
	data_set_max_min.ymax = y_max;
	data_set_max_min.zmin = z_min;
	data_set_max_min.zmax = z_max;

	return data_set_max_min;
}


int CalculateHash(struct ThreeDimPoint point_data, My_MaxMin data_set_max_min, const int split_grids_s)
{
#pragma HLS INLINE
	//copy the point_data to data_x,y,z
	type_point data_x = point_data.x;
	type_point data_y = point_data.y;
	type_point data_z = point_data.z;

	type_point x_min = data_set_max_min.xmin;
	type_point y_min = data_set_max_min.ymin;
	type_point z_min = data_set_max_min.zmin;
	type_point x_max = data_set_max_min.xmax;
	type_point y_max = data_set_max_min.ymax;
	type_point z_max = data_set_max_min.zmax;

	int cpx = (data_x - x_min) / (x_max - x_min) * split_grids_s;
	int cpy = (data_y - y_min) / (y_max - y_min) * split_grids_s;
	int cpz = (data_z - z_min) / (z_max - z_min) * split_grids_s;

	int max_index = split_grids_s - 1;
	if (cpx < 0) cpx = 0;
	if (cpx > max_index) cpx = max_index;
	if (cpy < 0) cpy = 0;
	if (cpy > max_index) cpy = max_index;
	if (cpz < 0) cpz = 0;
	if (cpz > max_index) cpz = max_index;

	//transform 3d index to a 1d index
	int data_index = cpx * split_grids_s * split_grids_s + cpy * split_grids_s + cpz;

	return data_index;
}

void DataClassify(My_Points data_set[k_data_set_size], const int input_dataset_size,
	const My_MaxMin data_set_max_min, const int real_split_grid, int cell_first_index[k_cells_number_max],
	int original_data_index[k_data_set_size])
{
	int total_calculated_cell_size = real_split_grid * real_split_grid * real_split_grid;
	int data_set_hash[k_data_set_size];

	int cell_occupied_number[k_cells_number_max];		//the first index of each grid when merge them in ordered hash array.
	int count_cell_size[k_cells_number_max];		//the first index of each grid when merge them in ordered hash array.

	int data_ordered_by_hash[k_data_set_size];		//for reorder the dataset by hash value

	for (int i = 0; i < total_calculated_cell_size; i++)
	{
		cell_occupied_number[i] = 0;
		count_cell_size[i] = 0;
	}

loop_calculate_data_set_hash:
	for (int i = 0; i < input_dataset_size; i++)
	{
		//If remove the dependency of inter loop, the result will be fault.
		int data_hash = CalculateHash(data_set[i], data_set_max_min, real_split_grid);

		data_set_hash[i] = data_hash;
		if (count_cell_size[data_hash] < 10000)
		{
			count_cell_size[data_hash] = count_cell_size[data_hash] + 1;
		}
		else
		{
#ifdef DEBUG_INFO
			std::cout << "error occurs, too many points in a cell, over 10000" << std::endl;
#endif
		}
	}

loop_calculate_first_index:
	cell_first_index[0] = 0;
	for (int i = 1; i < total_calculated_cell_size; i++)
	{
		cell_first_index[i] = cell_first_index[i - 1] + count_cell_size[i - 1];
	}

	for (int i = 0; i < input_dataset_size; i++)
	{
		int current_cell_index = data_set_hash[i];
		int insert_index = (cell_first_index[current_cell_index] + cell_occupied_number[current_cell_index]);
		data_ordered_by_hash[i] = insert_index;
		original_data_index[i] = i;
		cell_occupied_number[current_cell_index] = cell_occupied_number[current_cell_index] + 1;
	}

	//reorder the dataset,  GPU can accelerate the process ...
	for (int i = 0; i < input_dataset_size;)
	{
		int current_pos_in_orderd_dataset = data_ordered_by_hash[i];
		if (current_pos_in_orderd_dataset != i)
		{

			// exchange a data to its ought to be locate
			My_Points data_temp = data_set[current_pos_in_orderd_dataset];
			data_set[current_pos_in_orderd_dataset] = data_set[i];
			data_set[i] = data_temp;

			int index_temp = data_ordered_by_hash[current_pos_in_orderd_dataset];
			data_ordered_by_hash[current_pos_in_orderd_dataset] = data_ordered_by_hash[i];
			data_ordered_by_hash[i] = index_temp;

			int origin_index_temp = original_data_index[current_pos_in_orderd_dataset];
			original_data_index[current_pos_in_orderd_dataset] = original_data_index[i];
			original_data_index[i] = origin_index_temp;

#ifdef DEBUG_INFO
			if (data_ordered_by_hash[i] == data_ordered_by_hash[current_pos_in_orderd_dataset])
			{
				std::cout << "error occurs,  ";
				std::cout << "data_ordered_by_hash[i]= " << data_ordered_by_hash[i] << " data_ordered_by_hash[current_pos_in_orderd_dataset]= " << data_ordered_by_hash[current_pos_in_orderd_dataset] << " " << std::endl;
				getchar();
				break;
			}
#endif

		}
		else i++;

	}
}

void SearchKNNGBDS(My_Points data_set[k_data_set_size], const int input_dataset_size,
	const My_Points current_query, const int K,
	const My_MaxMin data_set_max_min, const int real_split_grid, 
	int cell_first_index[k_cells_number_max], int original_data_index[k_data_set_size],
	int current_knn_index[k_nearest_number_max], type_point  current_knn_distance[k_nearest_number_max])
{
	int valid_near_regions[k_search_near_regions_max];		//the valid regions index contains points
	int valid_near_region_size = 0;							//size of valid_near_region.  or number of valid_near_region
	int query_hash = CalculateHash(current_query, data_set_max_min, real_split_grid);

	int search_distance = 0;
	int search_continue = 0;
	int sum_near_points = 0;										// sum of points found in the near regions


loop_while_find_sufficient_points:
	for (int while_count = 0; while_count < 10; while_count++)
	{
		for (int ix = -search_distance; ix <= search_distance; ix++)
		{
			int iy_higher_bound = search_distance - abs(ix);
			int iy_lower_bound = -iy_higher_bound;

			for (int iy = iy_lower_bound; iy <= iy_higher_bound; iy++)
			{

				int iz_abs = search_distance - abs(ix) - abs(iy);
				if (iz_abs >= 0)
				{
					char iz = iz_abs;
					int cal_index = query_hash + ix * real_split_grid * real_split_grid + iy * real_split_grid + iz;
					//make sure the index is in the array's range
					if ((cal_index >= 0) && (cal_index < (real_split_grid * real_split_grid * real_split_grid)))
					{
						int current_cell_size = cell_first_index[cal_index + 1] - cell_first_index[cal_index];
						if ((current_cell_size) > 0)
						{
							sum_near_points += current_cell_size;
							valid_near_regions[valid_near_region_size] = cal_index;
							valid_near_region_size++;
						}

					}

					char iz_minus = -iz_abs;
					int cal_index_minus = query_hash + ix * real_split_grid * real_split_grid + iy * real_split_grid + iz_minus;
					//make sure the index is in the array's range
					if ((cal_index_minus >= 0) && (cal_index_minus < (real_split_grid * real_split_grid * real_split_grid)))
					{
						int current_cell_size = cell_first_index[cal_index_minus + 1] - cell_first_index[cal_index_minus];
						if (current_cell_size > 0)
						{
							sum_near_points += current_cell_size;
							valid_near_regions[valid_near_region_size] = cal_index_minus;
							valid_near_region_size++;
						}

					}
				}

			}
		}
		if (sum_near_points >= K)
		{
			if (search_continue < 3)
			{
				search_continue++;
				search_distance++;
			}
			else break;
		}
		else
			search_distance++;
	}

	for (int i = 0; i < valid_near_region_size; i++)
	{
#pragma HLS loop_tripcount min=1 max=15
		int current_search_hash = valid_near_regions[i];
		int current_cell_size = cell_first_index[current_search_hash + 1] - cell_first_index[current_search_hash];

	loop_knn_each_cell:
		for (int j = 0; j < current_cell_size; j++)
		{
#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=20
			//calculate the distance between query and point j;
			int searching_data_index = cell_first_index[current_search_hash] + j;
			struct ThreeDimPoint point_in_dataset = data_set[searching_data_index];
			int original_dataset_index = original_data_index[searching_data_index];

			type_point distance = ((current_query.x - point_in_dataset.x) * (current_query.x - point_in_dataset.x)
				+ (current_query.y - point_in_dataset.y) * (current_query.y - point_in_dataset.y)
				+ (current_query.z - point_in_dataset.z) * (current_query.z - point_in_dataset.z)
				);	//data_set_PL[index]
			
			//if distance is smaller than the biggest distance in result array,
			//then store the distance and the index of the point to replace the old biggest one.
			if (distance < current_knn_distance[K - 1])	//the gate of step in the result array
			{
				current_knn_distance[K - 1] = distance;
				current_knn_index[K - 1] = original_dataset_index;		//the indx of this point
				//Sort the result array
				for (int k = K - 2; k >= 0; k--)
				{
#pragma HLS loop_tripcount min=1 max=10
					if (distance < current_knn_distance[k])
					{
						current_knn_distance[k + 1] = current_knn_distance[k];
						current_knn_distance[k] = distance;
						current_knn_index[k + 1] = current_knn_index[k];
						current_knn_index[k] = original_dataset_index;
					}
					else break;	//if the new data is bigger than any data in the vector, the loop will be break.
				}
			}

		}
	}


}