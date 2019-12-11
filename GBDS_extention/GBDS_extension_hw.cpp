#undef __ARM_NEON__
#undef __ARM_NEON
#include "GBDS_extention.h"
#define __ARM_NEON__
#define __ARM_NEON

// variables stored in PL
static type_point split_precise_PL;

static struct ThreeDimPoint data_set_PL[k_data_set_size];
static struct MaxMin data_max_min_PL;
static int data_set_size_PL;

static type_point x_split_array_PL[k_axis_cell_max];
static type_point y_split_array_PL[k_axis_cell_max];
static type_point z_split_array_PL[k_axis_cell_max];
static struct ThreeDimPoint split_unit_PL;
static struct SplitArraySize split_array_size_PL;
static int total_calculated_cell_size_PL;


static int data_hash_PL[k_data_set_size];			//each data's hash
static int count_cell_size[k_cells_number_max];		//count the number of data in each grid
static int data_ordered_by_hash_PL[k_data_set_size];	//reorder the data's index by hash
static int cell_first_index[k_cells_number_max];		//the first index of each cell when merge them in ordered hash array.
static struct ThreeDimPoint ordered_data_set_PL[k_data_set_size];	//new dataset ordered by hash. 


static struct ThreeDimPoint query_data_PL;
static int K_PL;

static int nearest_index_PL[k_nearest_number_max];
static type_point nearest_distance_PL[k_nearest_number_max];
static bool sort_kick_flag[k_nearest_number_max];

void ExGetMaxMin_hw(struct ThreeDimPoint data_set[])
{

	type_point x_max, x_min, y_max, y_min, z_max, z_min;

	x_max = x_min = 0;
	y_max = y_min = 0;
	z_max = z_min = 0;

loop_cache_and_min_max:
	for (int i = 0; i<data_set_size_PL; i++)
	{
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=1 max=100000
		type_point data_x = data_set[i].x;
		type_point data_y = data_set[i].y;
		type_point data_z = data_set[i].z;
		data_set_PL[i].x = data_set[i].x;
		data_set_PL[i].y = data_set[i].y;
		data_set_PL[i].z = data_set[i].z;

		//compare x
		if (x_max<data_x)
			x_max = data_x;
		if (x_min>data_x)
			x_min = data_x;


		//compare y
		if (y_max<data_y)
			y_max = data_y;
		if (y_min>data_y)
			y_min = data_y;


		//compare z
		if (z_max<data_z)
			z_max = data_z;
		if (z_min>data_z)
			z_min = data_z;

	}
	//store these min max in a structure for convenient usage
	data_max_min_PL.xmin = x_min;
	data_max_min_PL.xmax = x_max;
	data_max_min_PL.ymin = y_min;
	data_max_min_PL.ymax = y_max;
	data_max_min_PL.zmin = z_min;
	data_max_min_PL.zmax = z_max;
}

void ExSplitSubCell_hw()
{
	int calculated_cell_size = (data_set_size_PL / k_ideal_cell_size + 1);		//evaluate the sub-spaces in need
	float x_range = data_max_min_PL.xmax - data_max_min_PL.xmin;							//the range of the data on x axis
	float y_range = data_max_min_PL.ymax - data_max_min_PL.ymin;
	float z_range = data_max_min_PL.zmax - data_max_min_PL.zmin;

	//calculate the max range of x,y,z; to avoid exceed k_axis_spaces_max
	float max_range = x_range;
	if (y_range > max_range)
		max_range = y_range;
	if (z_range > max_range)
		max_range = z_range;

	float x_times_max = x_range / max_range;
	float y_times_max = y_range / max_range;
	float z_times_max = z_range / max_range;
	float temp = x_times_max * y_times_max * z_times_max;

	float calculated_split_size = cbrt(calculated_cell_size / temp);
	float max_split_size = cbrt(k_cells_number_max / temp);
	if (calculated_split_size > max_split_size)
		calculated_split_size = max_split_size;

	//ceil is pretty ok, it won't exceed the array size
	//calculate the split size of x,y,z axis
	int x_split_size = floor(calculated_split_size * x_times_max);
	int y_split_size = floor(calculated_split_size * y_times_max);
	int z_split_size = floor(calculated_split_size * z_times_max);

	type_point x_unit = x_range / x_split_size;
	type_point y_unit = y_range / y_split_size;
	type_point z_unit = z_range / z_split_size;

	//when know the split unit and range, we can calculate the each split point and store it in a array.

	type_point x_min = data_max_min_PL.xmin;
	type_point y_min = data_max_min_PL.ymin;
	type_point z_min = data_max_min_PL.zmin;

	//consider parallel? the three loops can be executed simutaneously
loop_split_space_x:
	for (int i = 1; i <= x_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		x_split_array_PL[i] = x_min + i * x_unit;
	}
loop_split_space_y:
	for (int i = 1; i <= y_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		x_split_array_PL[i] = y_min + i * y_unit;
	}
loop_split_space_z:
	for (int i = 1; i <= z_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		x_split_array_PL[i] = z_min + i * z_unit;
	}

	//the split array range [min,max];(contain the min and max)
	split_array_size_PL.x_array_size = x_split_size;
	split_array_size_PL.y_array_size = y_split_size;
	split_array_size_PL.z_array_size = z_split_size;
	split_unit_PL.x = x_unit;
	split_unit_PL.y = y_unit;
	split_unit_PL.z = z_unit;
	total_calculated_cell_size_PL = x_split_size * y_split_size * z_split_size;
}


void ExDataClassify_hw()
{
	int cell_occupied_number[k_cells_number_max];		//the first index of each grid when merge them in ordered hash array.

														//reset the array
	for (int i = 0; i < total_calculated_cell_size_PL; i++)
	{
		count_cell_size[i] = 0;
		cell_occupied_number[i] = 0;
	}

	//	static int data_hash_PL[k_data_set_size];
	//	static int data_ordered_by_hash_PL[k_data_set_size];

	for (int i = 0; i < data_set_size_PL; i++)
	{
		int data_hash = ExCalculateHash_hw(data_set_PL[i]);

		data_hash_PL[i] = data_hash;
		count_cell_size[data_hash] = count_cell_size[data_hash] + 1;
	}

	cell_first_index[0] = 0;
	for (int i = 1; i < total_calculated_cell_size_PL; i++)
	{
		cell_first_index[i] = cell_first_index[i - 1] + count_cell_size[i - 1];
	}

	for (int i = 0; i < data_set_size_PL; i++)
	{
		int current_cell_index = data_hash_PL[i];
		int insert_index = (cell_first_index[current_cell_index] + cell_occupied_number[current_cell_index]);
		data_ordered_by_hash_PL[insert_index] = i;
		ordered_data_set_PL[insert_index] = data_set_PL[i];
		cell_occupied_number[current_cell_index] = cell_occupied_number[current_cell_index] + 1;
	}

	//for (int i = 0; i < k_sub_region_max; i++)
	//{
	//	if (count_cell_size[i] > 0)
	//	{
	//		std::cout << "hash number: " << i  << " has " << count_cell_size[i] << "data , they are: "<< std::endl;
	//		for (int j = 0; j < count_cell_size[i]; j++)
	//		{
	//			std::cout << data_ordered_by_hash_PL[cell_first_index[i] + j] << "  ";
	//		}
	//		std::cout << std::endl;
	//	}
	//}
}

int ExCalculateHash_hw(struct ThreeDimPoint point_data)
{
#pragma HLS INLINE
	//#pragma HLS UNROLL REGION

	//copy the point_data to data_x,y,z
	float data_x = point_data.x;
	float data_y = point_data.y;
	float data_z = point_data.z;
	int x_split_array_size = split_array_size_PL.x_array_size;
	int y_split_array_size = split_array_size_PL.y_array_size;
	int z_split_array_size = split_array_size_PL.z_array_size;
	//default x,y,z index as the max index
	int x_index;
	int y_index;
	int z_index;

	type_point x_min = data_max_min_PL.xmin;
	type_point y_min = data_max_min_PL.ymin;
	type_point z_min = data_max_min_PL.zmin;

	type_point x_unit = split_unit_PL.x;
	type_point y_unit = split_unit_PL.y;
	type_point z_unit = split_unit_PL.z;


	if (data_x <= x_split_array_PL[0])
		x_index = 0;
	else if (data_x >= x_split_array_PL[x_split_array_size - 1])
		x_index = x_split_array_size - 1;
	else
		x_index = (int)((data_x - x_min) / x_unit);

	if (data_y <= y_split_array_PL[0])
		y_index = 0;
	else if (data_y >= y_split_array_PL[y_split_array_size - 1])
		y_index = y_split_array_size - 1;
	else
		y_index = (int)((data_y - y_min) / y_unit);

	if (data_z <= z_split_array_PL[0])
		z_index = 0;
	else if (data_z >= z_split_array_PL[z_split_array_size - 1])
		z_index = z_split_array_size - 1;
	else
		z_index = (int)((data_z - z_min) / z_unit);

	//transform 3d index to a 1d index
	int data_index = x_index * y_split_array_size * z_split_array_size + y_index * z_split_array_size + z_index;

	return data_index;
}


void ExSearchKNNGBDS_hw()
{
	//#pragma HLS data_pack variable=query_data struct_level
	int valid_near_regions[k_search_near_regions_max];		//the valid regions index contains points

#pragma HLS ARRAY_PARTITION variable=valid_near_regions dim=1 cyclic factor=10

	int valid_near_region_size = 0;							//size of valid_near_region.  or number of valid_near_region
	int sum_near_points = 0;										// sum of points found in the near regions
	int query_index = ExCalculateHash_hw(query_data_PL);

	//find the near regions contains enough points...;  from distance 0 to infinite?...
	//while (sum_near_points < K* k_search_times)
	//consider pipeline the whole loop

	if (count_cell_size[query_index] > 0)
	{
		sum_near_points += count_cell_size[query_index];
		valid_near_regions[valid_near_region_size] = query_index;
		valid_near_region_size++;
	}


	if (sum_near_points < K_PL* k_search_points_times)
	{
		int search_distance = 1;
		for (int while_count = 0; while_count < 6; while_count++)
		{
		loop_Find_Near_Regions_ix:
			for (int ix = -search_distance; ix <= search_distance; ix++)
			{
#pragma HLS loop_tripcount min=1 max=40
				//#pragma HLS pipeline II=1

				int iy_higher_bound = search_distance - abs(ix);
				int iy_lower_bound = -iy_higher_bound;

			loop_Find_Near_Regions_iy:
				for (int iy = iy_lower_bound; iy <= iy_higher_bound; iy++)
				{
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=1 max=40
					int iz_abs = search_distance - abs(ix) - abs(iy);
					if (iz_abs >= 0)
					{
						char iz = iz_abs;
						int cal_index = query_index + ix * split_array_size_PL.y_array_size * split_array_size_PL.z_array_size + iy * split_array_size_PL.z_array_size + iz;
						//make sure the index is in the array's range
						if ((cal_index >= 0) && (cal_index < (total_calculated_cell_size_PL)))
						{
							if (count_cell_size[cal_index] > 0)
							{
								sum_near_points += count_cell_size[cal_index];
								valid_near_regions[valid_near_region_size] = cal_index;
								valid_near_region_size++;
							}

						}

						char iz_minus = -iz_abs;
						int cal_index_minus = query_index + ix * split_array_size_PL.y_array_size * split_array_size_PL.z_array_size + iy * split_array_size_PL.z_array_size + iz_minus;
						//make sure the index is in the array's range
						if ((cal_index_minus >= 0) && (cal_index_minus < (total_calculated_cell_size_PL)))
						{
							if (count_cell_size[cal_index_minus] > 0)
							{
								sum_near_points += count_cell_size[cal_index_minus];
								valid_near_regions[valid_near_region_size] = cal_index_minus;
								valid_near_region_size++;
							}

						}
					}

				}
			}
			if (sum_near_points >= K_PL * k_search_points_times)
				break;
			else
				search_distance++;
		}
	}


	////////////////////////////////find K_PL nearest neighbors in valid near sub-regions

	for (int i = 0; i < valid_near_region_size; i++)
	{
		int current_search_hash = valid_near_regions[i];
		for (int j = 0; j < count_cell_size[current_search_hash]; j++)
		{
			int index = data_ordered_by_hash_PL[cell_first_index[current_search_hash] + j];
			struct ThreeDimPoint test_p = ordered_data_set_PL[cell_first_index[current_search_hash] + j];
			type_point distance = ExEucDist_hw(query_data_PL, ordered_data_set_PL[cell_first_index[current_search_hash] + j]);	//data_set_PL[index]

		loop_knn_result_flag:
			for (int k = 0; k < k_nearest_number_max; k++)
			{
#pragma HLS loop_tripcount min=1 max=10
#pragma HLS unroll

				if (distance < nearest_distance_PL[k])
					sort_kick_flag[k] = true;
				else
					sort_kick_flag[k] = false;
			}

		loop_knn_result_shift:
			for (int k = k_nearest_number_max - 1; k > 0; k--)
			{

#pragma HLS loop_tripcount min=1 max=9
#pragma HLS unroll
#pragma HLS DEPENDENCE variable=nearest_distance_PL intra WAR true
#pragma HLS dependence variable=nearest_index_PL intra WAR true
				if (sort_kick_flag[k - 1] && sort_kick_flag[k])	//11
				{
					nearest_distance_PL[k] = nearest_distance_PL[k - 1];
					nearest_index_PL[k] = nearest_index_PL[k - 1];
				}
				if (!sort_kick_flag[k - 1] && sort_kick_flag[k])	//**0111
				{
					nearest_distance_PL[k] = nearest_distance_PL[k - 1];
					nearest_index_PL[k] = nearest_index_PL[k - 1];
					nearest_distance_PL[k - 1] = distance;
					nearest_index_PL[k - 1] = index;
				}
			}
			if (sort_kick_flag[0])
			{
				nearest_distance_PL[0] = distance;
				nearest_index_PL[0] = index;
			}
		}
	}
}

type_point ExEucDist_hw(struct ThreeDimPoint data1, struct ThreeDimPoint data2)
{
#pragma HLS INLINE
	return ((data1.x - data2.x)*(data1.x - data2.x)
		+ (data1.y - data2.y)*(data1.y - data2.y)
		+ (data1.z - data2.z)*(data1.z - data2.z)
		);
}

void ExBuildGBDS_hw(struct ThreeDimPoint data_set[k_data_set_size])
{
	ExGetMaxMin_hw(data_set);
	if (split_precise_PL == 0)
		ExSplitSubCell_hw();
	else
		ExSplitSubSpacePrecise_hw();
	ExDataClassify_hw();
}

void ExGBDSIPCore_hw(bool select_build_GBDS, My_Points data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, My_Points query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max], float split_precise)
{
	//#pragma HLS ARRAY_PARTITION variable=data_max_min_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=x_split_array_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=y_split_array_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=z_split_array_PL complete dim=0
	//#pragma HLS ARRAY_PARTITION variable=split_array_size_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=data_set_PL dim=1 cyclic factor=10
#pragma HLS data_pack variable=data_set struct_level
#pragma HLS ARRAY_PARTITION variable=sub_sets_size_PL dim=1 cyclic factor=10
#pragma HLS ARRAY_PARTITION variable=sub_sets_PL dim=1 cyclic factor=10
#pragma HLS ARRAY_PARTITION variable=nearest_index_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=nearest_distance_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=sort_kick_flag  complete dim=0

	if (select_build_GBDS)
	{
		//cache the data_set to PL
		data_set_size_PL = data_set_size;
		//my_rand_number_seed_PL = rand_seed;
		split_precise_PL = split_precise;

		ExBuildGBDS_hw(data_set);
	}
	else
	{
		//BUFF the data and then transmit to function

		for (int i = 0; i < k_nearest_number_max; i++)
		{
#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=10
			nearest_index_PL[i] = -1;
			nearest_distance_PL[i] = 1000000;
		}

		query_data_PL = query_data;
		K_PL = K;

		//SearchKNNGBDS(K_PL, query_data_PL, nearest_index_PL, nearest_distance_PL);
		ExSearchKNNGBDS_hw();

		// transmit the data to outside
	loop_output:
		for (int i = 0; i < K_PL; i++)
		{
#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=10
			nearest_index[i] = nearest_index_PL[i];
			nearest_distance[i] = nearest_distance_PL[i];
		}
	}
}

void ExSplitSubSpacePrecise_hw()
{
	float x_split_unit = split_precise_PL;
	float y_split_unit = split_precise_PL;
	float z_split_unit = split_precise_PL;
	int x_split_size = ((data_max_min_PL.xmax - data_max_min_PL.xmin) / split_precise_PL) + 1;	//int浼氭埅鏂紝姝ゅ+1纭繚鍒嗗壊鐨勭┖闂存暟杈惧埌闇�姹�;姝ゅ姞涓�绛夊悓浜巆eil()
	int y_split_size = ((data_max_min_PL.ymax - data_max_min_PL.ymin) / split_precise_PL) + 1;
	int z_split_size = ((data_max_min_PL.zmax - data_max_min_PL.zmin) / split_precise_PL) + 1;
	if (x_split_size > k_axis_cell_max)
		x_split_size = k_axis_cell_max;
	if (y_split_size > k_axis_cell_max)
		y_split_size = k_axis_cell_max;
	if (z_split_size > k_axis_cell_max)
		z_split_size = k_axis_cell_max;
	//if the calculated size is too big, then the total_..may be minus number
	float total_sub_spaces = abs((x_split_size) * (y_split_size) * (z_split_size));	//鍘熸潵鏄繖涓湴鏂圭殑闂銆傘�傚啓浠ｇ爜杩樻槸寰楁�濊檻鍛ㄥ叏鍟娿�傘��
																					//ensure the number of sub-spaces won't exceed the k_sub_region_max
	if (total_sub_spaces > k_cells_number_max)
	{
		//std::cout << "total_sub_spaces / k_sub_region_max: " << (total_sub_spaces / k_sub_region_max) <<std::endl;
		float split_down_size = cbrt(total_sub_spaces / k_cells_number_max);	//姹傜珛鏂规牴
																				//std::cout << "split_down_size: " << split_down_size <<std::endl;
		x_split_size = (x_split_size / split_down_size);
		y_split_size = (y_split_size / split_down_size);		//杩欎釜灏廱uG鎵句簡鎴戝ソ涔咃紝銆傘�傛垜璇存�庝箞浼氱畻鍑烘潵瓒呰繃鑼冨洿鐨勬暟;;
		z_split_size = (z_split_size / split_down_size);

		//绌洪棿涓嶅娌″姙娉曪紝閫�鍖栧洖涔嬪墠鐨勬柟娉�
		x_split_unit = (data_max_min_PL.xmax - data_max_min_PL.xmin) / x_split_size;
		y_split_unit = (data_max_min_PL.ymax - data_max_min_PL.ymin) / y_split_size;
		z_split_unit = (data_max_min_PL.zmax - data_max_min_PL.zmin) / z_split_size;
	}

	//when know the split unit and range, we can calculate the each split point and store it in a array.
	//stored data range: [min+true_split_precise, min + split_size*true_split_precise]
	//Use comparator "<" to calculate the index of the sub-spaces. so the first data ought to be min+true_split_precise

	type_point x_min = data_max_min_PL.xmin;
	type_point y_min = data_max_min_PL.ymin;
	type_point z_min = data_max_min_PL.zmin;

loop_split_space_precise_x:
	for (int i = 0; i < x_split_size; i++)
	{
		//consider parallel
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=50
		x_split_array_PL[i] = x_min + (i + 1) * x_split_unit;
	}
loop_split_space_precise_y:
	for (int i = 0; i < y_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=50
		x_split_array_PL[i] = y_min + (i + 1) * y_split_unit;
	}
loop_split_space_precise_z:
	for (int i = 0; i < z_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=50
		x_split_array_PL[i] = z_min + (i + 1) * z_split_unit;
	}

	//the split array range [min,max];(contain the min and max)
	split_array_size_PL.x_array_size = x_split_size;
	split_array_size_PL.y_array_size = y_split_size;
	split_array_size_PL.z_array_size = z_split_size;

	split_unit_PL.x = x_split_unit;
	split_unit_PL.y = y_split_unit;
	split_unit_PL.z = z_split_unit;
	total_calculated_cell_size_PL = x_split_size * y_split_size * z_split_size;
}