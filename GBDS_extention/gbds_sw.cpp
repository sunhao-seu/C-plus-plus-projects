#undef __ARM_NEON__
#undef __ARM_NEON
#include "gbds_pcl.h"
#define __ARM_NEON__
#define __ARM_NEON


// variables stored in PL
static struct ThreeDimPoint data_set_PL[k_data_set_size];

static struct MaxMin data_max_min_PL;

static int data_set_size_PL;
static type_point x_split_array_PL[k_axis_spaces_max];
static type_point y_split_array_PL[k_axis_spaces_max];
static type_point z_split_array_PL[k_axis_spaces_max];

static struct SplitArraySize split_array_size_PL;

static int sub_sets_PL[k_sub_region_max][k_sub_set_space_size];

static int sub_sets_size_PL[k_sub_region_max];
static unsigned int my_rand_number_seed_PL;
static int nearest_index_PL[k_nearest_number_max];
static type_point nearest_distance_PL[k_nearest_number_max];

static int K_PL;
static struct ThreeDimPoint query_data_PL;
static float split_precise_PL;

//new 
static int data_hash[k_data_set_size];
static int data_ordered_by_hash[k_data_set_size];



void GetMaxMin_sw(struct ThreeDimPoint data_set[k_data_set_size], struct MaxMin & data_max_min, int data_set_size)
{

	type_point x_max, x_min, y_max, y_min, z_max, z_min;

	x_max = x_min = 0;
	y_max = y_min = 0;
	z_max = z_min = 0;

loop_min_max:
	for (int i = 0; i<data_set_size; i ++)
	{
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=1 max=100000
		type_point data_x = data_set[i].x;
		type_point data_y = data_set[i].y;
		type_point data_z = data_set[i].z;

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
	data_max_min.xmin = x_min;
	data_max_min.xmax = x_max;
	data_max_min.ymin = y_min;
	data_max_min.ymax = y_max;
	data_max_min.zmin = z_min;
	data_max_min.zmax = z_max;
//	std::cout << "data_max_min.xmin: " << data_max_min.xmin <<std::endl;
//	std::cout << "data_max_min.xmax: " << data_max_min.xmax <<std::endl;
//	std::cout << "data_max_min.ymin: " << data_max_min.ymin <<std::endl;
//	std::cout << "data_max_min.ymax: " << data_max_min.ymax <<std::endl;
//	std::cout << "data_max_min.zmin: " << data_max_min.zmin <<std::endl;
//	std::cout << "data_max_min.zmax: " << data_max_min.zmax <<std::endl;
}

// great when points are evently disributed....more general;
//while split_precise id more specify for LOAM...for a certain precise search...
void SplitSubSpace_sw(int data_set_size, struct MaxMin data_max_min, float split_precise, type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize & split_array_size)
{
	//memset(x_split_array, 0, sizeof(type_point) * split_array_size.x_array_size);
	//memset(y_split_array, 0, sizeof(type_point) * split_array_size.y_array_size);
	//memset(z_split_array, 0, sizeof(type_point) * split_array_size.z_array_size);

	int calculated_space_size = (data_set_size / k_sub_space_data_size + 1);		//evaluate the sub-spaces in need
	float x_range = data_max_min.xmax - data_max_min.xmin;							//the range of the data on x axis
	float y_range = data_max_min.ymax - data_max_min.ymin;
	float z_range = data_max_min.zmax - data_max_min.zmin;

	//calculate the max range of x,y,z; to avoid exceed k_axis_spaces_max
	float max_range = x_range;
	if ( y_range > max_range)
		max_range = y_range;
	if (z_range > max_range)
		max_range = z_range;

	float x_times_max = x_range / max_range;
	float y_times_max = y_range / max_range;
	float z_times_max = z_range / max_range;
	float temp = x_times_max * y_times_max * z_times_max;

	int max_split_size = k_axis_spaces_max;	//defaut maximum splited spaces in a axis
	//calculate the split size of the max range on x,y,z
loop_split_space_1:
	for (int i = 0; i < k_axis_spaces_max; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30

		float total_sub_spaces = temp * i * i * i;
		if (total_sub_spaces > calculated_space_size)
		{
			max_split_size = i;
			break;
		}
	}

	//ceil is pretty ok, it won't exceed the array size
	//calculate the split size of x,y,z axis
	int x_split_size = ceil(max_split_size * x_times_max);
	int y_split_size = ceil(max_split_size * y_times_max);
	int z_split_size = ceil(max_split_size * z_times_max);

	if(x_split_size > k_axis_sub_region_max)
		x_split_size = k_axis_sub_region_max;
	if(y_split_size > k_axis_sub_region_max)
		y_split_size = k_axis_sub_region_max;
	if(z_split_size > k_axis_sub_region_max)
		z_split_size = k_axis_sub_region_max;
	float total_sub_spaces = abs((x_split_size) * (y_split_size ) * (z_split_size ));	//鍘熸潵鏄繖涓湴鏂圭殑闂銆傘�傚啓浠ｇ爜杩樻槸寰楁�濊檻鍛ㄥ叏鍟娿�傘��
	//ensure the number of sub-spaces won't exceed the k_sub_region_max
	if (total_sub_spaces > k_sub_region_max)
	{
		//std::cout << "total_sub_spaces / k_sub_region_max: " << (total_sub_spaces / k_sub_region_max) <<std::endl;
		float split_down_size = cbrt(total_sub_spaces / k_sub_region_max);	//姹傜珛鏂规牴
		//std::cout << "split_down_size: " << split_down_size <<std::endl;
		x_split_size = (x_split_size / split_down_size);
		y_split_size = (y_split_size / split_down_size);		//杩欎釜灏廱uG鎵句簡鎴戝ソ涔咃紝銆傘�傛垜璇存�庝箞浼氱畻鍑烘潵瓒呰繃鑼冨洿鐨勬暟;;
		z_split_size = (z_split_size / split_down_size);
	}

	type_point x_unit = x_range / ( ceil(max_split_size * x_times_max) );
	type_point y_unit = y_range / (ceil(max_split_size * y_times_max));
	type_point z_unit = z_range / (ceil(max_split_size * z_times_max));

	//when know the split unit and range, we can calculate the each split point and store it in a array.

	type_point x_min = data_max_min.xmin;
	type_point y_min = data_max_min.ymin;
	type_point z_min = data_max_min.zmin;

loop_split_space_x:
	for (int i = 1; i <= x_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		x_split_array[i] = x_min + i * x_unit;
	}
loop_split_space_y:
	for (int i = 1; i <= y_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		y_split_array[i] = y_min + i * y_unit;
	}
loop_split_space_z:
	for (int i = 1; i <= z_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		z_split_array[i] = z_min + i * z_unit;
	}

	//the split array range [min,max];(contain the min and max)
	split_array_size.x_array_size = x_split_size;
	split_array_size.y_array_size = y_split_size;
	split_array_size.z_array_size = z_split_size;
}

//#pragma SDS data mem_attribute(x_split_array:NON_CACHEABLE|PHYSICAL_CONTIGUOUS, y_split_array:NON_CACHEABLE|PHYSICAL_CONTIGUOUS, z_split_array:NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
//#pragma SDS data access_pattern("x_split_array":SEQUENTIAL,"y_split_array":SEQUENTIAL,"z_split_array":SEQUENTIAL)
//#pragma SDS data data_mover(rand_x:AXIDMA_SIMPLE, rand_y:AXIDMA_SIMPLE, rand_angle:AXIDMA_SIMPLE)
void SplitSubSpacePrecise_sw(int data_set_size, struct MaxMin data_max_min, float split_precise, type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize & split_array_size)
{
	float x_split_unit = split_precise;
	float y_split_unit = split_precise;
	float z_split_unit = split_precise;
	int x_split_size = ((data_max_min.xmax - data_max_min.xmin) / split_precise)+1;	//int浼氭埅鏂紝姝ゅ+1纭繚鍒嗗壊鐨勭┖闂存暟杈惧埌闇�姹�;姝ゅ姞涓�绛夊悓浜巆eil()
	int y_split_size = ((data_max_min.ymax - data_max_min.ymin) / split_precise)+1;
	int z_split_size = ((data_max_min.zmax - data_max_min.zmin) / split_precise) + 1;
	if(x_split_size > k_axis_sub_region_max)
		x_split_size = k_axis_sub_region_max;
	if(y_split_size > k_axis_sub_region_max)
		y_split_size = k_axis_sub_region_max;
	if(z_split_size > k_axis_sub_region_max)
		z_split_size = k_axis_sub_region_max;
	//if the calculated size is too big, then the total_..may be minus number
	float total_sub_spaces = abs((x_split_size) * (y_split_size ) * (z_split_size ));	//鍘熸潵鏄繖涓湴鏂圭殑闂銆傘�傚啓浠ｇ爜杩樻槸寰楁�濊檻鍛ㄥ叏鍟娿�傘��
	//ensure the number of sub-spaces won't exceed the k_sub_region_max
	if (total_sub_spaces > k_sub_region_max)
	{
		//std::cout << "total_sub_spaces / k_sub_region_max: " << (total_sub_spaces / k_sub_region_max) <<std::endl;
		float split_down_size = cbrt(total_sub_spaces / k_sub_region_max);	//姹傜珛鏂规牴
		//std::cout << "split_down_size: " << split_down_size <<std::endl;
		x_split_size = (x_split_size / split_down_size);
		y_split_size = (y_split_size / split_down_size);		//杩欎釜灏廱uG鎵句簡鎴戝ソ涔咃紝銆傘�傛垜璇存�庝箞浼氱畻鍑烘潵瓒呰繃鑼冨洿鐨勬暟;;
		z_split_size = (z_split_size / split_down_size);

		//绌洪棿涓嶅娌″姙娉曪紝閫�鍖栧洖涔嬪墠鐨勬柟娉�
		x_split_unit = (data_max_min.xmax - data_max_min.xmin) / x_split_size;
		y_split_unit = (data_max_min.ymax - data_max_min.ymin) / y_split_size;
		z_split_unit = (data_max_min.zmax - data_max_min.zmin) / z_split_size;
	}

	//when know the split unit and range, we can calculate the each split point and store it in a array.
	//stored data range: [min+true_split_precise, min + split_size*true_split_precise]
	//Use comparator "<" to calculate the index of the sub-spaces. so the first data ought to be min+true_split_precise

	type_point x_min = data_max_min.xmin;
	type_point y_min = data_max_min.ymin;
	type_point z_min = data_max_min.zmin;

loop_split_space_precise_x:
	for (int i = 0; i < x_split_size; i++)
	{
		//consider parallel
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		x_split_array[i] = x_min + (i+1) * x_split_unit;
	}
loop_split_space_precise_y:
	for (int i = 0; i < y_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		y_split_array[i] = y_min + (i + 1) * y_split_unit;
	}
loop_split_space_precise_z:
	for (int i = 0; i < z_split_size; i++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=30
		z_split_array[i] = z_min + (i + 1) * z_split_unit;
	}

	//the split array range [min,max];(contain the min and max)
	split_array_size.x_array_size = x_split_size;
	split_array_size.y_array_size = y_split_size;
	split_array_size.z_array_size = z_split_size;

}


//#pragma SDS data mem_attribute("data_set":NON_CACHEABLE|PHYSICAL_CONTIGUOUS, "x_split_array":NON_CACHEABLE|PHYSICAL_CONTIGUOUS, "y_split_array":NON_CACHEABLE|PHYSICAL_CONTIGUOUS, "z_split_array":NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
//#pragma SDS data access_pattern("data_set":SEQUENTIAL,"x_split_array":SEQUENTIAL,"y_split_array":SEQUENTIAL,"z_split_array":SEQUENTIAL)
//#pragma SDS data copy(data_set[0:k_data_set_size])
//#pragma SDS data mem_attribute("sub_sets":NON_CACHEABLE|PHYSICAL_CONTIGUOUS, "sub_sets_size":NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
//#pragma SDS data access_pattern("sub_sets":SEQUENTIAL, "sub_sets_size":SEQUENTIAL)
void DataClassify_sw(struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize split_array_size, int sub_sets[k_sub_region_max][k_sub_set_space_size], int sub_sets_size[k_sub_region_max], unsigned int my_rand_number_seed)
{
	//	srand((unsigned)time(NULL));
	unsigned int rand_number_generate = my_rand_number_seed;
	//reset the sub_set_size
loop_reset_sub_sets_size:
	for(int i = 0; i < k_sub_region_max; i ++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=12500
		sub_sets_size[i] = 0;
	}

loop_clarify_data:
	for (int i = 0; i < data_set_size; i++)
	{
//#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=100000
		int data_index = PCLCalculateIndex(data_set[i], x_split_array, y_split_array, z_split_array, split_array_size);

		my_rand_generate(&rand_number_generate);
		int rand_number = (int)(rand_number_generate % 10);

//		std::cout << "rand_number: " << rand_number << std::endl;
//		std::cout << "data_index: " << data_index << std::endl;
		//The following code ensure the data size won't exceed the allocated array size,
		// also for sampling evenly
		if (sub_sets_size[data_index] < k_sub_set_space_size)		// 1.0 / 3 *  arrray size between 0 and 1/3*k_sub_set_space_size
		{
			sub_sets[data_index][sub_sets_size[data_index]] = i;
			sub_sets_size[data_index] = sub_sets_size[data_index] + 1;
		}
		else if (sub_sets_size[data_index] < 2.0 / 3 * k_sub_set_space_size)	//arrray size between 1/3*k_sub_set_space_size and 2/3*k_sub_set_space_size
		{

			if (rand_number < 3)
				sub_sets[data_index][sub_sets_size[data_index]] = i;
			else
				sub_sets_size[data_index] = sub_sets_size[data_index] - 1;
		}

		else if (sub_sets_size[data_index] < 4.0 / 5 * k_sub_set_space_size)	//arrray size between 2/3*k_sub_set_space_size and 4/5*k_sub_set_space_size
		{
			//1/5 probability
			if (rand_number < 2)
				sub_sets[data_index][sub_sets_size[data_index]] = i;
			else
				sub_sets_size[data_index] = sub_sets_size[data_index] - 1;
		}
		else if(sub_sets_size[data_index] < k_sub_set_space_size)	//arrray size between 4/5*k_sub_set_space_size and _sub_space_array_size
		{
			// 1/10 probability
			if (rand_number < 1)
				sub_sets[data_index][sub_sets_size[data_index]] = i;
			else
				sub_sets_size[data_index] = sub_sets_size[data_index] - 1;
		}
		else
		{
			sub_sets_size[data_index] = sub_sets_size[data_index] - 1;
		}
		sub_sets_size[data_index] = sub_sets_size[data_index] + 1;
	}
}


int PCLCalculateIndex_sw(struct ThreeDimPoint point_data, type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize split_array_size)
{
//#pragma HLS INLINE

	//copy the point_data to data_x,y,z
	float data_x = point_data.x;
	float data_y = point_data.y;
	float data_z = point_data.z;
	//default x,y,z index as the max index
	int x_index = split_array_size.x_array_size-1;
	int y_index = split_array_size.y_array_size-1;
	int z_index = split_array_size.z_array_size-1;
	//similar to look-up-table
loop_cal_index_x:
	for (int jx = 0; jx < split_array_size.x_array_size; jx++)
	{
#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=30
		if (data_x <= x_split_array[jx])
		{
			x_index = jx;
			break;
		}

	}
loop_cal_index_y:
	for (int jy = 0; jy < split_array_size.y_array_size; jy++)
	{
#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=30
		if (data_y <= y_split_array[jy])
		{
			y_index = jy;
			break;
		}
	}
loop_cal_index_z:
	for (int jz = 0; jz < split_array_size.z_array_size; jz++)
	{
#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=30
		if (data_z <= z_split_array[jz])
		{
			z_index = jz;
			break;
		}

	}

	//transform 3d index to a 1d index
	int data_index = x_index * split_array_size.y_array_size * split_array_size.z_array_size + y_index * split_array_size.z_array_size + z_index;

	return data_index;
}




void SearchKNearestNeighbors_sw(int K, struct ThreeDimPoint query_data, struct ThreeDimPoint data_set[k_data_set_size], int sub_sets[k_sub_region_max][k_sub_set_space_size], int sub_sets_size[k_sub_region_max], type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize split_array_size, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max])
{
	int query_index = PCLCalculateIndex(query_data, x_split_array, y_split_array, z_split_array, split_array_size);
	int search_distance = 0;								//distance of near_region and current region
	int near_regions[k_search_near_regions_max];			//store the near_regions finded by Find_Near_Regions() function
	int valid_near_regions[k_search_near_regions_max];		//the valid regions index contains points
	int near_region_size = 0;								//size of near_regions.  or number of near_regions
	int valid_near_region_size = 0;							//size of valid_near_region.  or number of valid_near_region
	int sum_near_points = 0;										// sum of points found in the near regions

	//find the near regions contains enough points...;  from distance 0 to infinite?...
	while (sum_near_points < K* k_search_times)
	{
		Find_Near_Regions(query_index, search_distance, near_regions, near_region_size, split_array_size);	//find the index of near regions with distance = near_distance;
		//count the valid_regions and copy it to valid_near_regions
		for (int i = 0; i < near_region_size; i++)
		{
#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=1000
			if (sub_sets_size[near_regions[i]] > 0)
			{
				sum_near_points += sub_sets_size[near_regions[i]];
				valid_near_regions[valid_near_region_size] = near_regions[i];
				valid_near_region_size++;
			}

		}
		//if the points are not enough for K , enlarge the distance
		search_distance++;
	}

	//find K nearest neighbors in valid near sub-regions
	//initialize the output array
//	for (int i = 0; i < K; i++)
//	{
//		nearest_distance[i] = 10000;
//		nearest_index[i] = 10000;
//	}

	//For every valid sub-regions
	for (int i = 0; i < valid_near_region_size; i++)
	{
		//for every points in i-th sub_region
		//int test_k = sub_sets_size[valid_near_regions[i]];
		for (int j = 0; j < sub_sets_size[valid_near_regions[i]]; j++)
		{
#pragma HLS PIPELINE
#pragma HLS loop_tripcount min=1 max=1000
			//calculate the distance between query and point j;
			type_point distance = PCLEucDist(query_data, data_set[sub_sets[ valid_near_regions[i] ][j] ]);

			//if distance is smaller than the biggest distance in result array,
			//then store the distance and the index of the point to replace the old biggest one.
			if (distance < nearest_distance[K - 1])	//the gate of step in the result array
			{
				nearest_distance[K - 1] = distance;
				nearest_index[K - 1] = sub_sets[valid_near_regions[i]][j];		//the indx of this point
				//Sort the result array
				for (int k = K - 2; k >= 0; k--)
				{
					if (distance < nearest_distance[k])
					{
						nearest_distance[k + 1] = nearest_distance[k];
						nearest_distance[k] = distance;
						nearest_index[k + 1] = nearest_index[k];
						nearest_index[k] = sub_sets[valid_near_regions[i]][j];
					}
					else break;	//if the new data is bigger than any data in the vector, the loop will be break.
				}
			}

		}
	}


}

void Find_Near_Regions_sw(int current_index, int search_distance, int near_regions[k_search_near_regions_max], int & near_region_size, struct SplitArraySize split_array_size)
{
	near_region_size = 0;		//init the near_region size to 0
	for (int ix = -search_distance; ix <= search_distance; ix++)
	{
#pragma HLS loop_tripcount min=1 max=100
		for (int iy = -search_distance; iy <= search_distance; iy++)
		{
#pragma HLS loop_tripcount min=1 max=100
loop_Find_Near_Regions:
			for (int iz = -search_distance; iz <= search_distance; iz++)
			{
#pragma HLS pipeline II=1
#pragma HLS loop_tripcount min=1 max=100
				// only consider the regions with certain(search_distance) distance
				if ((abs(ix) + abs(iy) + abs(iz)) == search_distance)
				{
					int cal_index = current_index + ix * split_array_size.y_array_size * split_array_size.z_array_size + iy * split_array_size.z_array_size + iz;
					//make sure the index is in the array's range
					if ((cal_index >= 0) && (cal_index < (split_array_size.x_array_size *split_array_size.y_array_size * split_array_size.z_array_size)))
					{
						if(near_region_size < k_search_near_regions_max)
						{
							near_regions[near_region_size] = cal_index;
							near_region_size++;
						}

					}

				}
			}
		}
	}
}

/** @brief Calculate the euclidean distance between two vector3 data
*
*  @param data1,data2
*  @return the euclidean distance
*/
type_point PCLEucDist_sw(struct ThreeDimPoint data1, struct ThreeDimPoint data2)
{
#pragma HLS INLINE
	return ((data1.x - data2.x)*(data1.x - data2.x)
		+ (data1.y - data2.y)*(data1.y - data2.y)
		+ (data1.z - data2.z)*(data1.z - data2.z)
	);
}

void my_rand_generate_sw(unsigned int *value)
{
#pragma HLS INLINE

	//LSFR 64位随机数生成
	unsigned int x;
	x = *value;	/* The state must be seeded with a nonzero value. */
	x ^= x >> 12; // a
	x ^= x << 25; // b
	x ^= x >> 27; // c
	*value = x;
}


//#pragma SDS data mem_attribute("data_set":NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
//#pragma SDS data access_pattern("data_set":SEQUENTIAL)
//#pragma SDS data copy(data_set[0:k_data_set_size])
void BuildGBDS_sw(struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, float split_precise)
{


	GetMaxMin(data_set, data_max_min_PL, data_set_size);
	if(split_precise == 0)
		SplitSubSpace(data_set_size, data_max_min_PL, split_precise, x_split_array_PL, y_split_array_PL, z_split_array_PL, split_array_size_PL);
	else
		SplitSubSpacePrecise(data_set_size, data_max_min_PL, split_precise, x_split_array_PL, y_split_array_PL, z_split_array_PL, split_array_size_PL);
	DataClassify(data_set, data_set_size, x_split_array_PL, y_split_array_PL, z_split_array_PL, split_array_size_PL, sub_sets_PL, sub_sets_size_PL, rand_seed);

}


//#pragma SDS data mem_attribute("nearest_index":NON_CACHEABLE|PHYSICAL_CONTIGUOUS, "nearest_distance":NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
//#pragma SDS data access_pattern("nearest_index":SEQUENTIAL, "nearest_distance":SEQUENTIAL)
//#pragma SDS data copy(nearest_index[0:k_nearest_number_max], nearest_distance[0:k_nearest_number_max])		// tell the compiler the size of data_set;
//void SearchKNNGBDS(int K, struct ThreeDimPoint query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max])
void SearchKNNGBDS_sw()
{
//#pragma HLS data_pack variable=query_data struct_level

	int search_distance = 0;								//distance of near_region and current region
	int near_regions[k_search_near_regions_max];			//store the near_regions found by Find_Near_Regions() function
	int valid_near_regions[k_search_near_regions_max];		//the valid regions index contains points
	int near_region_size = 0;								//size of near_regions.  or number of near_regions
	int valid_near_region_size = 0;							//size of valid_near_region.  or number of valid_near_region
	int sum_near_points = 0;										// sum of points found in the near regions
	int query_index = PCLCalculateIndex_sw(query_data_PL, x_split_array_PL, y_split_array_PL, z_split_array_PL, split_array_size_PL);

	//find the near regions contains enough points...;  from distance 0 to infinite?...
	//while (sum_near_points < K* k_search_times)
	for(int while_count = 0; while_count < 100; while_count ++)
	{
#pragma HLS loop_tripcount min=1 max=100
		if(sum_near_points < K_PL* k_search_times)
		{
			Find_Near_Regions_sw(query_index, search_distance, near_regions, near_region_size, split_array_size_PL);	//find the index of near regions with distance = near_distance;
			//count the valid_regions and copy it to valid_near_regions
			for (int i = 0; i < near_region_size; i++)
			{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=1000
				if (sub_sets_size_PL[near_regions[i]] > 0)
				{
					sum_near_points += sub_sets_size_PL[near_regions[i]];
					valid_near_regions[valid_near_region_size] = near_regions[i];
					valid_near_region_size++;
				}

			}
			//if the points are not enough for K_PL , enlarge the distance
			search_distance++;
		}
		else break;

	}

////////////////////////////////find K_PL nearest neighbors in valid near sub-regions
	//For every valid sub-regions
	for (int i = 0; i < valid_near_region_size; i++)
	{
#pragma HLS loop_tripcount min=1 max=100
		//for every points in i-th sub_region
		//int test_k = sub_sets_size_PL[valid_near_regions[i]];
		for (int j = 0; j < sub_sets_size_PL[valid_near_regions[i]]; j++)
		{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=20

			//calculate the distance between query and point j;
			int index = sub_sets_PL[ valid_near_regions[i] ][j];
			type_point distance = PCLEucDist_sw(query_data_PL, data_set_PL[index]);


			//if distance is smaller than the biggest distance in result array,
			//then store the distance and the index of the point to replace the old biggest one.
			if (distance < nearest_distance_PL[K_PL - 1])	//the gate of step in the result array
			{

				nearest_distance_PL[K_PL - 1] = distance;
				nearest_index_PL[K_PL - 1] = index;		//the indx of this point
				//Sort the result array
				for (int k = K_PL - 2; k >= 0; k--)
				{
#pragma HLS loop_tripcount min=1 max=10
					if (distance < nearest_distance_PL[k])
					{
						nearest_distance_PL[k + 1] = nearest_distance_PL[k];
						nearest_distance_PL[k] = distance;
						nearest_index_PL[k + 1] = nearest_index_PL[k];
						nearest_index_PL[k] = index;
					}
					else break;	//if the new data is bigger than any data in the vector, the loop will be break.
				}
			}
		}
	}

}

//#pragma SDS data mem_attribute("data_set":NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
#pragma SDS data access_pattern("data_set":SEQUENTIAL)
#pragma SDS data copy("data_set"[0:data_set_size])
//#pragma SDS data mem_attribute("nearest_index":NON_CACHEABLE|PHYSICAL_CONTIGUOUS, "nearest_distance":NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
//#pragma SDS data access_pattern("nearest_index":SEQUENTIAL, "nearest_distance":SEQUENTIAL)
//#pragma SDS data copy(nearest_index[0:k_nearest_number_max], nearest_distance[0:k_nearest_number_max])		// tell the compiler the size of data_set;
//#pragma SDS data copy("data_set"[0:k_data_set_size], "nearest_index"[0:k_nearest_number_max], "nearest_distance"[0:k_nearest_number_max])
#pragma SDS data mem_attribute("data_set":PHYSICAL_CONTIGUOUS, "nearest_index":PHYSICAL_CONTIGUOUS, "nearest_distance":PHYSICAL_CONTIGUOUS)
void GBDSIPCore_sw(bool select_build_GBDS, My_Points data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, My_Points query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max], float split_precise)
{
//#pragma HLS ARRAY_PARTITION variable=data_set_PL dim=1 cyclic factor=3
//#pragma HLS ARRAY_PARTITION variable=data_max_min_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=x_split_array_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=y_split_array_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=z_split_array_PL complete dim=0
//#pragma HLS ARRAY_PARTITION variable=split_array_size_PL complete dim=0
#pragma HLS ARRAY_PARTITION variable=data_set_PL dim=1 cyclic factor=2
#pragma HLS data_pack variable=data_set struct_level

	if(select_build_GBDS)
	{
		//cache the data_set to PL
		data_set_size_PL = data_set_size;
		my_rand_number_seed_PL = rand_seed;
		split_precise_PL = split_precise;

loop_cache_dataset:
		for (int i = 0; i < data_set_size_PL; i ++)
		{
	#pragma HLS pipeline II=1
	#pragma HLS loop_tripcount min=1 max=100000
			data_set_PL[i].x = data_set[i].x;
			data_set_PL[i].y = data_set[i].y;
			data_set_PL[i].z = data_set[i].z;
		}

		//std::cout << "start" <<std::endl;
		BuildGBDS_sw(data_set_PL, data_set_size_PL, my_rand_number_seed_PL, split_precise_PL);
	}
	else
	{
		//BUFF the data and then transmit to function

		for(int i = 0; i < k_nearest_number_max; i ++)
		{
	#pragma HLS PIPELINE
	#pragma HLS loop_tripcount min=1 max=10
			nearest_index_PL[i] = 0;
			nearest_distance_PL[i] = 100000;
		}

		query_data_PL = query_data;
		K_PL = K;

		//SearchKNNGBDS(K_PL, query_data_PL, nearest_index_PL, nearest_distance_PL);
		SearchKNNGBDS_sw();

		// transmit the data to outside
		for(int i = 0; i < K_PL; i ++)
		{
	#pragma HLS PIPELINE
	#pragma HLS loop_tripcount min=1 max=10
			nearest_index[i] = nearest_index_PL[i];
			nearest_distance[i] = nearest_distance_PL[i];
		}
	}
}
