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
static bool sort_kick_flag[k_nearest_number_max];

static int K_PL;
static struct ThreeDimPoint query_data_PL;
static float split_precise_PL;


void GetMaxMin(struct ThreeDimPoint data_set[k_data_set_size], struct MaxMin & data_max_min, int data_set_size)
{

	type_point x_max, x_min, y_max, y_min, z_max, z_min;

	x_max = x_min = 0;
	y_max = y_min = 0;
	z_max = z_min = 0;

loop_cache_and_min_max:
	for (int i = 0; i<data_set_size; i ++)
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
void SplitSubSpace(int data_set_size, struct MaxMin data_max_min, float split_precise, type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize & split_array_size)
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
	//int max_split_size = k_axis_spaces_max;	//defaut maximum splited spaces in a axis
	//calculate the split size of the max range on x,y,z
	//it seems better to calculate the result directly... multiply is resource cost...

	float calculated_split_size = cbrt(calculated_space_size/temp);
	float max_split_size = cbrt(k_sub_region_max/temp);
	if(calculated_split_size > max_split_size)
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

	type_point x_min = data_max_min.xmin;
	type_point y_min = data_max_min.ymin;
	type_point z_min = data_max_min.zmin;

	//consider parallel? the three loops can be executed simutaneously
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
void SplitSubSpacePrecise(int data_set_size, struct MaxMin data_max_min, float split_precise, type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize & split_array_size)
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
void DataClassify(struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize split_array_size, int sub_sets[k_sub_region_max][k_sub_set_space_size], int sub_sets_size[k_sub_region_max], unsigned int my_rand_number_seed)
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
#pragma HLS PIPELINE II=1
#pragma HLS loop_tripcount min=1 max=100000
#pragma HLS DEPENDENCE variable=sub_sets_size intra WAR true
#pragma HLS DEPENDENCE variable=sub_sets_size inter false



		int data_index = PCLCalculateIndex(data_set[i], x_split_array, y_split_array, z_split_array, split_array_size);

		my_rand_generate(&rand_number_generate);
		unsigned char rand_number = (unsigned char)(rand_number_generate % 10);

//		std::cout << "rand_number: " << rand_number << std::endl;
//		std::cout << "data_index: " << data_index << std::endl;
		//The following code ensure the data size won't exceed the allocated array size,
		// also for sampling evenly
		int step1 = 1.0 / 3 * k_sub_set_space_size;
		int step2 = 2.0 / 3 * k_sub_set_space_size;
		int step3 = 4.0 / 5 * k_sub_set_space_size;
		int step4 = k_sub_set_space_size - 1;	//remove the loop dependancy..
		int index_sub_sets_size = sub_sets_size[data_index];
		if ( (index_sub_sets_size < step1)
				|| (index_sub_sets_size < step2 && index_sub_sets_size >= step1 && rand_number < 6)
				|| (index_sub_sets_size < step3 && index_sub_sets_size >= step2 && rand_number < 4)
				|| (index_sub_sets_size >= step3 && index_sub_sets_size < step4 && rand_number < 2) )
		{
			sub_sets[data_index][index_sub_sets_size] = i;
			sub_sets_size[data_index] = index_sub_sets_size + 1;
		}

	}
}


int PCLCalculateIndex(struct ThreeDimPoint point_data, type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize split_array_size)
{
#pragma HLS INLINE
//#pragma HLS UNROLL REGION

	//copy the point_data to data_x,y,z
	float data_x = point_data.x;
	float data_y = point_data.y;
	float data_z = point_data.z;
	int x_split_array_size = split_array_size.x_array_size;
	int y_split_array_size = split_array_size.y_array_size;
	int z_split_array_size = split_array_size.z_array_size;
	//default x,y,z index as the max index
	int x_index;
	int y_index;
	int z_index;

	type_point x_min = x_split_array[0];
	type_point y_min = y_split_array[0];
	type_point z_min = z_split_array[0];

	type_point x_unit = x_split_array[1] - x_split_array[0];
	type_point y_unit = y_split_array[1] - y_split_array[0];
	type_point z_unit = z_split_array[1] - z_split_array[0];
	//similar to look-up-table
//loop_cal_index_x:
//	for (int jx = 0; jx < split_array_size.x_array_size; jx++)
//	{
//#pragma HLS UNROLL
//#pragma HLS loop_tripcount min=1 max=30
//		if (data_x <= x_split_array[jx])
//		{
//			x_index = jx;
//			break;
//		}
//
//	}
//loop_cal_index_y:
//	for (int jy = 0; jy < split_array_size.y_array_size; jy++)
//	{
//#pragma HLS UNROLL
//#pragma HLS loop_tripcount min=1 max=30
//		if (data_y <= y_split_array[jy])
//		{
//			y_index = jy;
//			break;
//		}
//	}
//loop_cal_index_z:
//	for (int jz = 0; jz < split_array_size.z_array_size; jz++)
//	{
//#pragma HLS UNROLL
//#pragma HLS loop_tripcount min=1 max=30
//		if (data_z <= z_split_array[jz])
//		{
//			z_index = jz;
//			break;
//		}
//
//	}

	if(data_x <= x_split_array[0])
		x_index = 0;
	else if(data_x >= x_split_array[x_split_array_size - 1])
		x_index = x_split_array_size-1;
	else
		x_index = (int)( (data_x - x_min)/x_unit);

	if(data_y <= y_split_array[0])
		y_index = 0;
	else if(data_y >= y_split_array[y_split_array_size - 1])
		y_index = y_split_array_size-1;
	else
		y_index = (int)( (data_y - y_min)/y_unit);

	if(data_z <= z_split_array[0])
		z_index = 0;
	else if(data_z >= z_split_array[z_split_array_size - 1])
		z_index = z_split_array_size-1;
	else
		z_index = (int)( (data_z - z_min)/z_unit);

	//transform 3d index to a 1d index
	int data_index = x_index * y_split_array_size * z_split_array_size + y_index * z_split_array_size + z_index;

	return data_index;
}




//void SearchKNearestNeighbors(int K, struct ThreeDimPoint query_data, struct ThreeDimPoint data_set[k_data_set_size], int sub_sets[k_sub_region_max][k_sub_set_space_size], int sub_sets_size[k_sub_region_max], type_point x_split_array[k_axis_spaces_max], type_point y_split_array[k_axis_spaces_max], type_point z_split_array[k_axis_spaces_max], struct SplitArraySize split_array_size, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max])
//{
//	int query_index = PCLCalculateIndex(query_data, x_split_array, y_split_array, z_split_array, split_array_size);
//	int search_distance = 0;								//distance of near_region and current region
//	int near_regions[k_search_near_regions_max];			//store the near_regions finded by Find_Near_Regions() function
//	int valid_near_regions[k_search_near_regions_max];		//the valid regions index contains points
//	int near_region_size = 0;								//size of near_regions.  or number of near_regions
//	int valid_near_region_size = 0;							//size of valid_near_region.  or number of valid_near_region
//	int sum_near_points = 0;										// sum of points found in the near regions
//
//	//find the near regions contains enough points...;  from distance 0 to infinite?...
//	while (sum_near_points < K* k_search_times)
//	{
//		Find_Near_Regions(query_index, search_distance, near_regions, near_region_size, split_array_size);	//find the index of near regions with distance = near_distance;
//		//count the valid_regions and copy it to valid_near_regions
//		for (int i = 0; i < near_region_size; i++)
//		{
//#pragma HLS PIPELINE
//#pragma HLS loop_tripcount min=1 max=1000
//			if (sub_sets_size[near_regions[i]] > 0)
//			{
//				sum_near_points += sub_sets_size[near_regions[i]];
//				valid_near_regions[valid_near_region_size] = near_regions[i];
//				valid_near_region_size++;
//			}
//
//		}
//		//if the points are not enough for K , enlarge the distance
//		search_distance++;
//	}
//
//	//find K nearest neighbors in valid near sub-regions
//	//initialize the output array
////	for (int i = 0; i < K; i++)
////	{
////		nearest_distance[i] = 10000;
////		nearest_index[i] = 10000;
////	}
//
//	//For every valid sub-regions
//	for (int i = 0; i < valid_near_region_size; i++)
//	{
//		//for every points in i-th sub_region
//		//int test_k = sub_sets_size[valid_near_regions[i]];
//		for (int j = 0; j < sub_sets_size[valid_near_regions[i]]; j++)
//		{
//#pragma HLS PIPELINE
//#pragma HLS loop_tripcount min=1 max=1000
//			//calculate the distance between query and point j;
//			type_point distance = PCLEucDist(query_data, data_set[sub_sets[ valid_near_regions[i] ][j] ]);
//
//			//if distance is smaller than the biggest distance in result array,
//			//then store the distance and the index of the point to replace the old biggest one.
//			if (distance < nearest_distance[K - 1])	//the gate of step in the result array
//			{
//				nearest_distance[K - 1] = distance;
//				nearest_index[K - 1] = sub_sets[valid_near_regions[i]][j];		//the indx of this point
//				//Sort the result array
//				for (int k = K - 2; k >= 0; k--)
//				{
//					if (distance < nearest_distance[k])
//					{
//						nearest_distance[k + 1] = nearest_distance[k];
//						nearest_distance[k] = distance;
//						nearest_index[k + 1] = nearest_index[k];
//						nearest_index[k] = sub_sets[valid_near_regions[i]][j];
//					}
//					else break;	//if the new data is bigger than any data in the vector, the loop will be break.
//				}
//			}
//
//		}
//	}
//
//
//}

void Find_Near_Regions(int current_index, int search_distance, int near_regions[k_search_near_regions_max], int & near_region_size, struct SplitArraySize split_array_size)
{

	near_region_size = 0;		//init the near_region size to 0
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
			if(iz_abs >= 0)
			{
				char iz = iz_abs;
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

				char iz_minus = -iz_abs;
				int cal_index_minus = current_index + ix * split_array_size.y_array_size * split_array_size.z_array_size + iy * split_array_size.z_array_size + iz_minus;
				//make sure the index is in the array's range
				if ((cal_index_minus >= 0) && (cal_index_minus < (split_array_size.x_array_size *split_array_size.y_array_size * split_array_size.z_array_size)))
				{
					if(near_region_size < k_search_near_regions_max)
					{
						near_regions[near_region_size] = cal_index_minus;
						near_region_size++;
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
type_point PCLEucDist(struct ThreeDimPoint data1, struct ThreeDimPoint data2)
{
#pragma HLS INLINE
	return ((data1.x - data2.x)*(data1.x - data2.x)
		+ (data1.y - data2.y)*(data1.y - data2.y)
		+ (data1.z - data2.z)*(data1.z - data2.z)
	);
}

float my_cbrt(float value)
{
	float result;
	for(int i = 1; i < 10; i ++)
	{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=10
		int k = i * i * i;
		if( k > value)
		{
			result = i-1;
			break;
		}
	}

//	for(float i = 0; i < 1; i=i+0.2)
//	{
//#pragma HLS pipeline
//#pragma HLS loop_tripcount min=1 max=5
//		float precise_result = result + i;
//		int k = precise_result * precise_result * precise_result;
//		if( k > value)
//		{
//			result = precise_result;
//			break;
//		}
//	}
	return result;
}


void my_rand_generate(unsigned int *value)
{
#pragma HLS INLINE

	//LSFR 64位随机数生成
	unsigned int x;
	x = *value;	/* The state must be seeded with a nonzero value. */
//	x ^= x >> 12; // a
//	x ^= x << 25; // b
//	x ^= x >> 27; // c
	x ^= x >> 3; // a
	x ^= x << 5; // b
	x ^= x >> 7; // c
	*value = x;
}


//#pragma SDS data mem_attribute("data_set":NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
//#pragma SDS data access_pattern("data_set":SEQUENTIAL)
//#pragma SDS data copy(data_set[0:k_data_set_size])
void BuildGBDS(struct ThreeDimPoint data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, float split_precise)
{


	GetMaxMin(data_set, data_max_min_PL, data_set_size);
	if(split_precise == 0)
		SplitSubSpace(data_set_size, data_max_min_PL, split_precise, x_split_array_PL, y_split_array_PL, z_split_array_PL, split_array_size_PL);
	else
		SplitSubSpacePrecise(data_set_size, data_max_min_PL, split_precise, x_split_array_PL, y_split_array_PL, z_split_array_PL, split_array_size_PL);
	DataClassify(data_set_PL, data_set_size, x_split_array_PL, y_split_array_PL, z_split_array_PL, split_array_size_PL, sub_sets_PL, sub_sets_size_PL, rand_seed);

}


//#pragma SDS data mem_attribute("nearest_index":NON_CACHEABLE|PHYSICAL_CONTIGUOUS, "nearest_distance":NON_CACHEABLE|PHYSICAL_CONTIGUOUS)
//#pragma SDS data access_pattern("nearest_index":SEQUENTIAL, "nearest_distance":SEQUENTIAL)
//#pragma SDS data copy(nearest_index[0:k_nearest_number_max], nearest_distance[0:k_nearest_number_max])		// tell the compiler the size of data_set;
//void SearchKNNGBDS(int K, struct ThreeDimPoint query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max])
void SearchKNNGBDS()
{
//#pragma HLS data_pack variable=query_data struct_level
	int near_regions[k_search_near_regions_max];			//store the near_regions found by Find_Near_Regions() function
	int valid_near_regions[k_search_near_regions_max];		//the valid regions index contains points

#pragma HLS ARRAY_PARTITION variable=near_regions dim=1 cyclic factor=10
#pragma HLS ARRAY_PARTITION variable=valid_near_regions dim=1 cyclic factor=10

	int near_region_size = 0;								//size of near_regions.  or number of near_regions
	int valid_near_region_size = 0;							//size of valid_near_region.  or number of valid_near_region
	int sum_near_points = 0;										// sum of points found in the near regions
	int query_index = PCLCalculateIndex(query_data_PL, x_split_array_PL, y_split_array_PL, z_split_array_PL, split_array_size_PL);

	//find the near regions contains enough points...;  from distance 0 to infinite?...
	//while (sum_near_points < K* k_search_times)
	//consider pipeline the whole loop

	if(sub_sets_size_PL[query_index] > 0)
	{
		sum_near_points += sub_sets_size_PL[query_index];
		valid_near_regions[valid_near_region_size] = query_index;
		valid_near_region_size++;
	}


	//cost a lot of LUT.... can be modify..
//	if(sum_near_points < K_PL* k_search_times)
//	{
//loop_Find_nearest_regions:
//		for(int ix = -1; ix <= 1; ix++)
//		{
//#pragma HLS loop_tripcount min=3 max=3
//#pragma HLS pipeline
//			for(int iy = -1; iy <= 1; iy++)
//			{
//#pragma HLS loop_tripcount min=3 max=3
//				for(int iz = -1; iz <= 1; iz++)
//				{
//#pragma HLS loop_tripcount min=3 max=3
//					if(abs(ix) + abs(ix) + abs(ix) == 1)
//					{
//						int cal_index = query_index + ix * split_array_size_PL.y_array_size * split_array_size_PL.z_array_size + iy * split_array_size_PL.z_array_size + iz;
//						//make sure the index is in the array's range
//						if ( (cal_index != query_index) && (cal_index >= 0) && (cal_index < (split_array_size_PL.x_array_size *split_array_size_PL.y_array_size * split_array_size_PL.z_array_size)))
//						{
//							if(sub_sets_size_PL[cal_index] > 0)
//							{
//								sum_near_points += sub_sets_size_PL[cal_index];
//								valid_near_regions[valid_near_region_size] = cal_index;
//								valid_near_region_size++;
//							}
//
//						}
//					}
//				}
//			}
//		}
//	}
//
//	if(sum_near_points < K_PL* k_search_times)
//		{
//	loop_Find_second_regions:
//			for(int ix = -2; ix <= 2; ix++)
//			{
//	#pragma HLS loop_tripcount min=3 max=3
//	#pragma HLS pipeline
//				for(int iy = -2; iy <= 2; iy++)
//				{
//	#pragma HLS loop_tripcount min=3 max=3
//					for(int iz = -2; iz <= 2; iz++)
//					{
//	#pragma HLS loop_tripcount min=3 max=3
//						if(abs(ix) + abs(ix) + abs(ix) == 2)
//						{
//							int cal_index = query_index + ix * split_array_size_PL.y_array_size * split_array_size_PL.z_array_size + iy * split_array_size_PL.z_array_size + iz;
//							//make sure the index is in the array's range
//							if ( (cal_index != query_index) && (cal_index >= 0) && (cal_index < (split_array_size_PL.x_array_size *split_array_size_PL.y_array_size * split_array_size_PL.z_array_size)))
//							{
//								if(sub_sets_size_PL[cal_index] > 0)
//								{
//									sum_near_points += sub_sets_size_PL[cal_index];
//									valid_near_regions[valid_near_region_size] = cal_index;
//									valid_near_region_size++;
//								}
//
//							}
//						}
//					}
//				}
//			}
//		}


	if(sum_near_points < K_PL* k_search_times)
	{
		int search_distance = 1;
		for(int while_count = 0; while_count < 6; while_count ++)
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
					if(iz_abs >= 0)
					{
						char iz = iz_abs;
						int cal_index = query_index + ix * split_array_size_PL.y_array_size * split_array_size_PL.z_array_size + iy * split_array_size_PL.z_array_size + iz;
						//make sure the index is in the array's range
						if ((cal_index >= 0) && (cal_index < (split_array_size_PL.x_array_size *split_array_size_PL.y_array_size * split_array_size_PL.z_array_size)))
						{
							if(sub_sets_size_PL[cal_index] > 0)
							{
								sum_near_points += sub_sets_size_PL[cal_index];
								valid_near_regions[valid_near_region_size] = cal_index;
								valid_near_region_size++;
							}

						}

						char iz_minus = -iz_abs;
						int cal_index_minus = query_index + ix * split_array_size_PL.y_array_size * split_array_size_PL.z_array_size + iy * split_array_size_PL.z_array_size + iz_minus;
						//make sure the index is in the array's range
						if ((cal_index_minus >= 0) && (cal_index_minus < (split_array_size_PL.x_array_size *split_array_size_PL.y_array_size * split_array_size_PL.z_array_size)))
						{
							if(sub_sets_size_PL[cal_index_minus] > 0)
							{
								sum_near_points += sub_sets_size_PL[cal_index_minus];
								valid_near_regions[valid_near_region_size] = cal_index_minus;
								valid_near_region_size++;
							}

						}
					}

				}
			}
			if(sum_near_points >= K_PL* k_search_times)
				break;
			else
				search_distance++;
		}
	}


//	for(int while_count = 0; while_count < 20; while_count ++)
//	{
//#pragma HLS loop_tripcount min=1 max=20
////#pragma HLS pipeline
//		if(sum_near_points < K_PL* k_search_times)
//		{
//			Find_Near_Regions(query_index, search_distance, near_regions, near_region_size, split_array_size_PL);	//find the index of near regions with distance = near_distance;
//			//count the valid_regions and copy it to valid_near_regions
//loop_Find_sufficient_near_points:
//			for (int i = 0; i < near_region_size; i++)
//			{
//#pragma HLS pipeline
//#pragma HLS loop_tripcount min=1 max=100
//				if (sub_sets_size_PL[near_regions[i]] > 0)
//				{
//					sum_near_points += sub_sets_size_PL[near_regions[i]];
//					valid_near_regions[valid_near_region_size] = near_regions[i];
//					valid_near_region_size++;
//				}
//
//			}
//			//if the points are not enough for K_PL , enlarge the distance
//			search_distance++;
//		}
//		else break;
//
//	}

////////////////////////////////find K_PL nearest neighbors in valid near sub-regions
	//For every valid sub-regions
	type_point array_biggest_distance  = 10000;

loop_knn_result_region:
	for (int i = 0; i < valid_near_region_size; i++)
	{
#pragma HLS loop_tripcount min=1 max=100
		//for every points in i-th sub_region
		//int test_k = sub_sets_size_PL[valid_near_regions[i]];
loop_knn_result_points:
		for (int j = 0; j < sub_sets_size_PL[valid_near_regions[i]]; j++)
		{
#pragma HLS pipeline
#pragma HLS loop_tripcount min=1 max=20

			//calculate the distance between query and point j;
			int index = sub_sets_PL[ valid_near_regions[i] ][j];
			type_point distance = PCLEucDist(query_data_PL, data_set_PL[index]);

			//if distance is smaller than the biggest distance in result array,
			//then store the distance and the index of the point to replace the old biggest one.

loop_knn_result_flag:
			for(int k = 0; k < k_nearest_number_max; k++)
			{
#pragma HLS loop_tripcount min=1 max=10
#pragma HLS unroll

				if(distance < nearest_distance_PL[k])
					sort_kick_flag[k] = true;
				else
					sort_kick_flag[k] = false;
			}

loop_knn_result_shift:
			for(int k = k_nearest_number_max-1; k > 0; k--)
			{

#pragma HLS loop_tripcount min=1 max=9
#pragma HLS unroll
#pragma HLS DEPENDENCE variable=nearest_distance_PL intra WAR true
#pragma HLS dependence variable=nearest_index_PL intra WAR true
				if(sort_kick_flag[k-1] && sort_kick_flag[k])	//11
				{
					nearest_distance_PL[k] = nearest_distance_PL[k-1];
					nearest_index_PL[k] = nearest_index_PL[k-1];
				}
				if(!sort_kick_flag[k-1] && sort_kick_flag[k])	//**0111
				{
					nearest_distance_PL[k] = nearest_distance_PL[k-1];
					nearest_index_PL[k] = nearest_index_PL[k-1];
					nearest_distance_PL[k-1] = distance;
					nearest_index_PL[k-1] = index;
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
void GBDSIPCore(bool select_build_GBDS, My_Points data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, My_Points query_data, int nearest_index[k_nearest_number_max], type_point nearest_distance[k_nearest_number_max], float split_precise)
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

	if(select_build_GBDS)
	{
		//cache the data_set to PL
		data_set_size_PL = data_set_size;
		my_rand_number_seed_PL = rand_seed;
		split_precise_PL = split_precise;

//loop_cache_dataset:
//for (int i = 0; i < data_set_size_PL; i ++)
//		{
//	#pragma HLS pipeline II=1
//	#pragma HLS loop_tripcount min=1 max=100000
//			data_set_PL[i].x = data_set[i].x;
//			data_set_PL[i].y = data_set[i].y;
//			data_set_PL[i].z = data_set[i].z;
//		}

		//std::cout << "start" <<std::endl;
		BuildGBDS(data_set, data_set_size_PL, my_rand_number_seed_PL, split_precise_PL);
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
		SearchKNNGBDS();

		//sort the search result
//		for(int i = 0; i < K_PL; i ++)
//
//	#pragma HLS loop_tripcount min=1 max=10
//			for(int j = i+1; j < K_PL; j ++)
//			{
//				#pragma HLS PIPELINE
//				if(nearest_distance_PL[i] > nearest_distance_PL[j])
//				{
//					type_point  temp_distance = nearest_distance_PL[i];
//					int temp_index = nearest_index_PL[i];
//					nearest_index_PL[i] = nearest_index_PL[j];
//					nearest_distance_PL[i] = nearest_distance_PL[j];
//					nearest_index_PL[j] = temp_index;
//					nearest_distance_PL[j] = temp_distance;;
//				}
//			}
//		}

		// transmit the data to outside
loop_output:		for(int i = 0; i < K_PL; i ++)
		{
	#pragma HLS PIPELINE
	#pragma HLS loop_tripcount min=1 max=10
			nearest_index[i] = nearest_index_PL[i];
			nearest_distance[i] = nearest_distance_PL[i];
		}
	}
}



//void GBDSIPCore_queryset(bool select_build_GBDS, My_Points data_set[k_data_set_size], int data_set_size, unsigned int rand_seed, int K, My_Points query_data[k_query_set_size], int nearest_index[k_query_set_size][k_nearest_number_max], type_point nearest_distance[k_query_set_size][k_nearest_number_max], float split_precise)
//{
//}

