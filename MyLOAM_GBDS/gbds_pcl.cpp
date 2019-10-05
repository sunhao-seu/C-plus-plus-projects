#include "gbds_pcl.h"


//template<int DATA_SIZE>
void MyPointCloudToArray(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_data_set, struct ThreeDimPoint data_set[], int & data_set_size)
{
	//ampty the array
	memset(data_set, 0, sizeof(struct ThreeDimPoint) * data_set_size);
	int count = 0;
	for (int i = 0; i < point_cloud_data_set->points.size(); i++)
	{
		// if point data is null, then cannot enter this loop
		//if ( (point_cloud_data_set->points[i].x == point_cloud_data_set->points[i].x) && (point_cloud_data_set->points[i].y == point_cloud_data_set->points[i].y) && (point_cloud_data_set->points[i].z == point_cloud_data_set->points[i].z) )
		if(point_cloud_data_set->points[i].x == point_cloud_data_set->points[i].x)
		{
			data_set[count].x = point_cloud_data_set->points[i].x;
			data_set[count].y = point_cloud_data_set->points[i].y;
			data_set[count].z = point_cloud_data_set->points[i].z;
			count++;
		}
	}
	data_set_size = count;		//the size of valid points
}

void MyPointXYZIToThreeDimPoint(pcl::PointXYZI point, struct ThreeDimPoint & point_array)
{
	point_array.x = point.x;
	point_array.y = point.y;
	point_array.z = point.z;
}



//template<int DATA_SIZE>
void GetMaxMin(struct ThreeDimPoint data_set[], struct MaxMin & data_max_min, const int data_set_size)
{
	int i;
	int data_size = data_set_size;
	type_point x_max, x_min, y_max, y_min, z_max, z_min;
	if (data_size % 2 == 0)
	{
		//compare x
		if (data_set[0].x>data_set[1].x)
		{
			x_max = data_set[0].x;
			x_min = data_set[1].x;
		}
		else
		{
			x_max = data_set[1].x;
			x_min = data_set[0].x;
		}

		//compare y
		if (data_set[0].y>data_set[1].y)
		{
			y_max = data_set[0].y;
			y_min = data_set[1].y;
		}
		else
		{
			y_max = data_set[1].y;
			y_min = data_set[0].y;
		}

		////compare z
		if (data_set[0].z>data_set[1].z)
		{
			z_max = data_set[0].z;
			z_min = data_set[1].z;
		}
		else
		{
			z_max = data_set[1].z;
			z_min = data_set[0].z;
		}


		for (i = 2; i<data_size - 1; i = i + 2)
		{
			//compare x
			if (data_set[i].x>data_set[i + 1].x)
			{
				if (x_max<data_set[i].x)
					x_max = data_set[i].x;
				if (x_min>data_set[i + 1].x)
					x_min = data_set[i + 1].x;
			}
			else
			{
				if (x_max<data_set[i + 1].x)
					x_max = data_set[i + 1].x;
				if (x_min>data_set[i].x)
					x_min = data_set[i].x;
			}

			//compare y
			if (data_set[i].y>data_set[i + 1].y)
			{
				if (y_max<data_set[i].y)
					y_max = data_set[i].y;
				if (y_min>data_set[i + 1].y)
					y_min = data_set[i + 1].y;
			}
			else
			{
				if (y_max<data_set[i + 1].y)
					y_max = data_set[i + 1].y;
				if (y_min>data_set[i].y)
					y_min = data_set[i].y;
			}


			//compare z
			if (data_set[i].z>data_set[i + 1].z)
			{
				if (z_max<data_set[i].z)
					z_max = data_set[i].z;
				if (z_min>data_set[i + 1].z)
					z_min = data_set[i + 1].z;
			}
			else
			{
				if (z_max<data_set[i + 1].z)
					z_max = data_set[i + 1].z;
				if (z_min>data_set[i].z)
					z_min = data_set[i].z;
			}

		}
	}
	else
	{
		x_max = x_min = data_set[0].x;
		y_max = y_min = data_set[0].y;
		z_max = z_min = data_set[0].z;

		for (i = 1; i<data_size - 1; i = i + 2)
		{
			//compare x
			if (data_set[i].x>data_set[i + 1].x)
			{
				if (x_max<data_set[i].x)
					x_max = data_set[i].x;
				if (x_min>data_set[i + 1].x)
					x_min = data_set[i + 1].x;
			}
			else
			{

				if (x_max<data_set[i + 1].x)
					x_max = data_set[i + 1].x;
				if (x_min>data_set[i].x)
					x_min = data_set[i].x;
			}

			//compare y
			if (data_set[i].y>data_set[i + 1].y)
			{
				if (y_max<data_set[i].y)
					y_max = data_set[i].y;
				if (y_min>data_set[i + 1].y)
					y_min = data_set[i + 1].y;
			}
			else
			{

				if (y_max<data_set[i + 1].y)
					y_max = data_set[i + 1].y;
				if (y_min>data_set[i].y)
					y_min = data_set[i].y;
			}


			//compare z
			if (data_set[i].z>data_set[i + 1].z)
			{
				if (z_max<data_set[i].z)
					z_max = data_set[i].z;
				if (z_min>data_set[i + 1].z)
					z_min = data_set[i + 1].z;
			}
			else
			{

				if (z_max<data_set[i + 1].z)
					z_max = data_set[i + 1].z;
				if (z_min>data_set[i].z)
					z_min = data_set[i].z;
			}


		}
	}
	//store these min max in a structure for convenient usage
	data_max_min.xmin = x_min;
	data_max_min.xmax = x_max;
	data_max_min.ymin = y_min;
	data_max_min.ymax = y_max;
	data_max_min.zmin = z_min;
	data_max_min.zmax = z_max;
}

void SplitSubSpace(const int data_set_size, const struct MaxMin data_max_min, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize & split_array_size)
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

	int max_split_size = k_axis_spaces_max;	//defaut maximum splited spaces in a axis
	//calculate the split size of the max range on x,y,z
	for (int i = 0; i < k_axis_spaces_max; i++)
	{
		float total_sub_spaces = (x_times_max*i) * (y_times_max*i)*(z_times_max*i);
		if (total_sub_spaces > calculated_space_size)
		{
			max_split_size = i;
			break;
		}
	}

	//calculate the split size of x,y,z axis
	int x_split_size = ceil(max_split_size * x_times_max);
	int y_split_size = ceil(max_split_size * y_times_max);
	int z_split_size = ceil(max_split_size * z_times_max);

	type_point x_unit = x_range / ( ceil(max_split_size * x_times_max) );
	type_point y_unit = y_range / (ceil(max_split_size * y_times_max));
	type_point z_unit = z_range / (ceil(max_split_size * z_times_max));

	//when know the split unit and range, we can calculate the each split point and store it in a array.
	for (int i = 0; i <= x_split_size; i++)
	{
		x_split_array[i] = data_max_min.xmin + i * x_unit;
	}
	for (int i = 0; i <= y_split_size; i++)
	{
		y_split_array[i] = data_max_min.ymin + i * y_unit;
	}
	for (int i = 0; i <= z_split_size; i++)
	{
		z_split_array[i] = data_max_min.zmin + i * z_unit;
	}

	//the split array range [min,max];(contain the min and max)
	split_array_size.x_array_size = x_split_size + 1;
	split_array_size.y_array_size = y_split_size + 1;
	split_array_size.z_array_size = z_split_size + 1;

}

void SplitSubSpacePrecise(const int data_set_size, const struct MaxMin data_max_min, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize & split_array_size)
{
	int x_split_size = (data_max_min.xmax - data_max_min.xmin) / k_split_precise;
	int y_split_size = (data_max_min.ymax - data_max_min.ymin) / k_split_precise;
	int z_split_size = (data_max_min.zmax - data_max_min.zmin) / k_split_precise;
	int total_sub_spaces = (x_split_size * y_split_size * z_split_size);

	//ensure the number of sub-spaces won't exceed the k_sub_region_max
	if (total_sub_spaces > k_sub_region_max)
	{
		int split_down_size = pow((k_sub_region_max / total_sub_spaces),1.0/3) +1;
		x_split_size = x_split_size / split_down_size;
		y_split_size = y_split_size / split_down_size;
		z_split_size = z_split_size / split_down_size;
	}

	//when know the split unit and range, we can calculate the each split point and store it in a array.
	for (int i = 0; i <= x_split_size; i++)
	{
		x_split_array[i] = data_max_min.xmin + i * k_split_precise;
	}
	for (int i = 0; i <= y_split_size; i++)
	{
		y_split_array[i] = data_max_min.ymin + i * k_split_precise;
	}
	for (int i = 0; i <= z_split_size; i++)
	{
		z_split_array[i] = data_max_min.zmin + i * k_split_precise;
	}

	//the split array range [min,max];(contain the min and max)
	split_array_size.x_array_size = x_split_size + 1;
	split_array_size.y_array_size = y_split_size + 1;
	split_array_size.z_array_size = z_split_size + 1;
}

//template<int DATA_SIZE, int SUBSETS_NUM, int SUBSPACE_NUM>
void DataClassify(struct ThreeDimPoint data_set[], int data_set_size, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size, int sub_sets[][k_sub_space_array_size], int sub_sets_size[])
{
	memset(sub_sets, 0, sizeof(int) * k_sub_space_array_size * k_sub_region_max);
	memset(sub_sets_size, 0, sizeof(int) * k_sub_region_max);
	srand((unsigned)time(NULL));

	for (int i = 0; i < data_set_size; i++)
	{
		int data_index = PCLCalculateIndex(data_set[i], x_split_array, y_split_array, z_split_array, split_array_size);
		int index_size = sub_sets_size[data_index] + 1;		//此数据存放的下标

		sub_sets_size[data_index] = sub_sets_size[data_index] + 1;

		//以下操作确保数据不会超过给定空间的范围，同时还保证了一定的均匀采点
		if (index_size < 1.0/3*k_sub_space_array_size)		//存储数据在0到1/3*k_sub_space_array_size之间
			sub_sets[data_index][sub_sets_size[data_index]] = i;
		else if (index_size < 2.0 / 3 * k_sub_space_array_size)	//存储数据在1/3*k_sub_space_array_size到2/3*k_sub_space_array_size之间
		{
			//三分之一的概率存起来
			int rand_number = rand() % 10;
			if (rand_number < 3)
				sub_sets[data_index][sub_sets_size[data_index]] = i;
			else
				sub_sets_size[data_index] = sub_sets_size[data_index] - 1;
		}
			
		else if (index_size < 4.0 / 5 * k_sub_space_array_size)	//存储数据在2/3*k_sub_space_array_size到4/5*k_sub_space_array_size之间
		{
			//五分之一的概率存起来
			int rand_number = rand() % 10;
			if (rand_number < 2)
				sub_sets[data_index][sub_sets_size[data_index]] = i;
			else
				sub_sets_size[data_index] = sub_sets_size[data_index] - 1;
		}
		else if(index_size < k_sub_space_array_size)	//存储数据在4/5*k_sub_space_array_size到k_sub_space_array_size之间
		{
			//10分之一的概率存起来
			int rand_number = rand() % 10;
			if (rand_number < 1)
				sub_sets[data_index][sub_sets_size[data_index]] = i;
			else
				sub_sets_size[data_index] = sub_sets_size[data_index] - 1;
		}
		else sub_sets_size[data_index]=sub_sets_size[data_index] - 1;

	}
}


int PCLCalculateIndex(struct ThreeDimPoint point_data, type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size)
{
	//copy the point_data to data_x,y,z
	float data_x = point_data.x;
	float data_y = point_data.y;
	float data_z = point_data.z;
	//default x,y,z index as the max index
	int x_index = split_array_size.x_array_size-1;
	int y_index = split_array_size.y_array_size-1;
	int z_index = split_array_size.z_array_size-1;
	//similar to look-up-table
	for (int jx = 0; jx < split_array_size.x_array_size; jx++)
	{
		if (data_x <= x_split_array[jx])
		{
			x_index = jx;
			break;
		}

	}
	for (int jy = 0; jy < split_array_size.y_array_size; jy++)
	{
		if (data_y <= y_split_array[jy])
		{
			y_index = jy;
			break;
		}
	}
	for (int jz = 0; jz < split_array_size.z_array_size; jz++)
	{
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


void SearchKNearestNeighbors(int K, struct ThreeDimPoint query_data, struct ThreeDimPoint data_set[], int sub_sets[][k_sub_space_array_size], int sub_sets_size[], type_point x_split_array[], type_point y_split_array[], type_point z_split_array[], struct SplitArraySize split_array_size, int nearest_index[], type_point nearest_distance[])
{
	memset(nearest_index, 0, sizeof(int) * k_query_set_size);
	memset(nearest_distance, 0, sizeof(int) * k_query_set_size);


	int query_index = PCLCalculateIndex(query_data, x_split_array, y_split_array, z_split_array, split_array_size);
	int search_distance = 0;								//distance of near_region and current region
	int near_regions[k_search_near_regions_max];			//store the near_regions finded by Find_Near_Regions() function
	int valid_near_regions[k_search_near_regions_max];		//the valid regions index contains points
	int near_region_size = 0;								//size of near_regions.  or number of near_regions
	int valid_near_region_size = 0;							//size of valid_near_region.  or number of valid_near_region
	int sum_points = 0;										// sum of points found in the near regions

	//find the near regions contains enough points...;  from distance 0 to infinite?...
	while (sum_points < K* k_search_times)
	{
		Find_Near_Regions(query_index, search_distance, near_regions, near_region_size, split_array_size);	//find the index of near regions with distance = near_distance;
		//count the valid_regions and copy it to valid_near_regions
		for (int i = 0; i < near_region_size; i++)
		{
			if (sub_sets_size[near_regions[i]] > 0)
			{
				sum_points += sub_sets_size[near_regions[i]];
				valid_near_regions[valid_near_region_size] = near_regions[i];
				valid_near_region_size++;
			}
			
		}
		//if the points are not enough for K , enlarge the distance
		search_distance++;
	}

	//find K nearest neighbors in valid near sub-regions
	//initialize the output array
	for (int i = 0; i < K; i++)
	{
		nearest_distance[i] = 10000;
		nearest_index[i] = 10000;
	}

	//For every valid sub-regions
	for (int i = 0; i < valid_near_region_size; i++)
	{
		//for every points in i-th sub_region
		int test_k = sub_sets_size[valid_near_regions[i]];
		for (int j = 0; j < sub_sets_size[valid_near_regions[i]]; j++)
		{
			//calculate the distance between query and point j;
			int testa = valid_near_regions[i];
			int testb = sub_sets[valid_near_regions[i]][j];
			struct ThreeDimPoint testc = data_set[sub_sets[valid_near_regions[i]][j]];
			type_point distance = PCLEucDist(query_data, data_set[sub_sets[ valid_near_regions[i] ][j] ]);
			/*type_point distance2 = sqrt((query_data.x - data_set[sub_sets[valid_near_regions[i]][j]].x)*(query_data.x - data_set[sub_sets[valid_near_regions[i]][j]].x)
				+ (query_data.y - data_set[sub_sets[valid_near_regions[i]][j]].y)*(query_data.y - data_set[sub_sets[valid_near_regions[i]][j]].y)
				+ (query_data.z - data_set[sub_sets[valid_near_regions[i]][j]].z)*(query_data.z - data_set[sub_sets[valid_near_regions[i]][j]].z)
			);*/

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

void Find_Near_Regions(int current_index, int search_distance, int near_regions[], int & near_region_size, struct SplitArraySize split_array_size)
{
	near_region_size = 0;		//init the near_region size to 0
	for (int ix = -search_distance; ix <= search_distance; ix++)
	{
		for (int iy = -search_distance; iy <= search_distance; iy++)
		{
			for (int iz = -search_distance; iz <= search_distance; iz++)
			{
				// only consider the regions with certain(search_distance) distance
				if ((abs(ix) + abs(iy) + abs(iz)) == search_distance)
				{
					int cal_index = current_index + ix * split_array_size.y_array_size * split_array_size.z_array_size + iy * split_array_size.z_array_size + iz;
					//make sure the index is in the array's range
					if ((cal_index >= 0) && (cal_index < split_array_size.x_array_size *split_array_size.y_array_size * split_array_size.z_array_size))
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
type_point PCLEucDist(struct ThreeDimPoint data1, struct ThreeDimPoint data2)
{
	return ((data1.x - data2.x)*(data1.x - data2.x)
		+ (data1.y - data2.y)*(data1.y - data2.y)
		+ (data1.z - data2.z)*(data1.z - data2.z)
	);
}