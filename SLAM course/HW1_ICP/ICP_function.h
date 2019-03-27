#pragma once
#include "time.h"
#include "Eigen\Dense"
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>	//pointcloud transform
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/voxel_grid.h>	//FILTER
#include <pcl/registration/icp.h>	//ICP
#include <pcl/kdtree/kdtree_flann.h> //kd_tree
#include <algorithm>	//FIND
#include<read_files.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

#define depth_picture_rows 480
#define depth_picture_cols 640

typedef struct {
	std::vector<float> distances;
	std::vector<int> indices;
} NEIGHBOR;

// icp iteration; return rotation and translation matrix
Eigen::Matrix4d ICP_iterate_p2p(PointCloud::Ptr source_cloud, PointCloud::Ptr target_corres);

//preprocessing;  find correspondance; remove the outliers.
//use kd_tree find the neaest neighbor
//input: source points cloud and target points cloud
//output: corresponces points cloud of source_points and target_points
void Correspondance_search(PointCloud::Ptr cloud_src, PointCloud::Ptr cloud_dst, PointCloud::Ptr source_corres, PointCloud::Ptr target_corres);


/*
// pre_process the picture data
input: depth_file name and depth_file indice;
output: points cloud after reading from depth file and pre_processing.
*/
void get_cloud(std::vector<std::string> & depth_name, int picture_ind, PointCloud::Ptr PointCloud);