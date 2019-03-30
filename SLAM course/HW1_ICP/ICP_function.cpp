#include<ICP_function.h>


/*
// pre_process the picture data
input: depth_file name and depth_file indice;
output: points cloud after reading from depth file and pre_processing.
*/
void get_cloud(std::vector<std::string> & depth_name, int picture_ind, PointCloud::Ptr PointCloud)
{
	
	Eigen::Matrix3f Camera_K;
	Eigen::Matrix3f Camera_K_inv;
	Eigen::Vector3f Global_xyz;
	Eigen::Vector3f Camera_xyz;
	Camera_K << 525, 0, 319.5, 0, 525, 239.5, 0, 0, 1;
	//std::cout << Camera_K << std::endl;
	Camera_K_inv = Camera_K.inverse();
	Eigen::ArrayXXf depth_data;
	PointT p;	//store the global pose for a moment

	// read depth file and store the data in Eigen::ArrayXXf depth_data;
	read_depth_file(depth_name[picture_ind], depth_data);	//read depth.png files  get x

	//transform image coordinate to global coordinate, 
	//and store the two pointclouds in pointcloud1 and pointcloud2
	for (int i = 0; i<depth_picture_rows; i++)
	{
		for (int j = 0; j < depth_picture_cols; j++)
		{
			if (depth_data(i, j) != -1)
			{
				Camera_xyz << i, j, 1;
				Global_xyz = depth_data(i, j)*(Camera_K_inv * Camera_xyz);
				p.x = Global_xyz(0);
				p.y = Global_xyz(1);
				p.z = Global_xyz(2);
				PointCloud->points.push_back(p);
			}
		}
	}

	//downsample by voxel filter; set 0.01f, downSample ten times
	pcl::VoxelGrid<PointT> filter;
	filter.setInputCloud(PointCloud);
	filter.setLeafSize(0.01f, 0.01f, 0.01f);	//SET the unit voxel to 0.1m*0.1m*0.1m
	filter.filter(*PointCloud);
}


/*
input:	source_corres:for source frame points cloud after nearest search correspondance to target frame.
		target_corres:for next(target) frame points cloud after correspondance
output: homogenous coordinate transform from cloud1 to cloud2. Matrix4d4*4		
*/
Eigen::Matrix4d ICP_iterate_p2p(PointCloud::Ptr source_corres, PointCloud::Ptr target_corres)
{
	//step1: calculate center of the points
	//it seems that N1 has no usage...
	int N = source_corres->size();
	Eigen::Vector3d p1_sum = Eigen::Vector3d::Zero();
	Eigen::Vector3d p2_sum = Eigen::Vector3d::Zero();
	for (int i = 0; i < N; i++)
	{
		p1_sum[0] += source_corres->points[i].x;
		p1_sum[1] += source_corres->points[i].y;
		p1_sum[2] += source_corres->points[i].z;
	}
	for (int i = 0; i < N; i++)
	{
		p2_sum[0] += target_corres->points[i].x;
		p2_sum[1] += target_corres->points[i].y;
		p2_sum[2] += target_corres->points[i].z;
	}


	p1_sum[0] = p1_sum[0] / N;
	p1_sum[1] = p1_sum[1] / N;
	p1_sum[2] = p1_sum[2] / N;
	p2_sum[0] = p2_sum[0] / N;
	p2_sum[1] = p2_sum[1] / N;
	p2_sum[2] = p2_sum[2] / N;

	//step2: remove the center
	std::vector<PointT> Points_rmcen1(N), Points_rmcen2(N);
	for (int i = 0; i < N; i++)
	{
		Points_rmcen1[i].x = source_corres->points[i].x - p1_sum[0];
		Points_rmcen1[i].y = source_corres->points[i].y - p1_sum[1];
		Points_rmcen1[i].z = source_corres->points[i].z - p1_sum[2];
	}
	for (int i = 0; i < N; i++)
	{
		Points_rmcen2[i].x = target_corres->points[i].x - p2_sum[0];
		Points_rmcen2[i].y = target_corres->points[i].y - p2_sum[1];
		Points_rmcen2[i].z = target_corres->points[i].z - p2_sum[2];
	}

	//q=R*p+T
	//step3: calculate Points_rmcen1*Points_rmcen2^   p*q^(T)  
	Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; i++)
	{
		H += Eigen::Vector3d(Points_rmcen1[i].x, Points_rmcen1[i].y, Points_rmcen1[i].z)*
			Eigen::Vector3d(Points_rmcen2[i].x, Points_rmcen2[i].y, Points_rmcen2[i].z).transpose();
	}
	//std::cout << "H " << H << std::endl;

	//STEP4: SVD on H;  H=U*S*V'
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	Eigen::Matrix3d Vt = svd.matrixV().transpose();
	Eigen::Vector3d sigma = svd.singularValues();
	//std::cout << "U= " << U << std::endl;
	//std::cout << "V= " << V << std::endl;

	Eigen::Matrix3d ICP_R = V * (U.transpose());
	if (ICP_R.determinant() < 0)
	{
		std::cout << "SVD fails" << std::endl;
		Vt.block<1, 3>(2, 0) *= -1;
		ICP_R = Vt.transpose()*U.transpose();
	}
	Eigen::Vector3d ICP_T = p2_sum - ICP_R * p1_sum;

	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> ICP_transformation;
	//Eigen::Matrix4d ICP_transformation = Eigen::Matrix4d::Identity();
	//ICP_transformation = Eigen::Matrix4d::Identity();
	// column master!!!!
	ICP_transformation << ICP_R(0, 0), ICP_R(0, 1), ICP_R(0, 2), ICP_T[0],
		ICP_R(1, 0), ICP_R(1, 1), ICP_R(1, 2), ICP_T[1],
		ICP_R(2, 1), ICP_R(2, 1), ICP_R(2, 2), ICP_T[2],
		0, 0, 0, 1;
	return ICP_transformation;
}

void Correspondance_search(PointCloud::Ptr cloud_src, PointCloud::Ptr cloud_dst, PointCloud::Ptr source_corres, PointCloud::Ptr target_corres)
{
	clock_t time_start, time_finish;
	double  time_duration;
	time_start = clock();

	pcl::KdTreeFLANN<PointT> kdtreeICP;			//???why ptr will abort??
	kdtreeICP.setInputCloud(cloud_dst);	//build kd_tree
	PointT pointSel;
	int k = 1;	//searching k nearest neighbors
	std::vector<int> pointSearchInd(k);
	std::vector<float> pointSearchSqDis(k);
	NEIGHBOR dst_nearest_neighbor;	//cloud1中与cloud2里面最近的点； 基于cloud1建立的kd_tree
	float dis_threshold = 0.1;
	int corres_count = 0;

	for (int i = 0; i < cloud_src->points.size(); i++)
	{
		bool store_flag = 1;
		pointSel = cloud_src->points[i];
		kdtreeICP.nearestKSearch(pointSel, k, pointSearchInd, pointSearchSqDis);	//在kd树种查找pointSel的最邻近点
		if (pointSearchSqDis[0] <= dis_threshold)	//compare the correspondance point distance with threshold.
		{
			for (int j = 0; j < dst_nearest_neighbor.indices.size(); j++)
			{
				if (dst_nearest_neighbor.indices[j] == pointSearchInd[0])
				{
					store_flag = 0;
					// if new corresponce distance are bigger than old, break without store;
					if (pointSearchSqDis[0] >= dst_nearest_neighbor.distances[j])
					{
						break;
					}
					//else, erase the old and push_back the new
					else
					{
						//erase the old
						auto iter1 = dst_nearest_neighbor.indices.erase(std::begin(dst_nearest_neighbor.indices) + j); //Delete the j-th element
						auto iter2 = dst_nearest_neighbor.distances.erase(std::begin(dst_nearest_neighbor.distances) + j); //Delete the j-th element
						auto iter3 = source_corres->points.erase(std::begin(source_corres->points) + j); //Delete the j-th element
						auto iter4 = target_corres->points.erase(std::begin(target_corres->points) + j); //Delete the j-th element
						//store the new
						source_corres->points.push_back(pointSel);
						target_corres->points.push_back(cloud_dst->points[pointSearchInd[0]]);
						corres_count++;
						dst_nearest_neighbor.distances.push_back(pointSearchSqDis[0]);
						dst_nearest_neighbor.indices.push_back(pointSearchInd[0]);
						break;
					}
				}
					
			}
			if (store_flag == 1)
			{
				//store the new
				source_corres->points.push_back(pointSel);
				target_corres->points.push_back(cloud_dst->points[pointSearchInd[0]]);
				corres_count++;
				dst_nearest_neighbor.distances.push_back(pointSearchSqDis[0]);
				dst_nearest_neighbor.indices.push_back(pointSearchInd[0]);
			}
		}
		
	}
	time_finish = clock();
	time_duration = (double)(time_finish - time_start) / CLOCKS_PER_SEC;
	std::cout << "Correspondance_search time " << time_duration << std::endl;
	// remove these points related to one point
}

/*
Get cloud surface normals
*/
void Get_Cloud_Normal(PointCloud::Ptr cloud_src, pcl::PointCloud<pcl::Normal>::Ptr Normals)
{
	//normal test
	clock_t time_start, time_finish;
	double  time_duration;
	time_start = clock();
	// clear the points in normals
	Normals->clear();
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	//建立kdtree来进行近邻点集搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//为kdtree添加点运数据
	//tree->setInputCloud(cloud_src);
	n.setInputCloud(cloud_src);
	n.setSearchMethod(tree);
	//点云法向计算时，需要所搜的近邻点大小
	n.setKSearch(10);
	//开始进行法向计算
	n.compute(*Normals);
	time_finish = clock();
	time_duration = (double)(time_finish - time_start) / CLOCKS_PER_SEC;
	std::cout << "normals time " << time_duration << std::endl;
}

/*
P2P Absolute error between icp cloud and target cloud
*/
double Get_P2P_Absolute_error(PointCloud::Ptr cloud_icp, PointCloud::Ptr cloud_target)
{
	double E_icp_absolute = 0.0;
	//double E_icp1 = 0.0;
	for (int i = 0; i < cloud_icp->size(); i++)
	{
		//absolute error
		E_icp_absolute += ((cloud_target->points[i].x - cloud_icp->points[i].x)*(cloud_target->points[i].x - cloud_icp->points[i].x)
			+ (cloud_target->points[i].y - cloud_icp->points[i].y)*(cloud_target->points[i].y - cloud_icp->points[i].y)
			+ (cloud_target->points[i].z - cloud_icp->points[i].z)*(cloud_target->points[i].z - cloud_icp->points[i].z));
	}
	E_icp_absolute = E_icp_absolute / cloud_icp->size();
	return E_icp_absolute;
}

/*
P2P Relative error between icp cloud and target cloud
*/
double Get_P2P_Relative_error(PointCloud::Ptr cloud_src, PointCloud::Ptr cloud_icp)
{
	double E_icp_realative = 0.0;
	for (int i = 0; i < cloud_icp->size(); i++)
	{
		//E_icp1 += ((Points_rmcen1[i].x)*(Points_rmcen1[i].x) + (Points_rmcen1[i].y)*(Points_rmcen1[i].y) + (Points_rmcen1[i].z)*(Points_rmcen1[i].z));
		E_icp_realative += ((cloud_src->points[i].x - cloud_icp->points[i].x)*(cloud_src->points[i].x - cloud_icp->points[i].x)
			+ (cloud_src->points[i].y - cloud_icp->points[i].y)*(cloud_src->points[i].y - cloud_icp->points[i].y)
			+ (cloud_src->points[i].z - cloud_icp->points[i].z)*(cloud_src->points[i].z - cloud_icp->points[i].z));
	}
	E_icp_realative = E_icp_realative / cloud_icp->size();
	return E_icp_realative;
}

/*
P2L Relative error between icp cloud and target cloud
*/
double Get_P2L_Absolute_error(PointCloud::Ptr cloud_icp, PointCloud::Ptr cloud_target, pcl::PointCloud<pcl::Normal>::Ptr Normals)
{
	double E_P2L_absolute = 0.0;
	//double E_icp1 = 0.0;
	for (int i = 0; i < cloud_icp->size(); i++)
	{
		//absolute error
		E_P2L_absolute += ((cloud_target->points[i].x - cloud_icp->points[i].x)*Normals->points[i].normal_x
			+ (cloud_target->points[i].y - cloud_icp->points[i].y)*Normals->points[i].normal_y
			+ (cloud_target->points[i].z - cloud_icp->points[i].z)*Normals->points[i].normal_z)
			*((cloud_target->points[i].x - cloud_icp->points[i].x)*Normals->points[i].normal_x
				+ (cloud_target->points[i].y - cloud_icp->points[i].y)*Normals->points[i].normal_y
				+ (cloud_target->points[i].z - cloud_icp->points[i].z)*Normals->points[i].normal_z);
	}
	E_P2L_absolute = E_P2L_absolute / cloud_icp->size();
	return E_P2L_absolute;
}

Eigen::Matrix4d ICP_iterate_P2L(PointCloud::Ptr source_cloud, PointCloud::Ptr target_corres, pcl::PointCloud<pcl::Normal>::Ptr Normals)
{

	int pts_num = source_cloud->size();
	Eigen::MatrixXd A(pts_num, 6);
	Eigen::MatrixXd b(pts_num, 1);
	Eigen::MatrixXd x_opt;

	//build A and b
	for (int i = 0; i<pts_num; i++) 
	{
		A(i,0) = Normals->points[i].normal_z*source_cloud->points[i].y - Normals->points[i].normal_y*source_cloud->points[i].z;
		A(i,1) = Normals->points[i].normal_x*source_cloud->points[i].z - Normals->points[i].normal_z*source_cloud->points[i].x;
		A(i,2) = Normals->points[i].normal_y*source_cloud->points[i].x - Normals->points[i].normal_x*source_cloud->points[i].y;
		A(i,3) = Normals->points[i].normal_x;
		A(i,4) = Normals->points[i].normal_y;
		A(i,5) = Normals->points[i].normal_z;


		b(i) = ((target_corres->points[i].x - source_cloud->points[i].x)*Normals->points[i].normal_x
			+ (target_corres->points[i].y - source_cloud->points[i].y)*Normals->points[i].normal_y
			+ (target_corres->points[i].z - source_cloud->points[i].z)*Normals->points[i].normal_z);
	}
	//std::cout << "A " << A << "  b  " << b << std::endl;
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
	//Eigen::MatrixXd U = svd.matrixU();
	//Eigen::MatrixXd V = svd.matrixV();
	//Eigen::MatrixXd Vt = svd.matrixV().transpose();
	//Eigen::MatrixXd sigma = svd.singularValues();
	//Eigen::MatrixXd sigma_plus;
	//for (int i = 0; i < 3; i++)
	//{
	//	sigma_plus(i,i) = (sigma(i,i) = 0) ? 0 : (1 / sigma(i,i));
	//}
	//Eigen::MatrixXd A_plus(6, pts_num);
	//A_plus = V * sigma_plus*(U.transpose());
	////x_opt=(alpha,beta,gama,t_x,t_y,t_z)
	//x_opt = A_plus * b;


	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
	//double  pinvtoler = 1.e-8; //tolerance
	//int row = A.rows();
	//int col = A.cols();
	//int k = 6;
	//Eigen::MatrixXd A_plus = Eigen::MatrixXd::Zero(col, row);
	//Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
	//Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
	//for (long i = 0; i<k; ++i) {
	//	if (singularValues_inv(i) > pinvtoler)
	//		singularValues_inv(i) = 1.0 / singularValues_inv(i);
	//	else singularValues_inv(i) = 0;
	//}
	//for (long i = 0; i < k; ++i)
	//{
	//	singularValues_inv_mat(i, i) = singularValues_inv(i);
	//}
	//A_plus = (svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*
	//x_opt = A_plus * b;

	// solve x
	x_opt = ((A.transpose()*A).inverse())*(A.transpose())*b;

	//get the R and T;
	Eigen::Matrix3d R;
	double sin_a = sin(x_opt(0));
	double cos_a = cos(x_opt(0));
	double sin_b = sin(x_opt(1));
	double cos_b = cos(x_opt(1));
	double sin_y = sin(x_opt(2));
	double cos_y = cos(x_opt(2));
	R(0, 0) = cos_y * cos_b;
	R(0, 1) = -sin_y * cos_a + cos_y * sin_b*sin_a;
	R(0, 2) = sin_y * sin_a + cos_y * sin_b*cos_a;
	R(1, 0) = sin_y * cos_b;
	R(1, 1) = cos_y * cos_a + sin_y * sin_b*sin_a;
	R(1, 2) = -cos_y * sin_a + sin_y * sin_b*cos_a;
	R(2, 0) = -sin_b;
	R(2, 1) = cos_b * sin_a;
	R(2, 2) = cos_b * cos_a;

	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> ICP_transformation;
	//Eigen::Matrix4d ICP_transformation = Eigen::Matrix4d::Identity();
	//ICP_transformation = Eigen::Matrix4d::Identity();
	// column master!!!!
	ICP_transformation << R(0, 0), R(0, 1), R(0, 2), x_opt(3),
		R(1, 0), R(1, 1), R(1, 2), x_opt(4),
		R(2, 0), R(2, 1), R(2, 2), x_opt(5),
		0, 0, 0, 1;
	return ICP_transformation;

}
