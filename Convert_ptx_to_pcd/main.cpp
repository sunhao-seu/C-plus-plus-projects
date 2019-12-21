/*
2019/12/21
author: sunhao
email: sunhao@shanghaitech.edu.cn

Tips: This project is based on PCL library and Eigen library.

Function: Convert a set of 3D point cloud .ptx to .pcd files.

1: Search the input directory and it's sub_directory, get all the files name and deepest directory name
2: For each deepest directory, compare and obain the files belong to it, transform the .ptx to pcd, merge all the .ptx in a directory to one .pcd file
3: Visualizaton function..easy..
*/

// Eigen part
#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//read files
#include <io.h>
#include <string>
#include <vector>
#include<algorithm>

bool new_dir = true;	//to avoid to store the directory's name repeatly.

//find the deepest directories under the input parameter "path", store the result in directory_names
//find name of all files with "format".., store the result in files_names
void findfiles(std::string path, std::string format, std::vector<std::string>& files_names, std::vector<std::string>& directory_names)
{
	_finddata_t file;
	intptr_t file_handle;
	std::string Onepath = path + format;
	file_handle = _findfirst(Onepath.c_str(), &file);
	
	if (file_handle == -1)
	{
		cout << "can not match the folder path" << endl;
		system("pause");
	}
	do {
		//judge thether has sub directory
		if (file.attrib & _A_SUBDIR)
		{
			//.:current directory,,,,    ..:last directory
			if ((strcmp(file.name, ".") != 0) && (strcmp(file.name, "..") != 0))
			{
				std::string newPath = path + "\\" + file.name;
				new_dir = true;
				findfiles(newPath, format, files_names, directory_names);
				
			}
		}
		else
		{
			cout << path + "\\" + file.name << " " << endl;
			files_names.push_back(path + "\\" + file.name);
			if (new_dir)
			{
				directory_names.push_back(path);
				new_dir = false;
			}
		}
	} while (_findnext(file_handle, &file) == 0);
	_findclose(file_handle);

	return;
}

void Read_PCD_colored_Show(std::string filename)
{
	/*Show colored point cloud*/
	pcl::PointCloud<pcl::PointXYZI>::Ptr Mapcloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile(filename, *Mapcloud);
	//VOCEL FILTER
	//pcl::VoxelGrid<PointT> TESTFilterMap;
	//TESTFilterMap.setLeafSize(0.4, 0.4, 0.4);		// set the grid spacing; unit grid is 0.2
	//TESTFilterMap.setInputCloud(Mapcloud);
	//TESTFilterMap.filter(*Mapcloud);
	pcl::visualization::CloudViewer TestViewer("file viewer");
	TestViewer.showCloud(Mapcloud);
	while (!TestViewer.wasStopped())
	{
	}
}


using std::ios;
using std::ifstream;



int main(int argc, char** argv)
{
	std::string input_path;
	if (argc == 2)
	{
		input_path = argv[1];
		std::cout << "the input path is: " << input_path << std::endl;
	}
	else
	{
		std::cout << "error number of arguments,,,," << (argc - 1) << "You ought to give only 1 arguments indicating the search path!" << std::endl; 
		return -1;
	}


	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
	unsigned int rows_number;
	unsigned int cols_number;
	Eigen::Matrix3d scan_rotation_matrix;
	Eigen::Matrix<double, 1, 3> scan_translation_matrix;
	//Eigen::Matrix4d scan_transform_matrix;
	Eigen::Matrix<double, 4, 3> scan_transform_matrix;
	double temp;

	std::string search_path = input_path;		//the path of the outer directory..
	std::string search_format = "\\*.*";
	std::string ptx_format = ".ptx";
	int ptx_format_length = ptx_format.length();

	std::vector<std::string> files_names;
	std::vector<std::string> directory_names;
	findfiles(search_path, search_format, files_names, directory_names);
	std::cout << "files size: " << files_names.size() << std::endl;
	std::cout << "directory_names size: " << directory_names.size() << std::endl;

	//for each directory.... get the files under the directory and obtain the final pcd file. named as directory_name.pcd
	for (int dir_index = 0; dir_index < directory_names.size(); dir_index++)
	{
		std::cout << std::endl << std::endl << std::endl;
		cloudPtr->clear();
		std::cout << "Clear the point cloud size to " << cloudPtr->points.size() << std::endl;
		std::string dir_name = directory_names[dir_index];
		int dir_slash_num = count(dir_name.begin(), dir_name.end(), '\\');	//count the number of "\" of directory's name

		for (int i = 0; i < files_names.size(); i++)
		{
			//Read File from file.3d to cloud
			std::string current_file = files_names[i];
			int file_slash_num = count(current_file.begin(), current_file.end(), '\\');		//count the number of "\" of file's name
			//std::cout << "dir_slash_num " << dir_slash_num << "file_slash_num " << file_slash_num << std::endl;
			bool compare_name = dir_name.compare(0, dir_name.length(), current_file, 0, dir_name.length());//compare the dir_name.length() characters of dir name and file name
			
			//string suffix_name = .assign(s, 0, s.size() - n);
			bool compare_format = current_file.compare( (current_file.length() - ptx_format_length), current_file.length(), ptx_format, 0, ptx_format_length);

			if (compare_format != 0)
				std::cout << "error format file! " << current_file << std::endl;
			// if compare_name==0 and the (file_slash_num - dir_slash_num)=1; then the file is under the directory..
			if (compare_name == 0 && ((file_slash_num - dir_slash_num) == 1) )	//the file belong to dir_name
			{
				std::cout << std::endl;
				std::cout << "current_file: " << current_file << std::endl;

				ifstream inFile(current_file, ios::in);

				inFile >> rows_number;
				inFile >> cols_number;
				inFile >> scan_translation_matrix(0, 0) >> scan_translation_matrix(0, 1) >> scan_translation_matrix(0, 2);
				inFile >> scan_rotation_matrix(0, 0) >> scan_rotation_matrix(0, 1) >> scan_rotation_matrix(0, 2);
				inFile >> scan_rotation_matrix(1, 0) >> scan_rotation_matrix(1, 1) >> scan_rotation_matrix(1, 2);
				inFile >> scan_rotation_matrix(2, 0) >> scan_rotation_matrix(2, 1) >> scan_rotation_matrix(2, 2);

				inFile >> scan_transform_matrix(0, 0) >> scan_transform_matrix(0, 1) >> scan_transform_matrix(0, 2) >> temp;
				inFile >> scan_transform_matrix(1, 0) >> scan_transform_matrix(1, 1) >> scan_transform_matrix(1, 2) >> temp;
				inFile >> scan_transform_matrix(2, 0) >> scan_transform_matrix(2, 1) >> scan_transform_matrix(2, 2) >> temp;
				inFile >> scan_transform_matrix(3, 0) >> scan_transform_matrix(3, 1) >> scan_transform_matrix(3, 2) >> temp;

				//std::cout << "scan_transform_matrix " << std::endl << scan_transform_matrix << std::endl;

				double px, py, pz, pi, pr, pg, pb;
				unsigned int count = 0;;
				while (inFile >> px >> py >> pz >> pi >> pr >> pg >> pb) //read each row
				{
					Eigen::Matrix<double, 1, 4> point4d;
					point4d << px, py, pz, 1;
					Eigen::Vector3d point_transformed;
					point_transformed = point4d * scan_transform_matrix;

					//std::cout << "point4d " << std::endl << point4d << std::endl;
					//std::cout << "scan_transform_matrix " << std::endl << scan_transform_matrix << std::endl;
					//std::cout << "point_transformed " << std::endl << point_transformed << std::endl;


					pcl::PointXYZI basic_point;
					basic_point.x = point_transformed.x();
					basic_point.y = point_transformed.y();
					basic_point.z = point_transformed.z();
					basic_point.intensity = pi;
					cloudPtr->points.push_back(basic_point);

					count++;
					if (count % 500000 == 0)
						std::cout << "registrate points number " << count << std::endl;
				}

			}
		}

		cloudPtr->width = cloudPtr->points.size();
		cloudPtr->height = 1;
		//Write to pcd file

		std::string out_pcd_name = dir_name.replace(dir_name.find("\\"), 1, "_") + ".pcd";  // replace / by _ in the string ;
		std::cout << out_pcd_name << std::endl;
		
		std::cout << "Save the pcd ....." << std::endl;
		remove(out_pcd_name.c_str());
		pcl::io::savePCDFileASCII(out_pcd_name, *cloudPtr);

		std::cout << "Saved " << cloudPtr->points.size() << " data points to " << out_pcd_name  << std::endl;
	}
	
	//visualize the pcd file
	//Read_PCD_colored_Show(out_pcd_name);

	std::cout << "Press any key to exit..." << std::endl;
	getchar();
	return (0);
}