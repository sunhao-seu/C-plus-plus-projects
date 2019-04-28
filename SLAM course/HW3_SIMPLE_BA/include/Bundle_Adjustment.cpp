#include <Bundle_Adjustment.h>

void read_picture_txt(std::string & file_name, std::vector<std::string> & picture_name, std::vector<double> & picture_time_stamp)
{
	std::string file_path = "/home/sunhaotest/Documents/rgbd_dataset_freiburg2_xyz/";
	std::string file_read = file_path + file_name;
	std::string get_filename;
	std::ifstream fp_file;	// file stream
	fp_file.open(file_read);
	std::string file_line_info;	//line information of rgb file
	double time_stamp;	//time stamp
						//std::string png_name[3700];	//png file name array
						//int png_count = 0;	//png count
	if (!fp_file.is_open())
	{
		std::cerr << "File " << file_read << " could not be opened for reading\n";
		fp_file.clear();
	}
	while (std::getline(fp_file, file_line_info))
	{
		//std::cout << file_line_info << std::endl;
		if (file_line_info[0] != '#')
		{
			std::istringstream string_divide(file_line_info);
			string_divide >> time_stamp;
			string_divide >> get_filename;
			//picture_name[png_count] = file_path + picture_name[png_count];
			picture_name.push_back(file_path + get_filename);
			picture_time_stamp.push_back(time_stamp);
			//std::cout << png_name[png_count].c_str() << std::endl;
		}
		//if (png_count == 100)
		//{
		//	fp_file.close();
		//	break;
		//}
	}
	fp_file.close();
}

void read_depth_file(std::string & file_name, Eigen::ArrayXXf & depthArray)
{
	char header[8];    // 8 is the maximum size that can be checked

					   // open file and test for it being a png
	FILE *fp = fopen(file_name.c_str(), "rb");
	if (!fp)
		std::cout << "[read_png_file] File " << file_name << " could not be opened for reading\n";
	fread(header, 1, 8, fp);
	if (png_sig_cmp((unsigned char *)header, 0, 8))
		std::cout << "[read_png_file] File " << file_name << " is not recognized as a PNG file\n";

	// initialize stuff
	png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

	if (!png_ptr)
		std::cout << "[read_png_file] png_create_read_struct failed\n";

	png_infop info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr)
		std::cout << "[read_png_file] png_create_info_struct failed\n";

	if (setjmp(png_jmpbuf(png_ptr)))
		std::cout << "[read_png_file] Error during init_io\n";

	png_init_io(png_ptr, fp);
	png_set_sig_bytes(png_ptr, 8);
	png_read_info(png_ptr, info_ptr);

	int width = png_get_image_width(png_ptr, info_ptr);
	int height = png_get_image_height(png_ptr, info_ptr);
	png_byte color_type = png_get_color_type(png_ptr, info_ptr);
	png_byte bit_depth = png_get_bit_depth(png_ptr, info_ptr);

	//This is the variable to be returned
	depthArray = Eigen::ArrayXXf(height, width);

	//int number_of_passes = png_set_interlace_handling(png_ptr);
	png_read_update_info(png_ptr, info_ptr);

	//read file
	if (setjmp(png_jmpbuf(png_ptr)))
		std::cout << "[read_png_file] Error during read_image\n";

	png_bytep * row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * height);
	for (int y = 0; y < height; y++)
		row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png_ptr, info_ptr));

	png_read_image(png_ptr, row_pointers);
	fclose(fp);

	int step = width;
	uint16_t * depthImg = (uint16_t *)malloc(height * 2 * step);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			memcpy(((unsigned char *)depthImg) + i * 2 * step + j * 2 + 0, row_pointers[i] + j * 2 + 1, 1);
			memcpy(((unsigned char *)depthImg) + i * 2 * step + j * 2 + 1, row_pointers[i] + j * 2 + 0, 1);
			float depth = (float)depthImg[i*step + j] / 5000.0f;
			if (depth >= 0.5f && depth <= 5.0f)
				depthArray(i, j) = depth;
			else
				depthArray(i, j) = -1;
		}
	}

	free((void*)depthImg);
	for (int y = 0; y < height; y++)
		free((void*)row_pointers[y]);
}

/*
Find the groundtruth and transform it to RT
*/
void GetGroundTruth(std::string & truth_file_name, std::vector<double> & picture_time_stamp, std::vector<Eigen::Matrix4d> & ground_truth_RT)
{
	std::string file_path = "/home/sunhaotest/Documents/rgbd_dataset_freiburg2_xyz/";
	std::string file_read = file_path + truth_file_name;
	double tx, ty, tz, qx, qy, qz, qw;
	std::ifstream fp_file;	// file stream
	fp_file.open(file_read);
	std::string file_line_info;	//line information of rgb file
	int file_count = 0;
	double time_stamp;	//time stamp
						//std::string png_name[3700];	//png file name array
						//int png_count = 0;	//png count
	if (!fp_file.is_open())
	{
		std::cerr << "File " << file_read << " could not be opened for reading\n";
		fp_file.clear();
	}
	while (std::getline(fp_file, file_line_info))
	{
		//std::cout << file_line_info << std::endl;
		if (file_line_info[0] != '#')
		{
			std::istringstream string_divide(file_line_info);
			string_divide >> time_stamp;
			string_divide >> tx;
			string_divide >> ty;
			string_divide >> tz;
			string_divide >> qx;
			string_divide >> qy;
			string_divide >> qz;
			string_divide >> qw;

			// transform quaternion to rotation matrix
			//Eigen中四元数赋值的顺序，实数w在首；但是实际上它的内部存储顺序是[x y z w]
			Eigen::Quaterniond xyzw(qw, qx, qy, qz);
			//xyzw.w = qw;
			//xyzw.x = qx;
			//xyzw.y = qy;
			//xyzw.z = qz;
			xyzw.normalized();	//important
			Eigen::Matrix3d Rotation_store = xyzw.toRotationMatrix();

			Eigen::Matrix4d ground_truth_RT_store;
			ground_truth_RT_store << Rotation_store(0, 0), Rotation_store(0, 1), Rotation_store(0, 2), tx,
				Rotation_store(1, 0), Rotation_store(1, 1), Rotation_store(1, 2), ty,
				Rotation_store(2, 0), Rotation_store(2, 1), Rotation_store(2, 2), tz,
				0, 0, 0, 1;

			Eigen::Quaterniond q = Eigen::Quaterniond(Rotation_store);

			ground_truth_RT.push_back(ground_truth_RT_store);
			picture_time_stamp.push_back(time_stamp);

			//file_count++;
			//if (file_count % 1000 == 0)
			//{
			//	std::cout << "Rotation_store " << Rotation_store << std::endl;
			//	std::cout << "xyzw.coeffs() " << xyzw.coeffs() << std::endl;
			//	std::cout << "xyzw= " << qx << " " << qy << " " << qz << " " << qw << std::endl;
			//	std::cout << "ground_truth_RT_store= " << ground_truth_RT_store << std::endl;
			//}
		}
	}
	fp_file.close();
}

/*
return groundtruth of specify timestamp
input: timestamp, goundtruth timestamp[vector], groundtruth homogenous matrix[vector]
output:specify homegenous matrix
*/
Eigen::Matrix4d Get_time_stamp_Matrix(double time_stamp, std::vector<double> & picture_time_stamp, std::vector<Eigen::Matrix4d> & ground_truth_RT)
{
	for (int i = 0; i < picture_time_stamp.size(); i++)
	{
		if (picture_time_stamp[i] >= time_stamp)
		{
			return ground_truth_RT[i];
		}
	}
}

int Get_depth_corres_indice(const double rgb_time_stamp, const std::vector<double> & depth_time_stamp)
{
	for (int i = 0; i < depth_time_stamp.size(); i++)
	{
		if (depth_time_stamp[i] >= rgb_time_stamp)
		{
			return i;
		}
	}
	std::cout << "can not find related depth!!" << std::endl;
	return -1;
}


void DrawFeaturePoints(const cv::Mat source_img, const std::vector<cv::Point> Feature_Points)
{
	for (size_t i = 0; i < Feature_Points.size(); i++)
	{
		cv::circle(source_img, Feature_Points[i], 4, cv::Scalar(0, 0, 255));//在图像中画出特征点，2是圆的半径
	}
}

void DrawMovingLines(const cv::Mat source_img, const std::vector<cv::Point> Feature_Points1, const std::vector<cv::Point> Feature_Points2)
{
	for (size_t i = 0; i < Feature_Points1.size(); i++)
	{
		cv::line(source_img, Feature_Points1[i], Feature_Points2[i], cv::Scalar(0, 255, 255));
	}
}


cv::Point2f pixel2cam(const cv::Point& p)
{
	return cv::Point2f
	(
		(p.x - Camera_K.at<double>(0, 2)) / Camera_K.at<double>(0, 0),
		(p.y - Camera_K.at<double>(1, 2)) / Camera_K.at<double>(1, 1)
	);
}

cv::Point2f cam2pixel(const cv::Point2f& p)
{
	return cv::Point2f
	(
		(p.x * Camera_K.at<double>(0, 0) + Camera_K.at<double>(0, 2)),
		(p.x * Camera_K.at<double>(1, 1) + Camera_K.at<double>(1, 2))
	);
}

void Feature_Extract_Match(const cv::Mat & img_1, const cv::Mat & img_2, std::vector<cv::KeyPoint> & keypoints_1, std::vector<cv::KeyPoint> & keypoints_2, std::vector< cv::DMatch > & good_matches)
{
	// feature extracting and matching
	std::vector<cv::Point> good_match_img1, good_match_img2;
	//****************Feature extraction and matching:*****************************
	cv::Mat descriptors_1, descriptors_2;					//store the descriptors;
	cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();	//define a dector to search the  key-points and correspondance descriptors.
	cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
	// Ptr<FeatureDetector> detector = FeatureDetector::create(detector_name);
	// Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create(descriptor_name);
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");		//brute force hamming match

																										//-- 第一步:检测 Oriented FAST 角点位置
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);

	//-- 第二步:根据角点位置计算 BRIEF 描述子
	descriptor->compute(img_1, keypoints_1, descriptors_1);
	descriptor->compute(img_2, keypoints_2, descriptors_2);

	//-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
	std::vector<cv::DMatch> matches;
	//BFMatcher matcher ( NORM_HAMMING );
	matcher->match(descriptors_1, descriptors_2, matches);

	//-- 第四步:匹配点对筛选
	double min_dist = 10000, max_dist = 0;

	//找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
	//only use the distance information
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n", min_dist);

	//当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (matches[i].distance <= std::max(2 * min_dist, 30.0))
		{
			good_matches.push_back(matches[i]);
		}
	}

	//-- 第五步:绘制匹配结果
	cv::Mat img_match;
	cv::Mat img_goodmatch;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
	drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
	//cv::imshow("所有匹配点对", img_match);
	//cv::imshow("优化后匹配点对", img_goodmatch);

	//draw hw requir result
	//std::vector<cv::Point> good_match_img1;		//store the good match points.
	//std::vector<cv::Point> good_match_img2;
	std::cout << "The number of good matches:" << good_matches.size() << std::endl;
	for (int i = 0; i < good_matches.size(); i++)
	{
		good_match_img1.push_back(cv::Point(keypoints_1[good_matches[i].queryIdx].pt.x, keypoints_1[good_matches[i].queryIdx].pt.y));
		good_match_img2.push_back(cv::Point(keypoints_2[good_matches[i].trainIdx].pt.x, keypoints_2[good_matches[i].trainIdx].pt.y));
		cv::line(img_2, good_match_img1[i], good_match_img2[i], cv::Scalar(0, 255, 255));
	}

#ifdef SHOW_IMAGE
	cv::Mat outimg2;		//draw the keypoints of img2;
	cv::drawKeypoints(img_2, keypoints_2, outimg2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
	cv::imshow("ORB特征点", outimg2);
	cv::waitKey(0);
#endif
}

void Ransac_Inliers(const std::vector< cv::DMatch > & good_matches, const std::vector<cv::KeyPoint> & keypoints_1, const std::vector<cv::KeyPoint> & keypoints_2, std::vector <cv::DMatch> & Ransaced_Matches, std::vector <cv::KeyPoint> & Ransaced_keypoints_1, std::vector <cv::KeyPoint> & Ransaced_keypoints_2)
{
	//**************** Relative pose estimation *****************************
	//RANSAC匹配过程  Using opencv library;
	std::vector<cv::DMatch> m_Matches;
	m_Matches = good_matches;
	//坐标转换为float类型
	std::vector <cv::KeyPoint> RAN_KP1, RAN_KP2;
	//size_t是标准C库中定义的，应为unsigned int，在64位系统中为long unsigned int,在C++中为了适应不同的平台，增加可移植性。
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		RAN_KP1.push_back(keypoints_1[good_matches[i].queryIdx]);
		RAN_KP2.push_back(keypoints_2[good_matches[i].trainIdx]);
		//RAN_KP1是要存储img01中能与img02匹配的点
		//goodMatches存储了这些匹配点对的img01和img02的索引值
	}
	//坐标变换
	std::vector <cv::Point2f> p01, p02;
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		p01.push_back(RAN_KP1[i].pt);
		p02.push_back(RAN_KP2[i].pt);
	}

	//求基础矩阵 Fundamental,3*3的基础矩阵
	std::vector<uchar> RansacStatus;
	cv::Mat Fundamental = cv::findFundamentalMat(p01, p02, RansacStatus, cv::FM_RANSAC);
#ifdef SHOW_INFORMATION
	std::cout << "Fundamental " << Fundamental << std::endl;
#endif
	//重新定义关键点RR_KP和Ransaced_Matches来存储新的关键点和基础矩阵，通过RansacStatus来删除误匹配点
	//std::vector <cv::KeyPoint> Ransaced_keypoints_1, Ransaced_keypoints_2;
	//std::vector <cv::DMatch> Ransaced_Matches;
	int index = 0;
	for (size_t i = 0; i < m_Matches.size(); i++)
	{
		if (RansacStatus[i] != 0)
		{
			Ransaced_keypoints_1.push_back(RAN_KP1[i]);
			Ransaced_keypoints_2.push_back(RAN_KP2[i]);
			m_Matches[i].queryIdx = index;
			m_Matches[i].trainIdx = index;
			Ransaced_Matches.push_back(m_Matches[i]);
			index++;
		}
	}
}

void Get_RT(const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	const std::vector< cv::DMatch > & Ransaced_Matches,
	cv::Mat & R, cv::Mat & t)
{
	//-- 把匹配点转换为vector<Point2f>的形式
	std::vector<cv::Point2f> points1;
	std::vector<cv::Point2f> points2;

	for (int i = 0; i < (int)Ransaced_Matches.size(); i++)
	{
		points1.push_back(keypoints_1[Ransaced_Matches[i].queryIdx].pt);
		points2.push_back(keypoints_2[Ransaced_Matches[i].trainIdx].pt);
	}


	//Eigen::Vector3f testpoint1, testpoint2, testnormalizedpoint1, testnormalizedpoint2;
	//Eigen::Matrix3f K;
	//K << 525, 0, 319.5, 0, 525, 239.5, 0, 0, 1;
	//Eigen::Matrix3f inver_K = K.inverse();
	//cv::Point2f norma_point_store;
	//std::vector<cv::Point2f> normapoints1;
	//std::vector<cv::Point2f> normapoints2;
	//for (int i = 0; i < (int)Ransaced_Matches.size(); i++)
	//{
	//	points1.push_back(keypoints_1[Ransaced_Matches[i].queryIdx].pt);
	//	points2.push_back(keypoints_2[Ransaced_Matches[i].trainIdx].pt);

	//	testpoint1 << points1[i].x, points1[i].y, 1;
	//	testpoint2 << points2[i].x, points2[i].y, 1;

	//	testnormalizedpoint1 = inver_K * testpoint1;
	//	testnormalizedpoint2 = inver_K * testpoint2;

	//	norma_point_store.x = testnormalizedpoint1[0];
	//	norma_point_store.y = testnormalizedpoint1[1];
	//	normapoints1.push_back(norma_point_store);
	//	norma_point_store.x = testnormalizedpoint2[0];
	//	norma_point_store.y = testnormalizedpoint2[1];
	//	normapoints2.push_back(norma_point_store);

	//}



	//-- 计算基础矩阵
	cv::Mat fundamental_matrix1;
	fundamental_matrix1 = findFundamentalMat(points1, points2, cv::FM_8POINT);
#ifdef SHOW_INFORMATION
	std::cout << "fundamental_matrix1 is " << std::endl << fundamental_matrix1 << std::endl;
#endif
	//cv::Mat fundamental_matrix;
	//fundamental_matrix = findFundamentalMat(normapoints1, normapoints2, cv::FM_8POINT);
	//std::cout << "fundamental_matrix is " << std::endl << fundamental_matrix << std::endl;
	//Eigen::Matrix3f F;
	//F << fundamental_matrix.at<double>(0, 0), fundamental_matrix.at<double>(0, 1), fundamental_matrix.at<double>(0, 2),
	//	fundamental_matrix.at<double>(1, 0), fundamental_matrix.at<double>(1, 1), fundamental_matrix.at<double>(1, 2),
	//	fundamental_matrix.at<double>(2, 0), fundamental_matrix.at<double>(2, 1), fundamental_matrix.at<double>(2, 2)
	//	;
	//std::vector<Eigen::Vector2f> TESTpoints1, TESTpoints2;
	//for (int i = 0; i < points1.size(); i++)
	//{
	//	Eigen::Vector2f point_store;
	//	point_store[0] = points1[i].x;
	//	point_store[1] = points1[i].y;
	//	TESTpoints1.push_back(point_store);
	//	point_store[0] = points2[i].x;
	//	point_store[1] = points2[i].y;
	//	TESTpoints2.push_back(point_store);
	//}

	//Eigen::Matrix3f  testR;
	//Eigen::Vector3f testT;
	//find_R_t(F, TESTpoints1, TESTpoints2, testR, testT);
	//std::cout << "TESTR is " << std::endl << testR << std::endl;
	//std::cout << "TESTt is " << std::endl << testT << std::endl;


	//cv::Mat essential_test = Camera_K.t() * fundamental_matrix * Camera_K;
	//std::cout << "essential_test is " << std::endl << essential_test << std::endl;


	//-- 计算本质矩阵
	cv::Mat essential_matrix;
	essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point, cv::RANSAC);
#ifdef SHOW_INFORMATION
	std::cout << "essential_matrix is " << std::endl << essential_matrix << std::endl;
#endif
	////-- 计算单应矩阵
	//cv::Mat homography_matrix;
	//homography_matrix = findHomography(points1, points2, cv::FM_RANSAC, 3);
	//std::cout << "homography_matrix is " << std::endl << homography_matrix << std::endl;

	//-- 从本质矩阵中恢复旋转和平移信息.
	recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
#ifdef SHOW_INFORMATION
	std::cout << "R is " << std::endl << R << std::endl;
	std::cout << "t is " << std::endl << t << std::endl;
#endif
	//recoverPose(essential_test, points1, points2, R, t, focal_length, principal_point);
	//std::cout << "test R is " << std::endl << R << std::endl;
	//std::cout << "test t is " << std::endl << t << std::endl;
}

void Verify_R_T(const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	const std::vector< cv::DMatch > & Ransaced_Matches,
	cv::Mat & R, cv::Mat & t)
{
	//-- 验证E=t^R*scale
	cv::Mat Transform_T_x = (cv::Mat_<double>(3, 3) <<
		0, -t.at<double>(2, 0), t.at<double>(1, 0),
		t.at<double>(2, 0), 0, -t.at<double>(0, 0),
		-t.at<double>(1.0), t.at<double>(0, 0), 0);

	std::cout << "t^R=" << std::endl << Transform_T_x * R << std::endl;

	//-- 验证对极约束
	for (cv::DMatch m : Ransaced_Matches)
	{
		cv::Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt);
		cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
		cv::Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt);
		cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
		cv::Mat d = y2.t() * Transform_T_x * R * y1;
		std::cout << "epipolar constraint = " << d << std::endl;
	}
}

void triangulation(
	const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	const std::vector< cv::DMatch > & matches,
	const cv::Mat& R, const cv::Mat& t,
	std::vector< cv::Point3d >& points)
{
	points.clear();
	cv::Mat T1 = (cv::Mat_<double>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	cv::Mat T2 = (cv::Mat_<double>(3, 4) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));

	//cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	std::vector<cv::Point2f> pts_1, pts_2;
	for (cv::DMatch m : matches)
	{
		// 将像素坐标转换至相机坐标
		pts_1.push_back(pixel2cam(keypoints_1[m.queryIdx].pt));
		pts_2.push_back(pixel2cam(keypoints_2[m.queryIdx].pt));
	}

	cv::Mat pts_4d;
	cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

	// 转换成非齐次坐标
	for (int i = 0; i<pts_4d.cols; i++)
	{
		cv::Mat x = pts_4d.col(i);
		x /= x.at<float>(3, 0); // 归一化
		cv::Point3d p(
			x.at<float>(0, 0),
			x.at<float>(1, 0),
			x.at<float>(2, 0)
		);
		points.push_back(p);
	}
}

void Verify_Triangulation(const std::vector<cv::KeyPoint> & keypoints_1,
	const std::vector<cv::KeyPoint> & keypoints_2,
	const std::vector< cv::DMatch > & matches,
	const cv::Mat& R, const cv::Mat& t,
	const std::vector< cv::Point3d >& points)
{
	for (int i = 0; i<matches.size(); i++)
	{
		cv::Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt);
		cv::Point2d pt1_cam_3d(
			points[i].x / points[i].z,
			points[i].y / points[i].z
		);

		std::cout << "point in the first camera frame: " << pt1_cam << std::endl;
		std::cout << "point projected from 3D " << pt1_cam_3d << ", d=" << points[i].z << std::endl;

		// 第二个图
		cv::Point2f pt2_cam = pixel2cam(keypoints_2[matches[i].trainIdx].pt);
		cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
		pt2_trans /= pt2_trans.at<double>(2, 0);
		std::cout << "point in the second camera frame: " << pt2_cam << std::endl;
		std::cout << "point reprojected from second frame: " << pt2_trans.t() << std::endl;
		std::cout << std::endl;
	}
}

void Local3D_to_World3D(const cv::Point3f & Local_point, const cv::Mat Camera_pose, cv::Point3f & World_point)
{
	cv::Mat Lpoint = (cv::Mat_<double>(3, 1) << Local_point.x, Local_point.y, Local_point.z);
	cv::Mat R = ( cv::Mat_<double>(3, 3) << Camera_pose.at<double>(0, 0), Camera_pose.at<double>(0, 1), Camera_pose.at<double>(0, 2),
		Camera_pose.at<double>(1, 0), Camera_pose.at<double>(1, 1), Camera_pose.at<double>(1, 2),
		Camera_pose.at<double>(2, 0), Camera_pose.at<double>(2, 1), Camera_pose.at<double>(2, 2) );
	cv::Mat t = ( cv::Mat_<double>(3, 1) << Camera_pose.at<double>(0, 3), Camera_pose.at<double>(1, 3), Camera_pose.at<double>(2, 3) );
	//cv::Mat Wpoint = R.t() * (Lpoint - t);		//Pc= R*Pw +t
	cv::Mat Wpoint = R * Lpoint + t;		//Pc= R*Pw +t  Tc = Twc  Pw=Twc*Pc  Pw = Tc * Pc
	World_point.x = Wpoint.at<double>(0, 0);
	World_point.y = Wpoint.at<double>(1, 0);
	World_point.z = Wpoint.at<double>(2, 0);
}

void World3D_to_CameraUV(const cv::Point3f & World_point, const cv::Mat Camera_pose, cv::Point2f & Local_point)
{
	cv::Mat Wpoint = (cv::Mat_<double>(3, 1) << World_point.x, World_point.y, World_point.z);
	cv::Mat Camera_pose_inv = Camera_pose.inv();//*********************
	cv::Mat R = (cv::Mat_<double>(3, 3) << Camera_pose_inv.at<double>(0, 0), Camera_pose_inv.at<double>(0, 1), Camera_pose_inv.at<double>(0, 2),
		Camera_pose_inv.at<double>(1, 0), Camera_pose_inv.at<double>(1, 1), Camera_pose_inv.at<double>(1, 2),
		Camera_pose_inv.at<double>(2, 0), Camera_pose_inv.at<double>(2, 1), Camera_pose_inv.at<double>(2, 2));
	cv::Mat t = (cv::Mat_<double>(3, 1) << Camera_pose_inv.at<double>(0, 3), Camera_pose_inv.at<double>(1, 3), Camera_pose_inv.at<double>(2, 3));
	//std::cout << "Camera_pose " << Camera_pose << std::endl;
	//std::cout << "Camera_pose_inv " << Camera_pose_inv << std::endl;
	//std::cout << "R " << R << std::endl;
	//std::cout << "t " << t << std::endl;
	cv::Mat Lpoint = R * Wpoint + t;		//R，T求逆了， 世界坐标系的点回到相机坐标系
	//cv::Mat Lpoint = R.t() * ( Wpoint - t );		//世界坐标系的点回到相机坐标系
	Lpoint = Lpoint / Lpoint.at<double>(2, 0);	//归一化
	std::cout << "Lpoint " << Lpoint << std::endl;
	Local_point = cam2pixel(cv::Point2f(Lpoint.at<double>(0, 0), Lpoint.at<double>(1, 0)));
	std::cout << "Local_point " << Local_point << std::endl;
}