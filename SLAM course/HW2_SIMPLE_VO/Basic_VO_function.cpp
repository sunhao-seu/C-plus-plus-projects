#include <Basic_VO_function.h>



void Feature_Extract_Match(const cv::Mat img_1, const cv::Mat img_2, std::vector<cv::Point> & good_match_img1, std::vector<cv::Point> & good_match_img2)
{
	//****************Feature extraction and matching:*****************************
	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;		//store the key-points
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
	std::vector< cv::DMatch > good_matches;
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
	cv::Mat outimg2;		//draw the keypoints of img2;
	cv::drawKeypoints(img_2, keypoints_2, outimg2, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
	cv::imshow("ORB特征点", outimg2);
}



void GetFundamentalLameda(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, std::vector<Eigen::Matrix3d> & Fundmental_Matrix)
{
	Eigen::Matrix<double, 7, 9> A;
	for (int i = 0; i<7; i++)
	{
		int RanSample = int(rand() % good_match_img1.size());
		A(i, 0) = good_match_img1[RanSample].x *good_match_img2[RanSample].x;
		A(i, 1) = good_match_img1[RanSample].x *good_match_img2[RanSample].y;
		A(i, 2) = good_match_img1[RanSample].x;
		A(i, 3) = good_match_img1[RanSample].y *good_match_img2[RanSample].x;
		A(i, 4) = good_match_img1[RanSample].y *good_match_img2[RanSample].y;
		A(i, 5) = good_match_img1[RanSample].y;
		A(i, 6) = good_match_img2[RanSample].x;
		A(i, 7) = good_match_img2[RanSample].y;
		A(i, 8) = 1.0;
	}
	// SVD on A;  A=U*S*V'
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::MatrixXd U = svd.matrixU();
	Eigen::MatrixXd V = svd.matrixV();
	Eigen::MatrixXd Vt = svd.matrixV().transpose();
	Eigen::VectorXd sigma = svd.singularValues();
	Eigen::VectorXd f1 = V.col(V.cols() - 1);
	Eigen::VectorXd f2 = V.col(V.cols() - 2);

	Eigen::Matrix3d F1, F2, F;
	F1 << f1(0), f1(1), f1(2),
		f1(3), f1(4), f1(5),
		f1(6), f1(7), f1(8);
	F2 << f2(0), f2(1), f2(2),
		f2(3), f2(4), f2(5),
		f2(6), f2(7), f2(8);

	//std::cout << "V : " << V << std::endl;
	//std::cout << "F1 : " << F1 << std::endl;

	F = F2.inverse()*F1;
	Eigen::EigenSolver<Eigen::MatrixXd> solve_eigenvalue(F);
	Eigen::MatrixXcd F_EV = solve_eigenvalue.eigenvalues();//特征值
														   //std::cout << "The eigenvalues of F_EV are:" << std::endl << F_EV << std::endl;

	Eigen::MatrixXd realF_EV = F_EV.real();//获取特征值实数部分
	Eigen::MatrixXd imagF_EV = F_EV.imag();//获取特征值实数部分

	//std::cout << "imagF_EV:" << std::endl << imagF_EV << std::endl;
	std::vector<double> lamedas;
	for (size_t i = 0; i < imagF_EV.size(); i++)
	{
		if (imagF_EV(i) == 0)
		{
			lamedas.push_back(realF_EV(i));
			Fundmental_Matrix.push_back(F1 + realF_EV(i) * F2);
		}
	}


}



void triangulation(
	const std::vector<cv::Point> keypoint_1,
	const std::vector<cv::Point> keypoint_2,
	const cv::Mat& R, const cv::Mat& t,
	std::vector< cv::Point3d >& points)
{
	cv::Mat T1 = (cv::Mat_<float>(3, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	cv::Mat T2 = (cv::Mat_<float>(3, 4) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
		);

	//cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	std::vector<cv::Point2f> pts_1, pts_2;
	for (size_t i = 0; i<keypoint_1.size(); i++)
	{
		// 将像素坐标转换至相机坐标
		pts_1.push_back(pixel2cam(keypoint_1[i], Camera_K));
		pts_2.push_back(pixel2cam(keypoint_2[i], Camera_K));
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

void CalculateInlierNumbers(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, const std::vector< cv::Point3d >& points, const cv::Mat& R, const cv::Mat& t, int &inliers_count, double &TotalError)
{
	inliers_count = 0;		//count number of inliers
	TotalError = 0.0;
	for (size_t j = 0; j < good_match_img1.size(); j++)
	{
		cv::Point pt1 = good_match_img1[j];
		cv::Point pt1_reprojected = cam3d2pixel(points[j]);

		//std::cout << "point in the first camera frame: " << pt1_cam << std::endl;
		//std::cout << "point projected from 3D " << pt1_cam_3d << ", d=" << points[j].z << std::endl;

		// 第二个图
		cv::Point2f pt2 = good_match_img2[j];
		cv::Mat pt2_trans = R * (cv::Mat_<double>(3, 1) << points[j].x, points[j].y, points[j].z) + t;
		cv::Point3d pt2_3d_points(
			pt2_trans.at<double>(0, 0),
			pt2_trans.at<double>(1, 0),
			pt2_trans.at<double>(2, 0)
		);
		cv::Point pt2_reprojected = cam3d2pixel(pt2_3d_points);

		//std::cout << "point in the second camera frame: " << pt2_cam << std::endl;
		//std::cout << "point reprojected from second frame: " << pt2_trans.t() << std::endl;
		//std::cout << std::endl;

		double error_1, error_2;
		error_1 = getDistance(pt1_reprojected, pt1);
		error_2 = getDistance(pt2_reprojected, pt2);

		if ((error_1 < inlier_threshold) && (error_2 < inlier_threshold))
		{
			inliers_count++;
			TotalError = TotalError + error_1 * error_1 + error_2 * error_2;
		}
	}
	if (inliers_count < 2)
	{
		TotalError = TotalError_default;
	}
}


cv::Point2f pixel2cam(const cv::Point& p, const cv::Mat& K)
{
	return cv::Point2f
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
}


//三维坐标x,y,z 到像素平面坐标 u,v
cv::Point cam3d2pixel(const cv::Point3d& p)
{

	return cv::Point
	(
		(Camera_K.at<double>(0, 0)*(p.x/p.z)+ Camera_K.at<double>(0, 1)*(p.y / p.z) + Camera_K.at<double>(0, 2) ),
		(Camera_K.at<double>(1, 0)*(p.x / p.z) + Camera_K.at<double>(1, 1)*(p.y / p.z) + Camera_K.at<double>(1, 2))
	);
}


/***** 求两点间距离*****/
double getDistance(cv::Point2f pointO, cv::Point2f pointA)
{
	float distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	distance = sqrtf(distance);
	return distance;
}