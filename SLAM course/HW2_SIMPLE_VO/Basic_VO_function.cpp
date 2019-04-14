#include <Basic_VO_function.h>



void Feature_Extract_Match(const cv::Mat img_1, const cv::Mat img_2, std::vector<cv::Point> & good_match_img1, std::vector<cv::Point> & good_match_img2)
{
	good_match_img1.clear();
	good_match_img2.clear();
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
	Fundmental_Matrix.clear();
	Eigen::Matrix<double, 7, 9> A;
	for (int i = 0; i<7; i++)
	{
		int RanSample = int(rand() % good_match_img1.size());
		//A(i, 0) = good_match_img1[RanSample].x *good_match_img2[RanSample].x;
		//A(i, 1) = good_match_img1[RanSample].x *good_match_img2[RanSample].y;
		//A(i, 2) = good_match_img1[RanSample].x;
		//A(i, 3) = good_match_img1[RanSample].y *good_match_img2[RanSample].x;
		//A(i, 4) = good_match_img1[RanSample].y *good_match_img2[RanSample].y;
		//A(i, 5) = good_match_img1[RanSample].y;
		//A(i, 6) = good_match_img2[RanSample].x;
		//A(i, 7) = good_match_img2[RanSample].y;
		//A(i, 8) = 1.0;

		const float u1 = good_match_img1[RanSample].x;
		const float v1 = good_match_img1[RanSample].y;
		const float u2 = good_match_img2[RanSample].x;
		const float v2 = good_match_img2[RanSample].y;

		A(i, 0) = u2 * u1;
		A(i, 1) = u2 * v1;
		A(i, 2) = u2;
		A(i, 3) = v2 * u1;
		A(i, 4) = v2 * v1;
		A(i, 5) = v2;
		A(i, 6) = u1;
		A(i, 7) = v1;
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


void ScaleTriangulation(
	const std::vector<cv::Point> keypoint_1,
	const std::vector<cv::Point> keypoint_2,
	const cv::Mat& R, const cv::Mat& t,
	std::vector< cv::Point3d >& points, const cv::Mat & T_Pose_Pre)
{
	points.clear();
	cv::Mat T1 = (cv::Mat_<double>(3, 4) <<
		T_Pose_Pre.at<double>(0, 0), T_Pose_Pre.at<double>(0, 1), T_Pose_Pre.at<double>(0, 2), T_Pose_Pre.at<double>(0, 3),
		T_Pose_Pre.at<double>(1, 0), T_Pose_Pre.at<double>(1, 1), T_Pose_Pre.at<double>(1, 2), T_Pose_Pre.at<double>(1, 3),
		T_Pose_Pre.at<double>(2, 0), T_Pose_Pre.at<double>(2, 1), T_Pose_Pre.at<double>(2, 2), T_Pose_Pre.at<double>(2, 3));
	cv::Mat Transform_Matrix = (cv::Mat_<double>(4, 4) <<
		R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
		R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
		R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0),
		0,0,0,1
		);
	cv::Mat Homogenous_T2 = Transform_Matrix * T_Pose_Pre;
	cv::Mat T2 = (cv::Mat_<double>(3, 4) <<
		Homogenous_T2.at<double>(0, 0), Homogenous_T2.at<double>(0, 1), Homogenous_T2.at<double>(0, 2), Homogenous_T2.at<double>(0, 3),
		Homogenous_T2.at<double>(1, 0), Homogenous_T2.at<double>(1, 1), Homogenous_T2.at<double>(1, 2), Homogenous_T2.at<double>(1, 3),
		Homogenous_T2.at<double>(2, 0), Homogenous_T2.at<double>(2, 1), Homogenous_T2.at<double>(2, 2), Homogenous_T2.at<double>(2, 3));

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

void CalculateInlierNumbers(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, const std::vector< cv::Point3d > points, const cv::Mat& R, const cv::Mat& t, int &inliers_count, double &TotalError)
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


void GetInliers(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, const cv::Mat Fundmental_most, std::vector<cv::Point> & inliers_img1, std::vector<cv::Point> & inliers_img2)
{
	inliers_img1.clear();
	inliers_img2.clear();
	cv::Mat Essential_most = (Camera_K.t())*Fundmental_most*Camera_K;
	cv::Mat seven_point_R, seven_point_T;				//r,t CALCULATED BY Essential_most
	cv::recoverPose(Essential_most, good_match_img1, good_match_img2, seven_point_R, seven_point_T, focal_length, principal_point);
	std::vector<cv::Point3d> points;
	triangulation(good_match_img1, good_match_img2, seven_point_R, seven_point_T, points);
	for (size_t j = 0; j < good_match_img1.size(); j++)
	{
		cv::Point pt1 = good_match_img1[j];
		cv::Point pt1_reprojected = cam3d2pixel(points[j]);

		// 第二个图
		cv::Point2f pt2 = good_match_img2[j];
		cv::Mat pt2_trans = seven_point_R * (cv::Mat_<double>(3, 1) << points[j].x, points[j].y, points[j].z) + seven_point_T;
		cv::Point3d pt2_3d_points(
			pt2_trans.at<double>(0, 0),
			pt2_trans.at<double>(1, 0),
			pt2_trans.at<double>(2, 0)
		);
		cv::Point pt2_reprojected = cam3d2pixel(pt2_3d_points);

		double error_1, error_2;
		error_1 = getDistance(pt1_reprojected, pt1);
		error_2 = getDistance(pt2_reprojected, pt2);

		if ((error_1 < inlier_threshold) && (error_2 < inlier_threshold))
		{
			inliers_img1.push_back(good_match_img1[j]);
			inliers_img2.push_back(good_match_img2[j]);
		}
	}
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

void NormanizePoints(const std::vector<cv::Point> Source_Points, std::vector<cv::Point2f> & Normalized_Points, cv::Mat & T)
{
	Normalized_Points.clear();
	std::vector<cv::Point2f> Source_Points_RC;
	float RC_Points_ave_distance = 0.0;
	float cx = 0.0;
	float cy = 0.0;
	//取平均值
	for (size_t i = 0; i < Source_Points.size(); i++)
	{
		cx = Source_Points[i].x + cx;
		cy = Source_Points[i].y + cy;
	}
	cx = cx / Source_Points.size();
	cy = cy / Source_Points.size();

	//所有点的坐标减去平均值，求各点到所有点的平均值的平均欧氏距离
	cv::Point2f P_store;
	for (size_t i = 0; i < Source_Points.size(); i++)
	{
		P_store.x = Source_Points[i].x - cx;
		P_store.y = Source_Points[i].y - cy;
		Source_Points_RC.push_back(P_store);
		RC_Points_ave_distance = sqrtf(P_store.x*P_store.x + P_store.y*P_store.y);
	}
	RC_Points_ave_distance = RC_Points_ave_distance/ Source_Points.size();

	//将平均距离缩放为根号2
	for (size_t i = 0; i < Source_Points.size(); i++)
	{
		P_store.x = sqrt(2) / RC_Points_ave_distance * Source_Points_RC[i].x;
		P_store.y = sqrt(2) / RC_Points_ave_distance * Source_Points_RC[i].y;
		Normalized_Points.push_back(P_store);
	}
	T = (cv::Mat_<double>(3, 3) << sqrt(2)/ RC_Points_ave_distance,0, -(sqrt(2) / RC_Points_ave_distance * cx),
		0, sqrt(2) / RC_Points_ave_distance, -(sqrt(2) / RC_Points_ave_distance * cy),
		0,0,1);

}

cv::Mat Get_Eight_Points_Fundamental(const std::vector<cv::Point2f> Normalized_inliers_img1, const std::vector<cv::Point2f> Normalized_inliers_img2, const cv::Mat T1, const cv::Mat T2)
{
	cv::Mat Eigeht_Point_A(Normalized_inliers_img1.size(), 9, CV_32F);
	for (size_t i = 0; i < Normalized_inliers_img1.size(); i++)
	{
		const float u1 = Normalized_inliers_img1[i].x;
		const float v1 = Normalized_inliers_img1[i].y;
		const float u2 = Normalized_inliers_img2[i].x;
		const float v2 = Normalized_inliers_img2[i].y;

		float* ai = Eigeht_Point_A.ptr<float>(i);

		ai[0] = u2 * u1;
		ai[1] = u2 * v1;
		ai[2] = u2;
		ai[3] = v2 * u1;
		ai[4] = v2 * v1;
		ai[5] = v2;
		ai[6] = u1;
		ai[7] = v1;
		ai[8] = 1;

		//ai[0] = u1 * u2;
		//ai[1] = u1 * v2;
		//ai[2] = u1;
		//ai[3] = v1 * u2;
		//ai[4] = v1 * v2;
		//ai[5] = v1;
		//ai[6] = u2;
		//ai[7] = v2;
		//ai[8] = 1;
	}

	cv::Mat u, w, vt;

	//求A^T * A的特征值 ???
	cv::eigen(Eigeht_Point_A.t()*Eigeht_Point_A, w, vt);
	//cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

	cv::Mat Fpre = vt.row(8).reshape(0, 3);

	cv::SVDecomp(Fpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

	//w.at<float>(0) = 1;
	//w.at<float>(1) = 1;
	w.at<float>(2) = 0;

	cv::Mat F_norm = u * cv::Mat::diag(w)*vt;

	/*F = (cv::Mat_<double>(3, 3) << F_norm.at<float>(0, 0), F_norm.at<float>(0, 1), F_norm.at<float>(0, 2),
	F_norm.at<float>(1, 0), F_norm.at<float>(1,1), F_norm.at<float>(1,2),
	F_norm.at<float>(2, 0), F_norm.at<float>(2,1), F_norm.at<float>(2,2)
	);*/

	F_norm.convertTo(F_norm, CV_64F);		//float cannot multiply double

	cv::Mat Fundamental_EightPoint = (T2.t())*  F_norm * T1;
	//cv::Mat Fundamental_EightPoint = (T1.t())*  F_norm * T2;
	double F22 = Fundamental_EightPoint.at<double>(2, 2);
	if (fabs(F22) > FLT_EPSILON)
		Fundamental_EightPoint /= F22;

	//这里经过去归一化之后，得到还是Fundamental；
	return Fundamental_EightPoint;
}

void GetEssentialInliers(const std::vector<cv::Point> good_match_img1, const std::vector<cv::Point> good_match_img2, const cv::Mat Essential_Matrix, std::vector<cv::Point> & inliers_img1, std::vector<cv::Point> & inliers_img2)
{
	inliers_img1.clear();
	inliers_img2.clear();
	cv::Mat seven_point_R, seven_point_T;				//r,t CALCULATED BY Essential_most
	cv::recoverPose(Essential_Matrix, good_match_img1, good_match_img2, seven_point_R, seven_point_T, focal_length, principal_point);
	std::vector<cv::Point3d> points;
	triangulation(good_match_img1, good_match_img2, seven_point_R, seven_point_T, points);
	for (size_t j = 0; j < good_match_img1.size(); j++)
	{
		cv::Point pt1 = good_match_img1[j];
		cv::Point pt1_reprojected = cam3d2pixel(points[j]);

		// 第二个图
		cv::Point2f pt2 = good_match_img2[j];
		cv::Mat pt2_trans = seven_point_R * (cv::Mat_<double>(3, 1) << points[j].x, points[j].y, points[j].z) + seven_point_T;
		cv::Point3d pt2_3d_points(
			pt2_trans.at<double>(0, 0),
			pt2_trans.at<double>(1, 0),
			pt2_trans.at<double>(2, 0)
		);
		cv::Point pt2_reprojected = cam3d2pixel(pt2_3d_points);

		double error_1, error_2;
		error_1 = getDistance(pt1_reprojected, pt1);
		error_2 = getDistance(pt2_reprojected, pt2);

		if ((error_1 < inlier_threshold) && (error_2 < inlier_threshold))
		{
			inliers_img1.push_back(good_match_img1[j]);
			inliers_img2.push_back(good_match_img2[j]);
		}
	}
}

void DrawEpipolarLine(const cv::Mat source_img, const std::vector<cv::Point> Inlier_Points, const cv::Mat Fundamental_Matrix)
{
	
}
//画出外极线,对应点已经经过了RANSAC优化
void DrawEpiLines(const cv::Mat& img_1, const cv::Mat& img_2, std::vector<cv::Point>points1, std::vector<cv::Point>points2, const cv::Mat Fundamental_Matrix) {

	std::vector<cv::Vec<float, 3>> epilines1, epilines2;
	cv::computeCorrespondEpilines(points1, 1, Fundamental_Matrix, epilines1);//计算对应点的外极线epilines是一个三元组(a,b,c)，表示点在另一视图中对应的外极线ax+by+c=0;
	cv::computeCorrespondEpilines(points2, 2, Fundamental_Matrix, epilines2);
	//将图片转换为RGB图，画图的时候外极线用彩色绘制
	cv::Mat img1, img2;
	if (img_1.type() == CV_8UC3)
	{
		img_1.copyTo(img1);
		img_2.copyTo(img2);
	}
	else if (img_1.type() == CV_8UC1)
	{
		cvtColor(img_1, img1, cv::COLOR_GRAY2BGR);
		cvtColor(img_2, img2, cv::COLOR_GRAY2BGR);
	}
	else
	{
		std::cout << "unknow img type\n" << std::endl;
		exit(0);
	}

	cv::RNG& rng = cv::theRNG();
	for (uint i = 0; i < points2.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng(256), rng(256), rng(256));//随机产生颜色

		circle(img2, points2[i], 5, color);//在视图2中把关键点用圆圈画出来，然后再绘制在对应点处的外极线
		line(img2, cv::Point(0, -epilines1[i][2] / epilines1[i][1]), cv::Point(img2.cols, -(epilines1[i][2] + epilines1[i][0] * img2.cols) / epilines1[i][1]), color);
		//绘制外极线的时候，选择两个点，一个是x=0处的点，一个是x为图片宽度处
		circle(img1, points1[i], 4, color);
		line(img1, cv::Point(0, -epilines2[i][2] / epilines2[i][1]), cv::Point(img1.cols, -(epilines2[i][2] + epilines2[i][0] * img1.cols) / epilines2[i][1]), color);

	}
	cv::imshow("img2 epiline1", img2);
	cv::imshow("img1 epiline2", img1);
}