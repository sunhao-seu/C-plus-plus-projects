#include "MyLaserOdometry.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

const float LaserOdometry::scanPeriod = 0.1;

LaserOdometry::LaserOdometry() : cornerPointsSharp(new pcl::PointCloud<PointType>()),
cornerPointsLessSharp(new pcl::PointCloud<PointType>()),
surfPointsFlat(new pcl::PointCloud<PointType>()),
surfPointsLessFlat(new pcl::PointCloud<PointType>()),
laserCloudCornerLast(new pcl::PointCloud<PointType>()),
laserCloudSurfLast(new pcl::PointCloud<PointType>()),
laserCloudOri(new pcl::PointCloud<PointType>()), 
coeffSel(new pcl::PointCloud<PointType>()),
laserCloudFullRes(new pcl::PointCloud<PointType>()),
imuTrans(new pcl::PointCloud<pcl::PointXYZ>()), 
kdtreeCornerLast(new pcl::KdTreeFLANN<PointType>()),
kdtreeSurfLast(new pcl::KdTreeFLANN<PointType>())
{
	systemInited = false;

	newCornerPointsSharp = false;   newCornerPointsLessSharp = false; 
	newSurfPointsFlat = false;newSurfPointsLessFlat = false; 
	newLaserCloudFullRes = false; newImuTrans = false;
}

void LaserOdometry::TransformToStart(PointType const * const pi, PointType * const po)
{
	//插值系数计算，云中每个点的相对时间/点云周期10
	float s = 10 * (pi->intensity - int(pi->intensity));//？？这里的10其实是跟0.1对应的，而不是单纯的为了求十分位的数

	//线性插值：根据每个点在点云中的相对位置关系，乘以相应的旋转平移系数
	Angle rx = s * transform.rot_x.value();
	Angle ry = s * transform.rot_y.value();
	Angle rz = s * transform.rot_z.value();

	//减去位移，再绕z,x,y旋转； 旋转表示为Y1X2Z3
	Vector3 v0(Vector3(*pi) - s * transform.pos);
	Vector3 v1 = rotateZ(v0, -rz);
	Vector3 v2 = rotateX(v1, -rx);
	Vector3 v3 = rotateY(v2, -ry);

	po->x = v3.x();
	po->y = v3.y();
	po->z = v3.z();
	po->intensity = pi->intensity;
}

void LaserOdometry::AccumulateRotation(Angle cx, Angle cy, Angle cz, Angle lx, Angle ly, Angle lz, Angle &ox, Angle &oy, Angle &oz)
{
	float srx = lx.cos()*cx.cos()*ly.sin()*cz.sin() - cx.cos()*cz.cos()*lx.sin() - lx.cos()*ly.cos()*cx.sin();
	ox = -asin(srx);

	float srycrx = lx.sin()*(cy.cos()*cz.sin() - cz.cos()*cx.sin()*cy.sin()) + lx.cos()*ly.sin()*(cy.cos()*cz.cos()
		+ cx.sin()*cy.sin()*cz.sin()) + lx.cos()*ly.cos()*cx.cos()*cy.sin();
	float crycrx = lx.cos()*ly.cos()*cx.cos()*cy.cos() - lx.cos()*ly.sin()*(cz.cos()*cy.sin()
		- cy.cos()*cx.sin()*cz.sin()) - lx.sin()*(cy.sin()*cz.sin() + cy.cos()*cz.cos()*cx.sin());
	oy = atan2(srycrx / ox.cos(), crycrx / ox.cos());

	float srzcrx = cx.sin()*(lz.cos()*ly.sin() - ly.cos()*lx.sin()*lz.sin()) + cx.cos()*cz.sin()*(ly.cos()*lz.cos()
		+ lx.sin()*ly.sin()*lz.sin()) + lx.cos()*cx.cos()*cz.cos()*lz.sin();
	float crzcrx = lx.cos()*lz.cos()*cx.cos()*cz.cos() - cx.cos()*cz.sin()*(ly.cos()*lz.sin()
		- lz.cos()*lx.sin()*ly.sin()) - cx.sin()*(ly.sin()*lz.sin() + ly.cos()*lz.cos()*lx.sin());
	oz = atan2(srzcrx / ox.cos(), crzcrx / ox.cos());
}

void LaserOdometry::PluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
	const Angle& blx, const Angle& bly, const Angle& blz,
	const Angle& alx, const Angle& aly, const Angle& alz,
	Angle &acx, Angle &acy, Angle &acz)
{
	float sbcx = bcx.sin();
	float cbcx = bcx.cos();
	float sbcy = bcy.sin();
	float cbcy = bcy.cos();
	float sbcz = bcz.sin();
	float cbcz = bcz.cos();

	float sblx = blx.sin();
	float cblx = blx.cos();
	float sbly = bly.sin();
	float cbly = bly.cos();
	float sblz = blz.sin();
	float cblz = blz.cos();

	float salx = alx.sin();
	float calx = alx.cos();
	float saly = aly.sin();
	float caly = aly.cos();
	float salz = alz.sin();
	float calz = alz.cos();

	float srx = -sbcx * (salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly)
		- cbcx * cbcz*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
			- calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
		- cbcx * sbcz*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
			- calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz);
	acx = -asin(srx);

	float srycrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
		- calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
		- (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
			- calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
		+ cbcx * sbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
	float crycrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly * sblx*sblz)
		- calx * saly*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sblz)
		- (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz * sblx*sbly)
			- calx * caly*(sbly*sblz + cbly * cblz*sblx) + cblx * cblz*salx)
		+ cbcx * cbcy*(salx*sblx + calx * caly*cblx*cbly + calx * cblx*saly*sbly);
	acy = atan2(srycrx / acx.cos(), crycrx / acx.cos());

	float srzcrx = sbcx * (cblx*cbly*(calz*saly - caly * salx*salz)
		- cblx * sbly*(caly*calz + salx * saly*salz) + calx * salz*sblx)
		- cbcx * cbcz*((caly*calz + salx * saly*salz)*(cbly*sblz - cblz * sblx*sbly)
			+ (calz*saly - caly * salx*salz)*(sbly*sblz + cbly * cblz*sblx)
			- calx * cblx*cblz*salz) + cbcx * sbcz*((caly*calz + salx * saly*salz)*(cbly*cblz
				+ sblx * sbly*sblz) + (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz)
				+ calx * cblx*salz*sblz);
	float crzcrx = sbcx * (cblx*sbly*(caly*salz - calz * salx*saly)
		- cblx * cbly*(saly*salz + caly * calz*salx) + calx * calz*sblx)
		+ cbcx * cbcz*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
			+ (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly)
			+ calx * calz*cblx*cblz) - cbcx * sbcz*((saly*salz + caly * calz*salx)*(cblz*sbly
				- cbly * sblx*sblz) + (caly*salz - calz * salx*saly)*(cbly*cblz + sblx * sbly*sblz)
				- calx * calz*cblx*sblz);
	acz = atan2(srzcrx / acx.cos(), crzcrx / acx.cos());


}

void LaserOdometry::TransformToEnd(PointType const * const pi, PointType * const po)
{
	//po->x = pi->x;
	//po->y = pi->y;
	//po->z = pi->z;
	//po->intensity = int(pi->intensity);

	float s = 10 * (pi->intensity - int(pi->intensity));
	Angle rx = s * transform.rot_x.value();
	Angle ry = s * transform.rot_y.value();
	Angle rz = s * transform.rot_z.value();

	Vector3 v0(Vector3(*pi) - s * transform.pos);
	Vector3 v1 = rotateZ(v0, -rz);
	Vector3 v2 = rotateX(v1, -rx);
	Vector3 v3 = rotateY(v2, -ry);

	Vector3 v4 = rotateY(v3, transform.rot_y);
	Vector3 v5 = rotateX(v4, transform.rot_x);
	Vector3 v6 = rotateZ(v5, transform.rot_z);
	v6 += transform.pos - imuShiftFromStart;

	Vector3 v7 = rotateZ(v6, imuRollStart);
	Vector3 v8 = rotateX(v7, imuPitchStart);
	Vector3 v9 = rotateY(v8, imuYawStart);

	Vector3 v10 = rotateY(v9, -imuYawLast);
	Vector3 v11 = rotateX(v10, -imuPitchLast);
	Vector3 v12 = rotateZ(v11, -imuRollLast);

	po->x = v12.x();
	po->y = v12.y();
	po->z = v12.z();
	po->intensity = int(pi->intensity);
}

void LaserOdometry::laserCloudSharpHandler(const pcl::PointCloud<PointType>& cornerPointsSharp2)
{
	cornerPointsSharp->clear();
	*cornerPointsSharp = cornerPointsSharp2;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
	newCornerPointsSharp = true;
}

void LaserOdometry::laserCloudLessSharpHandler(const pcl::PointCloud<PointType>& cornerPointsLessSharp2)
{
	cornerPointsLessSharp->clear();
	*cornerPointsLessSharp = cornerPointsLessSharp2;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, indices);
	newCornerPointsLessSharp = true;
}

void LaserOdometry::laserCloudFlatHandler(const pcl::PointCloud<PointType>& surfPointsFlat2)
{
	surfPointsFlat->clear();

	*surfPointsFlat = surfPointsFlat2;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*surfPointsFlat, *surfPointsFlat, indices);
	newSurfPointsFlat = true;
}

void LaserOdometry::laserCloudLessFlatHandler(const pcl::PointCloud<PointType>& surfPointsLessFlat2)
{
	surfPointsLessFlat->clear();

	*surfPointsLessFlat = surfPointsLessFlat2;

	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, indices);
	newSurfPointsLessFlat = true;
}

void LaserOdometry::laserCloudFullResHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudFullRes2)
{
	laserCloudFullRes->clear();

	*laserCloudFullRes = *laserCloudFullRes2;
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudFullRes, *laserCloudFullRes, indices);
	newLaserCloudFullRes = true;
}

void LaserOdometry::imuTransHandler(const pcl::PointCloud<pcl::PointXYZ>& imuTrans2)
{
	// 跟imu有关的操作，，由register返回的，此代码没有用到Imu。。

	imuTrans->clear();
	*imuTrans = imuTrans2;

	imuPitchStart = imuTrans->points[0].x;
	imuYawStart = imuTrans->points[0].y;
	imuRollStart = imuTrans->points[0].z;

	imuPitchLast = imuTrans->points[1].x;
	imuYawLast = imuTrans->points[1].y;
	imuRollLast = imuTrans->points[1].z;

	imuShiftFromStart.x() = imuTrans->points[2].x;
	imuShiftFromStart.y() = imuTrans->points[2].y;
	imuShiftFromStart.z() = imuTrans->points[2].z;

	imuVeloFromStart.x() = imuTrans->points[3].x;
	imuVeloFromStart.y() = imuTrans->points[3].y;
	imuVeloFromStart.z() = imuTrans->points[3].z;


	newImuTrans = true;
}

LaserOdometryBack LaserOdometry::MyLaserOdometryHandler(const ScanRegistrationBack& ScanValueBack)
{
	LaserOdometryBack OdometryValueBack;

	//就是取值操作，将Registration返回的值复制到本地的数组中，，还是指针？
	laserCloudSharpHandler(ScanValueBack.cornerPointsSharp);
	laserCloudLessSharpHandler(ScanValueBack.cornerPointsLessSharp);
	laserCloudFlatHandler(ScanValueBack.surfPointsFlat);
	laserCloudLessFlatHandler(ScanValueBack.surfPointsLessFlat);
	laserCloudFullResHandler(ScanValueBack.laserCloud);
	imuTransHandler(ScanValueBack.imuTrans);


	PointType pointSel;	//当前帧的特征点集中取出一个点
	PointType tripod1, tripod2, tripod3;	//最近点与次邻近点; 次次对应点（面需要三个点）
	PointType pointProj;	//存储偏导信息，jacobian..不对，这个存储的是pointSel减去偏导
	PointType coeff; // 每个特征点对应的Jaccobian矩阵的三个元素都保存在coeffSel中，后面采用L - M方法解算的时候直接调用就行了。
	PointType pointOri;	//构建L-M矩阵时使用，正在处理的点

	bool isDegenerate = false;	//判断是否发生退化
	Eigen::Matrix<float, 6, 6> matP;	//预测矩阵P，退化时使用
	int frameCount = skipFrameNum;		//隔多少帧处理一次。。这里每帧都处理

	std::vector<int> pointSearchInd;		//最邻近搜索的最近点
	std::vector<float> pointSearchSqDis;    //最邻近搜索的最近点到该点的距离


	//同步作用，确保同时收到同一个点云的特征点以及IMU信息才进入
	if (newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat &&
		newSurfPointsLessFlat && newLaserCloudFullRes && newImuTrans)
	{
		newCornerPointsSharp = false;
		newCornerPointsLessSharp = false;
		newSurfPointsFlat = false;
		newSurfPointsLessFlat = false;
		newLaserCloudFullRes = false;
		newImuTrans = false;

		//对第一帧做初始化工作，赋给laserCloudCornerLast。。
		if (!systemInited)
		{
			//registration中的lesssharp在这里都存入cornerlast里面，构建kd树
			pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
			cornerPointsLessSharp = laserCloudCornerLast;
			laserCloudCornerLast = laserCloudTemp;
			// ROS_INFO("init");

			//exchange the data between surfPointsLessFlat and laserCloudSurfLast;
			//surf points in last sweep is regarded as lessflat points this sweep
			laserCloudTemp = surfPointsLessFlat;
			surfPointsLessFlat = laserCloudSurfLast;
			laserCloudSurfLast = laserCloudTemp;


			transformSum.rot_x += imuPitchStart;
			transformSum.rot_z += imuRollStart;

			//build kd tree of last corner points and surf points
			kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
			kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

			laserCloudCornerLastNum = laserCloudCornerLast->points.size();
			laserCloudSurfLastNum = laserCloudSurfLast->points.size();

			OdometryValueBack.laserCloudCornerLast = laserCloudCornerLast;
			OdometryValueBack.laserCloudSurfLast = laserCloudSurfLast;
			OdometryValueBack.laserCloudFullRes.reset(new pcl::PointCloud<PointType>());
			memset(OdometryValueBack.transformSum, 0, sizeof(OdometryValueBack.transformSum));

			systemInited = true;

			return OdometryValueBack;
		}


		//transform.pos -= imuVeloFromStart * scanPeriod;

		//只有特征点的个数足够才能够进行后面的匹配工作，不够的话就不算，关键帧的概念
		if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
		{
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices); // 剔除异常点
			int cornerPointsSharpNum = cornerPointsSharp->points.size(); // 当前时刻特征角点的个数
			int surfPointsFlatNum = surfPointsFlat->points.size(); // 当前时刻特征平坦点的个数

			//迭代次数为25次，以保证运算效率
			//1：对特征边 / 面上的点进行处理，2：构建Jaccobian矩阵，3：L - M运动估计求解。
			//L-M方法其实就是非线性最小二乘，是Gauss-Newton优化的一种改进（增加了一个阻尼因子，代码中的s），所以关键在于如何把点云配准和运动估计的问题转换为L-M优化求解的问题。
			//主要思路就是：构建约束方程 -> 约束方程求偏导构建Jaccobian矩阵 -> L-M求解。
			for (int iterCount = 0; iterCount < 25; iterCount++)
			{

				//process corner points; figure7(a)    cycle number: 36或者1
				for (int i = 0; i < cornerPointsSharpNum; i++)
				{
					//每次迭代后，得到的增量加到总的转换transform中，然后下一次迭代用新的转换后的数据来计算
					TransformToStart(&cornerPointsSharp->points[i], &pointSel);		//将角点转换到点云初始时刻；而kd树里面存储的是上一帧的最终时刻，也就是这个时刻的点云数据。pointSel为转换后点

					//每迭代五次，重新查找最近点
					if (iterCount % 5 == 0)
					{
						std::vector<int> indices;
						pcl::removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);
						//kd-tree查找一个最近距离点，边沿点未经过体素栅格滤波，一般边沿点本来就比较少，不做滤波
						kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
						int closestPointInd = -1, minPointInd2 = -1;

						//距离小于25才认为是有效的最邻近点
						if (pointSearchSqDis[0] < 25)
						{
							closestPointInd = pointSearchInd[0];
							int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity); // 搜索到的点j所在线数

							float pointSqDis, minPointSqDis2 = 25;
							//在上一帧向后找次邻近点；SCAN差值超过2.5的直接跳过； 结果存入minPointSqDis2 and minPointInd2中
							for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
							{
								//SCAN差值超过2.5的直接跳过
								if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5)
								{
									break;
								}

								pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
									(laserCloudCornerLast->points[j].x - pointSel.x) +
									(laserCloudCornerLast->points[j].y - pointSel.y) *
									(laserCloudCornerLast->points[j].y - pointSel.y) +
									(laserCloudCornerLast->points[j].z - pointSel.z) *
									(laserCloudCornerLast->points[j].z - pointSel.z);

								if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan)
								{
									if (pointSqDis < minPointSqDis2) {
										minPointSqDis2 = pointSqDis;
										minPointInd2 = j;
									}
								}
							}

							//向前找次邻近点；SCAN差值超过2.5的直接跳过
							for (int j = closestPointInd - 1; j >= 0; j--)
							{
								if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5)
								{
									break;
								}

								pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
									(laserCloudCornerLast->points[j].x - pointSel.x) +
									(laserCloudCornerLast->points[j].y - pointSel.y) *
									(laserCloudCornerLast->points[j].y - pointSel.y) +
									(laserCloudCornerLast->points[j].z - pointSel.z) *
									(laserCloudCornerLast->points[j].z - pointSel.z);

								if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan)
								{
									if (pointSqDis < minPointSqDis2)
									{
										minPointSqDis2 = pointSqDis;
										minPointInd2 = j;
									}
								}
							}

						}
						pointSearchCornerInd1[i] = closestPointInd;  // 当前所有边特征点在上一时刻边特征点云中对应的最邻近点的索引
						pointSearchCornerInd2[i] = minPointInd2;  // 当前所有边特征点在上一时刻边特征点云中对应的次邻近点的索引

					}

					if (pointSearchCornerInd2[i] >= 0)//大于等于0，说明两个点都找到了；默认为-1
					{
						//公式2，求点到线距离
						//pointSel-->i   tripod1-->j	tripod2-->l
						tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
						tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

						// 选择的特征点记为O，kd - tree最近距离点记为A，另一个最近距离点记为B
						float x0 = pointSel.x;
						float y0 = pointSel.y;
						float z0 = pointSel.z;
						float x1 = tripod1.x;
						float y1 = tripod1.y;
						float z1 = tripod1.z;
						float x2 = tripod2.x;
						float y2 = tripod2.y;
						float z2 = tripod2.z;
						//a012=|(pointSel - tripod1) cross (pointSel - tripod2)|	cross 表示叉乘  公式（2）
						//向量OA = (x0 - x1, y0 - y1, z0 - z1), 向量OB = (x0 - x2, y0 - y2, z0 - z2)，向量AB = （x1 - x2, y1 - y2, z1 - z2）
						//向量OA OB的向量积(即叉乘)为：
						//|  i      j      k  |
						//|x0-x1  y0-y1  z0-z1|
						//|x0-x2  y0-y2  z0-z2|
						//模为：
						float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
							* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
							+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
							* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
						//l12=distance of tripod1 and tripod2
						float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

						// 向量[la；lb；lc] 为距离ld2分别对x0 y0 z0的偏导,也就是当前点i  Jacobian matrix 最终要transform的偏导
						//AB方向的单位向量与OAB平面的单位法向量的向量积在各轴上的分量（d的方向）
						//x轴分量i
						//sh: la就是ld2对x0的求导；lb是ld2对y0的求导；；此处的求导带入
						float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

						//y轴分量j
						float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

						//z轴分量k
						float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
							+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

						//点到线的距离，d = |向量OA 叉乘 向量OB|/|AB|
						float ld2 = a012 / l12;		//空间中点到直线的距离，就是论文中的de

						//unused
						pointProj = pointSel;
						pointProj.x -= la * ld2;
						pointProj.y -= lb * ld2;
						pointProj.z -= lc * ld2;

						//权重计算，距离越大权重越小，距离越小权重越大，得到的权重范围<=1
						float  s = 1;	//传说中的阻尼因子,取1应该是一开始没有权重
						if (iterCount >= 5)
						{
							//5次迭代之后开始增加权重因素
							s = 1 - 1.8 * fabs(ld2);  // 点到直线距离越小阻尼因子越大
						}
						coeff.x = s * la;
						coeff.y = s * lb;
						coeff.z = s * lc;
						coeff.intensity = s * ld2;

						// ？？满足阈值(ld2 < 0.5)，将特征点插入
						if (s > 0.1 && ld2 != 0)
						{
							laserCloudOri->push_back(cornerPointsSharp->points[i]);
							coeffSel->push_back(coeff); //每个特征点对应的Jaccobian矩阵的三个元素都保存在coeffSel中，后面采用L - M方法解算的时候直接调用就行了。
						}

					}
				}

				//process palanr points; figure7(b)    cycle number: 202或者2
				for (int i = 0; i < surfPointsFlatNum; i++)
				{
					TransformToStart(&surfPointsFlat->points[i], &pointSel);
					//每5次迭代重新找一次最邻近点
					if (iterCount % 5 == 0)
					{
						kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
						int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

						if (pointSearchSqDis[0] < 25)
						{
							closestPointInd = pointSearchInd[0];
							int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

							float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;

							//向后找
							for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++)
							{
								if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5)
								{
									break;
								}

								pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
									(laserCloudSurfLast->points[j].x - pointSel.x) +
									(laserCloudSurfLast->points[j].y - pointSel.y) *
									(laserCloudSurfLast->points[j].y - pointSel.y) +
									(laserCloudSurfLast->points[j].z - pointSel.z) *
									(laserCloudSurfLast->points[j].z - pointSel.z);

								if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan)
								{
									if (pointSqDis < minPointSqDis2)
									{
										minPointSqDis2 = pointSqDis;
										minPointInd2 = j;
									}
								}
								else
								{
									if (pointSqDis < minPointSqDis3)
									{
										minPointSqDis3 = pointSqDis;
										minPointInd3 = j;
									}
								}
							}
							//向前找
							for (int j = closestPointInd - 1; j >= 0; j--)
							{
								if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5)
								{
									break;
								}

								pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
									(laserCloudSurfLast->points[j].x - pointSel.x) +
									(laserCloudSurfLast->points[j].y - pointSel.y) *
									(laserCloudSurfLast->points[j].y - pointSel.y) +
									(laserCloudSurfLast->points[j].z - pointSel.z) *
									(laserCloudSurfLast->points[j].z - pointSel.z);

								if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan)
								{
									if (pointSqDis < minPointSqDis2)
									{
										minPointSqDis2 = pointSqDis;
										minPointInd2 = j;
									}
								}
								else
								{
									if (pointSqDis < minPointSqDis3)
									{
										minPointSqDis3 = pointSqDis;
										minPointInd3 = j;
									}
								}
							}

						}
						pointSearchSurfInd1[i] = closestPointInd;
						pointSearchSurfInd2[i] = minPointInd2;
						pointSearchSurfInd3[i] = minPointInd3;

					}

					if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0)
					{
						tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]]; //A点
						tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]]; //B点
						tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]]; //C点

						//pa pb pc  // 向量[pa；pb；pc] = 点到面的距离对x0 y0 z0的偏导
						//向量AB AC的向量积（即叉乘），得到的是法向量
						//x轴方向分向量i
						float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
							- (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
						//y轴方向分向量j
						float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
							- (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
						float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
							- (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
						float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

						//法向量的模
						float ps = sqrt(pa * pa + pb * pb + pc * pc);
						//pa pb pc为法向量各方向上的单位向量
						pa /= ps;
						pb /= ps;
						pc /= ps;
						pd /= ps;

						//点到面的距离：向量OA与与法向量的点积除以法向量的模
						float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

						pointProj = pointSel;
						pointProj.x -= pa * pd2;
						pointProj.y -= pb * pd2;
						pointProj.z -= pc * pd2;

						float s = 1;
						if (iterCount >= 5) {
							s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
								+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));

							// s = 1 - 1.8 * fabs(pd2) ;
						}

						coeff.x = s * pa;
						coeff.y = s * pb;
						coeff.z = s * pc;//
						coeff.intensity = s * pd2;

						if (s > 0.1 && pd2 != 0) {
							//保存原始点与相应的系数
							laserCloudOri->push_back(surfPointsFlat->points[i]);
							coeffSel->push_back(coeff);
						}
					}

				}

				//后面应该是解公式9。。L-M的解算过程         奇怪，L-M的解算部分几乎不需要时间，只需要2，3个时钟周期。。。
				//雷达的运动是连续的。将所有对应到的点求到直线的距离到面的距离之和最短然后按照Levenberg-Marquardt算法迭代计算，得到两帧之间的变换，最后通过累计计算odom
				int pointSelNum = laserCloudOri->points.size(); // 匹配到的点的个数(即存在多少个约束)
				if (pointSelNum < 10)	//特征点太少;
				{
					continue;
				}

				Eigen::Matrix<float, Eigen::Dynamic, 6> matA(pointSelNum, 6);	//n*6
				Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, pointSelNum);	//6*n  A的转置
				Eigen::Matrix<float, 6, 6> matAtA;								//6*6
				Eigen::VectorXf matB(pointSelNum);								//n*1
				Eigen::Matrix<float, 6, 1> matAtB;								//6*1								
				Eigen::Matrix<float, 6, 1> matX;

				// 遍历每一个特征点，构建Jaccobian矩阵  需要得到的是距离对坐标变换的偏导数
				// 构建matA and matB
				for (int i = 0; i < pointSelNum; i++)
				{
					/**
					* 采用Levenberg-Marquardt计算
					* 首先建立当前时刻Lidar坐标系下提取到的特征点与点到直线/平面
					* 的约束方程。而后对约束方程求对坐标变换(3旋转+3平移)的偏导
					* 公式参见论文(2)-(8)
					*有几个对应点就有几个约束方程
					*/
					pointOri = laserCloudOri->points[i];// 当前处理的点
					coeff = coeffSel->points[i];// 该点所对应的偏导数

					float s = 1;
					//transform就是TkL(t)，存储的 分别是角度和位置信息
					float srx = sin(s * transform.rot_x.value());
					float crx = cos(s * transform.rot_x.value());
					float sry = sin(s * transform.rot_y.value());
					float cry = cos(s * transform.rot_y.value());
					float srz = sin(s * transform.rot_z.value());
					float crz = cos(s * transform.rot_z.value());
					float tx = s * transform.pos.x();
					float ty = s * transform.pos.y();
					float tz = s * transform.pos.z();

					//loss对RT矩阵中的参数求偏导 ？？？？
					//构建LM方法里面的A矩阵，求出来的MATX是增量deltaX
					float arx = (-s * crx*sry*srz*pointOri.x + s * crx*crz*sry*pointOri.y + s * srx*sry*pointOri.z
						+ s * tx*crx*sry*srz - s * ty*crx*crz*sry - s * tz*srx*sry) * coeff.x
						+ (s*srx*srz*pointOri.x - s * crz*srx*pointOri.y + s * crx*pointOri.z
							+ s * ty*crz*srx - s * tz*crx - s * tx*srx*srz) * coeff.y
						+ (s*crx*cry*srz*pointOri.x - s * crx*cry*crz*pointOri.y - s * cry*srx*pointOri.z
							+ s * tz*cry*srx + s * ty*crx*cry*crz - s * tx*crx*cry*srz) * coeff.z;

					float ary = ((-s * crz*sry - s * cry*srx*srz)*pointOri.x
						+ (s*cry*crz*srx - s * sry*srz)*pointOri.y - s * crx*cry*pointOri.z
						+ tx * (s*crz*sry + s * cry*srx*srz) + ty * (s*sry*srz - s * cry*crz*srx)
						+ s * tz*crx*cry) * coeff.x
						+ ((s*cry*crz - s * srx*sry*srz)*pointOri.x
							+ (s*cry*srz + s * crz*srx*sry)*pointOri.y - s * crx*sry*pointOri.z
							+ s * tz*crx*sry - ty * (s*cry*srz + s * crz*srx*sry)
							- tx * (s*cry*crz - s * srx*sry*srz)) * coeff.z;

					float arz = ((-s * cry*srz - s * crz*srx*sry)*pointOri.x + (s*cry*crz - s * srx*sry*srz)*pointOri.y
						+ tx * (s*cry*srz + s * crz*srx*sry) - ty * (s*cry*crz - s * srx*sry*srz)) * coeff.x
						+ (-s * crx*crz*pointOri.x - s * crx*srz*pointOri.y
							+ s * ty*crx*srz + s * tx*crx*crz) * coeff.y
						+ ((s*cry*crz*srx - s * sry*srz)*pointOri.x + (s*crz*sry + s * cry*srx*srz)*pointOri.y
							+ tx * (s*sry*srz - s * cry*crz*srx) - ty * (s*crz*sry + s * cry*srx*srz)) * coeff.z;

					float atx = -s * (cry*crz - srx * sry*srz) * coeff.x + s * crx*srz * coeff.y
						- s * (crz*sry + cry * srx*srz) * coeff.z;

					float aty = -s * (cry*srz + crz * srx*sry) * coeff.x - s * crx*crz * coeff.y
						- s * (sry*srz - cry * crz*srx) * coeff.z;

					float atz = s * crx*sry * coeff.x - s * srx * coeff.y - s * crx*cry * coeff.z;

					//距离，点线距离，点面距离
					float d2 = coeff.intensity;

					matA(i, 0) = arx;
					matA(i, 1) = ary;
					matA(i, 2) = arz;
					matA(i, 3) = atx;
					matA(i, 4) = aty;
					matA(i, 5) = atz;
					matB(i, 0) = -0.05 * d2;
				}
				// 最小二乘计算(QR分解法)
				matAt = matA.transpose();
				matAtA = matAt * matA;
				matAtB = matAt * matB;

				//通过矩阵分解来求解矩阵方程，速度比求逆快  matAtA*matX=matAtB；解matX
				matX = matAtA.colPivHouseholderQr().solve(matAtB);

				//第一次迭代才进行的操作，应该是进行了某些操作避免局部最优
				if (iterCount == 0)
				{
					Eigen::Matrix<float, 1, 6> matE;	//特征值1*6矩阵
					Eigen::Matrix<float, 6, 6> matV;	//特征向量6*6矩阵
					Eigen::Matrix<float, 6, 6> matV2;

					// 计算矩阵的特征向量E及特征向量的反对称阵V
					Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
					matE = esolver.eigenvalues().real();
					matV = esolver.eigenvectors().real();

					matV2 = matV;

					isDegenerate = false;
					float eignThre[6] = { 10, 10, 10, 10, 10, 10 }; //特征值取值门槛
					for (int i = 5; i >= 0; i--)//从小到大查找
					{
						if (matE(0, i) < eignThre[i])  ////特征值太小，则认为处在兼并环境中，发生了退化
						{
							for (int j = 0; j < 6; j++)//对应的特征向量置为0
							{
								matV2(i, j) = 0;
							}
							isDegenerate = true;
						}
						else {
							break;
						}
					}
					matP = matV.inverse() * matV2;//计算P矩阵
				}

				if (isDegenerate) //如果发生退化，只使用预测矩阵P计算
				{
					Eigen::Matrix<float, 6, 1> matX2;
					matX2 = matX;

					matX = matP * matX2;
				}

				//迭代中操作，累积旋转角度与位移
				//解出来的MatX包含的信息应该是一个旋转平移量，表示经过如此旋转平移后误差最小
				transform.rot_x = transform.rot_x.value() + matX(0, 0);
				transform.rot_y = transform.rot_y.value() + matX(1, 0);
				transform.rot_z = transform.rot_z.value() + matX(2, 0);
				transform.pos.x() += matX(3, 0);
				transform.pos.y() += matX(4, 0);
				transform.pos.z() += matX(5, 0);

				//如果出现了nan数据，重置该数据。。
				if (isnan(transform.rot_x.value())) transform.rot_x = Angle();
				if (isnan(transform.rot_y.value())) transform.rot_y = Angle();
				if (isnan(transform.rot_z.value())) transform.rot_z = Angle();

				if (isnan(transform.pos.x())) transform.pos.x() = 0.0;
				if (isnan(transform.pos.y())) transform.pos.y() = 0.0;
				if (isnan(transform.pos.z())) transform.pos.z() = 0.0;

				//deltaR表示最小二乘拟合的精度；如果不符合要求，则iter+1,继续迭代
				//还有个deltaT;;对角度和位置分别拟合
				float deltaR = sqrt(
					pow(rad2deg(matX(0, 0)), 2) +
					pow(rad2deg(matX(1, 0)), 2) +
					pow(rad2deg(matX(2, 0)), 2));
				float deltaT = sqrt(
					pow(matX(3, 0) * 100, 2) +
					pow(matX(4, 0) * 100, 2) +
					pow(matX(5, 0) * 100, 2));

				if (deltaR < 0.1 && deltaT < 0.1) 
				{
					break;
				}

			}

		}

		//迭代完毕，算出了两坨点云间的相对运动，但他们是在这两帧点云的局部坐标系下的，
		//我们需要把它转换到世界坐标系下，因此需要进行转换
		Angle rx, ry, rz;

		//计算旋转角的累积变化量，返回rx, ry, rz
		AccumulateRotation(transformSum.rot_x,
			transformSum.rot_y,
			transformSum.rot_z,
			-transform.rot_x,
			-transform.rot_y.value() * 1.05,	//??1.05?? //求相对于原点的旋转量,垂直方向上1.05倍修正?
			-transform.rot_z,
			rx, ry, rz);

		//当前位姿映射回初始坐标系; imuShiftFromStart是位移的累积量
		Vector3 v0(transform.pos.x() - imuShiftFromStart.x(),
			transform.pos.y() - imuShiftFromStart.y(),
			transform.pos.z()*1.05 - imuShiftFromStart.z());

		Vector3 v1 = rotateZ(v0, rz);
		Vector3 v2 = rotateX(v1, rx);
		Vector3 v3 = rotateY(v2, ry);
		Vector3 trans = transformSum.pos - v3;

		//再插入IMU的旋转测量量，这里没有使用IMU，可以不用这句话。//根据IMU修正旋转量
		//PluginIMURotation(rx, ry, rz,
		//	imuPitchStart, imuYawStart, imuRollStart,
		//	imuPitchLast, imuYawLast, imuRollLast,
		//	rx, ry, rz);

		transformSum.rot_x = rx;
		transformSum.rot_y = ry;
		transformSum.rot_z = rz;
		transformSum.pos = trans;

		//返回值的第一项，总transform，Global层次的//得到世界坐标系下的转移矩阵
		OdometryValueBack.transformSum[0] = rx.value();
		OdometryValueBack.transformSum[1] = ry.value();
		OdometryValueBack.transformSum[2] = rz.value();
		OdometryValueBack.transformSum[3] = trans.x();
		OdometryValueBack.transformSum[4] = trans.y();
		OdometryValueBack.transformSum[5] = trans.z();

		int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
		//得到Pk-;也就是将k-1到K之间找到的特征点全部映射到k时刻；对应于伪代码第22行；
		//对点云的曲率比较大和比较小的点投影到扫描结束位置
		//为下次迭代做准备
		// for every less sharp point
		for (int i = 0; i < cornerPointsLessSharpNum; i++) 
		{
			TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
		}

		int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
		// for every less flat point
		for (int i = 0; i < surfPointsLessFlatNum; i++) 
		{
			TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
		}

		frameCount++;
		//点云全部点，每间隔一个点云数据相对点云最后一个点进行畸变校正？？？？
		if (frameCount >= skipFrameNum + 1) {
			int laserCloudFullResNum = laserCloudFullRes->points.size();
			for (int i = 0; i < laserCloudFullResNum; i++) {
				TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
			}
		}

		//交换cornerPointsLessSharp 与 laserCloudCornerLast； 注意，当前帧变成上一帧，位处理下一帧做准备
		//与SystemInit功能类似
		//surfPointsLessFlat 与 laserCloudSurfLast
		pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
		cornerPointsLessSharp = laserCloudCornerLast;
		laserCloudCornerLast = laserCloudTemp;

		laserCloudTemp = surfPointsLessFlat;
		surfPointsLessFlat = laserCloudSurfLast;
		laserCloudSurfLast = laserCloudTemp;

		laserCloudCornerLastNum = laserCloudCornerLast->points.size();
		laserCloudSurfLastNum = laserCloudSurfLast->points.size();
		//点足够多就构建kd-tree，否则弃用此帧，沿用上一帧数据的kd-tree
		if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
			//构建kd树，把当前特征点存入kd树种便于下次搜索； 可并行
			kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
			kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
		}

		//odometry 返回值
		OdometryValueBack.laserCloudCornerLast = laserCloudCornerLast;
		OdometryValueBack.laserCloudFullRes = laserCloudFullRes;
		OdometryValueBack.laserCloudSurfLast = laserCloudSurfLast;

	}

	return OdometryValueBack;
}