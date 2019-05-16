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
	//��ֵϵ�����㣬����ÿ��������ʱ��/��������10
	float s = 10 * (pi->intensity - int(pi->intensity));//���������10��ʵ�Ǹ�0.1��Ӧ�ģ������ǵ�����Ϊ����ʮ��λ����

	//���Բ�ֵ������ÿ�����ڵ����е����λ�ù�ϵ��������Ӧ����תƽ��ϵ��
	Angle rx = s * transform.rot_x.value();
	Angle ry = s * transform.rot_y.value();
	Angle rz = s * transform.rot_z.value();

	//��ȥλ�ƣ�����z,x,y��ת�� ��ת��ʾΪY1X2Z3
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
	// ��imu�йصĲ���������register���صģ��˴���û���õ�Imu����

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

	//����ȡֵ��������Registration���ص�ֵ���Ƶ����ص������У�������ָ�룿
	laserCloudSharpHandler(ScanValueBack.cornerPointsSharp);
	laserCloudLessSharpHandler(ScanValueBack.cornerPointsLessSharp);
	laserCloudFlatHandler(ScanValueBack.surfPointsFlat);
	laserCloudLessFlatHandler(ScanValueBack.surfPointsLessFlat);
	laserCloudFullResHandler(ScanValueBack.laserCloud);
	imuTransHandler(ScanValueBack.imuTrans);


	PointType pointSel;	//��ǰ֡�������㼯��ȡ��һ����
	PointType tripod1, tripod2, tripod3;	//���������ڽ���; �δζ�Ӧ�㣨����Ҫ�����㣩
	PointType pointProj;	//�洢ƫ����Ϣ��jacobian..���ԣ�����洢����pointSel��ȥƫ��
	PointType coeff; // ÿ���������Ӧ��Jaccobian���������Ԫ�ض�������coeffSel�У��������L - M���������ʱ��ֱ�ӵ��þ����ˡ�
	PointType pointOri;	//����L-M����ʱʹ�ã����ڴ���ĵ�

	bool isDegenerate = false;	//�ж��Ƿ����˻�
	Eigen::Matrix<float, 6, 6> matP;	//Ԥ�����P���˻�ʱʹ��
	int frameCount = skipFrameNum;		//������֡����һ�Ρ�������ÿ֡������

	std::vector<int> pointSearchInd;		//���ڽ������������
	std::vector<float> pointSearchSqDis;    //���ڽ�����������㵽�õ�ľ���


	//ͬ�����ã�ȷ��ͬʱ�յ�ͬһ�����Ƶ��������Լ�IMU��Ϣ�Ž���
	if (newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat &&
		newSurfPointsLessFlat && newLaserCloudFullRes && newImuTrans)
	{
		newCornerPointsSharp = false;
		newCornerPointsLessSharp = false;
		newSurfPointsFlat = false;
		newSurfPointsLessFlat = false;
		newLaserCloudFullRes = false;
		newImuTrans = false;

		//�Ե�һ֡����ʼ������������laserCloudCornerLast����
		if (!systemInited)
		{
			//registration�е�lesssharp�����ﶼ����cornerlast���棬����kd��
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

		//ֻ��������ĸ����㹻���ܹ����к����ƥ�乤���������Ļ��Ͳ��㣬�ؼ�֡�ĸ���
		if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
		{
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices); // �޳��쳣��
			int cornerPointsSharpNum = cornerPointsSharp->points.size(); // ��ǰʱ�������ǵ�ĸ���
			int surfPointsFlatNum = surfPointsFlat->points.size(); // ��ǰʱ������ƽ̹��ĸ���

			//��������Ϊ25�Σ��Ա�֤����Ч��
			//1���������� / ���ϵĵ���д���2������Jaccobian����3��L - M�˶�������⡣
			//L-M������ʵ���Ƿ�������С���ˣ���Gauss-Newton�Ż���һ�ָĽ���������һ���������ӣ������е�s�������Թؼ�������ΰѵ�����׼���˶����Ƶ�����ת��ΪL-M�Ż��������⡣
			//��Ҫ˼·���ǣ�����Լ������ -> Լ��������ƫ������Jaccobian���� -> L-M��⡣
			for (int iterCount = 0; iterCount < 25; iterCount++)
			{

				//process corner points; figure7(a)    cycle number: 36����1
				for (int i = 0; i < cornerPointsSharpNum; i++)
				{
					//ÿ�ε����󣬵õ��������ӵ��ܵ�ת��transform�У�Ȼ����һ�ε������µ�ת���������������
					TransformToStart(&cornerPointsSharp->points[i], &pointSel);		//���ǵ�ת�������Ƴ�ʼʱ�̣���kd������洢������һ֡������ʱ�̣�Ҳ�������ʱ�̵ĵ������ݡ�pointSelΪת�����

					//ÿ������Σ����²��������
					if (iterCount % 5 == 0)
					{
						std::vector<int> indices;
						pcl::removeNaNFromPointCloud(*laserCloudCornerLast, *laserCloudCornerLast, indices);
						//kd-tree����һ���������㣬���ص�δ��������դ���˲���һ����ص㱾���ͱȽ��٣������˲�
						kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
						int closestPointInd = -1, minPointInd2 = -1;

						//����С��25����Ϊ����Ч�����ڽ���
						if (pointSearchSqDis[0] < 25)
						{
							closestPointInd = pointSearchInd[0];
							int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity); // �������ĵ�j��������

							float pointSqDis, minPointSqDis2 = 25;
							//����һ֡����Ҵ��ڽ��㣻SCAN��ֵ����2.5��ֱ�������� �������minPointSqDis2 and minPointInd2��
							for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++)
							{
								//SCAN��ֵ����2.5��ֱ������
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

							//��ǰ�Ҵ��ڽ��㣻SCAN��ֵ����2.5��ֱ������
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
						pointSearchCornerInd1[i] = closestPointInd;  // ��ǰ���б�����������һʱ�̱����������ж�Ӧ�����ڽ��������
						pointSearchCornerInd2[i] = minPointInd2;  // ��ǰ���б�����������һʱ�̱����������ж�Ӧ�Ĵ��ڽ��������

					}

					if (pointSearchCornerInd2[i] >= 0)//���ڵ���0��˵�������㶼�ҵ��ˣ�Ĭ��Ϊ-1
					{
						//��ʽ2����㵽�߾���
						//pointSel-->i   tripod1-->j	tripod2-->l
						tripod1 = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
						tripod2 = laserCloudCornerLast->points[pointSearchCornerInd2[i]];

						// ѡ����������ΪO��kd - tree���������ΪA����һ�����������ΪB
						float x0 = pointSel.x;
						float y0 = pointSel.y;
						float z0 = pointSel.z;
						float x1 = tripod1.x;
						float y1 = tripod1.y;
						float z1 = tripod1.z;
						float x2 = tripod2.x;
						float y2 = tripod2.y;
						float z2 = tripod2.z;
						//a012=|(pointSel - tripod1) cross (pointSel - tripod2)|	cross ��ʾ���  ��ʽ��2��
						//����OA = (x0 - x1, y0 - y1, z0 - z1), ����OB = (x0 - x2, y0 - y2, z0 - z2)������AB = ��x1 - x2, y1 - y2, z1 - z2��
						//����OA OB��������(�����)Ϊ��
						//|  i      j      k  |
						//|x0-x1  y0-y1  z0-z1|
						//|x0-x2  y0-y2  z0-z2|
						//ģΪ��
						float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
							* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
							+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
							* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));
						//l12=distance of tripod1 and tripod2
						float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

						// ����[la��lb��lc] Ϊ����ld2�ֱ��x0 y0 z0��ƫ��,Ҳ���ǵ�ǰ��i  Jacobian matrix ����Ҫtransform��ƫ��
						//AB����ĵ�λ������OABƽ��ĵ�λ���������������ڸ����ϵķ�����d�ķ���
						//x�����i
						//sh: la����ld2��x0���󵼣�lb��ld2��y0���󵼣����˴����󵼴���
						float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

						//y�����j
						float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
							- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

						//z�����k
						float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
							+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

						//�㵽�ߵľ��룬d = |����OA ��� ����OB|/|AB|
						float ld2 = a012 / l12;		//�ռ��е㵽ֱ�ߵľ��룬���������е�de

						//unused
						pointProj = pointSel;
						pointProj.x -= la * ld2;
						pointProj.y -= lb * ld2;
						pointProj.z -= lc * ld2;

						//Ȩ�ؼ��㣬����Խ��Ȩ��ԽС������ԽСȨ��Խ�󣬵õ���Ȩ�ط�Χ<=1
						float  s = 1;	//��˵�е���������,ȡ1Ӧ����һ��ʼû��Ȩ��
						if (iterCount >= 5)
						{
							//5�ε���֮��ʼ����Ȩ������
							s = 1 - 1.8 * fabs(ld2);  // �㵽ֱ�߾���ԽС��������Խ��
						}
						coeff.x = s * la;
						coeff.y = s * lb;
						coeff.z = s * lc;
						coeff.intensity = s * ld2;

						// ����������ֵ(ld2 < 0.5)�������������
						if (s > 0.1 && ld2 != 0)
						{
							laserCloudOri->push_back(cornerPointsSharp->points[i]);
							coeffSel->push_back(coeff); //ÿ���������Ӧ��Jaccobian���������Ԫ�ض�������coeffSel�У��������L - M���������ʱ��ֱ�ӵ��þ����ˡ�
						}

					}
				}

				//process palanr points; figure7(b)    cycle number: 202����2
				for (int i = 0; i < surfPointsFlatNum; i++)
				{
					TransformToStart(&surfPointsFlat->points[i], &pointSel);
					//ÿ5�ε���������һ�����ڽ���
					if (iterCount % 5 == 0)
					{
						kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
						int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

						if (pointSearchSqDis[0] < 25)
						{
							closestPointInd = pointSearchInd[0];
							int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

							float pointSqDis, minPointSqDis2 = 25, minPointSqDis3 = 25;

							//�����
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
							//��ǰ��
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
						tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]]; //A��
						tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]]; //B��
						tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]]; //C��

						//pa pb pc  // ����[pa��pb��pc] = �㵽��ľ����x0 y0 z0��ƫ��
						//����AB AC��������������ˣ����õ����Ƿ�����
						//x�᷽�������i
						float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
							- (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
						//y�᷽�������j
						float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
							- (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
						float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
							- (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
						float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

						//��������ģ
						float ps = sqrt(pa * pa + pb * pb + pc * pc);
						//pa pb pcΪ�������������ϵĵ�λ����
						pa /= ps;
						pb /= ps;
						pc /= ps;
						pd /= ps;

						//�㵽��ľ��룺����OA���뷨�����ĵ�����Է�������ģ
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
							//����ԭʼ������Ӧ��ϵ��
							laserCloudOri->push_back(surfPointsFlat->points[i]);
							coeffSel->push_back(coeff);
						}
					}

				}

				//����Ӧ���ǽ⹫ʽ9����L-M�Ľ������         ��֣�L-M�Ľ��㲿�ּ�������Ҫʱ�䣬ֻ��Ҫ2��3��ʱ�����ڡ�����
				//�״���˶��������ġ������ж�Ӧ���ĵ���ֱ�ߵľ��뵽��ľ���֮�����Ȼ����Levenberg-Marquardt�㷨�������㣬�õ���֮֡��ı任�����ͨ���ۼƼ���odom
				int pointSelNum = laserCloudOri->points.size(); // ƥ�䵽�ĵ�ĸ���(�����ڶ��ٸ�Լ��)
				if (pointSelNum < 10)	//������̫��;
				{
					continue;
				}

				Eigen::Matrix<float, Eigen::Dynamic, 6> matA(pointSelNum, 6);	//n*6
				Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, pointSelNum);	//6*n  A��ת��
				Eigen::Matrix<float, 6, 6> matAtA;								//6*6
				Eigen::VectorXf matB(pointSelNum);								//n*1
				Eigen::Matrix<float, 6, 1> matAtB;								//6*1								
				Eigen::Matrix<float, 6, 1> matX;

				// ����ÿһ�������㣬����Jaccobian����  ��Ҫ�õ����Ǿ��������任��ƫ����
				// ����matA and matB
				for (int i = 0; i < pointSelNum; i++)
				{
					/**
					* ����Levenberg-Marquardt����
					* ���Ƚ�����ǰʱ��Lidar����ϵ����ȡ������������㵽ֱ��/ƽ��
					* ��Լ�����̡������Լ�������������任(3��ת+3ƽ��)��ƫ��
					* ��ʽ�μ�����(2)-(8)
					*�м�����Ӧ����м���Լ������
					*/
					pointOri = laserCloudOri->points[i];// ��ǰ����ĵ�
					coeff = coeffSel->points[i];// �õ�����Ӧ��ƫ����

					float s = 1;
					//transform����TkL(t)���洢�� �ֱ��ǽǶȺ�λ����Ϣ
					float srx = sin(s * transform.rot_x.value());
					float crx = cos(s * transform.rot_x.value());
					float sry = sin(s * transform.rot_y.value());
					float cry = cos(s * transform.rot_y.value());
					float srz = sin(s * transform.rot_z.value());
					float crz = cos(s * transform.rot_z.value());
					float tx = s * transform.pos.x();
					float ty = s * transform.pos.y();
					float tz = s * transform.pos.z();

					//loss��RT�����еĲ�����ƫ�� ��������
					//����LM���������A�����������MATX������deltaX
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

					//���룬���߾��룬�������
					float d2 = coeff.intensity;

					matA(i, 0) = arx;
					matA(i, 1) = ary;
					matA(i, 2) = arz;
					matA(i, 3) = atx;
					matA(i, 4) = aty;
					matA(i, 5) = atz;
					matB(i, 0) = -0.05 * d2;
				}
				// ��С���˼���(QR�ֽⷨ)
				matAt = matA.transpose();
				matAtA = matAt * matA;
				matAtB = matAt * matB;

				//ͨ������ֽ��������󷽳̣��ٶȱ������  matAtA*matX=matAtB����matX
				matX = matAtA.colPivHouseholderQr().solve(matAtB);

				//��һ�ε����Ž��еĲ�����Ӧ���ǽ�����ĳЩ��������ֲ�����
				if (iterCount == 0)
				{
					Eigen::Matrix<float, 1, 6> matE;	//����ֵ1*6����
					Eigen::Matrix<float, 6, 6> matV;	//��������6*6����
					Eigen::Matrix<float, 6, 6> matV2;

					// ����������������E�����������ķ��Գ���V
					Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
					matE = esolver.eigenvalues().real();
					matV = esolver.eigenvectors().real();

					matV2 = matV;

					isDegenerate = false;
					float eignThre[6] = { 10, 10, 10, 10, 10, 10 }; //����ֵȡֵ�ż�
					for (int i = 5; i >= 0; i--)//��С�������
					{
						if (matE(0, i) < eignThre[i])  ////����ֵ̫С������Ϊ���ڼ沢�����У��������˻�
						{
							for (int j = 0; j < 6; j++)//��Ӧ������������Ϊ0
							{
								matV2(i, j) = 0;
							}
							isDegenerate = true;
						}
						else {
							break;
						}
					}
					matP = matV.inverse() * matV2;//����P����
				}

				if (isDegenerate) //��������˻���ֻʹ��Ԥ�����P����
				{
					Eigen::Matrix<float, 6, 1> matX2;
					matX2 = matX;

					matX = matP * matX2;
				}

				//�����в������ۻ���ת�Ƕ���λ��
				//�������MatX��������ϢӦ����һ����תƽ��������ʾ���������תƽ�ƺ������С
				transform.rot_x = transform.rot_x.value() + matX(0, 0);
				transform.rot_y = transform.rot_y.value() + matX(1, 0);
				transform.rot_z = transform.rot_z.value() + matX(2, 0);
				transform.pos.x() += matX(3, 0);
				transform.pos.y() += matX(4, 0);
				transform.pos.z() += matX(5, 0);

				//���������nan���ݣ����ø����ݡ���
				if (isnan(transform.rot_x.value())) transform.rot_x = Angle();
				if (isnan(transform.rot_y.value())) transform.rot_y = Angle();
				if (isnan(transform.rot_z.value())) transform.rot_z = Angle();

				if (isnan(transform.pos.x())) transform.pos.x() = 0.0;
				if (isnan(transform.pos.y())) transform.pos.y() = 0.0;
				if (isnan(transform.pos.z())) transform.pos.z() = 0.0;

				//deltaR��ʾ��С������ϵľ��ȣ����������Ҫ����iter+1,��������
				//���и�deltaT;;�ԽǶȺ�λ�÷ֱ����
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

		//������ϣ������������Ƽ������˶�����������������֡���Ƶľֲ�����ϵ�µģ�
		//������Ҫ����ת������������ϵ�£������Ҫ����ת��
		Angle rx, ry, rz;

		//������ת�ǵ��ۻ��仯��������rx, ry, rz
		AccumulateRotation(transformSum.rot_x,
			transformSum.rot_y,
			transformSum.rot_z,
			-transform.rot_x,
			-transform.rot_y.value() * 1.05,	//??1.05?? //�������ԭ�����ת��,��ֱ������1.05������?
			-transform.rot_z,
			rx, ry, rz);

		//��ǰλ��ӳ��س�ʼ����ϵ; imuShiftFromStart��λ�Ƶ��ۻ���
		Vector3 v0(transform.pos.x() - imuShiftFromStart.x(),
			transform.pos.y() - imuShiftFromStart.y(),
			transform.pos.z()*1.05 - imuShiftFromStart.z());

		Vector3 v1 = rotateZ(v0, rz);
		Vector3 v2 = rotateX(v1, rx);
		Vector3 v3 = rotateY(v2, ry);
		Vector3 trans = transformSum.pos - v3;

		//�ٲ���IMU����ת������������û��ʹ��IMU�����Բ�����仰��//����IMU������ת��
		//PluginIMURotation(rx, ry, rz,
		//	imuPitchStart, imuYawStart, imuRollStart,
		//	imuPitchLast, imuYawLast, imuRollLast,
		//	rx, ry, rz);

		transformSum.rot_x = rx;
		transformSum.rot_y = ry;
		transformSum.rot_z = rz;
		transformSum.pos = trans;

		//����ֵ�ĵ�һ���transform��Global��ε�//�õ���������ϵ�µ�ת�ƾ���
		OdometryValueBack.transformSum[0] = rx.value();
		OdometryValueBack.transformSum[1] = ry.value();
		OdometryValueBack.transformSum[2] = rz.value();
		OdometryValueBack.transformSum[3] = trans.x();
		OdometryValueBack.transformSum[4] = trans.y();
		OdometryValueBack.transformSum[5] = trans.z();

		int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
		//�õ�Pk-;Ҳ���ǽ�k-1��K֮���ҵ���������ȫ��ӳ�䵽kʱ�̣���Ӧ��α�����22�У�
		//�Ե��Ƶ����ʱȽϴ�ͱȽ�С�ĵ�ͶӰ��ɨ�����λ��
		//Ϊ�´ε�����׼��
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
		//����ȫ���㣬ÿ���һ������������Ե������һ������л���У����������
		if (frameCount >= skipFrameNum + 1) {
			int laserCloudFullResNum = laserCloudFullRes->points.size();
			for (int i = 0; i < laserCloudFullResNum; i++) {
				TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
			}
		}

		//����cornerPointsLessSharp �� laserCloudCornerLast�� ע�⣬��ǰ֡�����һ֡��λ������һ֡��׼��
		//��SystemInit��������
		//surfPointsLessFlat �� laserCloudSurfLast
		pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
		cornerPointsLessSharp = laserCloudCornerLast;
		laserCloudCornerLast = laserCloudTemp;

		laserCloudTemp = surfPointsLessFlat;
		surfPointsLessFlat = laserCloudSurfLast;
		laserCloudSurfLast = laserCloudTemp;

		laserCloudCornerLastNum = laserCloudCornerLast->points.size();
		laserCloudSurfLastNum = laserCloudSurfLast->points.size();
		//���㹻��͹���kd-tree���������ô�֡��������һ֡���ݵ�kd-tree
		if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
			//����kd�����ѵ�ǰ���������kd���ֱ����´������� �ɲ���
			kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
			kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
		}

		//odometry ����ֵ
		OdometryValueBack.laserCloudCornerLast = laserCloudCornerLast;
		OdometryValueBack.laserCloudFullRes = laserCloudFullRes;
		OdometryValueBack.laserCloudSurfLast = laserCloudSurfLast;

	}

	return OdometryValueBack;
}