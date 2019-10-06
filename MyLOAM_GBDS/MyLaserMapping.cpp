#include "MyLaserMapping.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
using namespace std;
const float LaserMapping::scanPeriod = 0.1;

LaserMapping::LaserMapping() :
	laserCloudCornerLast(new pcl::PointCloud<PointType>()),
	laserCloudSurfLast(new pcl::PointCloud<PointType>()),
	laserCloudCornerStack(new pcl::PointCloud<PointType>()),
	laserCloudSurfStack(new pcl::PointCloud<PointType>()),
	laserCloudCornerStack2(new pcl::PointCloud<PointType>()),
	laserCloudSurfStack2(new pcl::PointCloud<PointType>()),
	laserCloudOri(new pcl::PointCloud<PointType>()),
	coeffSel(new pcl::PointCloud<PointType>()),
	laserCloudSurround(new pcl::PointCloud<PointType>()),
	laserCloudSurround2(new pcl::PointCloud<PointType>()),
	laserCloudCornerFromMap(new pcl::PointCloud<PointType>()),
	laserCloudSurfFromMap(new pcl::PointCloud<PointType>()),
	laserCloudFullRes(new pcl::PointCloud<PointType>())
{
	//outfile2.open("/cornerLog.txt"); outfile.open("/SurfLog.txt");
	/*num_id = 0; num_id2 = 0; timeLaserCloudCornerLast = 0; timeLaserCloudSurfLast = 0; 
	timeLaserCloudSurfLast = 0; timeLaserCloudFullRes = 0;timeLaserOdometry = 0;  */
	newLaserCloudCornerLast = false;
	newLaserCloudSurfLast = false;
	newLaserCloudFullRes = false;
	newLaserOdometry = false;
	laserCloudCenWidth = 10;	 // 邻域宽度, cm为单位
	laserCloudCenHeight = 5;	// 邻域高度, cm为单位
	laserCloudCenDepth = 10;	// 邻域深度, cm为单位
	systemInit = true;
	//初始化工作，没有这几行会崩溃
	for (int i = 0; i < laserCloudNum; i++) {
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
	}
	frameCount = stackFrameNum - 1;
	mapFrameCount = mapFrameNum - 1;
}

void LaserMapping::laserCloudCornerLastHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudCornerLast2)
{


	laserCloudCornerLast->clear();
	*laserCloudCornerLast = *laserCloudCornerLast2;

	newLaserCloudCornerLast = true;
}

void LaserMapping::laserCloudSurfLastHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudSurfLast2)
{


	laserCloudSurfLast->clear();
	*laserCloudSurfLast = *laserCloudSurfLast2;

	newLaserCloudSurfLast = true;
}

void LaserMapping::laserCloudFullResHandler(const pcl::PointCloud<PointType>::Ptr& laserCloudFullRes2)
{


	laserCloudFullRes->clear();
	*laserCloudFullRes = *laserCloudFullRes2;
	newLaserCloudFullRes = true;
}

void LaserMapping::laserOdometryHandler(const float* transformSum2)
{
	transformSum.rot_x = transformSum2[0];
	transformSum.rot_y = transformSum2[1];
	transformSum.rot_z = transformSum2[2];

	transformSum.pos.x() = transformSum2[3];
	transformSum.pos.y() = transformSum2[4];
	transformSum.pos.z() = transformSum2[5];

	newLaserOdometry = true;
}

//基于匀速模型，根据上次微调的结果和odometry这次与上次计算的结果，猜测一个新的世界坐标系的转换矩阵transformTobeMapped
void LaserMapping::transformAssociateToMap()
{
	//*befmap里面记录的实际上就是里程计节点累加的结果 tobemap里面存的是正在迭代求解的对于全局的坐标变换
	// aftmap是在tobemap迭代结束以后，拿到tobemap的值（如果有Imu的话还会有个加权）
	// 所以一个新的tansformsum进来，他跟之前存下来的befmap做对比，其实就是得到了新的一帧相对于之前处理过
	// 那一帧过程中的相对运动，再把这个相对运动加到afrmap上，其实就得到的初始的相对与全局的坐标变换，作为tobemap进入迭代

	//首先计算 \bar{T}_{k}^{W}(t_{k+1}) ，而后根据 \bar{T}_{k}^{W}(t_{k+1}) 将测量得到的点坐标转换到世界坐标系{W}下。
	//两个坐标转换函数都采用欧拉角表示姿态旋转
	Vector3 v0 = transformBefMapped.pos - transformSum.pos;		//transformBefMapped：前一帧或者说上一次地图中雷达的位姿；；此处相减，得到当前帧雷达在全局地图中的位姿
	Vector3 v1 = rotateY(v0, -(transformSum.rot_y));
	Vector3 v2 = rotateX(v1, -(transformSum.rot_x));
	//平移增量
	transformIncre.pos = rotateZ(v2, -(transformSum.rot_z));

	//bc:before current		bl:before last		al:after last	
	//计算
	float sbcx = transformSum.rot_x.sin();
	float cbcx = transformSum.rot_x.cos();
	float sbcy = transformSum.rot_y.sin();
	float cbcy = transformSum.rot_y.cos();
	float sbcz = transformSum.rot_z.sin();
	float cbcz = transformSum.rot_z.cos();

	float sblx = transformBefMapped.rot_x.sin();
	float cblx = transformBefMapped.rot_x.cos();
	float sbly = transformBefMapped.rot_y.sin();
	float cbly = transformBefMapped.rot_y.cos();
	float sblz = transformBefMapped.rot_z.sin();
	float cblz = transformBefMapped.rot_z.cos();

	float salx = transformAftMapped.rot_x.sin();
	float calx = transformAftMapped.rot_x.cos();
	float saly = transformAftMapped.rot_y.sin();
	float caly = transformAftMapped.rot_y.cos();
	float salz = transformAftMapped.rot_z.sin();
	float calz = transformAftMapped.rot_z.cos();

	float srx = -sbcx * (salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz)
		- cbcx * sbcy*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
			- calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
		- cbcx * cbcy*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
			- calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx);
	transformTobeMapped.rot_x = -asin(srx);

	float srycrx = sbcx * (cblx*cblz*(caly*salz - calz * salx*saly)
		- cblx * sblz*(caly*calz + salx * saly*salz) + calx * saly*sblx)
		- cbcx * cbcy*((caly*calz + salx * saly*salz)*(cblz*sbly - cbly * sblx*sblz)
			+ (caly*salz - calz * salx*saly)*(sbly*sblz + cbly * cblz*sblx) - calx * cblx*cbly*saly)
		+ cbcx * sbcy*((caly*calz + salx * saly*salz)*(cbly*cblz + sblx * sbly*sblz)
			+ (caly*salz - calz * salx*saly)*(cbly*sblz - cblz * sblx*sbly) + calx * cblx*saly*sbly);
	float crycrx = sbcx * (cblx*sblz*(calz*saly - caly * salx*salz)
		- cblx * cblz*(saly*salz + caly * calz*salx) + calx * caly*sblx)
		+ cbcx * cbcy*((saly*salz + caly * calz*salx)*(sbly*sblz + cbly * cblz*sblx)
			+ (calz*saly - caly * salx*salz)*(cblz*sbly - cbly * sblx*sblz) + calx * caly*cblx*cbly)
		- cbcx * sbcy*((saly*salz + caly * calz*salx)*(cbly*sblz - cblz * sblx*sbly)
			+ (calz*saly - caly * salx*salz)*(cbly*cblz + sblx * sbly*sblz) - calx * caly*cblx*sbly);
	transformTobeMapped.rot_y = atan2(srycrx / transformTobeMapped.rot_x.cos(),
		crycrx / transformTobeMapped.rot_x.cos());

	float srzcrx = (cbcz*sbcy - cbcy * sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
		- calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
		- (cbcy*cbcz + sbcx * sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
			- calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
		+ cbcx * sbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
	float crzcrx = (cbcy*sbcz - cbcz * sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz * sblx*sbly)
		- calx * salz*(cbly*cblz + sblx * sbly*sblz) + cblx * salx*sbly)
		- (sbcy*sbcz + cbcy * cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly * sblx*sblz)
			- calx * calz*(sbly*sblz + cbly * cblz*sblx) + cblx * cbly*salx)
		+ cbcx * cbcz*(salx*sblx + calx * cblx*salz*sblz + calx * calz*cblx*cblz);
	transformTobeMapped.rot_z = atan2(srzcrx / transformTobeMapped.rot_x.cos(),
		crzcrx / transformTobeMapped.rot_x.cos());

	Vector3 v3;
	v1 = rotateZ(transformIncre.pos, transformTobeMapped.rot_z);
	v2 = rotateX(v1, transformTobeMapped.rot_x);
	v3 = rotateY(v2, transformTobeMapped.rot_y);
	transformTobeMapped.pos = transformAftMapped.pos - v3;

}

//特征点转换到世界坐标系
void LaserMapping::pointAssociateToMap(PointType const * const pi, PointType * const po)
{
	Vector3 v1 = rotateZ(*pi, transformTobeMapped.rot_z);
	Vector3 v2 = rotateX(v1, transformTobeMapped.rot_x);
	Vector3 v3 = rotateY(v2, transformTobeMapped.rot_y);
	v3 += transformTobeMapped.pos;

	po->x = v3.x();
	po->y = v3.y();
	po->z = v3.z();

	po->intensity = pi->intensity;
}

//特征点从世界坐标系转换会雷达坐标系
void LaserMapping::pointAssociateTobeMapped(PointType const * const pi, PointType * const po)
{
	//是上面一个函数的逆过程
	Vector3 v0 = Vector3(*pi) - transformTobeMapped.pos;
	Vector3 v1 = rotateY(v0, -transformTobeMapped.rot_y);
	Vector3 v2 = rotateX(v1, -transformTobeMapped.rot_x);
	Vector3 v3 = rotateZ(v2, -transformTobeMapped.rot_z);

	po->x = v3.x();
	po->y = v3.y();
	po->z = v3.z();

	po->intensity = pi->intensity;
}

void LaserMapping::transformUpdate()
{
	transformBefMapped = transformSum;
	transformAftMapped = transformTobeMapped;
}

LaserMappingBack LaserMapping::LaserMappingHandler(const LaserOdometryBack& OdometryValueBack)
{
	LaserMappingBack MappingBackValue;

	PointType pointSel;		//point selected
	PointType pointOri;		//original point, 雷达坐标系中的点 //构建L-M矩阵时使用，正在处理的点
	PointType pointProj;    //存储偏导信息，jacobian..不对，这个存储的是pointSel减去偏导
	PointType coeff; // 每个特征点对应的Jaccobian矩阵的三个元素都保存在coeffSel中，后面采用L - M方法解算的时候直接调用就行了。

	Eigen::Matrix3f matA1;		//特征角点的矩阵A 3*3
	Eigen::Matrix<float, 1, 3> matD1;	//A1分解得到的特征值
	Eigen::Matrix3f matV1;	//A1分解得到的特征向量
	Eigen::Matrix<float, 5, 3> matA0; //特征平面点的矩阵A
	Eigen::Matrix<float, 5, 1> matB0;
	Eigen::Vector3f matX0;

	matA0.setZero();
	matB0.setConstant(-1);	//全-1；  点到面的距离尽量优化，使之趋于0
	matX0.setZero();

	matA1.setZero();
	matD1.setZero();
	matV1.setZero();

	bool isDegenerate = false;		//判断是否出现退化？比如误差反而变大了
	Eigen::Matrix<float, 6, 6> matP;

	//清空旧数据，读入新数据
	laserCloudCornerLastHandler(OdometryValueBack.laserCloudCornerLast);
	laserCloudSurfLastHandler(OdometryValueBack.laserCloudSurfLast);
	laserCloudFullResHandler(OdometryValueBack.laserCloudFullRes);
	laserOdometryHandler(OdometryValueBack.transformSum);



	int frameCount = stackFrameNum - 1; // initialize as 0
	int mapFrameCount = mapFrameNum - 1; // initialize as 4

	time3 = clock();

	//总if,一直到最后  如果这些数据都得到了更新，就更新地图
	if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry)
	{
		frameCount++;
		//控制跳帧数，>=这里实际并没有跳帧，只取>或者增大stackFrameNum才能实现相应的跳帧处理
		//这个条件语句中先预测当前点云在世界坐标系中的转换矩阵Twc; 然后将特征点都转换到世界坐标系，存放在laserCloudCornerStack2，laserCloudSurfStack2中
		if (frameCount >= stackFrameNum)
		{
			//预测将当前点云转换到地图的全局坐标系中的转换矩阵，yes，得到transformTobeMapped
			transformAssociateToMap();

			//将特征点也转换到地图坐标中
			int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
			for (int i = 0; i < laserCloudCornerLastNum; i++)
			{
				pointAssociateToMap(&laserCloudCornerLast->points[i], &pointSel);
				laserCloudCornerStack2->push_back(pointSel);
			}

			int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
			for (int i = 0; i < laserCloudSurfLastNum; i++)
			{
				pointAssociateToMap(&laserCloudSurfLast->points[i], &pointSel);
				laserCloudSurfStack2->push_back(pointSel);
			}
		}

		//第二个总if，跟第一个if条件一样。。。
		if (frameCount >= stackFrameNum)
		{
			//找当前估计的Lidar位姿 \bar{T}_{k}^{W}(t_{k+1}) 属于哪个子cube。I、J、K对应了cube的索引。
			frameCount = 0;
			PointType pointOnYAxis;  // 当前Lidar坐标系{L}y轴上的一点(0,10,0)，在后面
			pointOnYAxis.x = 0.0;
			pointOnYAxis.y = 10.0;
			pointOnYAxis.z = 0.0;
			pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);  // 转到世界坐标系{W}下

			// cube中心位置索引；；；预测的Twc所属cube的索引；；
			//立方体中点在世界坐标系下的（原点）位置
			//过半取一（以50米进行四舍五入的效果），由于数组下标只能为正数，而地图可能建立在原点前后，因此
			//每一维偏移一个laserCloudCenWidth（该值会动态调整，以使得数组利用最大化，初始值为该维数组长度1/2）的量
			int centerCubeI = int((transformTobeMapped.pos.x() + 25.0) / 50.0) + laserCloudCenWidth;
			int centerCubeJ = int((transformTobeMapped.pos.y() + 25.0) / 50.0) + laserCloudCenHeight;
			int centerCubeK = int((transformTobeMapped.pos.z() + 25.0) / 50.0) + laserCloudCenDepth;

			//由于计算机求余是向零取整，为了不使（-50.0,50.0）求余后都向零偏移，当被求余数为负数时求余结果统一向左偏移一个单位，也即减一
			if (transformTobeMapped.pos.x() + 25.0 < 0) centerCubeI--;
			if (transformTobeMapped.pos.y() + 25.0 < 0) centerCubeJ--;
			if (transformTobeMapped.pos.z() + 25.0 < 0) centerCubeK--;

			//如果取到的子cube在整个大cube的边缘则将点对应的cube的索引向中心方向挪动一个单位，这样做主要是截取边沿cube。
			/*通过6个while循环，使I,J,K分别处于一个范围之内； 也就是3到(max-3);  3是因为之后查找邻近点在5*5*5里面查找。。
			具体见循环后注释
			//以下部分其实就在剪切地图，每次距离变远以后，就整个cube平移。最开始激光雷达位于坐标原点，
			// 实际上对应cube索引是第10 5 10。 当激光雷达位置距离边缘小于3个cube以后，整个大cube向
			//该方向平移一个cube（或几个，直到距离边缘大于3个为止），同理中间位置的索引也跟着变，这样
			// 下一次计算的时候不会出现问题。也就是平移过一次以后，如果位于原点的值在进行计算对应的就不是
			//10 5 10了，有可能是9 5 10。     ====整个大cube的范围是21,11,21

			version3: 雷达一开始在0,0,0处，则对应的cude中心I,J,K为10,5,10；
			假如雷达沿I走了475米，那么雷达的位置就是475,0,0；对应的I,J,K为20,5,10；
			这个时候20>21-3,太靠近边缘cube了，于是整个21*11*21大cube需要沿着I平移，【其实就是大cube的中心平移】
			直至雷达的位置不在边缘范围

			可是，移动的是啥数据？是剪切的地图数据，全地图太大了，在雷达附近剪切地图用于匹配就可以了、

			移动的是cube...把所有cube前后左右上下移动，使lidar所在cube前后左右上下至少有3个cube,为了后面临近点搜索做准备
			*/
			while (centerCubeI < 3)  //???这个3是什么意思---》雷达位置范围大于3小于(max-3),
			{
				// 将点的指针向中心方向平移   I->WIDTH   J->HEIGHT   K->DEPTH
				// 分割出各个小cube
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						//将数组内所有数据右移，类似循环移位。 先把最后一个值拿出来，然后依次移位，再把这最后一个值存进去
						//version2： 不是循环移位，就是单纯的右移一位； 而把最后一个数据拿出来赋给开始点，我觉得这样总比赋0好、、、
						int i = laserCloudWidth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i >= 1; i--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();

					}
				}
				//应该是先加laserCloudCenWidth，然后根据前面计算centerCubeI的公式，centerCubeI也需要加1
				centerCubeI++;
				laserCloudCenWidth++;
			}

			while (centerCubeI >= laserCloudWidth - 3) {
				for (int j = 0; j < laserCloudHeight; j++) {
					for (int k = 0; k < laserCloudDepth; k++) {
						int i = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i < laserCloudWidth - 1; i++) {
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI--;
				laserCloudCenWidth--;
			}

			while (centerCubeJ < 3) {
				for (int i = 0; i < laserCloudWidth; i++) {
					for (int k = 0; k < laserCloudDepth; k++) {
						int j = laserCloudHeight - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j >= 1; j--) {
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight*k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ++;
				laserCloudCenHeight++;
			}

			while (centerCubeJ >= laserCloudHeight - 3) {
				for (int i = 0; i < laserCloudWidth; i++) {
					for (int k = 0; k < laserCloudDepth; k++) {
						int j = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j < laserCloudHeight - 1; j++) {
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight*k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ--;
				laserCloudCenHeight--;
			}

			while (centerCubeK < 3) {
				for (int i = 0; i < laserCloudWidth; i++) {
					for (int j = 0; j < laserCloudHeight; j++) {
						int k = laserCloudDepth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k >= 1; k--) {
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k - 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK++;
				laserCloudCenDepth++;
			}

			while (centerCubeK >= laserCloudDepth - 3) {
				for (int i = 0; i < laserCloudWidth; i++) {
					for (int j = 0; j < laserCloudHeight; j++) {
						int k = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k < laserCloudDepth - 1; k++) {
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight*(k + 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK--;
				laserCloudCenDepth--;
			}

#ifdef LOAM_DEBUG
			std::cout << "I = " << centerCubeI << "   " << "J = " << centerCubeJ << "   " << "K = " << centerCubeK << "   " << std::endl;
#endif
			//以上部分其实就在剪切地图，每次距离变远以后，就整个cube平移。最开始激光雷达位于坐标原点，
			// 实际上对应cube索引是第10 5 10。 当激光雷达位置距离边缘小于3个cube以后，整个大cube向
			//该方向平移一个cube（或几个，直到距离边缘大于3个为止），同理中间位置的索引也跟着变，这样
			// 下一次计算的时候不会出现问题。也就是平移过一次以后，如果位于原点的值在进行计算对应的就不是
			//10 5 10了，有可能是9 5 10。     ====整个大cube的范围是21,11,21

			//version2: 我现在觉得应该是避免雷达位于cube的边沿，也就是让雷达的位置在（3，max-3）之中
			//tips: 很多变量可能看起来没有头绪，但其实这是个迭代进行的，变量的赋值啥的可能在处理完之后。。。

			//centerCube 对应的是预测激光雷达在的位置对应的索引,单位是cube

			//处理完毕边沿点，接下来就是在取到的子cube的5*5*5的邻域内找对应的配准点了。。。。论文里面说是10cm*10cm*10cm
			//isInLaserFOV： 没搞懂这个变量的作用
			//在每一维附近5个cube(前2个，后2个，中间1个)里进行查找（前后250米范围内，总共500米范围），三个维度总共125个cube
			//在这125个cube里面进一步筛选在视域范围内的cube
			int laserCloudValidNum = 0;		//雷达视域内的cube
			int laserCloudSurroundNum = 0;	//雷达周围的cube 5*5*5
			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
				{
					for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
					{
						//满足这些条件才在剪切的地图中
						if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight && k >= 0 && k < laserCloudDepth)
						{
							// 计算子cube对应的点坐标  version2:
							// NOTE: 由于i j k均为整数，坐标取值为中心点坐标
							float centerX = 50.0 * (i - laserCloudCenWidth);	//对应前面的int centerCubeI = int((transformTobeMapped.pos.x() + 25.0) / 50.0) + laserCloudCenWidth;
							float centerY = 50.0 * (j - laserCloudCenHeight);
							float centerZ = 50.0 * (k - laserCloudCenDepth);

							// 取邻近的8个点坐标，都可能是雷达的世界坐标系位置。
							//判断一下该配准点是否属于当前Lidar的可视范围内，可以根据余弦公式对距离范围进行推导。
							//根据代码中的式子，只要点在x轴±60°的范围内都认为是FOV中的点(作者这么做是因为Lidar里程计的估计结果抬不准确了，只能概略的取一个较大的范围)。
							bool isInLaserFOV = false;		//是否在雷达的视域中？？？？
							for (int ii = -1; ii <= 1; ii += 2)
							{
								for (int jj = -1; jj <= 1; jj += 2)
								{
									for (int kk = -1; kk <= 1; kk += 2)
									{
										//上下左右八个顶点坐标
										Vector3 corner;
										corner.x() = centerX + 25.0 * ii;
										corner.y() = centerY + 25.0 * jj;
										corner.z() = centerZ + 25.0 * kk;

										Vector3 point_on_axis(pointOnYAxis.x, pointOnYAxis.y, pointOnYAxis.z);
										//原点到顶点距离的平方和 //transformTobeMapped.pos - corner,将顶点转换到世界坐标系中？
										float squaredSide1 = (transformTobeMapped.pos - corner).squaredNorm();
										//pointOnYAxis到顶点距离的平方和
										float squaredSide2 = (point_on_axis - corner).squaredNorm();

										float check1 = 100.0 + squaredSide1 - squaredSide2
											- 10.0 * sqrt(3.0) * sqrt(squaredSide1);

										float check2 = 100.0 + squaredSide1 - squaredSide2
											+ 10.0 * sqrt(3.0) * sqrt(squaredSide1);

										//if |100 + squaredSide1 - squaredSide2| < 10.0 * sqrt(3.0) * sqrt(squaredSide1)
										if (check1 < 0 && check2 > 0) 
										{
											isInLaserFOV = true;
										}
									}
								}
							}

							//判断一下该cube是否属于当前Lidar的可视范围内，可以根据余弦公式对距离范围进行推导。
							//根据代码中的式子，只要点在x轴±60°的范围内都认为是FOV中的点(作者这么做是因为Lidar里程计的估计结果抬不准确了，只能概略的取一个较大的范围)。
							//于是我们就得到了在当前Lidar位置的邻域内有效的地图特征点。
							//记住视域范围内的cube索引，匹配用
							if (isInLaserFOV) {
								laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j
									+ laserCloudWidth * laserCloudHeight * k;
								laserCloudValidNum++;
							}
							////记住附近所有cube的索引，显示用   这个存储的也只是indice?对的，每个索引就是一个cube
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j
								+ laserCloudWidth * laserCloudHeight * k;
							laserCloudSurroundNum++;

						}
					}
				}
			}//删选雷达可视范围内的点

			//所以，我们就不需要对庞大的所有地图点云进行处理了，只需要处理这些邻域cube内的地图特征点即可，可以节省大量的运算资源。
			//为了保证当前帧的点云足够平滑，还对点云进行了滤波处理。
			laserCloudCornerFromMap->clear();
			laserCloudSurfFromMap->clear();
			//构建特征点地图，查找匹配使用
			for (int i = 0; i < laserCloudValidNum; i++) {
				//test the bunber of points
				//int size_array = laserCloudCornerArray[laserCloudValidInd[i]]->size();
				//if (size_array != 0)
				//{
				//	int t = 0;
				//}
				*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
				*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
			}
			int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();  // 有效的特征边上的点的个数
			int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();     // 有效的特征面上的点的个数

			//// 将世界坐标系下的当前帧特征点转到当前Lidar坐标系下,
			// 为啥要转回雷达坐标系再进行滤波？直接滤波不行吗；  
			//这里是为了转换回去，后面直接使用更新后的转换矩阵来将点转到世界坐标系。
			int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();  // 所有特征边上的点的个数
			for (int i = 0; i < laserCloudCornerStackNum2; i++) {
				pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
			}

			int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();  // 所有特征面上的点的个数
			for (int i = 0; i < laserCloudSurfStackNum2; i++) {
				pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
			}

			// 对所有当前帧特征点进行滤波处理 5cm*5cm*5cm
			pcl::VoxelGrid<PointType> downSizeFilterCorner;
			downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

			pcl::VoxelGrid<PointType> downSizeFilterSurf;
			downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);	//10cm*10cm*10cm

			laserCloudCornerStack->clear();
			downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
			downSizeFilterCorner.filter(*laserCloudCornerStack);
			//*laserCloudCornerStack = *laserCloudCornerStack2;

			int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

			laserCloudSurfStack->clear();
			downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
			downSizeFilterSurf.filter(*laserCloudSurfStack);

			//*laserCloudSurfStack = *laserCloudSurfStack2;
			int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

			laserCloudCornerStack2->clear();
			laserCloudSurfStack2->clear();

			/*做完这些工作以后，我们就有了在当前Lidar所在位置附近的所有地图特征点以及当前帧的点云特征点，
			后面的工作就是怎么把这两坨点匹配在一起啦！于是我们再次拿出KD树，来寻找最邻近的5个点。
			对点云协方差矩阵进行主成分分析：
			*若这五个点分布在直线上，协方差矩阵的特征值包含一个元素显著大于其余两个，与该特征值相关的特征向量表示所处直线的方向；
			*若这五个点分布在平面上，协方差矩阵的特征值存在一个显著小的元素，与该特征值相关的特征向量表示所处平面的法线方向。
			因此我们可以很轻易的根据特征向量找到直线上两点从而利用论文中点到直线的距离公式构建优化问题(对优化问题有疑问的同学请参考上一篇文章)。
			平面特征也是相同的思路。完成了优化问题的构建之后就可以对它进行求解了，求解方法还是L-M迭代。这部分代码与laserOdometry部分的几乎一致
			*/

			//laserCloudCornerFromMapNum
			//	laserCloudSurfFromMapNum
			//	laserCloudCornerStackNum
			//	laserCloudSurfStackNum
			Eigen::Matrix<int,1, 4> store_point_size;
			store_point_size << laserCloudCornerFromMapNum, laserCloudSurfFromMapNum, laserCloudCornerStackNum, laserCloudSurfStackNum;


			std::ofstream fout_point_size;
			fout_point_size.open("dataset_size.txt", std::ios::app);//在文件末尾追加写入
			fout_point_size << store_point_size << std::endl;//每次写完一个矩阵以后换行
			fout_point_size.close();

#ifdef USE_GBDS_ON
			std::cout << "USE_GBDS_ON:                        ";
#endif
#ifdef USE_KDTREE_ON
			std::cout << "USE_KDTREE_ON: " << std::endl;
#endif
			if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
			{
				time_start = clock();

#ifdef USE_GBDS_ON
/*********************************************************************************/
//build the grid-based data structure of corner_last
			MyPointCloudToArray(laserCloudCornerFromMap, map_corner_data_set, map_corner_useful_data_set_size);
			GetMaxMin(map_corner_data_set, map_corner_data_max_min, map_corner_useful_data_set_size);

			//SplitSubSpace(map_corner_useful_data_set_size, map_corner_data_max_min, map_corner_x_split_array, map_corner_y_split_array, map_corner_z_split_array, map_corner_split_array_size);
			SplitSubSpacePrecise(map_corner_useful_data_set_size, map_corner_data_max_min, map_corner_x_split_array, map_corner_y_split_array, map_corner_z_split_array, map_corner_split_array_size);
			DataClassify(map_corner_data_set, map_corner_useful_data_set_size, map_corner_x_split_array, map_corner_y_split_array, map_corner_z_split_array, map_corner_split_array_size, map_corner_sub_sets, map_corner_sub_sets_size);

			/*********************************************************************************/

/*********************************************************************************/
//build the grid-based data structure of surf_last
			MyPointCloudToArray(laserCloudSurfFromMap, map_surf_data_set, map_surf_useful_data_set_size);
			GetMaxMin(map_surf_data_set, map_surf_data_max_min, map_surf_useful_data_set_size);

			//SplitSubSpace(map_surf_useful_data_set_size, map_surf_data_max_min, map_surf_x_split_array, map_surf_y_split_array, map_surf_z_split_array, map_surf_split_array_size);
			SplitSubSpacePrecise(map_surf_useful_data_set_size, map_surf_data_max_min, map_surf_x_split_array, map_surf_y_split_array, map_surf_z_split_array, map_surf_split_array_size);
			DataClassify(map_surf_data_set, map_surf_useful_data_set_size, map_surf_x_split_array, map_surf_y_split_array, map_surf_z_split_array, map_surf_split_array_size, map_surf_sub_sets, map_surf_sub_sets_size);
/*********************************************************************************/
#endif
#ifdef USE_KDTREE_ON
				nanoflann::KdTreeFLANN<PointType> kdtreeCornerFromMap;
				nanoflann::KdTreeFLANN<PointType> kdtreeSurfFromMap;

				//将地图中的特征点存入KD树中
				kdtreeCornerFromMap.setInputCloud(laserCloudCornerFromMap);
				kdtreeSurfFromMap.setInputCloud(laserCloudSurfFromMap);
#endif // USE_GBDS_ON


				//重新定义查找点的个数和距离
				std::vector<int> pointSearchInd;
				std::vector<float> pointSearchSqDis;
				pointSearchInd.resize(5);
				pointSearchSqDis.resize(5);

				//跟odometry类似的迭代，然后求Jacobian Matrix,然后使用L-M方法来求解
				for (int iterCount = 0; iterCount < 10; iterCount++)
				{
					time1 = clock();

					laserCloudOri->clear();
					coeffSel->clear();

					num_corner = 0;
					num_surf = 0;

					//主要的耗时都在这两个循环里面
					//此循环使用LM方法匹配角点特征点***这里的匹配主要是求得那个偏导。便于后面综合牛顿迭代
					for (int i = 0; i < laserCloudCornerStackNum; i++)
					{
						//laserCloudCornerStack是雷达本地坐标系下，voxel滤波之后的特征点集
						pointOri = laserCloudCornerStack->points[i];
						pointAssociateToMap(&pointOri, &pointSel);

#ifdef USE_GBDS_ON
/*********************************************************************************/
						//std::vector<int> record_ind = pointSearchInd;
						//std::vector<float> record_dis = pointSearchSqDis;

						struct ThreeDimPoint my_point_sel;
						MyPointXYZIToThreeDimPoint(pointSel, my_point_sel);

						type_point map_corner_nearest_distance[k_query_set_size];
						int map_corner_nearest_index[k_query_set_size];
						SearchKNearestNeighbors(5, my_point_sel, map_corner_data_set, map_corner_sub_sets, map_corner_sub_sets_size, map_corner_x_split_array, map_corner_y_split_array, map_corner_z_split_array, map_corner_split_array_size, map_corner_nearest_index, map_corner_nearest_distance);

						for (int i = 0; i < 5; i++)
						{
							pointSearchInd[i] = map_corner_nearest_index[i];
							pointSearchSqDis[i] = map_corner_nearest_distance[i];
						}						
/*********************************************************************************/						
#endif
#ifdef USE_KDTREE_ON
//kd树中查找与指定点最近的5个点，按距离从小到大排序，返回点的索引与距离
						kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
#endif




						//因为有了这个判断，所以就算没找到最近邻的五个点问题也不大。。可以考虑将精度设为1.、、
						if (pointSearchSqDis[4] < 1.0)//5个点中最大距离不超过1才处理
						{
							

							Vector3 vc(0, 0, 0);
							//将五个最近点的坐标加和求平均
							for (int j = 0; j < 5; j++) 
							{
								vc.x() += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
								vc.y() += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
								vc.z() += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
							}
							vc /= 5.0;

							Eigen::Matrix3f mat_a;
							mat_a.setZero();

							for (int j = 0; j < 5; j++) {
								//减去均值，去中心化，然后构建matA1；  Point to line 用得是类似arun的方法
								float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - vc.x();
								float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - vc.y();
								float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - vc.z();

								mat_a(0, 0) += ax * ax;
								mat_a(0, 1) += ax * ay;
								mat_a(0, 2) += ax * az;
								mat_a(1, 1) += ay * ay;
								mat_a(1, 2) += ay * az;
								mat_a(2, 2) += az * az;
							}
							matA1 = mat_a / 5.0;

							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
							matD1 = esolver.eigenvalues().real();
							matV1 = esolver.eigenvectors().real();
							//若这五个点分布在直线上，协方差矩阵的特征值包含一个元素显著大于其余两个，
							//与该特征值相关的特征向量表示所处直线的方向
							if (matD1(0, 0) > 3 * matD1(0, 1))
							{
								float x0 = pointSel.x;
								float y0 = pointSel.y;
								float z0 = pointSel.z;
								//由于这些点都分布在直线上，为了方便直接取该点在直线左右0.1取两个临近点
								//然后后面是点线距离，求导，与ODOMETRY一样
								float x1 = vc.x() + 0.1 * matV1(0, 0);
								float y1 = vc.y() + 0.1 * matV1(0, 1);
								float z1 = vc.z() + 0.1 * matV1(0, 2);
								float x2 = vc.x() - 0.1 * matV1(0, 0);
								float y2 = vc.y() - 0.1 * matV1(0, 1);
								float z2 = vc.z() - 0.1 * matV1(0, 2);


								float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
									* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
									+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
									* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
									+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
									* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

								float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

								float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
									+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

								float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
									- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

								float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
									+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

								float ld2 = a012 / l12;
								//unused
								pointProj = pointSel;
								pointProj.x -= la * ld2;
								pointProj.y -= lb * ld2;
								pointProj.z -= lc * ld2;

								// float s = 1 - 0.9 * fabs(ld2)/sqrt(sqrt(pointSel.x * pointSel.x
								// + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
								//权重系数计算，就是距离近分配高一点的权重， 距离远分配低一点的权重，权重体现在后面牛顿迭代中，也就是论文里面说的LM方法
								float s = 1 - 0.9 * fabs(ld2);


								coeff.x = s * la;
								coeff.y = s * lb;
								coeff.z = s * lc;
								coeff.intensity = s * ld2;


								//用于测试，使得intensity等于实际的loss
								// coeff.x =  la;
								// coeff.y =  lb;
								// coeff.z =  lc;
								// coeff.intensity =  ld2;

								if (s > 0.1) //距离足够小才使用，权重太小了就认为是不可靠的数据，这相当于一个去噪
								{
									laserCloudOri->push_back(pointOri);
									coeffSel->push_back(coeff);
									num_corner++;
									//   outfile2<< iterCount<<","<<num_corner++<<","<<s<<endl;

								}
							}
						}
					}

					//主要的耗时都在这两个循环里面
					//此循环使用LM方法匹配平面特征点，得到偏导信息
					//平面方程为AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
					//其中(X,Y,Z)是点的坐标, 对应这里的mat_a0, 是已知数
					//A/D, B/D, C/D 对应mat_x0, 是待求的值
					//等式右边的-1对应mat_b0
					for (int i = 0; i < laserCloudSurfStackNum; i++)
					{
						pointOri = laserCloudSurfStack->points[i];
						pointAssociateToMap(&pointOri, &pointSel);

#ifdef USE_GBDS_ON
/*********************************************************************************/
						//std::vector<int> record_ind = pointSearchInd;
						//std::vector<float> record_dis = pointSearchSqDis;

						struct ThreeDimPoint my_point_sel;
						MyPointXYZIToThreeDimPoint(pointSel, my_point_sel);

						type_point map_surf_nearest_distance[k_query_set_size];
						int map_surf_nearest_index[k_query_set_size];
						SearchKNearestNeighbors(5, my_point_sel, map_surf_data_set, map_surf_sub_sets, map_surf_sub_sets_size, map_surf_x_split_array, map_surf_y_split_array, map_surf_z_split_array, map_surf_split_array_size, map_surf_nearest_index, map_surf_nearest_distance);

						for (int i = 0; i < 5; i++)
						{
							pointSearchInd[i] = map_surf_nearest_index[i];
							pointSearchSqDis[i] = map_surf_nearest_distance[i];
						}
/*********************************************************************************/
#endif
#ifdef USE_KDTREE_ON
						kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
#endif // USE_GBDS_ON

						if (pointSearchSqDis[4] < 1.0)
						{
							//构建五个最近点的坐标矩阵
							for (int j = 0; j < 5; j++) 
							{
								matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
								matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
								matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
							}
							//求解matA0*matX0=matB0  //求解 (A/D)X+(B/D)Y+(C/D)Z = -1 中的 A/D, B/D, C/D 
							matX0 = matA0.colPivHouseholderQr().solve(matB0);

							float pa = matX0(0, 0);
							float pb = matX0(1, 0);
							float pc = matX0(2, 0);

							//对应之前的-1, (A/D)X+(B/D)Y+(C/D)Z = -1 <=> (A/D)X+(B/D)Y+(C/D)Z +1 = 0
							float pd = 1;

							//ps为平面法向量的模
							//求得(pa, pb, pc)为法向量, 模为1, pd为平面到原点的距离
							float ps = sqrt(pa * pa + pb * pb + pc * pc);
							pa /= ps;
							pb /= ps;
							pc /= ps;
							pd /= ps;

							//确定拟合出的平面与用来拟合的点都足够接近, 表示平面拟合的有效性
							bool planeValid = true;
							for (int j = 0; j < 5; j++)
							{
								//判断平面拟合的误差
								if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
									pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
									pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
								{
									//平面无效, 本点匹配失败, 此点不对后续的优化贡献约束
									planeValid = false;
									break;
								}
							}

							if (planeValid)
							{
								//点到平面的距离, 参考点到平面距离公式, 分母部分为1
								float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

								//unused
								pointProj = pointSel;
								pointProj.x -= pa * pd2;
								pointProj.y -= pb * pd2;
								pointProj.z -= pc * pd2;

								float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
									+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));

								// float s = 1 - 0.9 * fabs(pd2);

								//(pa, pb, pc)是法向量, 点到平面的垂线, 也是点面距离相对与点坐标的偏导, 详见文章的公式推导部分.
								//...绝了。。点面距离也是跟法向量有关的，刚刚好。。
								coeff.x = s * pa;
								coeff.y = s * pb;
								coeff.z = s * pc;
								coeff.intensity = s * pd2;

								//用于测试，使得intensity等于实际的loss
								// coeff.x =  pa;
								// coeff.y =  pb;
								// coeff.z =  pc;
								// coeff.intensity = pd2;


								if (s > 0.1) 
								{
									laserCloudOri->push_back(pointOri);
									coeffSel->push_back(coeff);
									num_surf++;
									//  outfile<< iterCount<<","<<num_surf++<<","<<s<<endl;
								}
							}

						}
					}

#ifdef MY_SHOW_MAP_TIME_PROFILE
					time2 = clock();
					time_last2 = (double)(time2 - time1) / CLOCKS_PER_SEC;
					std::cout << "Search tree and calculate Time = " << time_last2 << std::endl;
#endif
					//至此，jacobian矩阵构建完成。 
					//角点与平面点放一起优化。使用牛顿迭代
					float srx = transformTobeMapped.rot_x.sin();
					float crx = transformTobeMapped.rot_x.cos();
					float sry = transformTobeMapped.rot_y.sin();
					float cry = transformTobeMapped.rot_y.cos();
					float srz = transformTobeMapped.rot_z.sin();
					float crz = transformTobeMapped.rot_z.cos();

					int laserCloudSelNum = laserCloudOri->points.size();
					if (laserCloudSelNum < 50) {
						continue;
					}

					Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
					Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
					Eigen::Matrix<float, 6, 6> matAtA;
					Eigen::VectorXf matB(laserCloudSelNum);
					Eigen::VectorXf matAtB;
					Eigen::VectorXf matX;

					for (int i = 0; i < laserCloudSelNum; i++)
					{
						pointOri = laserCloudOri->points[i];
						coeff = coeffSel->points[i];

						//链式求导第二步，具体求导过程可看知乎相关分析
						float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
							+ (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
							+ (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

						float ary = ((cry*srx*srz - crz * sry)*pointOri.x
							+ (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
							+ ((-cry * crz - srx * sry*srz)*pointOri.x
								+ (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

						float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
							+ (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
							+ ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

						matA(i, 0) = arx;
						matA(i, 1) = ary;
						matA(i, 2) = arz;
						matA(i, 3) = coeff.x;
						matA(i, 4) = coeff.y;
						matA(i, 5) = coeff.z;
						matB(i, 0) = -coeff.intensity;
					}

					matAt = matA.transpose();
					matAtA = matAt * matA;
					matAtB = matAt * matB;
					matX = matAtA.colPivHouseholderQr().solve(matAtB);

					//第一次迭代判断是否退化，跟odometry一样
					//具体描述见Loam作者Zhang J的<<On Degeneracy of Optimization-based State Estimation Problems>>
					//大概方法是通过Jacobian的eigenvalue判断哪个分量的约束不足, 不更新那个方向上的迭代
					//特征值分解
					if (iterCount == 0)
					{
						Eigen::Matrix<float, 1, 6> matE;
						Eigen::Matrix<float, 6, 6> matV;
						Eigen::Matrix<float, 6, 6> matV2;

						Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
						matE = esolver.eigenvalues().real();
						matV = esolver.eigenvectors().real();

						matV2 = matV;


						isDegenerate = false;
						float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
						//特征值从小往大判断，如果有小于阈值，则认为发生退化
						for (int i = 5; i >= 0; i--) 
						{
							if (matE(0, i) < eignThre[i]) 
							{
								for (int j = 0; j < 6; j++) 
								{
									matV2(i, j) = 0;

								}
								isDegenerate = true;
							}
							else 
							{
								break;
							}
						}
						matP = matV.inverse() * matV2;
					}

					if (isDegenerate) 
					{
						Eigen::Matrix<float, 6, 1> matX2(matX);
						matX = matP * matX2;
					}
					//积累每次的调整量
					transformTobeMapped.rot_x += matX(0, 0);
					transformTobeMapped.rot_y += matX(1, 0);
					transformTobeMapped.rot_z += matX(2, 0);
					transformTobeMapped.pos.x() += matX(3, 0);
					transformTobeMapped.pos.y() += matX(4, 0);
					transformTobeMapped.pos.z() += matX(5, 0);


					float deltaR = sqrt(
						pow(rad2deg(matX(0, 0)), 2) +
						pow(rad2deg(matX(1, 0)), 2) +
						pow(rad2deg(matX(2, 0)), 2));
					float deltaT = sqrt(
						pow(matX(3, 0) * 100, 2) +
						pow(matX(4, 0) * 100, 2) +
						pow(matX(5, 0) * 100, 2));

					//旋转平移增量足够小就停止迭代
					if (deltaR < 0.05 && deltaT < 0.05 /*&& matX.at<float>(0, 0) < 0.002*/) 
					{
						break;
					}

				}//迭代25次
				 //迭代结束更新相关的转移矩阵，也就是transformBefMapped与transformAftMapped
				transformUpdate();

				time_end = clock();
				time_last_compare = (double)(time_end - time_start) / CLOCKS_PER_SEC;
				std::cout << "Total mapping Time = " << time_last_compare << std::endl;

				time_store.push_back(time_last_compare);
				
			}//如果特征点个数足够

			//存储上面这个if用的总时间
#ifdef USE_GBDS_ON
			std::ofstream fout1;
			fout1.open("gbds_time_store.txt", std::ios::app);//在文件末尾追加写入
			for (int i = 0; i < time_store.size(); i++)
			{
				fout1 << time_store[i] << std::endl;//每次写完一个矩阵以后换行
			}
			fout1.close();
			time_store.clear();
#endif
#ifdef USE_KDTREE_ON
			std::ofstream fout2;
			fout2.open("kdtree_time_store.txt", std::ios::app);//在文件末尾追加写入
			for (int i = 0; i < time_store.size(); i++)
			{
				fout2 << time_store[i] << std::endl;//每次写完一个矩阵以后换行
			}
			fout2.close();
			time_store.clear();
#endif



#ifdef BOTH_KDTREE_AND_GBDS
			std::cout << "BOTH_KDTREE_AND_GBDS USE_KDTREE_ON:      " ;
			if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
			{
				time_start = clock();


				nanoflann::KdTreeFLANN<PointType> kdtreeCornerFromMap;
				nanoflann::KdTreeFLANN<PointType> kdtreeSurfFromMap;

				//将地图中的特征点存入KD树中
				kdtreeCornerFromMap.setInputCloud(laserCloudCornerFromMap);
				kdtreeSurfFromMap.setInputCloud(laserCloudSurfFromMap);

				//重新定义查找点的个数和距离
				std::vector<int> pointSearchInd;
				std::vector<float> pointSearchSqDis;
				pointSearchInd.resize(5);
				pointSearchSqDis.resize(5);

				//跟odometry类似的迭代，然后求Jacobian Matrix,然后使用L-M方法来求解
				for (int iterCount = 0; iterCount < 10; iterCount++)
				{
					time1 = clock();

					laserCloudOri->clear();
					coeffSel->clear();

					num_corner = 0;
					num_surf = 0;

					//主要的耗时都在这两个循环里面
					//此循环使用LM方法匹配角点特征点***这里的匹配主要是求得那个偏导。便于后面综合牛顿迭代
					for (int i = 0; i < laserCloudCornerStackNum; i++)
					{
						//laserCloudCornerStack是雷达本地坐标系下，voxel滤波之后的特征点集
						pointOri = laserCloudCornerStack->points[i];
						pointAssociateToMap(&pointOri, &pointSel);

						//kd树中查找与指定点最近的5个点，按距离从小到大排序，返回点的索引与距离
						kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

						//因为有了这个判断，所以就算没找到最近邻的五个点问题也不大。。可以考虑将精度设为1.、、
						if (pointSearchSqDis[4] < 1.0)//5个点中最大距离不超过1才处理
						{


							Vector3 vc(0, 0, 0);
							//将五个最近点的坐标加和求平均
							for (int j = 0; j < 5; j++)
							{
								vc.x() += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
								vc.y() += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
								vc.z() += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
							}
							vc /= 5.0;

							Eigen::Matrix3f mat_a;
							mat_a.setZero();

							for (int j = 0; j < 5; j++) {
								//减去均值，去中心化，然后构建matA1；  Point to line 用得是类似arun的方法
								float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - vc.x();
								float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - vc.y();
								float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - vc.z();

								mat_a(0, 0) += ax * ax;
								mat_a(0, 1) += ax * ay;
								mat_a(0, 2) += ax * az;
								mat_a(1, 1) += ay * ay;
								mat_a(1, 2) += ay * az;
								mat_a(2, 2) += az * az;
							}
							matA1 = mat_a / 5.0;

							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
							matD1 = esolver.eigenvalues().real();
							matV1 = esolver.eigenvectors().real();
							//若这五个点分布在直线上，协方差矩阵的特征值包含一个元素显著大于其余两个，
							//与该特征值相关的特征向量表示所处直线的方向
							if (matD1(0, 0) > 3 * matD1(0, 1))
							{
								float x0 = pointSel.x;
								float y0 = pointSel.y;
								float z0 = pointSel.z;
								//由于这些点都分布在直线上，为了方便直接取该点在直线左右0.1取两个临近点
								//然后后面是点线距离，求导，与ODOMETRY一样
								float x1 = vc.x() + 0.1 * matV1(0, 0);
								float y1 = vc.y() + 0.1 * matV1(0, 1);
								float z1 = vc.z() + 0.1 * matV1(0, 2);
								float x2 = vc.x() - 0.1 * matV1(0, 0);
								float y2 = vc.y() - 0.1 * matV1(0, 1);
								float z2 = vc.z() - 0.1 * matV1(0, 2);


								float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
									* ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
									+ ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
									* ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
									+ ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
									* ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

								float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

								float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
									+ (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

								float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
									- (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

								float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
									+ (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

								float ld2 = a012 / l12;
								//unused
								pointProj = pointSel;
								pointProj.x -= la * ld2;
								pointProj.y -= lb * ld2;
								pointProj.z -= lc * ld2;

								// float s = 1 - 0.9 * fabs(ld2)/sqrt(sqrt(pointSel.x * pointSel.x
								// + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
								//权重系数计算，就是距离近分配高一点的权重， 距离远分配低一点的权重，权重体现在后面牛顿迭代中，也就是论文里面说的LM方法
								float s = 1 - 0.9 * fabs(ld2);


								coeff.x = s * la;
								coeff.y = s * lb;
								coeff.z = s * lc;
								coeff.intensity = s * ld2;


								//用于测试，使得intensity等于实际的loss
								// coeff.x =  la;
								// coeff.y =  lb;
								// coeff.z =  lc;
								// coeff.intensity =  ld2;

								if (s > 0.1) //距离足够小才使用，权重太小了就认为是不可靠的数据，这相当于一个去噪
								{
									laserCloudOri->push_back(pointOri);
									coeffSel->push_back(coeff);
									num_corner++;
									//   outfile2<< iterCount<<","<<num_corner++<<","<<s<<endl;

								}
							}
						}
					}

					//主要的耗时都在这两个循环里面
					//此循环使用LM方法匹配平面特征点，得到偏导信息
					//平面方程为AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
					//其中(X,Y,Z)是点的坐标, 对应这里的mat_a0, 是已知数
					//A/D, B/D, C/D 对应mat_x0, 是待求的值
					//等式右边的-1对应mat_b0
					for (int i = 0; i < laserCloudSurfStackNum; i++)
					{
						pointOri = laserCloudSurfStack->points[i];
						pointAssociateToMap(&pointOri, &pointSel);

						kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

						if (pointSearchSqDis[4] < 1.0)
						{
							//构建五个最近点的坐标矩阵
							for (int j = 0; j < 5; j++)
							{
								matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
								matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
								matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
							}
							//求解matA0*matX0=matB0  //求解 (A/D)X+(B/D)Y+(C/D)Z = -1 中的 A/D, B/D, C/D 
							matX0 = matA0.colPivHouseholderQr().solve(matB0);

							float pa = matX0(0, 0);
							float pb = matX0(1, 0);
							float pc = matX0(2, 0);

							//对应之前的-1, (A/D)X+(B/D)Y+(C/D)Z = -1 <=> (A/D)X+(B/D)Y+(C/D)Z +1 = 0
							float pd = 1;

							//ps为平面法向量的模
							//求得(pa, pb, pc)为法向量, 模为1, pd为平面到原点的距离
							float ps = sqrt(pa * pa + pb * pb + pc * pc);
							pa /= ps;
							pb /= ps;
							pc /= ps;
							pd /= ps;

							//确定拟合出的平面与用来拟合的点都足够接近, 表示平面拟合的有效性
							bool planeValid = true;
							for (int j = 0; j < 5; j++)
							{
								//判断平面拟合的误差
								if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
									pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
									pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
								{
									//平面无效, 本点匹配失败, 此点不对后续的优化贡献约束
									planeValid = false;
									break;
								}
							}

							if (planeValid)
							{
								//点到平面的距离, 参考点到平面距离公式, 分母部分为1
								float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

								//unused
								pointProj = pointSel;
								pointProj.x -= pa * pd2;
								pointProj.y -= pb * pd2;
								pointProj.z -= pc * pd2;

								float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
									+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));

								// float s = 1 - 0.9 * fabs(pd2);

								//(pa, pb, pc)是法向量, 点到平面的垂线, 也是点面距离相对与点坐标的偏导, 详见文章的公式推导部分.
								//...绝了。。点面距离也是跟法向量有关的，刚刚好。。
								coeff.x = s * pa;
								coeff.y = s * pb;
								coeff.z = s * pc;
								coeff.intensity = s * pd2;

								//用于测试，使得intensity等于实际的loss
								// coeff.x =  pa;
								// coeff.y =  pb;
								// coeff.z =  pc;
								// coeff.intensity = pd2;


								if (s > 0.1)
								{
									laserCloudOri->push_back(pointOri);
									coeffSel->push_back(coeff);
									num_surf++;
									//  outfile<< iterCount<<","<<num_surf++<<","<<s<<endl;
								}
							}

						}
					}

#ifdef MY_SHOW_MAP_TIME_PROFILE
					time2 = clock();
					time_last2 = (double)(time2 - time1) / CLOCKS_PER_SEC;
					std::cout << "Search tree and calculate Time = " << time_last2 << std::endl;
#endif
					//至此，jacobian矩阵构建完成。 
					//角点与平面点放一起优化。使用牛顿迭代
					float srx = transformTobeMapped.rot_x.sin();
					float crx = transformTobeMapped.rot_x.cos();
					float sry = transformTobeMapped.rot_y.sin();
					float cry = transformTobeMapped.rot_y.cos();
					float srz = transformTobeMapped.rot_z.sin();
					float crz = transformTobeMapped.rot_z.cos();

					int laserCloudSelNum = laserCloudOri->points.size();
					if (laserCloudSelNum < 50) {
						continue;
					}

					Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
					Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
					Eigen::Matrix<float, 6, 6> matAtA;
					Eigen::VectorXf matB(laserCloudSelNum);
					Eigen::VectorXf matAtB;
					Eigen::VectorXf matX;

					for (int i = 0; i < laserCloudSelNum; i++)
					{
						pointOri = laserCloudOri->points[i];
						coeff = coeffSel->points[i];

						//链式求导第二步，具体求导过程可看知乎相关分析
						float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
							+ (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
							+ (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

						float ary = ((cry*srx*srz - crz * sry)*pointOri.x
							+ (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
							+ ((-cry * crz - srx * sry*srz)*pointOri.x
								+ (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

						float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
							+ (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
							+ ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

						matA(i, 0) = arx;
						matA(i, 1) = ary;
						matA(i, 2) = arz;
						matA(i, 3) = coeff.x;
						matA(i, 4) = coeff.y;
						matA(i, 5) = coeff.z;
						matB(i, 0) = -coeff.intensity;
					}

					matAt = matA.transpose();
					matAtA = matAt * matA;
					matAtB = matAt * matB;
					matX = matAtA.colPivHouseholderQr().solve(matAtB);

					//第一次迭代判断是否退化，跟odometry一样
					//具体描述见Loam作者Zhang J的<<On Degeneracy of Optimization-based State Estimation Problems>>
					//大概方法是通过Jacobian的eigenvalue判断哪个分量的约束不足, 不更新那个方向上的迭代
					//特征值分解
					if (iterCount == 0)
					{
						Eigen::Matrix<float, 1, 6> matE;
						Eigen::Matrix<float, 6, 6> matV;
						Eigen::Matrix<float, 6, 6> matV2;

						Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
						matE = esolver.eigenvalues().real();
						matV = esolver.eigenvectors().real();

						matV2 = matV;


						isDegenerate = false;
						float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
						//特征值从小往大判断，如果有小于阈值，则认为发生退化
						for (int i = 5; i >= 0; i--)
						{
							if (matE(0, i) < eignThre[i])
							{
								for (int j = 0; j < 6; j++)
								{
									matV2(i, j) = 0;

								}
								isDegenerate = true;
							}
							else
							{
								break;
							}
						}
						matP = matV.inverse() * matV2;
					}

					if (isDegenerate)
					{
						Eigen::Matrix<float, 6, 1> matX2(matX);
						matX = matP * matX2;
					}
					//积累每次的调整量
					transformTobeMapped.rot_x += matX(0, 0);
					transformTobeMapped.rot_y += matX(1, 0);
					transformTobeMapped.rot_z += matX(2, 0);
					transformTobeMapped.pos.x() += matX(3, 0);
					transformTobeMapped.pos.y() += matX(4, 0);
					transformTobeMapped.pos.z() += matX(5, 0);


					float deltaR = sqrt(
						pow(rad2deg(matX(0, 0)), 2) +
						pow(rad2deg(matX(1, 0)), 2) +
						pow(rad2deg(matX(2, 0)), 2));
					float deltaT = sqrt(
						pow(matX(3, 0) * 100, 2) +
						pow(matX(4, 0) * 100, 2) +
						pow(matX(5, 0) * 100, 2));

					//旋转平移增量足够小就停止迭代
					if (deltaR < 0.05 && deltaT < 0.05 /*&& matX.at<float>(0, 0) < 0.002*/)
					{
						break;
					}

				}//迭代25次
				 //迭代结束更新相关的转移矩阵，也就是transformBefMapped与transformAftMapped
				transformUpdate();

				time_end = clock();
				time_last_compare = (double)(time_end - time_start) / CLOCKS_PER_SEC;
				std::cout << "Total mapping Time = " << time_last_compare << std::endl << std::endl;

				time_store.push_back(time_last_compare);

			}//如果特征点个数足够

			 //存储上面这个if用的总时间

			std::ofstream fout2;
			fout2.open("kdtree_time_store.txt", std::ios::app);//在文件末尾追加写入
			for (int i = 0; i < time_store.size(); i++)
			{
				fout2 << time_store[i] << std::endl;//每次写完一个矩阵以后换行
			}
			fout2.close();
			time_store.clear();

#endif

			//截止到这里，我们就完成了当前帧点云与地图点云的配准，并对Lidar里程计的运动估计结果进行完了优化。
			//更新完成后，我们还需要将当前帧扫描得到的特征点云封装在不同的cube中，并在地图数组中保存。

			//将corner points按距离（比例尺缩小）归入相应的立方体
			std::vector<int> cubeIndStore;
			std::vector<PointType> cubeIndStorePoint;
			cubeIndStore.clear();
			cubeIndStorePoint.clear();
			for (int i = 0; i < laserCloudCornerStackNum; i++) 
			{
				//转移到世界坐标系
				pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);
				//按50的比例尺缩小，四舍五入，偏移laserCloudCen*的量，计算索引
				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0) cubeI--;
				if (pointSel.y + 25.0 < 0) cubeJ--;
				if (pointSel.z + 25.0 < 0) cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth) 
				{
					//只挑选-laserCloudCenWidth * 50.0 < point.x < laserCloudCenWidth * 50.0范围内的点，y和z同理
					//可以认为是只挑选一个cube范围的当前帧点存入地图中
					//按照尺度放进不同的组，每个组的点数量各异
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudCornerArray[cubeInd]->push_back(pointSel);

					//test, 判断是否会重复，，然而结果是会出现很多重复。。。
					//这不是重复，而是一个array中有很多点。。。。。
					//for (int i = 0; i <cubeIndStore.size(); i++)
					//{
					//	if (cubeInd == cubeIndStore[i])
					//	{
					//		std::cout << "repeat data indice:" << cubeInd << std::endl;
					//		std::cout << "OLD data " << cubeIndStorePoint[i] << std::endl;
					//		std::cout << "new data " << pointSel << std::endl;
					//	}
					//}
					//cubeIndStore.push_back(cubeInd);
					//cubeIndStorePoint.push_back(pointSel);
				}
			}
			//将surf points按距离（比例尺缩小）归入相应的立方体
			for (int i = 0; i < laserCloudSurfStackNum; i++) 
			{
				//使用更新后的转换矩阵将特征点转换到世界坐标系中
				pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);
				//surfCloudForRANSAC->push_back(pointSel);
				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0) cubeI--;
				if (pointSel.y + 25.0 < 0) cubeJ--;
				if (pointSel.z + 25.0 < 0) cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth) 
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudSurfArray[cubeInd]->push_back(pointSel);
				}
			}

			//特征点下采样
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				int ind = laserCloudValidInd[i];

				laserCloudCornerArray2[ind]->clear();
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
				downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

				laserCloudSurfArray2[ind]->clear();
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
				downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);
				
				//Array与Array2交换，即滤波后自我更新
				pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
				laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
				laserCloudCornerArray2[ind] = laserCloudTemp;

				laserCloudTemp = laserCloudSurfArray[ind];
				laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
				laserCloudSurfArray2[ind] = laserCloudTemp;
			}

			//最后就是将各种信息发布出去了。这里需要说明的是，为了保证运行效率环境点云每mapFrameNum帧发布一次。
			mapFrameCount++;
			if (mapFrameCount >= mapFrameNum)
			{
				mapFrameCount = 0;

				laserCloudSurround2->clear();
				for (int i = 0; i < laserCloudSurroundNum; i++)
				{
					int ind = laserCloudSurroundInd[i];
					*laserCloudSurround2 += *laserCloudCornerArray[ind];
					*laserCloudSurround2 += *laserCloudSurfArray[ind];
				}

				laserCloudSurround->clear();
				downSizeFilterCorner.setInputCloud(laserCloudSurround2);
				downSizeFilterCorner.filter(*laserCloudSurround);


				//雷达周围的点，角点与平面点都存进去了
				MappingBackValue.laserCloudSurround = laserCloudSurround;


				// outfile<<num_id2<<",cloud"<<std::endl;
				//num_id2++;//？？
			}

			int laserCloudFullResNum = laserCloudFullRes->points.size();
			for (int i = 0; i < laserCloudFullResNum; i++) {
				pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
			}
			//将当前雷达扫描到的所有的点，转换到世界坐标系后存入MappingBackValue.laserCloudFullRes
			MappingBackValue.laserCloudFullRes = laserCloudFullRes;

			MappingBackValue.transformAftMapped[0] = transformAftMapped.rot_x.value();
			MappingBackValue.transformAftMapped[1] = transformAftMapped.rot_y.value();
			MappingBackValue.transformAftMapped[2] = transformAftMapped.rot_z.value();
			MappingBackValue.transformAftMapped[3] = transformAftMapped.pos.x();
			MappingBackValue.transformAftMapped[4] = transformAftMapped.pos.y();
			MappingBackValue.transformAftMapped[5] = transformAftMapped.pos.z();

			// outfile<<num_id<<",odometry"<<std::endl;
			//num_id++;

		}//第二个总if，跟第一个if条件一样。。。
	}//总if

#ifdef MY_SHOW_MAP_TIME_PROFILE
	time4 = clock();
	time_last3 = (double)(time4 - time3) / CLOCKS_PER_SEC;
	std::cout << "Total mapping Time = " << time_last3 << std::endl;
#endif

	return MappingBackValue;
}