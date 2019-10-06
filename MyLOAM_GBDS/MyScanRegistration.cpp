#include "MyScanRegistration.h"
const double ScanRegistration::scanPeriod = 0.1;	//激光雷达扫描频率

//类的初始化函数
ScanRegistration::ScanRegistration()
{
	systemInited = false;
	imuPointerFront = 0; imuPointerLast = -1;

	imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
	imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

	imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
	imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

	imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
	imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

	imuShiftFromStartXCur = 0; imuShiftFromStartYCur = 0; imuShiftFromStartZCur = 0;
	imuVeloFromStartXCur = 0; imuVeloFromStartYCur = 0; imuVeloFromStartZCur = 0;

}
ScanRegistrationBack ScanRegistration::ScanRegistrationHandle(const PointCloud& laserCloudIn1)
{
	// N_SCANS=16; FOR VLP16 RADAR
	std::vector<int> scanStartInd(N_SCANS, 0);	//每条线的做操作的起始点索引与结束点索引
	std::vector<int> scanEndInd(N_SCANS, 0);

	PointCloud laserCloudIn;
	laserCloudIn.clear();
	laserCloudIn = laserCloudIn1;		//copy cloud data from input parameter  +=。。？？
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);	//remove nan points from cloud points

	int cloudSize = laserCloudIn.points.size();		//点云中点的个数
	//雷达点云pcap一帧不一定是一圈，可能只有1/5圈。。。
	//lidar scan开始点的旋转角,atan2范围[-pi,+pi],计算旋转角时取负号是因为velodyne是顺时针旋转
	float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);	//atan范围是[-pi,pi]
	//lidar scan结束点的旋转角，加2*pi使点云旋转周期为2*pi;  [pi,3pi]
	float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
		laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

	//结束方位角与开始方位角差值控制在(PI,3*PI)范围，允许lidar不是一个圆周扫描
	//正常情况下在这个范围内：pi < endOri - startOri < 3*pi，异常则修正
	//PCAP 包里的数据应该被特殊处理过，进不了这两个判断，每次开始点都是-pi/2,结束点3pi/2。刚好一个圆周。。。
	if (endOri - startOri > 3 * M_PI) {
		endOri -= 2 * M_PI;
	}
	else if (endOri - startOri < M_PI) {
		endOri += 2 * M_PI;
	}


	PointT point;
	int count = cloudSize;	//计数点云有效点的个数
	bool halfPassed = false;  // judge whether the point's z/x angle over pi； if over, compare with end_ori.
	std::vector<PointCloud> laserCloudScans(N_SCANS);
	// Calculate all SCANID of all points, and calculate intensity by SCANID+reltime
	for (int i = 0; i < cloudSize; i++)
	{
		/*imu为x轴向前,y轴向左,z轴向上的右手坐标系，
		velodyne lidar被安装为x轴向前, y轴向左, z轴向上的右手坐标系，
		scanRegistration会把两者通过交换坐标轴，都统一到z轴向前, x轴向左, y轴向上的右手坐标系
		，这是J.Zhang的论文里面使用的坐标系
		交换后：R = Ry(yaw)*Rx(pitch)*Rz(roll)*/
		point.x = laserCloudIn.points[i].y;
		point.y = laserCloudIn.points[i].z;
		point.z = laserCloudIn.points[i].x;

		float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI; //得到竖直角，也可以说是俯仰角
		int scanID;
		int roundedAngle = int(angle + (angle<0.0 ? -0.5 : +0.5));	//用int截断，相当于四舍五入角度
		// 角度大于零，由小到大划入偶数线（0->16）；角度小于零，由大到小划入奇数线(15->1);这是因为雷达的线排序就是这样排序的。。。
		//但这个代码是角度是正的就直接分给线号； 角度是负的就取反分给线号。。。
		//我明白了。。这个scanID只是为了按高度排好；便于计算以及显示
		if (roundedAngle > 0)
		{
			scanID = roundedAngle;
		}
		else
		{
			scanID = roundedAngle + (N_SCANS - 1);
		}
		if (scanID > (N_SCANS - 1) || scanID < 0) // 将16线以外的杂点剔除
		{
			count--;
			continue;

		}
		//std::cout << "angle: " << angle << std::endl << "ScanID: " << scanID << std::endl;

		//为啥是-atan2而不是atan2;顺时针旋转
		float ori = -atan2(point.x, point.z); 
		// z/x 竖直方向与x轴的夹角,之后的操作为调整此角度范围。。。错了，y才是竖直方向；这个是为了判断reltime
		//貌似是判断第一个处理的点有没有过半；过半了与endori比较； 稍微减小一点误差
		//而且只要变成true,只能等下一帧才能再变回false
		if (!halfPassed) {
			// adjust the ori to [startOri-1/2pi,startOri+3/2pi]
			if (ori < startOri - M_PI / 2) {
				ori += 2 * M_PI;
			}
			else if (ori > startOri + M_PI * 3 / 2) {
				ori -= 2 * M_PI;
			}

			if (ori - startOri > M_PI) {
				halfPassed = true;
			}
		}
		else {
			// [endOri - 3pi/2 ,endOri+pi/2]
			ori += 2 * M_PI;

			if (ori < endOri - M_PI * 3 / 2) {
				ori += 2 * M_PI;
			}
			else if (ori > endOri + M_PI / 2) {
				ori -= 2 * M_PI;
			}
		}

		//-0.5 < relTime < 1.5（点旋转的角度与整个周期旋转角度的比率, 即点云中点的相对时间）
		float relTime = (ori - startOri) / (endOri - startOri);
		//点强度=线号+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
		point.intensity = scanID + scanPeriod * relTime;

		//region for IMU process

		laserCloudScans[scanID].push_back(point);

	}
	cloudSize = count;	//排完序后点云中的点的个数

	PointCloud::Ptr PointCloudOrdered(new PointCloud());	//按线聚类之后的点集
	for (int i = 0; i < N_SCANS; i++) {
		*PointCloudOrdered += laserCloudScans[i];//所有被分层的点,一共count个点（不能被分层的点已经被去掉）
	}

	//求每个点的曲率，每条线前后五个点不要；
	int scanCount = -1;
	for (int i = 5; i < cloudSize - 5; i++)
	{
		float diffX = PointCloudOrdered->points[i - 5].x + PointCloudOrdered->points[i - 4].x
			+ PointCloudOrdered->points[i - 3].x + PointCloudOrdered->points[i - 2].x
			+ PointCloudOrdered->points[i - 1].x - 10 * PointCloudOrdered->points[i].x
			+ PointCloudOrdered->points[i + 1].x + PointCloudOrdered->points[i + 2].x
			+ PointCloudOrdered->points[i + 3].x + PointCloudOrdered->points[i + 4].x
			+ PointCloudOrdered->points[i + 5].x;
		float diffY = PointCloudOrdered->points[i - 5].y + PointCloudOrdered->points[i - 4].y
			+ PointCloudOrdered->points[i - 3].y + PointCloudOrdered->points[i - 2].y
			+ PointCloudOrdered->points[i - 1].y - 10 * PointCloudOrdered->points[i].y
			+ PointCloudOrdered->points[i + 1].y + PointCloudOrdered->points[i + 2].y
			+ PointCloudOrdered->points[i + 3].y + PointCloudOrdered->points[i + 4].y
			+ PointCloudOrdered->points[i + 5].y;
		float diffZ = PointCloudOrdered->points[i - 5].z + PointCloudOrdered->points[i - 4].z
			+ PointCloudOrdered->points[i - 3].z + PointCloudOrdered->points[i - 2].z
			+ PointCloudOrdered->points[i - 1].z - 10 * PointCloudOrdered->points[i].z
			+ PointCloudOrdered->points[i + 1].z + PointCloudOrdered->points[i + 2].z
			+ PointCloudOrdered->points[i + 3].z + PointCloudOrdered->points[i + 4].z
			+ PointCloudOrdered->points[i + 5].z;
		cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;

		//相关数组初始化工作
		cloudSortInd[i] = i;		//有效曲率点的索引
		cloudNeighborPicked[i] = 0;
		cloudNeighborPickedForCorner[i] = 0;
		cloudLabel[i] = 0;

		//startind and endind，每条线只进入这个if一次。为后面分成6等分做准备
		if (int(PointCloudOrdered->points[i].intensity) != scanCount)
		{
			scanCount = int(PointCloudOrdered->points[i].intensity);

			if (scanCount > 0 && scanCount < N_SCANS) {
				scanStartInd[scanCount] = i + 5;
				scanEndInd[scanCount - 1] = i - 5;//每一线前五个点和后五个点不要
			}
		}
	}
	//额外处理最开始点与最终点
	scanStartInd[0] = 5;
	scanEndInd.back() = cloudSize - 5;

	/*遍历所有点（除去前五个和后六个），判断该点及其周边点是否可以作为特征点位：
	当某点及其后点间的距离平方大于某阈值a（说明这两点有一定距离），且两向量夹角小于某阈值b时（夹角小就可能存在遮挡），
	将其一侧的临近6个点设为不可标记为特征点的点；若某点到其前后两点的距离均大于c倍的该点深度，
	则该点判定为不可标记特征点的点（入射角越小，点间距越大，即激光发射方向与投射到的平面越近似水平）。*/
	// 前面已经将所有点按线分好了之后再导入
	//先处理角点，edge points
	for (int i = 5; i < cloudSize - 6; i++)
	{
		//前后两个点的距离平方
		float diffX = PointCloudOrdered->points[i + 1].x - PointCloudOrdered->points[i].x;
		float diffY = PointCloudOrdered->points[i + 1].y - PointCloudOrdered->points[i].y;
		float diffZ = PointCloudOrdered->points[i + 1].z - PointCloudOrdered->points[i].z;
		float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

		//前后两个点距离超过阈值才可能是特征点
		if (diff > 0.1)
		{
			//点到激光雷达的距离
			float depth1 = sqrt(PointCloudOrdered->points[i].x * PointCloudOrdered->points[i].x +
				PointCloudOrdered->points[i].y * PointCloudOrdered->points[i].y +
				PointCloudOrdered->points[i].z * PointCloudOrdered->points[i].z);

			float depth2 = sqrt(PointCloudOrdered->points[i + 1].x * PointCloudOrdered->points[i + 1].x +
				PointCloudOrdered->points[i + 1].y * PointCloudOrdered->points[i + 1].y +
				PointCloudOrdered->points[i + 1].z * PointCloudOrdered->points[i + 1].z);

			/* 针对论文中(b)情况 */
			if (depth1 > depth2)
			{
				//如果深度不一样的话，将前后两个点拉到同一个球面上计算曲率
				diffX = PointCloudOrdered->points[i + 1].x - PointCloudOrdered->points[i].x * depth2 / depth1;
				diffY = PointCloudOrdered->points[i + 1].y - PointCloudOrdered->points[i].y * depth2 / depth1;
				diffZ = PointCloudOrdered->points[i + 1].z - PointCloudOrdered->points[i].z * depth2 / depth1;// ？？构建了一个等腰三角形的底向量

				//depth2可以理解成动态阈值，越远阈值越大；；其实就是看夹角
				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
					// 根据等腰三角形性质，这一判断threshold=0.1实际表示X[i]向量与X[i+1]的夹角小于5.732度
					// cloudNeighborPicked 是考虑一个特征点周围不能再设置成特征约束的判断标志位
					cloudNeighborPicked[i - 5] = 1;
					cloudNeighborPicked[i - 4] = 1;
					cloudNeighborPicked[i - 3] = 1;
					cloudNeighborPicked[i - 2] = 1;
					cloudNeighborPicked[i - 1] = 1;
					cloudNeighborPicked[i] = 1;
				}
			}
			else
			{
				diffX = PointCloudOrdered->points[i + 1].x * depth1 / depth2 - PointCloudOrdered->points[i].x;
				diffY = PointCloudOrdered->points[i + 1].y * depth1 / depth2 - PointCloudOrdered->points[i].y;
				diffZ = PointCloudOrdered->points[i + 1].z * depth1 / depth2 - PointCloudOrdered->points[i].z;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1)
				{
					cloudNeighborPicked[i + 1] = 1;
					cloudNeighborPicked[i + 2] = 1;
					cloudNeighborPicked[i + 3] = 1;
					cloudNeighborPicked[i + 4] = 1;
					cloudNeighborPicked[i + 5] = 1;
					cloudNeighborPicked[i + 6] = 1;
				}
			}
		}
		/* 针对论文中(a)情况 */
		//xie:去除论文里提到的线特征的边缘点，如果一个点是互相遮挡间断处的点，那么他周围的至少6个点不能要，因为计算曲率的时候用的5个点。
		float diffX2 = PointCloudOrdered->points[i].x - PointCloudOrdered->points[i - 1].x;
		float diffY2 = PointCloudOrdered->points[i].y - PointCloudOrdered->points[i - 1].y;
		float diffZ2 = PointCloudOrdered->points[i].z - PointCloudOrdered->points[i - 1].z;
		float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

		float dis = PointCloudOrdered->points[i].x * PointCloudOrdered->points[i].x
			+ PointCloudOrdered->points[i].y * PointCloudOrdered->points[i].y
			+ PointCloudOrdered->points[i].z * PointCloudOrdered->points[i].z;
		
		//xie:如果距离两边都很远就去掉这个点
		if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
			cloudNeighborPicked[i] = 1;
		}
		
	}

	//开始找sharp points and flat points
	PointCloud cornerPointsSharp;
	PointCloud cornerPointsLessSharp;
	PointCloud surfPointsFlat;
	PointCloud surfPointsLessFlat;

	for (int i = 0; i < N_SCANS; i++)
	{
		PointCloud::Ptr surfPointsLessFlatScan(new PointCloud);	//less flat点比较多，需要进行voxel滤波处理
		//将每一线等间距分成6份来处理； 冒泡排序算法；sp:六等分起点，start pointer
		for (int j = 0; j < 6; j++)
		{
			int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / 6;	//start point and end point
			int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / 6 - 1;

			//对一线的1/6，按曲率排序； 排序后的结果存放在cloudSortInd里面的
			for (int k = sp + 1; k <= ep; k++)
			{
				for (int l = k; l >= sp + 1; l--)
				{
					if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]])
					{
						int temp = cloudSortInd[l - 1];
						cloudSortInd[l - 1] = cloudSortInd[l];
						cloudSortInd[l] = temp;
					}
				}
			}

			//挑选每个分段的曲率很大和比较大的点
			int largestPickedNum = 0;	//	曲率较大且可选的点的个数
			for (int k = ep; k >= sp; k--)
			{
				int ind = cloudSortInd[k];
				if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
				{
					largestPickedNum++;
					//挑选曲率最大的前2个点放入sharp点集合
					if (largestPickedNum <= 2)
					{
						cloudLabel[ind] = 2;//2代表点曲率很大
						cornerPointsSharp.push_back(PointCloudOrdered->points[ind]);
						cornerPointsLessSharp.push_back(PointCloudOrdered->points[ind]);
					}
					else if (largestPickedNum <= 20)
					{
						cloudLabel[ind] = 1;//1代表点曲率比较尖锐
						cornerPointsLessSharp.push_back(PointCloudOrdered->points[ind]);
					}
					else
					{
						break;
					}

					cloudNeighborPicked[ind] = 1;//筛选标志置位,挑选过了设为1，不再挑选
					//将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
					for (int l = 1; l <= 5; l++)
					{
						float diffX = PointCloudOrdered->points[ind + l].x
							- PointCloudOrdered->points[ind + l - 1].x;
						float diffY = PointCloudOrdered->points[ind + l].y
							- PointCloudOrdered->points[ind + l - 1].y;
						float diffZ = PointCloudOrdered->points[ind + l].z
							- PointCloudOrdered->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
						{
							break;
						}
						cloudNeighborPicked[ind + l] = 1;
					}

					for (int l = -1; l >= -5; l--)
					{
						float diffX = PointCloudOrdered->points[ind + l].x
							- PointCloudOrdered->points[ind + l + 1].x;
						float diffY = PointCloudOrdered->points[ind + l].y
							- PointCloudOrdered->points[ind + l + 1].y;
						float diffZ = PointCloudOrdered->points[ind + l].z
							- PointCloudOrdered->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
						{
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}

				}
			}

			//挑选每个分段的曲率很小比较小的点，flat points
			int smallestPickedNum = 0;
			for (int k = sp; k <= ep; k++)
			{
				int ind = cloudSortInd[k];
				//如果曲率的确比较小，并且未被筛选出
				if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
				{

					cloudLabel[ind] = -1;//-1代表曲率很小的点
					surfPointsFlat.push_back(PointCloudOrdered->points[ind]);

					smallestPickedNum++;
					if (smallestPickedNum >= 4)
					{//只选最小的四个，剩下的Label==0,就都是曲率比较小的
						break;
					}

					cloudNeighborPicked[ind] = 1;
					for (int l = 1; l <= 5; l++) //同样防止特征点聚集
					{
						float diffX = PointCloudOrdered->points[ind + l].x
							- PointCloudOrdered->points[ind + l - 1].x;
						float diffY = PointCloudOrdered->points[ind + l].y
							- PointCloudOrdered->points[ind + l - 1].y;
						float diffZ = PointCloudOrdered->points[ind + l].z
							- PointCloudOrdered->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
						{
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--)
					{
						float diffX = PointCloudOrdered->points[ind + l].x
							- PointCloudOrdered->points[ind + l + 1].x;
						float diffY = PointCloudOrdered->points[ind + l].y
							- PointCloudOrdered->points[ind + l + 1].y;
						float diffZ = PointCloudOrdered->points[ind + l].z
							- PointCloudOrdered->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
						{
							break;
						}

						cloudNeighborPicked[ind + l] = 1;
					}

				}
			}

			//将剩余的点（包括之前被排除的点）全部归入平面点中less flat类别中
			for (int k = sp; k <= ep; k++)
			{
				if (cloudLabel[k] <= 0)
				{
					surfPointsLessFlatScan->push_back(PointCloudOrdered->points[k]);
				}
			}
		}

		//由于less flat点最多，对每个分段less flat的点进行体素栅格滤波
		pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
		pcl::VoxelGrid<PointType> downSizeFilter;
		downSizeFilter.setInputCloud(surfPointsLessFlatScan);
		downSizeFilter.setLeafSize(0.2, 0.2, 0.2);	//设置单位正方体为20cm
		downSizeFilter.filter(surfPointsLessFlatScanDS);

		//less flat点
		surfPointsLessFlat += surfPointsLessFlatScanDS;


	}

	ScanRegistrationBack ScanBackValue;

	ScanBackValue.imuTrans.points.resize(4);

	ScanBackValue.imuTrans.points[0].x = imuPitchStart;
	ScanBackValue.imuTrans.points[0].y = imuYawStart;
	ScanBackValue.imuTrans.points[0].z = imuRollStart;

	ScanBackValue.imuTrans.points[1].x = imuPitchCur;
	ScanBackValue.imuTrans.points[1].y = imuYawCur;
	ScanBackValue.imuTrans.points[1].z = imuRollCur;

	ScanBackValue.imuTrans.points[2].x = imuShiftFromStartXCur;
	ScanBackValue.imuTrans.points[2].y = imuShiftFromStartYCur;
	ScanBackValue.imuTrans.points[2].z = imuShiftFromStartZCur;

	ScanBackValue.imuTrans.points[3].x = imuVeloFromStartXCur;
	ScanBackValue.imuTrans.points[3].y = imuVeloFromStartYCur;
	ScanBackValue.imuTrans.points[3].z = imuVeloFromStartZCur;


	//返回值
	//特殊点
	ScanBackValue.cornerPointsLessSharp = cornerPointsLessSharp;	//一般尖锐点
	ScanBackValue.cornerPointsSharp = cornerPointsSharp;			//更尖锐点。。也就是该段内尖锐点的个数超过4时识别到的尖锐点存入这个数组
	ScanBackValue.laserCloud = PointCloudOrdered;							//按线聚类之后的点集
	ScanBackValue.surfPointsFlat = surfPointsFlat;
	ScanBackValue.surfPointsLessFlat = surfPointsLessFlat;
	return ScanBackValue;
}