#include "MyScanRegistration.h"
const double ScanRegistration::scanPeriod = 0.1;	//�����״�ɨ��Ƶ��

//��ĳ�ʼ������
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
	std::vector<int> scanStartInd(N_SCANS, 0);	//ÿ���ߵ�����������ʼ�����������������
	std::vector<int> scanEndInd(N_SCANS, 0);

	PointCloud laserCloudIn;
	laserCloudIn.clear();
	laserCloudIn = laserCloudIn1;		//copy cloud data from input parameter  +=��������
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);	//remove nan points from cloud points

	int cloudSize = laserCloudIn.points.size();		//�����е�ĸ���
	//�״����pcapһ֡��һ����һȦ������ֻ��1/5Ȧ������
	//lidar scan��ʼ�����ת��,atan2��Χ[-pi,+pi],������ת��ʱȡ��������Ϊvelodyne��˳ʱ����ת
	float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);	//atan��Χ��[-pi,pi]
	//lidar scan���������ת�ǣ���2*piʹ������ת����Ϊ2*pi;  [pi,3pi]
	float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
		laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

	//������λ���뿪ʼ��λ�ǲ�ֵ������(PI,3*PI)��Χ������lidar����һ��Բ��ɨ��
	//����������������Χ�ڣ�pi < endOri - startOri < 3*pi���쳣������
	//PCAP ���������Ӧ�ñ����⴦������������������жϣ�ÿ�ο�ʼ�㶼��-pi/2,������3pi/2���պ�һ��Բ�ܡ�����
	if (endOri - startOri > 3 * M_PI) {
		endOri -= 2 * M_PI;
	}
	else if (endOri - startOri < M_PI) {
		endOri += 2 * M_PI;
	}


	PointT point;
	int count = cloudSize;	//����������Ч��ĸ���
	bool halfPassed = false;  // judge whether the point's z/x angle over pi�� if over, compare with end_ori.
	std::vector<PointCloud> laserCloudScans(N_SCANS);
	// Calculate all SCANID of all points, and calculate intensity by SCANID+reltime
	for (int i = 0; i < cloudSize; i++)
	{
		/*imuΪx����ǰ,y������,z�����ϵ���������ϵ��
		velodyne lidar����װΪx����ǰ, y������, z�����ϵ���������ϵ��
		scanRegistration�������ͨ�����������ᣬ��ͳһ��z����ǰ, x������, y�����ϵ���������ϵ
		������J.Zhang����������ʹ�õ�����ϵ
		������R = Ry(yaw)*Rx(pitch)*Rz(roll)*/
		point.x = laserCloudIn.points[i].y;
		point.y = laserCloudIn.points[i].z;
		point.z = laserCloudIn.points[i].x;

		float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI; //�õ���ֱ�ǣ�Ҳ����˵�Ǹ�����
		int scanID;
		int roundedAngle = int(angle + (angle<0.0 ? -0.5 : +0.5));	//��int�ضϣ��൱����������Ƕ�
		// �Ƕȴ����㣬��С������ż���ߣ�0->16�����Ƕ�С���㣬�ɴ�С����������(15->1);������Ϊ�״�������������������ġ�����
		//����������ǽǶ������ľ�ֱ�ӷָ��ߺţ� �Ƕ��Ǹ��ľ�ȡ���ָ��ߺš�����
		//�������ˡ������scanIDֻ��Ϊ�˰��߶��źã����ڼ����Լ���ʾ
		if (roundedAngle > 0)
		{
			scanID = roundedAngle;
		}
		else
		{
			scanID = roundedAngle + (N_SCANS - 1);
		}
		if (scanID > (N_SCANS - 1) || scanID < 0) // ��16��������ӵ��޳�
		{
			count--;
			continue;

		}
		//std::cout << "angle: " << angle << std::endl << "ScanID: " << scanID << std::endl;

		//Ϊɶ��-atan2������atan2;˳ʱ����ת
		float ori = -atan2(point.x, point.z); 
		// z/x ��ֱ������x��ļн�,֮��Ĳ���Ϊ�����˽Ƕȷ�Χ���������ˣ�y������ֱ���������Ϊ���ж�reltime
		//ò�����жϵ�һ������ĵ���û�й��룻��������endori�Ƚϣ� ��΢��Сһ�����
		//����ֻҪ���true,ֻ�ܵ���һ֡�����ٱ��false
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

		//-0.5 < relTime < 1.5������ת�ĽǶ�������������ת�Ƕȵı���, �������е�����ʱ�䣩
		float relTime = (ori - startOri) / (endOri - startOri);
		//��ǿ��=�ߺ�+�����ʱ�䣨��һ������+һ��С���������������ߺţ�С�������Ǹõ�����ʱ�䣩,����ɨ�裺���ݵ�ǰɨ��ĽǶȺ�ɨ�����ڼ������ɨ����ʼλ�õ�ʱ��
		point.intensity = scanID + scanPeriod * relTime;

		//region for IMU process

		laserCloudScans[scanID].push_back(point);

	}
	cloudSize = count;	//�����������еĵ�ĸ���

	PointCloud::Ptr PointCloudOrdered(new PointCloud());	//���߾���֮��ĵ㼯
	for (int i = 0; i < N_SCANS; i++) {
		*PointCloudOrdered += laserCloudScans[i];//���б��ֲ�ĵ�,һ��count���㣨���ܱ��ֲ�ĵ��Ѿ���ȥ����
	}

	//��ÿ��������ʣ�ÿ����ǰ������㲻Ҫ��
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

		//��������ʼ������
		cloudSortInd[i] = i;		//��Ч���ʵ������
		cloudNeighborPicked[i] = 0;
		cloudNeighborPickedForCorner[i] = 0;
		cloudLabel[i] = 0;

		//startind and endind��ÿ����ֻ�������ifһ�Ρ�Ϊ����ֳ�6�ȷ���׼��
		if (int(PointCloudOrdered->points[i].intensity) != scanCount)
		{
			scanCount = int(PointCloudOrdered->points[i].intensity);

			if (scanCount > 0 && scanCount < N_SCANS) {
				scanStartInd[scanCount] = i + 5;
				scanEndInd[scanCount - 1] = i - 5;//ÿһ��ǰ�����ͺ�����㲻Ҫ
			}
		}
	}
	//���⴦���ʼ�������յ�
	scanStartInd[0] = 5;
	scanEndInd.back() = cloudSize - 5;

	/*�������е㣨��ȥǰ����ͺ����������жϸõ㼰���ܱߵ��Ƿ������Ϊ������λ��
	��ĳ�㼰�����ľ���ƽ������ĳ��ֵa��˵����������һ�����룩�����������н�С��ĳ��ֵbʱ���н�С�Ϳ��ܴ����ڵ�����
	����һ����ٽ�6������Ϊ���ɱ��Ϊ������ĵ㣻��ĳ�㵽��ǰ������ľ��������c���ĸõ���ȣ�
	��õ��ж�Ϊ���ɱ��������ĵ㣨�����ԽС������Խ�󣬼����ⷢ�䷽����Ͷ�䵽��ƽ��Խ����ˮƽ����*/
	// ǰ���Ѿ������е㰴�߷ֺ���֮���ٵ���
	//�ȴ���ǵ㣬edge points
	for (int i = 5; i < cloudSize - 6; i++)
	{
		//ǰ��������ľ���ƽ��
		float diffX = PointCloudOrdered->points[i + 1].x - PointCloudOrdered->points[i].x;
		float diffY = PointCloudOrdered->points[i + 1].y - PointCloudOrdered->points[i].y;
		float diffZ = PointCloudOrdered->points[i + 1].z - PointCloudOrdered->points[i].z;
		float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

		//ǰ����������볬����ֵ�ſ�����������
		if (diff > 0.1)
		{
			//�㵽�����״�ľ���
			float depth1 = sqrt(PointCloudOrdered->points[i].x * PointCloudOrdered->points[i].x +
				PointCloudOrdered->points[i].y * PointCloudOrdered->points[i].y +
				PointCloudOrdered->points[i].z * PointCloudOrdered->points[i].z);

			float depth2 = sqrt(PointCloudOrdered->points[i + 1].x * PointCloudOrdered->points[i + 1].x +
				PointCloudOrdered->points[i + 1].y * PointCloudOrdered->points[i + 1].y +
				PointCloudOrdered->points[i + 1].z * PointCloudOrdered->points[i + 1].z);

			/* ���������(b)��� */
			if (depth1 > depth2)
			{
				//�����Ȳ�һ���Ļ�����ǰ������������ͬһ�������ϼ�������
				diffX = PointCloudOrdered->points[i + 1].x - PointCloudOrdered->points[i].x * depth2 / depth1;
				diffY = PointCloudOrdered->points[i + 1].y - PointCloudOrdered->points[i].y * depth2 / depth1;
				diffZ = PointCloudOrdered->points[i + 1].z - PointCloudOrdered->points[i].z * depth2 / depth1;// ����������һ�����������εĵ�����

				//depth2�������ɶ�̬��ֵ��ԽԶ��ֵԽ�󣻣���ʵ���ǿ��н�
				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
					// ���ݵ������������ʣ���һ�ж�threshold=0.1ʵ�ʱ�ʾX[i]������X[i+1]�ļн�С��5.732��
					// cloudNeighborPicked �ǿ���һ����������Χ���������ó�����Լ�����жϱ�־λ
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
		/* ���������(a)��� */
		//xie:ȥ���������ᵽ���������ı�Ե�㣬���һ�����ǻ����ڵ���ϴ��ĵ㣬��ô����Χ������6���㲻��Ҫ����Ϊ�������ʵ�ʱ���õ�5���㡣
		float diffX2 = PointCloudOrdered->points[i].x - PointCloudOrdered->points[i - 1].x;
		float diffY2 = PointCloudOrdered->points[i].y - PointCloudOrdered->points[i - 1].y;
		float diffZ2 = PointCloudOrdered->points[i].z - PointCloudOrdered->points[i - 1].z;
		float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

		float dis = PointCloudOrdered->points[i].x * PointCloudOrdered->points[i].x
			+ PointCloudOrdered->points[i].y * PointCloudOrdered->points[i].y
			+ PointCloudOrdered->points[i].z * PointCloudOrdered->points[i].z;
		
		//xie:����������߶���Զ��ȥ�������
		if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
			cloudNeighborPicked[i] = 1;
		}
		
	}

	//��ʼ��sharp points and flat points
	PointCloud cornerPointsSharp;
	PointCloud cornerPointsLessSharp;
	PointCloud surfPointsFlat;
	PointCloud surfPointsLessFlat;

	for (int i = 0; i < N_SCANS; i++)
	{
		PointCloud::Ptr surfPointsLessFlatScan(new PointCloud);	//less flat��Ƚ϶࣬��Ҫ����voxel�˲�����
		//��ÿһ�ߵȼ��ֳ�6�������� ð�������㷨��sp:���ȷ���㣬start pointer
		for (int j = 0; j < 6; j++)
		{
			int sp = (scanStartInd[i] * (6 - j) + scanEndInd[i] * j) / 6;	//start point and end point
			int ep = (scanStartInd[i] * (5 - j) + scanEndInd[i] * (j + 1)) / 6 - 1;

			//��һ�ߵ�1/6������������ �����Ľ�������cloudSortInd�����
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

			//��ѡÿ���ֶε����ʺܴ�ͱȽϴ�ĵ�
			int largestPickedNum = 0;	//	���ʽϴ��ҿ�ѡ�ĵ�ĸ���
			for (int k = ep; k >= sp; k--)
			{
				int ind = cloudSortInd[k];
				if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
				{
					largestPickedNum++;
					//��ѡ��������ǰ2�������sharp�㼯��
					if (largestPickedNum <= 2)
					{
						cloudLabel[ind] = 2;//2��������ʺܴ�
						cornerPointsSharp.push_back(PointCloudOrdered->points[ind]);
						cornerPointsLessSharp.push_back(PointCloudOrdered->points[ind]);
					}
					else if (largestPickedNum <= 20)
					{
						cloudLabel[ind] = 1;//1��������ʱȽϼ���
						cornerPointsLessSharp.push_back(PointCloudOrdered->points[ind]);
					}
					else
					{
						break;
					}

					cloudNeighborPicked[ind] = 1;//ɸѡ��־��λ,��ѡ������Ϊ1��������ѡ
					//�����ʱȽϴ�ĵ��ǰ���5����������ȽϽ��ĵ�ɸѡ��ȥ����ֹ������ۼ���ʹ����������ÿ�������Ͼ����ֲ�����
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

			//��ѡÿ���ֶε����ʺ�С�Ƚ�С�ĵ㣬flat points
			int smallestPickedNum = 0;
			for (int k = sp; k <= ep; k++)
			{
				int ind = cloudSortInd[k];
				//������ʵ�ȷ�Ƚ�С������δ��ɸѡ��
				if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
				{

					cloudLabel[ind] = -1;//-1�������ʺ�С�ĵ�
					surfPointsFlat.push_back(PointCloudOrdered->points[ind]);

					smallestPickedNum++;
					if (smallestPickedNum >= 4)
					{//ֻѡ��С���ĸ���ʣ�µ�Label==0,�Ͷ������ʱȽ�С��
						break;
					}

					cloudNeighborPicked[ind] = 1;
					for (int l = 1; l <= 5; l++) //ͬ����ֹ������ۼ�
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

			//��ʣ��ĵ㣨����֮ǰ���ų��ĵ㣩ȫ������ƽ�����less flat�����
			for (int k = sp; k <= ep; k++)
			{
				if (cloudLabel[k] <= 0)
				{
					surfPointsLessFlatScan->push_back(PointCloudOrdered->points[k]);
				}
			}
		}

		//����less flat����࣬��ÿ���ֶ�less flat�ĵ��������դ���˲�
		pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
		pcl::VoxelGrid<PointType> downSizeFilter;
		downSizeFilter.setInputCloud(surfPointsLessFlatScan);
		downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
		downSizeFilter.filter(surfPointsLessFlatScanDS);

		//less flat��
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


	//����ֵ
	//�����
	ScanBackValue.cornerPointsLessSharp = cornerPointsLessSharp;	//һ������
	ScanBackValue.cornerPointsSharp = cornerPointsSharp;			//������㡣��Ҳ���Ǹö��ڼ����ĸ�������4ʱʶ�𵽵ļ��������������
	ScanBackValue.laserCloud = PointCloudOrdered;							//���߾���֮��ĵ㼯
	ScanBackValue.surfPointsFlat = surfPointsFlat;
	ScanBackValue.surfPointsLessFlat = surfPointsLessFlat;
	return ScanBackValue;
}