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
	laserCloudCenWidth = 10;	 // ������, cmΪ��λ
	laserCloudCenHeight = 5;	// ����߶�, cmΪ��λ
	laserCloudCenDepth = 10;	// �������, cmΪ��λ
	systemInit = true;
	//��ʼ��������û���⼸�л����
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

//��������ģ�ͣ������ϴ�΢���Ľ����odometry������ϴμ���Ľ�����²�һ���µ���������ϵ��ת������transformTobeMapped
void LaserMapping::transformAssociateToMap()
{
	//*befmap�����¼��ʵ���Ͼ�����̼ƽڵ��ۼӵĽ�� tobemap�����������ڵ������Ķ���ȫ�ֵ�����任
	// aftmap����tobemap���������Ժ��õ�tobemap��ֵ�������Imu�Ļ������и���Ȩ��
	// ����һ���µ�tansformsum����������֮ǰ��������befmap���Աȣ���ʵ���ǵõ����µ�һ֡�����֮ǰ�����
	// ��һ֡�����е�����˶����ٰ��������˶��ӵ�afrmap�ϣ���ʵ�͵õ��ĳ�ʼ�������ȫ�ֵ�����任����Ϊtobemap�������

	//���ȼ��� \bar{T}_{k}^{W}(t_{k+1}) ��������� \bar{T}_{k}^{W}(t_{k+1}) �������õ��ĵ�����ת������������ϵ{W}�¡�
	//��������ת������������ŷ���Ǳ�ʾ��̬��ת
	Vector3 v0 = transformBefMapped.pos - transformSum.pos;		//transformBefMapped��ǰһ֡����˵��һ�ε�ͼ���״��λ�ˣ����˴�������õ���ǰ֡�״���ȫ�ֵ�ͼ�е�λ��
	Vector3 v1 = rotateY(v0, -(transformSum.rot_y));
	Vector3 v2 = rotateX(v1, -(transformSum.rot_x));
	//ƽ������
	transformIncre.pos = rotateZ(v2, -(transformSum.rot_z));

	//bc:before current		bl:before last		al:after last	
	//����
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

//������ת������������ϵ
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

//���������������ϵת�����״�����ϵ
void LaserMapping::pointAssociateTobeMapped(PointType const * const pi, PointType * const po)
{
	//������һ�������������
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
	PointType pointOri;		//original point, �״�����ϵ�еĵ� //����L-M����ʱʹ�ã����ڴ���ĵ�
	PointType pointProj;    //�洢ƫ����Ϣ��jacobian..���ԣ�����洢����pointSel��ȥƫ��
	PointType coeff; // ÿ���������Ӧ��Jaccobian���������Ԫ�ض�������coeffSel�У��������L - M���������ʱ��ֱ�ӵ��þ����ˡ�

	Eigen::Matrix3f matA1;		//�����ǵ�ľ���A 3*3
	Eigen::Matrix<float, 1, 3> matD1;	//A1�ֽ�õ�������ֵ
	Eigen::Matrix3f matV1;	//A1�ֽ�õ�����������
	Eigen::Matrix<float, 5, 3> matA0; //����ƽ���ľ���A
	Eigen::Matrix<float, 5, 1> matB0;
	Eigen::Vector3f matX0;

	matA0.setZero();
	matB0.setConstant(-1);	//ȫ-1��  �㵽��ľ��뾡���Ż���ʹ֮����0
	matX0.setZero();

	matA1.setZero();
	matD1.setZero();
	matV1.setZero();

	bool isDegenerate = false;		//�ж��Ƿ�����˻����������������
	Eigen::Matrix<float, 6, 6> matP;

	//��վ����ݣ�����������
	laserCloudCornerLastHandler(OdometryValueBack.laserCloudCornerLast);
	laserCloudSurfLastHandler(OdometryValueBack.laserCloudSurfLast);
	laserCloudFullResHandler(OdometryValueBack.laserCloudFullRes);
	laserOdometryHandler(OdometryValueBack.transformSum);



	int frameCount = stackFrameNum - 1; // initialize as 0
	int mapFrameCount = mapFrameNum - 1; // initialize as 4

	//��if,һֱ�����  �����Щ���ݶ��õ��˸��£��͸��µ�ͼ
	if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry)
	{
		frameCount++;
		//������֡����>=����ʵ�ʲ�û����֡��ֻȡ>��������stackFrameNum����ʵ����Ӧ����֡����
		//��������������Ԥ�⵱ǰ��������������ϵ�е�ת������Twc; Ȼ�������㶼ת������������ϵ�������laserCloudCornerStack2��laserCloudSurfStack2��
		if (frameCount >= stackFrameNum)
		{
			//Ԥ�⽫��ǰ����ת������ͼ��ȫ������ϵ�е�ת������yes���õ�transformTobeMapped
			transformAssociateToMap();

			//��������Ҳת������ͼ������
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

		//�ڶ�����if������һ��if����һ��������
		if (frameCount >= stackFrameNum)
		{
			//�ҵ�ǰ���Ƶ�Lidarλ�� \bar{T}_{k}^{W}(t_{k+1}) �����ĸ���cube��I��J��K��Ӧ��cube��������
			frameCount = 0;
			PointType pointOnYAxis;  // ��ǰLidar����ϵ{L}y���ϵ�һ��(0,10,0)���ں���
			pointOnYAxis.x = 0.0;
			pointOnYAxis.y = 10.0;
			pointOnYAxis.z = 0.0;
			pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);  // ת����������ϵ{W}��

			// cube����λ������������Ԥ���Twc����cube����������
			//�������е�����������ϵ�µģ�ԭ�㣩λ��
			//����ȡһ����50�׽������������Ч���������������±�ֻ��Ϊ����������ͼ���ܽ�����ԭ��ǰ�����
			//ÿһάƫ��һ��laserCloudCenWidth����ֵ�ᶯ̬��������ʹ������������󻯣���ʼֵΪ��ά���鳤��1/2������
			int centerCubeI = int((transformTobeMapped.pos.x() + 25.0) / 50.0) + laserCloudCenWidth;
			int centerCubeJ = int((transformTobeMapped.pos.y() + 25.0) / 50.0) + laserCloudCenHeight;
			int centerCubeK = int((transformTobeMapped.pos.z() + 25.0) / 50.0) + laserCloudCenDepth;

			//���ڼ��������������ȡ����Ϊ�˲�ʹ��-50.0,50.0�����������ƫ�ƣ�����������Ϊ����ʱ������ͳһ����ƫ��һ����λ��Ҳ����һ
			if (transformTobeMapped.pos.x() + 25.0 < 0) centerCubeI--;
			if (transformTobeMapped.pos.y() + 25.0 < 0) centerCubeJ--;
			if (transformTobeMapped.pos.z() + 25.0 < 0) centerCubeK--;

			//���ȡ������cube��������cube�ı�Ե�򽫵��Ӧ��cube�����������ķ���Ų��һ����λ����������Ҫ�ǽ�ȡ����cube��
			/*ͨ��6��whileѭ����ʹI,J,K�ֱ���һ����Χ֮�ڣ� Ҳ����3��(max-3);  3����Ϊ֮������ڽ�����5*5*5������ҡ���
			�����ѭ����ע��
			//���²�����ʵ���ڼ��е�ͼ��ÿ�ξ����Զ�Ժ󣬾�����cubeƽ�ơ��ʼ�����״�λ������ԭ�㣬
			// ʵ���϶�Ӧcube�����ǵ�10 5 10�� �������״�λ�þ����ԵС��3��cube�Ժ�������cube��
			//�÷���ƽ��һ��cube���򼸸���ֱ�������Ե����3��Ϊֹ����ͬ���м�λ�õ�����Ҳ���ű䣬����
			// ��һ�μ����ʱ�򲻻�������⡣Ҳ����ƽ�ƹ�һ���Ժ����λ��ԭ���ֵ�ڽ��м����Ӧ�ľͲ���
			//10 5 10�ˣ��п�����9 5 10��     ====������cube�ķ�Χ��21,11,21

			version3: �״�һ��ʼ��0,0,0�������Ӧ��cude����I,J,KΪ10,5,10��
			�����״���I����475�ף���ô�״��λ�þ���475,0,0����Ӧ��I,J,KΪ20,5,10��
			���ʱ��20>21-3,̫������Եcube�ˣ���������21*11*21��cube��Ҫ����Iƽ�ƣ�����ʵ���Ǵ�cube������ƽ�ơ�
			ֱ���״��λ�ò��ڱ�Ե��Χ

			���ǣ��ƶ�����ɶ���ݣ��Ǽ��еĵ�ͼ���ݣ�ȫ��ͼ̫���ˣ����״︽�����е�ͼ����ƥ��Ϳ����ˡ�

			�ƶ�����cube...������cubeǰ�����������ƶ���ʹlidar����cubeǰ����������������3��cube,Ϊ�˺����ٽ���������׼��
			*/
			while (centerCubeI < 3)  //???���3��ʲô��˼---���״�λ�÷�Χ����3С��(max-3),
			{
				// �����ָ�������ķ���ƽ��   I->WIDTH   J->HEIGHT   K->DEPTH
				// �ָ������Сcube
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						//�������������������ƣ�����ѭ����λ�� �Ȱ����һ��ֵ�ó�����Ȼ��������λ���ٰ������һ��ֵ���ȥ
						//version2�� ����ѭ����λ�����ǵ���������һλ�� �������һ�������ó���������ʼ�㣬�Ҿ��������ܱȸ�0�á�����
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
				//Ӧ�����ȼ�laserCloudCenWidth��Ȼ�����ǰ�����centerCubeI�Ĺ�ʽ��centerCubeIҲ��Ҫ��1
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
			//���ϲ�����ʵ���ڼ��е�ͼ��ÿ�ξ����Զ�Ժ󣬾�����cubeƽ�ơ��ʼ�����״�λ������ԭ�㣬
			// ʵ���϶�Ӧcube�����ǵ�10 5 10�� �������״�λ�þ����ԵС��3��cube�Ժ�������cube��
			//�÷���ƽ��һ��cube���򼸸���ֱ�������Ե����3��Ϊֹ����ͬ���м�λ�õ�����Ҳ���ű䣬����
			// ��һ�μ����ʱ�򲻻�������⡣Ҳ����ƽ�ƹ�һ���Ժ����λ��ԭ���ֵ�ڽ��м����Ӧ�ľͲ���
			//10 5 10�ˣ��п�����9 5 10��     ====������cube�ķ�Χ��21,11,21

			//version2: �����ھ���Ӧ���Ǳ����״�λ��cube�ı��أ�Ҳ�������״��λ���ڣ�3��max-3��֮��
			//tips: �ܶ�������ܿ�����û��ͷ��������ʵ���Ǹ��������еģ������ĸ�ֵɶ�Ŀ����ڴ�����֮�󡣡���

			//centerCube ��Ӧ����Ԥ�⼤���״��ڵ�λ�ö�Ӧ������,��λ��cube

			//������ϱ��ص㣬������������ȡ������cube��5*5*5���������Ҷ�Ӧ����׼���ˡ���������������˵��10cm*10cm*10cm
			//isInLaserFOV�� û�㶮�������������
			//��ÿһά����5��cube(ǰ2������2�����м�1��)����в��ң�ǰ��250�׷�Χ�ڣ��ܹ�500�׷�Χ��������ά���ܹ�125��cube
			//����125��cube�����һ��ɸѡ������Χ�ڵ�cube
			int laserCloudValidNum = 0;		//�״������ڵ�cube
			int laserCloudSurroundNum = 0;	//�״���Χ��cube 5*5*5
			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
				{
					for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
					{
						//������Щ�������ڼ��еĵ�ͼ��
						if (i >= 0 && i < laserCloudWidth && j >= 0 && j < laserCloudHeight && k >= 0 && k < laserCloudDepth)
						{
							// ������cube��Ӧ�ĵ�����  version2:
							// NOTE: ����i j k��Ϊ����������ȡֵΪ���ĵ�����
							float centerX = 50.0 * (i - laserCloudCenWidth);	//��Ӧǰ���int centerCubeI = int((transformTobeMapped.pos.x() + 25.0) / 50.0) + laserCloudCenWidth;
							float centerY = 50.0 * (j - laserCloudCenHeight);
							float centerZ = 50.0 * (k - laserCloudCenDepth);

							// ȡ�ڽ���8�������꣬���������״����������ϵλ�á�
							//�ж�һ�¸���׼���Ƿ����ڵ�ǰLidar�Ŀ��ӷ�Χ�ڣ����Ը������ҹ�ʽ�Ծ��뷶Χ�����Ƶ���
							//���ݴ����е�ʽ�ӣ�ֻҪ����x���60��ķ�Χ�ڶ���Ϊ��FOV�еĵ�(������ô������ΪLidar��̼ƵĹ��ƽ��̧��׼ȷ�ˣ�ֻ�ܸ��Ե�ȡһ���ϴ�ķ�Χ)��
							bool isInLaserFOV = false;		//�Ƿ����״�������У�������
							for (int ii = -1; ii <= 1; ii += 2)
							{
								for (int jj = -1; jj <= 1; jj += 2)
								{
									for (int kk = -1; kk <= 1; kk += 2)
									{
										//�������Ұ˸���������
										Vector3 corner;
										corner.x() = centerX + 25.0 * ii;
										corner.y() = centerY + 25.0 * jj;
										corner.z() = centerZ + 25.0 * kk;

										Vector3 point_on_axis(pointOnYAxis.x, pointOnYAxis.y, pointOnYAxis.z);
										//ԭ�㵽��������ƽ���� //transformTobeMapped.pos - corner,������ת������������ϵ�У�
										float squaredSide1 = (transformTobeMapped.pos - corner).squaredNorm();
										//pointOnYAxis����������ƽ����
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

							//�ж�һ�¸�cube�Ƿ����ڵ�ǰLidar�Ŀ��ӷ�Χ�ڣ����Ը������ҹ�ʽ�Ծ��뷶Χ�����Ƶ���
							//���ݴ����е�ʽ�ӣ�ֻҪ����x���60��ķ�Χ�ڶ���Ϊ��FOV�еĵ�(������ô������ΪLidar��̼ƵĹ��ƽ��̧��׼ȷ�ˣ�ֻ�ܸ��Ե�ȡһ���ϴ�ķ�Χ)��
							//�������Ǿ͵õ����ڵ�ǰLidarλ�õ���������Ч�ĵ�ͼ�����㡣
							//��ס����Χ�ڵ�cube������ƥ����
							if (isInLaserFOV) {
								laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j
									+ laserCloudWidth * laserCloudHeight * k;
								laserCloudValidNum++;
							}
							////��ס��������cube����������ʾ��   ����洢��Ҳֻ��indice?�Եģ�ÿ����������һ��cube
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j
								+ laserCloudWidth * laserCloudHeight * k;
							laserCloudSurroundNum++;

						}
					}
				}
			}//ɾѡ�״���ӷ�Χ�ڵĵ�

			//���ԣ����ǾͲ���Ҫ���Ӵ�����е�ͼ���ƽ��д����ˣ�ֻ��Ҫ������Щ����cube�ڵĵ�ͼ�����㼴�ɣ����Խ�ʡ������������Դ��
			//Ϊ�˱�֤��ǰ֡�ĵ����㹻ƽ�������Ե��ƽ������˲�����
			laserCloudCornerFromMap->clear();
			laserCloudSurfFromMap->clear();
			//�����������ͼ������ƥ��ʹ��
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
			int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();  // ��Ч���������ϵĵ�ĸ���
			int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();     // ��Ч���������ϵĵ�ĸ���

			//// ����������ϵ�µĵ�ǰ֡������ת����ǰLidar����ϵ��,
			// ΪɶҪת���״�����ϵ�ٽ����˲���ֱ���˲�������  
			//������Ϊ��ת����ȥ������ֱ��ʹ�ø��º��ת������������ת����������ϵ��
			int laserCloudCornerStackNum2 = laserCloudCornerStack2->points.size();  // �����������ϵĵ�ĸ���
			for (int i = 0; i < laserCloudCornerStackNum2; i++) {
				pointAssociateTobeMapped(&laserCloudCornerStack2->points[i], &laserCloudCornerStack2->points[i]);
			}

			int laserCloudSurfStackNum2 = laserCloudSurfStack2->points.size();  // �����������ϵĵ�ĸ���
			for (int i = 0; i < laserCloudSurfStackNum2; i++) {
				pointAssociateTobeMapped(&laserCloudSurfStack2->points[i], &laserCloudSurfStack2->points[i]);
			}

			// �����е�ǰ֡����������˲����� 5cm*5cm*5cm
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

			/*������Щ�����Ժ����Ǿ������ڵ�ǰLidar����λ�ø��������е�ͼ�������Լ���ǰ֡�ĵ��������㣬
			����Ĺ���������ô���������ƥ����һ���������������ٴ��ó�KD������Ѱ�����ڽ���5���㡣
			�Ե���Э�������������ɷַ�����
			*���������ֲ���ֱ���ϣ�Э������������ֵ����һ��Ԫ���������������������������ֵ��ص�����������ʾ����ֱ�ߵķ���
			*���������ֲ���ƽ���ϣ�Э������������ֵ����һ������С��Ԫ�أ��������ֵ��ص�����������ʾ����ƽ��ķ��߷���
			������ǿ��Ժ����׵ĸ������������ҵ�ֱ��������Ӷ����������е㵽ֱ�ߵľ��빫ʽ�����Ż�����(���Ż����������ʵ�ͬѧ��ο���һƪ����)��
			ƽ������Ҳ����ͬ��˼·��������Ż�����Ĺ���֮��Ϳ��Զ�����������ˣ���ⷽ������L-M�������ⲿ�ִ�����laserOdometry���ֵļ���һ��
			*/

			


			if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100)
			{
				nanoflann::KdTreeFLANN<PointType> kdtreeCornerFromMap;
				nanoflann::KdTreeFLANN<PointType> kdtreeSurfFromMap;

				//����ͼ�е����������KD����
				kdtreeCornerFromMap.setInputCloud(laserCloudCornerFromMap);
				kdtreeSurfFromMap.setInputCloud(laserCloudSurfFromMap);

				//���¶�����ҵ�ĸ����;���
				std::vector<int> pointSearchInd;
				std::vector<float> pointSearchSqDis;
				pointSearchInd.resize(5);
				pointSearchSqDis.resize(5);

				//��odometry���Ƶĵ�����Ȼ����Jacobian Matrix,Ȼ��ʹ��L-M���������
				for (int iterCount = 0; iterCount < 10; iterCount++)
				{
					laserCloudOri->clear();
					coeffSel->clear();

					num_corner = 0;
					num_surf = 0;

					//��Ҫ�ĺ�ʱ����������ѭ������
					//��ѭ��ʹ��LM����ƥ��ǵ�������***�����ƥ����Ҫ������Ǹ�ƫ�������ں����ۺ�ţ�ٵ���
					for (int i = 0; i < laserCloudCornerStackNum; i++)
					{
						//laserCloudCornerStack���״ﱾ������ϵ�£�voxel�˲�֮��������㼯
						pointOri = laserCloudCornerStack->points[i];
						pointAssociateToMap(&pointOri, &pointSel);
						//kd���в�����ָ���������5���㣬�������С�������򣬷��ص�����������
						kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

						if (pointSearchSqDis[4] < 1.0)//5�����������벻����1�Ŵ���
						{
							Vector3 vc(0, 0, 0);
							//���������������Ӻ���ƽ��
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
								//��ȥ��ֵ��ȥ���Ļ���Ȼ�󹹽�matA1��  Point to line �õ�������arun�ķ���
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
							//���������ֲ���ֱ���ϣ�Э������������ֵ����һ��Ԫ��������������������
							//�������ֵ��ص�����������ʾ����ֱ�ߵķ���
							if (matD1(0, 0) > 3 * matD1(0, 1))
							{
								float x0 = pointSel.x;
								float y0 = pointSel.y;
								float z0 = pointSel.z;
								//������Щ�㶼�ֲ���ֱ���ϣ�Ϊ�˷���ֱ��ȡ�õ���ֱ������0.1ȡ�����ٽ���
								//Ȼ������ǵ��߾��룬�󵼣���ODOMETRYһ��
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
								//Ȩ��ϵ�����㣬���Ǿ���������һ���Ȩ�أ� ����Զ�����һ���Ȩ�أ�Ȩ�������ں���ţ�ٵ����У�Ҳ������������˵��LM����
								float s = 1 - 0.9 * fabs(ld2);


								coeff.x = s * la;
								coeff.y = s * lb;
								coeff.z = s * lc;
								coeff.intensity = s * ld2;


								//���ڲ��ԣ�ʹ��intensity����ʵ�ʵ�loss
								// coeff.x =  la;
								// coeff.y =  lb;
								// coeff.z =  lc;
								// coeff.intensity =  ld2;

								if (s > 0.1) //�����㹻С��ʹ�ã�Ȩ��̫С�˾���Ϊ�ǲ��ɿ������ݣ����൱��һ��ȥ��
								{
									laserCloudOri->push_back(pointOri);
									coeffSel->push_back(coeff);
									num_corner++;
									//   outfile2<< iterCount<<","<<num_corner++<<","<<s<<endl;

								}
							}
						}
					}

					//��Ҫ�ĺ�ʱ����������ѭ������
					//��ѭ��ʹ��LM����ƥ��ƽ�������㣬�õ�ƫ����Ϣ
					//ƽ�淽��ΪAX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
					//����(X,Y,Z)�ǵ������, ��Ӧ�����mat_a0, ����֪��
					//A/D, B/D, C/D ��Ӧmat_x0, �Ǵ����ֵ
					//��ʽ�ұߵ�-1��Ӧmat_b0
					for (int i = 0; i < laserCloudSurfStackNum; i++)
					{
						pointOri = laserCloudSurfStack->points[i];
						pointAssociateToMap(&pointOri, &pointSel);
						kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

						if (pointSearchSqDis[4] < 1.0)
						{
							//��������������������
							for (int j = 0; j < 5; j++) 
							{
								matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
								matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
								matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
							}
							//���matA0*matX0=matB0  //��� (A/D)X+(B/D)Y+(C/D)Z = -1 �е� A/D, B/D, C/D 
							matX0 = matA0.colPivHouseholderQr().solve(matB0);

							float pa = matX0(0, 0);
							float pb = matX0(1, 0);
							float pc = matX0(2, 0);

							//��Ӧ֮ǰ��-1, (A/D)X+(B/D)Y+(C/D)Z = -1 <=> (A/D)X+(B/D)Y+(C/D)Z +1 = 0
							float pd = 1;

							//psΪƽ�淨������ģ
							//���(pa, pb, pc)Ϊ������, ģΪ1, pdΪƽ�浽ԭ��ľ���
							float ps = sqrt(pa * pa + pb * pb + pc * pc);
							pa /= ps;
							pb /= ps;
							pc /= ps;
							pd /= ps;

							//ȷ����ϳ���ƽ����������ϵĵ㶼�㹻�ӽ�, ��ʾƽ����ϵ���Ч��
							bool planeValid = true;
							for (int j = 0; j < 5; j++)
							{
								//�ж�ƽ����ϵ����
								if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
									pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
									pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
								{
									//ƽ����Ч, ����ƥ��ʧ��, �˵㲻�Ժ������Ż�����Լ��
									planeValid = false;
									break;
								}
							}

							if (planeValid)
							{
								//�㵽ƽ��ľ���, �ο��㵽ƽ����빫ʽ, ��ĸ����Ϊ1
								float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

								//unused
								pointProj = pointSel;
								pointProj.x -= pa * pd2;
								pointProj.y -= pb * pd2;
								pointProj.z -= pc * pd2;

								float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
									+ pointSel.y * pointSel.y + pointSel.z * pointSel.z));

								// float s = 1 - 0.9 * fabs(pd2);

								//(pa, pb, pc)�Ƿ�����, �㵽ƽ��Ĵ���, Ҳ�ǵ�����������������ƫ��, ������µĹ�ʽ�Ƶ�����.
								//...���ˡ����������Ҳ�Ǹ��������йصģ��ոպá���
								coeff.x = s * pa;
								coeff.y = s * pb;
								coeff.z = s * pc;
								coeff.intensity = s * pd2;

								//���ڲ��ԣ�ʹ��intensity����ʵ�ʵ�loss
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

					//���ˣ�jacobian���󹹽���ɡ� 
					//�ǵ���ƽ����һ���Ż���ʹ��ţ�ٵ���
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

						//��ʽ�󵼵ڶ����������󵼹��̿ɿ�֪����ط���
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

					//��һ�ε����ж��Ƿ��˻�����odometryһ��
					//����������Loam����Zhang J��<<On Degeneracy of Optimization-based State Estimation Problems>>
					//��ŷ�����ͨ��Jacobian��eigenvalue�ж��ĸ�������Լ������, �������Ǹ������ϵĵ���
					//����ֵ�ֽ�
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
						//����ֵ��С�����жϣ������С����ֵ������Ϊ�����˻�
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
					//����ÿ�εĵ�����
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

					//��תƽ�������㹻С��ֹͣ����
					if (deltaR < 0.05 && deltaT < 0.05 /*&& matX.at<float>(0, 0) < 0.002*/) 
					{
						break;
					}

				}//����25��
				 //��������������ص�ת�ƾ���Ҳ����transformBefMapped��transformAftMapped
				transformUpdate();

			}//�������������㹻

			//��ֹ��������Ǿ�����˵�ǰ֡�������ͼ���Ƶ���׼������Lidar��̼Ƶ��˶����ƽ�����������Ż���
			//������ɺ����ǻ���Ҫ����ǰ֡ɨ��õ����������Ʒ�װ�ڲ�ͬ��cube�У����ڵ�ͼ�����б��档

			//��corner points�����루��������С��������Ӧ��������
			std::vector<int> cubeIndStore;
			std::vector<PointType> cubeIndStorePoint;
			cubeIndStore.clear();
			cubeIndStorePoint.clear();
			for (int i = 0; i < laserCloudCornerStackNum; i++) 
			{
				//ת�Ƶ���������ϵ
				pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);
				//��50�ı�������С���������룬ƫ��laserCloudCen*��������������
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
					//ֻ��ѡ-laserCloudCenWidth * 50.0 < point.x < laserCloudCenWidth * 50.0��Χ�ڵĵ㣬y��zͬ��
					//������Ϊ��ֻ��ѡһ��cube��Χ�ĵ�ǰ֡������ͼ��
					//���ճ߶ȷŽ���ͬ���飬ÿ����ĵ���������
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudCornerArray[cubeInd]->push_back(pointSel);

					//test, �ж��Ƿ���ظ�����Ȼ������ǻ���ֺܶ��ظ�������
					//�ⲻ���ظ�������һ��array���кܶ�㡣��������
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
			//��surf points�����루��������С��������Ӧ��������
			for (int i = 0; i < laserCloudSurfStackNum; i++) 
			{
				//ʹ�ø��º��ת������������ת������������ϵ��
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

			//�������²���
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				int ind = laserCloudValidInd[i];

				laserCloudCornerArray2[ind]->clear();
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
				downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

				laserCloudSurfArray2[ind]->clear();
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
				downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);
				
				//Array��Array2���������˲������Ҹ���
				pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
				laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
				laserCloudCornerArray2[ind] = laserCloudTemp;

				laserCloudTemp = laserCloudSurfArray[ind];
				laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
				laserCloudSurfArray2[ind] = laserCloudTemp;
			}

			//�����ǽ�������Ϣ������ȥ�ˡ�������Ҫ˵�����ǣ�Ϊ�˱�֤����Ч�ʻ�������ÿmapFrameNum֡����һ�Ρ�
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


				//�״���Χ�ĵ㣬�ǵ���ƽ��㶼���ȥ��
				MappingBackValue.laserCloudSurround = laserCloudSurround;


				// outfile<<num_id2<<",cloud"<<std::endl;
				//num_id2++;//����
			}

			int laserCloudFullResNum = laserCloudFullRes->points.size();
			for (int i = 0; i < laserCloudFullResNum; i++) {
				pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
			}
			//����ǰ�״�ɨ�赽�����еĵ㣬ת������������ϵ�����MappingBackValue.laserCloudFullRes
			MappingBackValue.laserCloudFullRes = laserCloudFullRes;

			MappingBackValue.transformAftMapped[0] = transformAftMapped.rot_x.value();
			MappingBackValue.transformAftMapped[1] = transformAftMapped.rot_y.value();
			MappingBackValue.transformAftMapped[2] = transformAftMapped.rot_z.value();
			MappingBackValue.transformAftMapped[3] = transformAftMapped.pos.x();
			MappingBackValue.transformAftMapped[4] = transformAftMapped.pos.y();
			MappingBackValue.transformAftMapped[5] = transformAftMapped.pos.z();

			// outfile<<num_id<<",odometry"<<std::endl;
			//num_id++;

		}//�ڶ�����if������һ��if����һ��������
	}//��if


	return MappingBackValue;
}