#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <opencv2/opencv.hpp>
#include <vector>
// #include <pair>
#include<algorithm>

using namespace std;

struct PCAInfo
{
	double lambda0, scale;
	cv::Matx31d normal, planePt;
	std::vector<int> idxAll, idxIn;

	PCAInfo &operator =(const PCAInfo &info)
	{
		this->lambda0 = info.lambda0;
		this->normal = info.normal;
		this->idxIn = info.idxIn;
		this->idxAll = info.idxAll;
		this->scale = scale;
		return *this;
	}
};

struct PLANE
{
	double scale;
	std::vector<std::vector<std::vector<cv::Point3d> > > lines3d;

	PLANE &operator =(const PLANE &info)
	{
		this->scale    = info.scale;
		this->lines3d     = info.lines3d;
		return *this;
	}
};

void Transpcd2txt(string filename,pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud)
{
	pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *inCloud);
	std::ofstream out("GlobalMap.txt");
	if (out.is_open()) 
	   {
	        for(auto it = inCloud->begin();it <  inCloud->end();it++)
			{
				out << it->x<<" "<<it->y<<" "<<it->z<<"\n";
			}
			
	    }
	out.close();
}

void Transpcd2txt(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud)
{
	// pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *inCloud);
	std::ofstream out("/home/liuhy/workspace/pcl/test1/Completion.txt");
	if (out.is_open()) 
	   {
	        for(auto it = inCloud->begin();it <  inCloud->end();it++)
			{
				out << it->x<<" "<<it->y<<" "<<it->z<<"\n";
			}
			
	    }
	out.close();
}

void Transply2txt(string filename,pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud)
{
	pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *inCloud);
	std::ofstream out("/home/liuhy/workspace/pcl/test1/dataseg_cloud.txt");
	if (out.is_open()) 
	   {
	        for(auto it = inCloud->begin();it <  inCloud->end();it++)
			{
				out << it->x<<" "<<it->y<<" "<<it->z<<"\n";
			}
			
	    }
	out.close();
}

void readDataFromtxt(string fileData,pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud) {
	std::ifstream ptReader( fileData );
	double x = 0, y = 0, z = 0;
	if( ptReader.is_open() )
	{
		while ( !ptReader.eof() ) 
		{
			//ptReader >> x >> y >> z >> a >> b >> c >> labelIdx;
			//ptReader >> x >> y >> z >> a >> b >> c >> color;
			//ptReader >> x >> y >> z >> color >> a >> b >> c;
			//ptReader >> x >> y >> z >> a >> b >> c ;
			pcl::PointXYZ cloud;
			ptReader >> x >> y >> z;
			cloud.x = x;
			cloud.y = y;
			cloud.z = 	z;
			//ptReader >> x >> y >> z >> color;
			//ptReader >> x >> y >> z >> nx >> ny >> nz;

			inCloud->push_back(cloud);

		}
		ptReader.close();
	}

	std::cout << "Total num of points: " << inCloud->size() << "\n";
}
 
void Normal_Calculation(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,pcl::PointCloud<pcl::Normal>::Ptr  & pcNormal,
pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud_with_normals
){
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(inCloud);
	ne.setInputCloud(inCloud);
	ne.setSearchMethod(tree);
	ne.setKSearch(20);
	//ne->setRadiusSearch (0.03); 
	ne.compute(*pcNormal);	
	pcl::concatenateFields(*inCloud, *pcNormal, *cloud_with_normals);
}

double meadian( std::vector<double> dataset )
{
	std::sort( dataset.begin(), dataset.end(), []( const double& lhs, const double& rhs ){ return lhs < rhs; } );
	if(dataset.size()%2 == 0)
	{
		return dataset[dataset.size()/2];
	}
	else
	{
		return (dataset[dataset.size()/2] + dataset[dataset.size()/2 + 1])/2.0;
	}
}

void MCMD_OutlierRemoval( std::vector<std::vector<double> > &pointData, PCAInfo &pcaInfo )
{
	double a = 1.4826;
	double thRz = 2.5;
	int num = pcaInfo.idxAll.size();

	// ODs
	cv::Matx31d h_mean( 0, 0, 0 );
	for( int j = 0; j < pcaInfo.idxIn.size(); ++j )
	{
		int idx = pcaInfo.idxIn[j];
		h_mean += cv::Matx31d( pointData[idx][0], pointData[idx][1], pointData[idx][2] );
	}
	h_mean *= ( 1.0 / pcaInfo.idxIn.size() );

	std::vector<double> ODs( num );
	for( int j = 0; j < num; ++j )
	{
		int idx = pcaInfo.idxAll[j];
		cv::Matx31d pt( pointData[idx][0], pointData[idx][1], pointData[idx][2] );
		cv::Matx<double, 1, 1> OD_mat = ( pt - h_mean ).t() * pcaInfo.normal;
		double OD = fabs( OD_mat.val[0] );
		ODs[j] = OD;
	}

	// calculate the Rz-score for all points using ODs
	std::vector<double> sorted_ODs( ODs.begin(), ODs.end() );
	double median_OD = meadian( sorted_ODs );
	std::vector<double>().swap( sorted_ODs );

	std::vector<double> abs_diff_ODs( num );
	for( int j = 0; j < num; ++j )
	{
		abs_diff_ODs[j] = fabs( ODs[j] - median_OD );
	}
	double MAD_OD = a * meadian( abs_diff_ODs ) + 1e-6;
	std::vector<double>().swap( abs_diff_ODs );

	// get inlier 
	std::vector<int> idxInlier;
	for( int j = 0; j < num; ++j )
	{
		double Rzi = fabs( ODs[j] - median_OD ) / MAD_OD;
		if ( Rzi < thRz ) 
		{
			int idx = pcaInfo.idxAll[j];
			idxInlier.push_back( idx );
		}
	}

	// 
	pcaInfo.idxIn = idxInlier;
} 

void Pca(int K ,pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,vector<PCAInfo> & pcainfo, double & scale, double & magnitd)
{

	cout<<"----- Normal Calculation ..."<<endl;

	double MINVALUE = 1e-7;

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(inCloud);

// #pragma omp parallel for
	for (int i = 0; i < inCloud->points.size();++i)
	{

	// std::cout << "K nearset neighbor search at (" <<
	// it->x << " " << it->y << " " << it->z <<
	// "), with K = " << K << std::endl;

		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		tree->nearestKSearch(inCloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);

		// for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
		// 	cout << "    " << inCloud->points[pointIdxNKNSearch[i]].x
		// 	<< " " << inCloud->points[pointIdxNKNSearch[i]].y
		// 	<< " " << inCloud->points[pointIdxNKNSearch[i]].z
		// 	<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << endl;

		// int scale = 0;
		pcainfo.resize(inCloud->points.size());
		
		double h_mean_x=0.0,h_mean_y=0.0,h_mean_z = 0.0;
		for(int j =0 ;j < pointIdxNKNSearch.size() ;++j)
			{
				h_mean_x += inCloud->points[pointIdxNKNSearch[j]].x;
				h_mean_y += inCloud->points[pointIdxNKNSearch[j]].y;
				h_mean_z += inCloud->points[pointIdxNKNSearch[j]].z;
			}
		h_mean_x *= 1.0/ pointIdxNKNSearch.size();
		h_mean_y *= 1.0/ pointIdxNKNSearch.size();
		h_mean_z *= 1.0/ pointIdxNKNSearch.size();

		double h_cov_1 = 0.0, h_cov_2 = 0.0, h_cov_3 = 0.0;
		double h_cov_5 = 0.0, h_cov_6 = 0.0;
		double h_cov_9 = 0.0;
		double dx = 0.0, dy = 0.0, dz = 0.0;

		for(int j =0;j<pointIdxNKNSearch.size();++j){
			dx = inCloud->points[pointIdxNKNSearch[j]].x - h_mean_x;
			dy = inCloud->points[pointIdxNKNSearch[j]].y - h_mean_y;
			dz = inCloud->points[pointIdxNKNSearch[j]].z - h_mean_z;

			h_cov_1 += dx*dx; h_cov_2 += dx*dy; h_cov_3 += dx*dz;
			h_cov_5 += dy*dy; h_cov_6 += dy*dz;
			h_cov_9 += dz*dz;
		}
		cv::Matx33d h_cov(
		h_cov_1, h_cov_2, h_cov_3, 
		h_cov_2, h_cov_5, h_cov_6, 
		h_cov_3, h_cov_6, h_cov_9);
		h_cov *= 1.0/pointIdxNKNSearch.size();

		cv::Matx33d h_cov_evectors;
		cv::Matx31d h_cov_evals;
		cv::eigen( h_cov, h_cov_evals, h_cov_evectors );

		pcainfo[i].idxAll.resize(pointIdxNKNSearch.size());
		for(int j=0;j<pointIdxNKNSearch.size();++j){
			pcainfo[i].idxAll[j] = pointIdxNKNSearch.at(j);
		}
		 int idx = pointIdxNKNSearch[3];
		 dx =  inCloud->points[i].x - inCloud->points[idx].x;
		 dy= inCloud->points[i].y - inCloud->points[idx].y;
		 dz = inCloud->points[i].z - inCloud->points[idx].z;
		double scaleTemp = sqrt(dx*dx + dy*dy+dz*dz);
		pcainfo[i].scale = scaleTemp;
		scale += scaleTemp;
		// cout << "scale: " << scale << endl;

		double t = h_cov_evals.row(0).val[0] + h_cov_evals.row(1).val[0] + h_cov_evals.row(2).val[0] + ( rand()%10 + 1 ) * MINVALUE;
		pcainfo[i].lambda0 = h_cov_evals.row(2).val[0] / t;
		pcainfo[i].normal = h_cov_evectors.row(2).t();

		pcainfo[i].idxIn = pcainfo[i].idxAll;
	}
	scale /= inCloud->points.size();
	magnitd = sqrt(inCloud->points[0].x*inCloud->points[0].x + inCloud->points[0].y*inCloud->points[0].y 
	+ inCloud->points[0].z*inCloud->points[0].z);
}

void PcaSingle(vector<vector<double>>   & pointscur,PCAInfo & pcainfo)
{
	int num = pointscur.size();
	cv::Matx31d h_mean(0,0,0);
	pcainfo.idxIn.resize(num);
	for(int i =0 ;i<pointscur.size();i++)
	{
		pcainfo.idxIn[i] = i;
		h_mean += cv::Matx31d(pointscur[i][0],pointscur[i][1],pointscur[i][2]);
	}
	h_mean *= 1.0/num;
	
	cv::Matx33d h_cov(0,0,0,0,0,0,0,0,0);
	for(int i =0 ;i<pointscur.size();i++)
	{
		cv::Matx31d hi = cv::Matx31d(pointscur[i][0],pointscur[i][1],pointscur[i][2]);
		h_cov += (hi-h_mean)*(hi-h_mean).t();
	}
	h_cov *= 1.0/num;

	cv::Matx33d h_cov_evectors;
	cv::Matx31d h_cov_evals;
	cv::eigen(h_cov,h_cov_evals,h_cov_evectors);

	pcainfo.idxAll = pcainfo.idxIn;
	pcainfo.lambda0 = h_cov_evals.row(2).val[0];
	// pcaInfo.lambda0 = h_cov_evals.row(2).val[0] / ( h_cov_evals.row(0).val[0] + h_cov_evals.row(1).val[0] + h_cov_evals.row(2).val[0] );
	pcainfo.normal = h_cov_evectors.row(2).t();
	pcainfo.planePt = h_mean;

	MCMD_OutlierRemoval(pointscur,pcainfo);
}

void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,pcl::PointCloud<pcl::Normal>::Ptr  & pcNormal ){
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  //创建显示对象
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (inCloud, 0, 255, 0);
    viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(inCloud,single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  //设置点云中每个点的大小
	//加载法向量点云  10表示每隔10个点显示一下法向量 0.05表示法向量的长度
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(inCloud, pcNormal, 5, 0.1, "normals");  

	viewer->addCoordinateSystem(1.0);
	viewer->spin();
}

void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,vector<PCAInfo> & pcainfo){
		pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);
			// 将Normal这个对象初始化，否则会无法进行后面的赋值操作
		pcNormal->points.resize(inCloud->points.size());
		for(int i =0;i<inCloud->points.size();i++)
		{
			pcNormal->points[i].normal_x = pcainfo[i].normal.row(0).val[0];
			pcNormal->points[i].normal_y = pcainfo[i].normal.row(1).val[0];
			pcNormal->points[i].normal_z = pcainfo[i].normal.row(2).val[0];
		}

		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  //创建显示对象
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (inCloud, 0, 255, 0);
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(inCloud,single_color, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  //设置点云中每个点的大小
		//加载法向量点云  10表示每隔10个点显示一下法向量 0.05表示法向量的长度
		viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(inCloud, pcNormal, 5, 0.1, "normals");  

		viewer->addCoordinateSystem(1.0);
		viewer->spin();
}

void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,vector<vector<int>> & regions){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  regions_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	srand(time(0));

	for(int i = 0; i < regions.size();i++) {
		int R = rand() %255;
		int G = rand() %255;
		int B = rand() %255;
		for(int j = 0; j < regions[i].size();j++) {
		 	pcl::PointXYZRGB points;
			points.x = inCloud->points[regions[i][j]].x;
			points.y = inCloud->points[regions[i][j]].y;
			points.z = inCloud->points[regions[i][j]].z;
			points.r = R;
			points.g = G;
			points.b = B;
			regions_cloud->push_back(points);
	 	}
	}

	//  cout<<regions_cloud->size()<<endl;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  //创建显示对象
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color (regions_cloud, rand()%255, rand()%255, rand()%255);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(regions_cloud,"sample cloud");
	viewer->addPointCloud<pcl::PointXYZ>(inCloud,"sample cloud2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  //设置点云中每个点的大小
	//加载法向量点云  10表示每隔10个点显示一下法向量 0.05表示法向量的长度
	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(inCloud, pcNormal, 5, 0.1, "normals");  

	viewer->addCoordinateSystem(1.0);
	viewer->spin();
}

void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,vector<PCAInfo> & pcainfo,int flag)
{
		pcl::PointCloud<pcl::Normal>::Ptr pcNormal(new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr Mean_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			// 将Normal这个对象初始化，否则会无法进行后面的赋值操作
		pcNormal->points.resize(pcainfo.size());
		for(int i =0;i<pcainfo.size();i++)
		{
			pcNormal->points[i].normal_x = pcainfo[i].normal.row(0).val[0];
			pcNormal->points[i].normal_y = pcainfo[i].normal.row(1).val[0];
			pcNormal->points[i].normal_z = pcainfo[i].normal.row(2).val[0];
			pcl::PointXYZ cloud;
			cloud.x = pcainfo[i].planePt.val[0];
			cloud.y = pcainfo[i].planePt.val[1];
			cloud.z = pcainfo[i].planePt.val[2];
			Mean_cloud->push_back(cloud);
		}

		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  //创建显示对象
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (Mean_cloud, 0, 255, 0);
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(inCloud, "sample cloud");
		viewer->addPointCloud<pcl::PointXYZ>(Mean_cloud,single_color,"Mean cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "Mean cloud");  //设置点云中每个点的大小
		//加载法向量点云  10表示每隔10个点显示一下法向量 0.05表示法向量的长度
		viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(Mean_cloud, pcNormal, 1, 0.5, "normals");  

		viewer->addCoordinateSystem(1.0);
		viewer->spin();
}

void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud){

		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  //创建显示对象
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (inCloud, 0, 255, 0);
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(inCloud,single_color, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  //设置点云中每个点的大小
		//加载法向量点云  10表示每隔10个点显示一下法向量 0.05表示法向量的长度
		// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(inCloud, pcNormal, 5, 0.1, "normals");  

		viewer->addCoordinateSystem(1.0);
		viewer->spin();
}

void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr & newCloud){

		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  //创建显示对象
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (newCloud, 0, 255, 0);
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZ>(newCloud,single_color, "sample cloud");
		viewer->addPointCloud<pcl::PointXYZ>(inCloud, "sample cloud2");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  //设置点云中每个点的大小
		//加载法向量点云  10表示每隔10个点显示一下法向量 0.05表示法向量的长度
		// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(inCloud, pcNormal, 5, 0.1, "normals");  

		viewer->addCoordinateSystem(1.0);
		viewer->spin();
}

void regionGrow(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,vector<PCAInfo> & pcainfo,vector<vector<int>> & regions){
		// 这里是保证两个点的法向量之间只是相差15度，作为一个阈值而存在
		cout<<"----- Region  Grow ..."<<endl;

		double thAngle = 15.0/180.0*CV_PI;
		double thNormal = cos(thAngle);

		//将所有点的曲率进行从小到大排序，保证更好的平面可以被选出
		vector<pair<int, double> >  idxsorted(inCloud->points.size());
		for(int i=0; i<inCloud->points.size();i++){
			idxsorted[i].first = i;
			idxsorted[i].second = pcainfo[i].lambda0;
		}
		sort(idxsorted.begin(), idxsorted.end(),[](const std::pair<int,double>& lhs, const std::pair<int,double>& rhs) { return lhs.second < rhs.second; } );

		// 选取前90%的点来进行平面的聚类
		double present = 0.9;
		int idx = int(inCloud->points.size()*present);
		vector<int> IsUsed(inCloud->points.size(),0);
		for(int i = 0; i < idx; ++i){
				int idxStrater =  idxsorted[i].first;
				//如果这个点在前面已经被使用过了，就跳过此次循环
				if(IsUsed[idxStrater]){ continue; }
				cv::Matx31d Startnor = pcainfo[idxStrater].normal;
				double xStrater = inCloud->points[idxStrater].x, yStrater = inCloud->points[idxStrater].y, zStrater = inCloud->points[idxStrater].z;
				double thRadius2 = pow(50*pcainfo[idxStrater].scale, 2);

				//创建一个list来进行存储
				vector<int> clustersTemp;
				// 使用reserve来进行空间的申请，但是并不会创建对象，所以现在size大小还是0
				clustersTemp.reserve(10000);
				clustersTemp.push_back(idxStrater);
				//遍历邻域内的所有点，看看有没有共面的点
				int count =0;

				while(count < clustersTemp.size())
				{
					int idxSeed = clustersTemp[count];
					cv::Matx31d norSeed = pcainfo[idxSeed].normal;
					double thOrtho = pcainfo[idxSeed].scale;

					int num = pcainfo[idxSeed].idxIn.size();

					for(int j = 0;j <num ;++j)
						{
							//邻域中找点
							int idxCur = pcainfo[idxSeed].idxIn[j];
							if(IsUsed[idxCur]){ continue;}
							//第一步判断两个的法向量是否接近平行  向量內积等于 |a||b|cos()   这里的向量已经进行过单位化 
							cv::Matx31d curNor = pcainfo[idxCur].normal;
							double normalDev = abs(curNor.val[0]*Startnor.val[0]+curNor.val[1]*Startnor.val[1]+curNor.val[2]*Startnor.val[2]);
							// double normalDev = abs(curNor.val[0]*norSeed.val[0]+curNor.val[1]*norSeed.val[1]+curNor.val[2]*norSeed.val[2]);
							if(normalDev <  thNormal)
							{
								continue;
							}

							//第二步 判断两者的正交距离  看看两个向量是投影后的距离是不是超过第三点的距离
							double dx = inCloud->points[idxCur].x - xStrater;
							double dy = inCloud->points[idxCur].y - yStrater;
							double dz = inCloud->points[idxCur].z - zStrater;
							// double dOrtho = abs(dx*Startnor.val[0] + dy*Startnor.val[1] + dz*Startnor.val[2]);
							// double dOrtho = abs(dx*norSeed.val[0] + dy*norSeed.val[1] + dz*norSeed.val[2]);
							double dOrtho = abs(dx*curNor.val[0] + dy*curNor.val[1] + dz*curNor.val[2]);
							if(dOrtho > thOrtho)
							{
								continue;
							}


							//第三步判断距离
							double dPara = dx*dx + dy*dy+dz*dz;
							if(dPara > thRadius2)
							{
								continue;
							}

							clustersTemp.push_back(idxCur);
							IsUsed[idxCur] =1;
						}
						count ++;
				}
		// cout << clustersTemp.size() << endl;
		// 对于大于30个点就设置为平面点
		if ( clustersTemp.size() > 0 )
			{
				regions.push_back( clustersTemp );
			}
			else
			{
				//否则就取消标志位，参与到后续的遍历中
				for (int j=0; j<clustersTemp.size(); ++j)
				{
					IsUsed[clustersTemp[j]] = 0;
				}
			}
		}
}

void regionMerging(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,vector<PCAInfo> & pcainfo,vector<vector<int>> & regions,vector<PCAInfo> & patch)
{

	cout<<"----- Region  Merging ..."<<endl;

	double thRegionSize = 600000;

	// vector<PCAInfo> patch ;
	patch.resize(regions.size());
	cout << regions.size() <<endl;
	//计算平面的法向量和中心点
	for(int i=0; i<regions.size(); ++i)
	{
		int pointsnums = regions[i].size();
		vector<vector<double>>  pointsDatacur (pointsnums);
		for(int j=0 ; j < pointsnums ; j++)
		{
			pointsDatacur[j].resize(3);
			pointsDatacur[j][0] = inCloud->points[regions[i][j]].x;
			pointsDatacur[j][1] = inCloud->points[regions[i][j]].y;
			pointsDatacur[j][2] = inCloud->points[regions[i][j]].z;
		}

		PcaSingle(pointsDatacur,patch[i]);

		patch[i].idxAll = regions[i];
		double scaleave = 0.0;

		for(int j=0 ; j< patch[i].idxAll.size(); j++)
		{
			int idx = regions[i][patch[i].idxIn[j]];
			patch[i].idxIn[j] = idx ;
			scaleave += pcainfo[idx].scale;
		}
		scaleave /= patch[i].idxIn.size();
		patch[i].scale = scaleave * 5.0;

	}

	// 设定点到区域的labelIdx
	vector<int> lable (inCloud->points.size(),-1);
	for(int i=0;i<regions.size();i++)
	{
		for(int j=0;j<regions[i].size();j++)
		{
			lable[regions[i][j]] = i;
		}
	}

	// 挨个平面的遍历所有的点，寻找相接的平面
	vector<vector<int> > patchAdjacent(patch.size());   //这个用来保存相接的平面
	for(int i=0;i<patch.size();i++)
	{
		vector<int> patchAdjacentTemp;
		vector<vector<int>> pointsAdjacentTemp;

		for(int j=0;j<patch[i].idxIn.size();j++)
		{
			int start_idx = patch[i].idxIn[j];
			int  start_lable =lable[start_idx];
			for(int k = 0; k<pcainfo[start_idx].idxIn.size();k++)
			{
				int cur_idx = pcainfo[start_idx].idxIn[k];
				int cur_lable  = lable[cur_idx];
				//如果标签属于同一个平面就进行跳过
				if( cur_lable== start_lable ||  cur_lable<0)
				{
					continue;
				}
				//如果出现不一样就反向判断idxcur的knn点中是否含有start_idx
				bool Isneighbor = false;
				for(int n=0;n<pcainfo[cur_idx].idxIn.size();n++){
					if(pcainfo[cur_idx].idxIn[n] == start_idx)
					{
						Isneighbor = true;
					}
				}
				if(!Isneighbor)
				{
					continue;
				}
				// 接受该点所在的平面为相接点
				bool IsIn = false;
				int  n =0;
				for(n =0;n<patchAdjacentTemp.size();++n)
				{
					if(patchAdjacentTemp[n] == cur_lable)
					{
						IsIn = true;
						break;
					}
				}
				if(IsIn)
				{
					pointsAdjacentTemp[n].push_back(cur_idx);
				}
				else{
					patchAdjacentTemp.push_back(cur_lable);
					vector<int> temp;
					temp.push_back(cur_idx);
					pointsAdjacentTemp.push_back(temp);
				}
			}
		}
		for(int j=0;j< pointsAdjacentTemp.size();j++)
		{
			sort(pointsAdjacentTemp[j].begin(),pointsAdjacentTemp[j].end());
			vector<int>::iterator new_end = unique(pointsAdjacentTemp[j].begin(),pointsAdjacentTemp[j].end());
			pointsAdjacentTemp[j].erase(new_end,pointsAdjacentTemp[j].end());
			if(pointsAdjacentTemp[j].size()>3)
			{
					// 如果可行的点超过三个，就认为这个平面是相接平面
					patchAdjacent[i].push_back(patchAdjacentTemp[j]);
			}
		}
	}
	
	//将相接的平面进行拼接
	// vector<vector<int>> new_regions;
	regions.clear();
	vector<int> mergeIdx(patchAdjacent.size(),0);
	for(int i = 0; i < patch.size();i++) {
		if(!mergeIdx[i])
		{
			int Start_Idx = i;
			cv::Matx31d Start_nor = patch[i].normal;
			cv::Matx31d Start_pt = patch[i].planePt;
			double thOrtho = patch[i].scale;

			vector<int> patchIdx;
			patchIdx.push_back(Start_Idx);

			int count=0;
			int total_points =0;
			bool is_Enough =false;
			while(count < patchIdx.size()) 
			{
				int Seed_idx = patchIdx[count];
				cv::Matx31d Seed_nor = patch[Seed_idx].normal;
				cv::Matx31d Seed_pt = patch[Seed_idx].planePt;
				for(int j = 0; j < patchAdjacent[Seed_idx].size();j++) {
					int Cur_Idx = patchAdjacent[Seed_idx][j];
					if(mergeIdx[Cur_Idx])
					{
						continue;
					}
					cv::Matx31d Cur_nor = patch[Cur_Idx].normal;
					cv::Matx31d Cur_pt = patch[Cur_Idx].planePt;
					
					double devAngle = 0.0;
					double devDis = 0.0;
					double thDev = 0.0;
					double thAngle = 15.0/180.0*CV_PI;

					cv::Matx31d Pt_v1 = Cur_pt - Start_pt;
					cv::Matx31d Pt_v2 = Cur_pt - Seed_pt;
					devAngle = acos(Start_nor.val[0]*Cur_nor.val[0]+Start_nor.val[1]*Cur_nor.val[1]+Start_nor.val[2]*Cur_nor.val[2]);
					devDis = abs(Pt_v1.val[0]*Start_nor.val[0]+Pt_v1.val[1]*Start_nor.val[1]+Pt_v1.val[2]*Start_nor.val[2]);

					//如果满足阈值条件，就将两个平面进行拼接
					if(min(devAngle,fabs(CV_PI-devAngle))<thAngle && devDis < thOrtho )
					{
							patchIdx.push_back(Cur_Idx);
							mergeIdx[Cur_Idx] = 1;

							total_points += patch[Cur_Idx].idxAll.size();
							if(total_points >thRegionSize)
							{
								is_Enough = true;
								break;
							}
					}
			}
			if(is_Enough)
					{
						break;
					}
					count ++;
			}
			// 根据结果合并点云
			vector<int> New_Clusters;
			for(int j = 0; j <patchIdx.size();j++){
				int Idx  = patchIdx[j];
				for(int n=0;n<patch[Idx].idxAll.size();n++)
				{
					New_Clusters.push_back(patch[Idx].idxAll[n]);
				}
			}
			if(New_Clusters.size() >0){
				regions.push_back(New_Clusters);
			}
		}
	}


}

void cloudCompletion(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,vector<PCAInfo> & pcainfo,vector<vector<int>> & regions,
pcl::PointCloud<pcl::PointXYZ>::Ptr & newCloud)
{
	cout<<"----- Cloud  Completion ..."<<endl;
	//创建每个点的lable标签
	vector<int> lable(inCloud->points.size(),-1);
	for(int i=0;i<regions.size();i++){
		for(int j=0;j<regions[i].size();j++){
				lable[regions[i][j]]=i;
		}
	}
	//点云补全思路，遍历该平面的所有点，每个点查找其knn点，如果处于同一个标签，就以两个的中值作为中心点插入其中
	for(int i=0;i<regions.size();i++){
		for(int j=0;j<regions[i].size();j++){
			int Start_idx = regions[i][j];
			int Start_lable = lable[Start_idx];
			for(int n=0;n<pcainfo[Start_idx].idxIn.size();n++){
				int Cur_idx = pcainfo[Start_idx].idxIn[n];
				int Cur_lable = lable[Cur_idx];
				//判断其领域内的点是不是属于同一个平面
				if(Start_lable == Cur_lable)
				{
					pcl::PointXYZ cloud;
					cloud.x = (inCloud->points[Start_idx].x+inCloud->points[Cur_idx].x)/2.0;
					cloud.y = (inCloud->points[Start_idx].y+inCloud->points[Cur_idx].y)/2.0;
					cloud.z = (inCloud->points[Start_idx].z+inCloud->points[Cur_idx].z)/2.0;
					inCloud->push_back(cloud);
				}
			}
		}
	}
	//这种处理方式会留下大量的重复点，所以要使用radius_NN进行点的删除
	//首先先创建kd_tree
	vector<int> flags(inCloud->points.size(),1);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(inCloud);
	std::vector<int> index(30);
	std::vector<float> diss(30);
	for(int i=0;i<inCloud->points.size();i++)
	{
		if(flags[i] && tree->nearestKSearch(inCloud->points[i],30,index,diss)>0){
			for(int j=1;j<index.size();j++){
				if(diss[j]<0.0001){
					flags[index[j]] = 0;
				}
			}
		}
	}
	//根据标志位来进行点云的擦除

	for(int i=0;i<inCloud->points.size();i++){
		if(flags[i])
			newCloud->push_back(inCloud->points[i]);
	}
	std::cout << "Total num of points with cloudCompletion : " << newCloud->size() << "\n";
}

void subDivision( std::vector<std::vector<cv::Point> > &straightString, std::vector<cv::Point> &contour, int first_index, int last_index
	, double min_deviation, int min_size )
{
	int clusters_count = straightString.size();

	cv::Point first = contour[first_index];
	cv::Point last = contour[last_index];

	// Compute the length of the straight line segment defined by the endpoints of the cluster.
	int x = first.x - last.x;
	int y = first.y - last.y;
	double length = sqrt( static_cast<double>( (x * x) + (y * y) ) );

	// Find the pixels with maximum deviation from the line segment in order to subdivide the cluster.
	int max_pixel_index = 0;
	double max_deviation = -1.0;

	for (int i=first_index, count=contour.size(); i!=last_index; i=(i+1)%count)
	{
		cv::Point current = contour[i];

		double deviation = static_cast<double>( abs( ((current.x - first.x) * (first.y - last.y)) + ((current.y - first.y) * (last.x - first.x)) ) );

		if (deviation > max_deviation)
		{
			max_pixel_index = i;
			max_deviation = deviation;
		}
	}
	max_deviation /= length;

	// 
	// 	// Compute the ratio between the length of the segment and the maximum deviation.
	// 	float ratio = length / std::max( max_deviation, min_deviation );

	// Test the number of pixels of the sub-clusters.
	int half_min_size=min_size/2;
	if ((max_deviation>=min_deviation) && ((max_pixel_index - first_index + 1) >= half_min_size) && ((last_index - max_pixel_index + 1) >= half_min_size))
	{
		subDivision( straightString, contour, first_index, max_pixel_index, min_deviation, min_size );
		subDivision( straightString, contour, max_pixel_index, last_index, min_deviation, min_size );
	}
	else
	{
		// 
		if ( last_index - first_index > min_size )
		{
			std::vector<cv::Point> straightStringCur;
			for ( int i=first_index; i<last_index; ++i )
			{
				straightStringCur.push_back(contour[i]);
			}
			straightString.push_back(straightStringCur);
			//terminalIds.push_back(std::pair<int,int>(first_index, last_index));
		}
	}
}

void lineFittingSVD(cv::Point *points, int length, std::vector<double> &parameters, double &maxDev)
{
	// 
	cv::Matx21d h_mean( 0, 0 );
	for( int i = 0; i < length; ++i )
	{
		h_mean += cv::Matx21d( points[i].x, points[i].y );
	}
	h_mean *= ( 1.0 / length );

	cv::Matx22d h_cov( 0, 0, 0, 0 );
	for( int i = 0; i < length; ++i )
	{
		cv::Matx21d hi = cv::Matx21d( points[i].x, points[i].y );
		h_cov += ( hi - h_mean ) * ( hi - h_mean ).t();
	}
	h_cov *=( 1.0 / length );

	// eigenvector
	cv::Matx22d h_cov_evectors;
	cv::Matx21d h_cov_evals;
	cv::eigen( h_cov, h_cov_evals, h_cov_evectors );

	cv::Matx21d normal = h_cov_evectors.row(1).t();

	// 
	if ( abs(normal.val[0]) < abs(normal.val[1]) )  // horizontal
	{
		parameters[0] = 0;
		parameters[1] = - normal.val[0] / normal.val[1];
		parameters[2] = h_mean.val[1] - parameters[1] * h_mean.val[0];
	}
	else  // vertical
	{
		parameters[0] = 1;
		parameters[1] = - normal.val[1] / normal.val[0];
		parameters[2] = h_mean.val[0] - parameters[1] * h_mean.val[1];
	}

	// maximal deviation
	maxDev = 0;
	for( int i = 0; i < length; ++i )
	{
		cv::Matx21d hi = cv::Matx21d( points[i].x, points[i].y );
		cv::Matx21d v = hi - h_mean;
		double dis2 = v.dot(v);
		double disNormal = v.dot(normal);
		double disOrtho = sqrt(dis2 - disNormal*disNormal);
		if ( disOrtho > maxDev )
		{
			maxDev = disOrtho;
		}
	}
}

void lineFitting( int rows, int cols, std::vector<cv::Point> &contour, double thMinimalLineLength, std::vector<std::vector<cv::Point2d> > &lines )
{
	// get straight strings from the contour
	double minDeviation = 6.0;
	std::vector<std::vector<cv::Point> > straightString;
	subDivision(straightString, contour, 0, contour.size()-1, minDeviation, int(thMinimalLineLength));
	if ( !straightString.size() )
	{
		return;
	}
	for ( int i=0; i<straightString.size(); ++i )
	{
		if ( straightString[i].size() < thMinimalLineLength )
		{
			continue;
		}

		std::vector<double> parameters( 4 );
		double maxDev = 0.0;
		//bool isOK = lineFittingLS( straightString[i], parameters, maxDev );
		lineFittingSVD(&straightString[i][0], straightString[i].size(), parameters, maxDev);
		//if ( isOK )
		{
			double k = parameters[1];
			double b = parameters[2];
			int lineLen = straightString[i].size();

			double xs = 0, ys = 0, xe = 0, ye = 0;
			if ( ! parameters[0] )  // horizontal
			{
				xs = straightString[i][0].x;
				ys = k * xs + b;
				xe = straightString[i][lineLen-1].x;
				ye = k * xe + b;
			}
			else   // vertical
			{
				ys = straightString[i][0].y;
				xs = k * ys + b;
				ye = straightString[i][lineLen-1].y;
				xe = k * ye + b;
			}

			if ( !( xs==xe && ys==ye ) )
			{
				std::vector<cv::Point2d> lineCur(2);
				lineCur[0] = cv::Point2d(xs, ys);
				lineCur[1] = cv::Point2d(xe, ye);

				lines.push_back( lineCur );
			}
		}
	}
}

bool maskFromPoint( std::vector<cv::Point2d> &pts2d, double radius, double &xmin, double &ymin, double &xmax, double &ymax
, int &margin, cv::Mat &mask )
{
	xmin=10000000, ymin = 10000000;
	xmax=-xmin;
	ymax=-ymin;
	for (int i=0; i<pts2d.size(); ++i)
	{
		if(pts2d[i].x < xmin) { xmin = pts2d[i].x; }
		if(pts2d[i].x > xmax) { xmax = pts2d[i].x; }

		if(pts2d[i].y < ymin) { ymin = pts2d[i].y; }
		if(pts2d[i].y > ymax) { ymax = pts2d[i].y; }
	}

	margin = 4;
	int cols = (xmax-xmin) / radius + 2*margin;
	int rows = (ymax-ymin) / radius + 2*margin;
	if ( cols < 10 || rows < 10 )
	{
		return false;
	}

	mask = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));
	for (int i=0; i<pts2d.size(); ++i)
	{
		int xInt = int((pts2d[i].x-xmin)/radius+0.5+margin);
		int yInt = int((pts2d[i].y-ymin)/radius+0.5+margin);
		mask.at<uchar>(yInt,xInt) = 255;
	}
	return true;
}

void lineFromMask( cv::Mat &mask, int thLineLengthPixel, std::vector<std::vector<std::vector<cv::Point2d> > > &lines )
{
	lines.clear();

	// get mask image via dilate and erode
	cv::Mat mask2;
	cv::dilate(mask, mask2, cv::Mat());
	cv::erode(mask2, mask2, cv::Mat());

	// A. contours
	double thLength = thLineLengthPixel;
	
	std::vector<std::vector<cv::Point> > contours;  
	std::vector<cv::Vec4i> hierarchy;  
	cv::findContours(mask2, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	cv::Mat contous_mask =  cv::Mat::zeros(mask2.size(),CV_8UC1);
	for(int i = 0; i < contours.size();i++){
		cv::drawContours(contous_mask,contours,i,cv::Scalar(255),1,8,hierarchy);
	}
	// cv::imshow("origin", mask);
	// cv::imshow("erode", mask2);
	// cv::imshow("contours", contous_mask);//显示
	// cv::waitKey(0);

	// B. line fitting from the contours
	for ( int i=0; i<contours.size(); ++i )
	{
		if ( contours[i].size() < 4*thLength  )
		{
			continue;
		}

		std::vector<std::vector<cv::Point2d> > lineTemp;
		// 这里通过拟合得出了线段的开始点和结束点
		lineFitting( mask2.rows, mask2.cols, contours[i], thLength, lineTemp );
		lines.push_back(lineTemp);
	}

	cv::Mat line_mask =  cv::Mat::zeros(mask2.size(),CV_8UC1);
	for(int i=0; i<lines.size(); ++i)
	{
		for(int j=0; j<lines[i].size(); ++j)
		{
			cv::Point2d start = lines[i][j][0] ;
			cv::Point2d end = lines[i][j][1];
			cv::line(mask, start, end, cv::Scalar(89,125,90), 3);
		}
	}
	cv::imshow("line", mask);
	cv::waitKey(0);


}

void planeBase3DLineDetection(pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud,
vector<vector<int>> & regions,double scale,vector<PLANE> & planes,
vector<PCAInfo> & pcainfo)
{
	double thAngle = 10.0/180*CV_PI;
	double thLineLength = 8.0* scale;
	int numPatchs = regions.size();
	vector<PCAInfo> patch(numPatchs);

	//	首先计算出每个平面的法向量
	for(int i = 0; i < numPatchs;i++) {

		int pointNumCur = regions[i].size();
		vector<vector<double >> pointsDataCur(pointNumCur);

		for(int j = 0; j < pointNumCur;j++) {
			int idx = regions[i][j];
			pointsDataCur[j].resize(3);
			pointsDataCur[j][0]= inCloud->points[idx].x;
			pointsDataCur[j][1] = inCloud->points[idx].y;
			pointsDataCur[j][2] = inCloud->points[idx].z;
		}
		PcaSingle(pointsDataCur, patch[i]);

		patch[i].idxAll = regions[i];
		for(int j=0; j<regions[i].size(); j++)
		{
			int idx = patch[i].idxIn[j];
			patch[i].idxIn[j]=regions[i][idx];
		}		
	}

	planes.resize(patch.size());
	//三维线段检测 每个平面来一次
	for(int i=0; i<patch.size(); i++)
	{
		// 步骤一 将三维点投影到二维平面
		vector<cv::Point2d> pts2d;
		vector<double> ptscales ;

		bool initialized = false;
		// vx 和 vy 是坐标轴的方向矢量
		cv::Mat_<double> vx,vy;
		cv::Mat_<double> planePt = (cv::Mat_<double>(3,1) << patch[i].planePt.val[0],patch[i].planePt.val[1],patch[i].planePt.val[2]);
		cv::Mat_<double> normal = (cv::Mat_<double>(3,1 ) << patch[i].normal.val[0],patch[i].normal.val[1],patch[i].normal.val[2]);
		
		//初始化坐标系
		for (int j = 0; j <patch[i].idxAll.size();j++)  {
			int id = patch[i].idxAll[j];
			cv::Mat_<double> pt3d = (cv::Mat_<double>(3,1) << inCloud->points[id].x,inCloud->points[id].y,inCloud->points[id].z);
			//这个就是pcpi的向量
			cv::Mat_<double> v3d = pt3d - planePt;  
			cv::Mat_<double> vOrtho = v3d.dot(normal)*normal;
			cv::Mat_<double> vPlane = v3d - vOrtho;
			cv::Mat_<double> ptplane = planePt + vPlane;
			if(!initialized)
			{
				vx = vPlane*1.0/(cv::norm(vPlane));
				vy = vx.cross(normal);
				vy = vy*1.0/(cv::norm(vy));
				initialized =true;
			}
			if(initialized)
			{
				double x = vPlane.dot(vx);
				double y = vPlane.dot(vy);
				pts2d.push_back(cv::Point2d(x,y));
				ptscales.push_back(pcainfo[id].scale);
			}
		}

		// 3D -2D 投影
		double gridSideLength = 0;
		sort(ptscales.begin(),ptscales.end(),[](const double & lhs,const double & rhs){
			return lhs < rhs;});
		int idxNinety = min(int(double(ptscales.size())*0.90),int(ptscales.size()-1));
		gridSideLength = ptscales[idxNinety]*0.75; 

		//3d 到2d的投影
		double xmin,ymin,xmax,ymax;
		int  margin = 0;
		cv::Mat mask;
		bool isok = maskFromPoint(pts2d,gridSideLength,xmin,ymin,xmax,ymax,margin,mask);
		if(!isok){
			continue;
		}
		// cv::Mat mask_big;
		// cv::resize(mask,mask_big,cv::Size(),2,2);
		// // cv::namedWindow("input", CV_WINDOW_AUTOSIZE);//构建一个窗口，自适应图片大小
        // cv::imshow("input", mask);//显示
        // cv::waitKey(0);

		// 2d线段检测
		int thLineLengthPixel = max(thLineLength/gridSideLength,10.0);
		std::vector<std::vector<std::vector<cv::Point2d> > > lines2d;
		lineFromMask( mask, thLineLengthPixel, lines2d );
		if (!lines2d.size())
		{
			continue;
		}

		// 2D-3D重投影  对于检测出来的每个连通域的每条线
		planes[i].scale = gridSideLength;
		for ( int m=0; m<lines2d.size(); ++m ) 
		{
			std::vector<std::vector<cv::Point3d> > temp;
			for (int n=0; n<lines2d[m].size(); ++n)
			{
				double length = abs(lines2d[m][n][1].x-lines2d[m][n][0].x) + abs(lines2d[m][n][1].y-lines2d[m][n][0].y);
				if ( length < thLineLengthPixel )
				{
					continue;
				}

				lines2d[m][n][0].x = (lines2d[m][n][0].x - margin) * gridSideLength + xmin;
				lines2d[m][n][0].y = (lines2d[m][n][0].y - margin) * gridSideLength + ymin;

				lines2d[m][n][1].x = (lines2d[m][n][1].x - margin) * gridSideLength + xmin;
				lines2d[m][n][1].y = (lines2d[m][n][1].y - margin) * gridSideLength + ymin;

				cv::Mat_<double> xs = lines2d[m][n][0].x * vx;
				cv::Mat_<double> ys = lines2d[m][n][0].y * vy;
				cv::Mat_<double> pts = planePt + xs + ys;

				cv::Mat_<double> xe = lines2d[m][n][1].x * vx;
				cv::Mat_<double> ye = lines2d[m][n][1].y * vy;
				cv::Mat_<double> pte = planePt + xe + ye;

				std::vector<cv::Point3d> line3dTemp(2);
				line3dTemp[0] = cv::Point3d(pts(0), pts(1), pts(2));
				line3dTemp[1] = cv::Point3d(pte(0), pte(1), pte(2));

				temp.push_back( line3dTemp );
			}
			if (temp.size())
			{
				planes[i].lines3d.push_back(temp);
			}
		}
	}

	int nums_line =0;
	for(int i = 0; i < planes.size();i++){
		for(int j = 0; j < planes[i].lines3d.size();j++)
		nums_line+=planes[i].lines3d[j].size();
	}
	cout << "nums_line" << nums_line << endl;
}

void writeOutLines( string filePath,pcl::PointCloud<pcl::PointXYZ>::Ptr & inCloud, std::vector<std::vector<cv::Point3d> > &lines, double scale )
{
	// write out bounding polygon result
	pcl::PointCloud<pcl::PointXYZ>::Ptr lines_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	string fileEdgePoints = filePath + "lines.txt";
	FILE *fp2 = fopen( fileEdgePoints.c_str(), "w");
	for (int p=0; p<lines.size(); ++p)
	{
		int R = rand()%255;
		int G = rand()%255;
		int B = rand()%255;

		cv::Point3d dev = lines[p][1] - lines[p][0];
		double L = sqrt(dev.x*dev.x + dev.y*dev.y + dev.z*dev.z);
		int k = L/(scale/10);

		double x = lines[p][0].x, y = lines[p][0].y, z = lines[p][0].z;
		double dx = dev.x/k, dy = dev.y/k, dz = dev.z/k;
		for ( int j=0; j<k; ++j)
		{
			x += dx;
			y += dy;
			z += dz;
			
			pcl::PointXYZ cloud;
			cloud.x = x;
			cloud.y = y;
			cloud.z = 	z;
			lines_cloud->push_back(cloud);

			fprintf( fp2, "%.6lf   %.6lf   %.6lf    ", x, y, z );
			fprintf( fp2, "%d   %d   %d   %d\n", R, G, B, p );
		}
	}
	fclose( fp2 );
	Visualization(inCloud,lines_cloud);
}

int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<PLANE> planes;
	vector<vector<cv::Point3d> >  lines;
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	vector<PCAInfo> pcainfo;
	vector<PCAInfo> patch;
	vector<vector<int>> regions;
	double scale,magnitd;
	
	
	int k = 20;
	
	string fileData = "/home/liuhy/workspace/pcl/test1/dataseg_cloud.txt";
	string fileOut  = "/home/liuhy/workspace/pcl/test1/data";
	// string filename = "/home/liuhy/workspace/pcl/test1/pt.ply";
	// 从txt中读取文件
	readDataFromtxt(fileData , inCloud);
	// readDataFromtxt(fileData , oldCloud);
	// Transpcd2txt(filename,inCloud);
	// Transply2txt(filename,inCloud);
	// pcl自带的点法向量计算公式
	// Normal_Calculation(inCloud,pcNormal,cloud_with_normals);
	// 计算k临近，并且计算它的pca信息 scale是点与第三临近点的比值 ，magnitd是第一个点到原点距离的平方
	Pca(k,inCloud,pcainfo,scale,magnitd);
	// 区域生长，将很多的点长到一块儿去
	regionGrow(inCloud,pcainfo,regions);

	// cloudCompletion(inCloud,pcainfo,regions,newCloud);
	//区域合并，将差不多角度的平面进行一个合并
	regionMerging(inCloud,pcainfo,regions,patch);
	// 将新的点云保存为txt文件
	// Transpcd2txt(newCloud);

	// 点云和它向量的可视化程序
	Visualization(inCloud,regions); 
	//步骤二  3D线段的检测
	planeBase3DLineDetection(inCloud,regions,scale,planes,pcainfo);

	// postProcessing( planes, lines );
	for (int i=0; i<planes.size(); ++i)
	{
		for (int m=0; m<planes[i].lines3d.size(); ++m)
		{
			for (int n=0; n<planes[i].lines3d[m].size(); ++n)
			{
				lines.push_back(planes[i].lines3d[m][n]);
			}
		}
	}

	cout<<"lines number: "<<lines.size()<<endl;
	cout<<"planes number: "<<planes.size()<<endl;

	writeOutLines( fileOut,inCloud, lines, scale );
	// pcl::io::savePCDFile("plane_cloud_out.pcd", *cloud_with_normals);
 
	return 0;
}

 