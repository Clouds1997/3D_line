#include <iostream>
#include <fstream>
using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry> 
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h> 
#include <pcl/io/pcd_io.h> 
#include <omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<pcl/point_cloud.h>
#include<pcl/kdtree/kdtree_flann.h>


// #include <pcl/visualization/cloud_viewer.h>

// 定义点云使用的格式：这里用的是XYZRGB
typedef pcl::PointXYZRGB PointT; 
typedef pcl::PointCloud<PointT> PointCloud;

bool dbscan(const PointCloud::Ptr& cloud_in, std::vector<std::vector<int>> &clusters_index, const double& r, const int& size)
{
    if (!cloud_in->size())
        return false;
    //LOG()
    pcl::KdTreeFLANN<PointT> tree;
    tree.setInputCloud(cloud_in);
    std::vector<bool> cloud_processed(cloud_in->size(), false);

    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        if (cloud_processed[i])
        {
            continue;
        }

        std::vector<int>seed_queue;
        //检查近邻数是否大于给定的size（判断是否是核心对象）
        std::vector<int> indices_cloud;
        std::vector<float> dists_cloud;
        if (tree.radiusSearch(cloud_in->points[i], r, indices_cloud, dists_cloud) >= size)
        {
            seed_queue.push_back(i);
            cloud_processed[i] = true;
        }
        else
        {
            //cloud_processed[i] = true;
            continue;
        }

        int seed_index = 0;
        int R = rand() %255;
		int G = rand() %255;
		int B = rand() %255;
        while (seed_index < seed_queue.size())
        {
            std::vector<int> indices;
            std::vector<float> dists;
            if (tree.radiusSearch(cloud_in->points[seed_queue[seed_index]], r, indices, dists) < size)//函数返回值为近邻数量
            {
                //cloud_processed[i] = true;//不满足<size可能是边界点，也可能是簇的一部分，不能标记为已处理
                ++seed_index;
                continue;
            }
            for (size_t j = 0; j < indices.size(); ++j)
            {
                if (cloud_processed[indices[j]])
                {
                    continue;
                }
                cloud_in->points[indices[j]].r = R;
                cloud_in->points[indices[j]].g = G;
                cloud_in->points[indices[j]].b = B;
                seed_queue.push_back(indices[j]);
                cloud_processed[indices[j]] = true;
            }
            ++seed_index;                
        }
        clusters_index.push_back(seed_queue);               
      
    }
   // std::cout << clusters_index.size() << std::endl;

    if (clusters_index.size())
        return true;
    else
        return false;
}

int main( int argc, char** argv )
{
    srand(time(0));
    cv::Mat colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;         // 相机位姿

    boost::format fmt( "../%s/%d.%s" ); //图像文件格式
    colorImgs = cv::imread( (fmt%"color"%1%"png").str());
    depthImgs = cv::imread( (fmt%"depth"%1%"png").str(), -1 ); 
    std::ifstream resultReader(  (fmt%"result"%1%"txt" ).str() );

    string name;
    double accuracy;
    double startv, startu, w, h;

    vector<string> names;
    vector<double> accuracys;
    vector<double> startvs, startus, ws, hs;

    if( resultReader.is_open() )
	{
		while ( !resultReader.eof() ) 
		{
			resultReader >> name >> accuracy >> startu >> startv >> w >> h;
            names.push_back(name);
            accuracys.push_back(accuracy);
            startvs.push_back(startv);
            startus.push_back(startu);
            ws.push_back(w);
            hs.push_back(h);
		}
		resultReader.close();
	}
    
    // 计算点云并拼接
    // 相机内参 
    // double cx = 319.50;
    // double cy = 239.50;
    // double fx = 481.20;
    // double fy = 480.00;
    // double depthScale = 1000.0;

    double cx = 320.1;
    double cy = 247.6;
    double fx = 535.4;
    double fy = 539.2;
    double depthScale = 1000.0;
    
    cout<<"正在将图像转换为点云..."<<endl;
    
    // 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud ); 

    for ( int v=startvs[1]; v<startvs[1]+ ws[1]; v++ )
        for ( int u=startus[1]; u<startus[1] + hs[1]; u++ )
        {
            unsigned int d = depthImgs.ptr<unsigned short> ( v )[u]; // 深度值
            if ( d==0 ) continue; // 为0表示没有测量到
            Eigen::Vector3d point; 
            point[2] = double(d)/depthScale; 
            point[0] = (u-cx)*point[2]/fx;
            point[1] = (v-cy)*point[2]/fy; 
            
            PointT p ;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            // p.b = colorImgs.data[ v*colorImgs.step+u*colorImgs.channels() ];
            // p.g = colorImgs.data[ v*colorImgs.step+u*colorImgs.channels()+1 ];
            // p.r = colorImgs.data[ v*colorImgs.step+u*colorImgs.channels()+2 ];
            pointCloud->points.push_back( p );
        }
    
    std::vector<std::vector<int>> clusters_index;
    dbscan(pointCloud, clusters_index, 0.3, 10);

    //  //显示点云
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("pointcloud"));
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));  //创建显示对象
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> depth_color(pointCloud);
    viewer->addPointCloud<PointT>(pointCloud,"depth_cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->spin();
    return 0;
}
