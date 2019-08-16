/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <chrono>
#include <unordered_map> 
#include <queue>

using namespace std;
using namespace pcl;
#include "pcl_utilities.hpp"


int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

   if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/rex/Desktop/GIT/PCL_Utilities/test.pcd", *cloud) == -1) //* load the file
   {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
   }
   std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
   pcl::PointCloud<pcl::PointXYZRGB> cloud_np;

   for(size_t i=0;i<cloud->points.size();i++)
   	cloud_np.push_back(cloud->points[i]);

   std::cout<<"Here"<<endl;

   vector<vector<double> > v = PCLUtilities::pclToVector<PointXYZRGB>(cloud_np);
   std::cout<<v.size()<<endl;
   
   PCLUtilities::pclToXYZ<PointXYZRGB>(cloud_np,"hello.csv");
  
   return (0);
}