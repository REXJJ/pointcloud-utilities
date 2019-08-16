#ifndef PCL_UTILITIES_HPP
#define PCL_UTILITIES_HPP

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
#include <fstream>
#include <thread>
#include <ctime>

using namespace std;
using namespace pcl;

class pcl_utilities
{  
public:

   vector<int> static splitRGBData(float rgb)
   {
   	uint32_t data = static_cast<uint32_t>(rgb);
   	vector<int> d;
   	uint32_t p=1;
    for(int i=0;i<3;i++)
   	{
   		d.push_back((data>>p) & 0x0000ff);
   		p=p*8;
   	}
   	reverse(d.begin(),d.end());
   	return d;
   }

   template <typename PointT> static vector<vector<double> > pclToVector(pcl::PointCloud<PointT> p)
	{
		vector<vector<double> > v;
		for(size_t i=0;i<p.points.size();i++){
			vector<double> d;
			for(size_t j=0;j<3;j++)
				d.push_back(p.points[i].data[j]);
			v.push_back(d);
		}
	return v;
	}

	template <typename PointT> static void pclToCSV(pcl::PointCloud<PointT> p, std::string filename)
	{
		ofstream f;
		f.open(filename);
		vector<vector<double> > v = pclToVector<PointT>(p);
		for(int i=0;i<v.size();i++)
		{
			for(int j=0;j<3;j++)
				f<<v[i][j]<<",";
			vector<int> c = splitRGBData(v[i][4]);
            for(int j=0;j<c.size()-1;j++)
            	f<<c[j]<<",";
			f<<c[c.size()-1]<<"\n";
		}
		f.close();
	}

	template <typename PointT> static void pclToXYZ(pcl::PointCloud<PointT> p, std::string filename)
	{
		ofstream f;
		f.open(filename);
		vector<vector<double> > v = pclToVector<PointT>(p);
		for(int i=0;i<v.size();i++)
		{
			for(int j=0;j<2;j++)
				f<<v[i][j]<<",";
			vector<int> c = splitRGBData(v[i][4]);
			f<<v[i][2]<<"\n";
		}
		f.close();
	}

};

#endif



