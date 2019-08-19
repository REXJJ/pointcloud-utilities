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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>

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

#include <Eigen/Dense>

using namespace std;
using namespace pcl;
using namespace std::chrono_literals;
using namespace Eigen;


class PCLUtilities
{  
public:

    static vector<int> splitRGBData(float rgb)
    {
	   	uint32_t data = *reinterpret_cast<int*>(&rgb);
	   	vector<int> d;
	   	int a[3]={16,8,1};
	    for(int i=0;i<3;i++)
	   	{
	   		d.push_back((data>>a[i]) & 0x0000ff);
	   	}
	   	return d;
    }


    template <typename PointT> static vector<double> pointsToVector(PointT t)
    {
    	vector<double> d;
    	for(int i=0;i<3;i++)
    		d.push_back(t.data[i]);
    	return d;
    }

    /***************************************************************************/
    //Point Cloud to other Data Structures
    /***************************************************************************/
    template <typename PointT> static vector<vector<double> > pclToVector(const pcl::PointCloud<PointT>& p)
	{
		vector<vector<double> > v;
		for(size_t i=0;i<p.points.size();i++)
			v.push_back(pointsToVector(p.points[i]));
	    return v;
	}

    template <typename PointT> static MatrixXf pclToEigen(const pcl::PointCloud<PointT>& p)
	{
        MatrixXf m = p.getMatrixXfMap().transpose();
        if(m.cols()<5) return m;
        for(int i=0;i<m.rows();i++)
        		{
        			vector<int> colors = splitRGBData(double(m(i,4)));
        			m(i,4) = float(colors[0]); 
        			m(i,5) = float(colors[1]);
        			m(i,6) = float(colors[2]);
        		}
        return m;
	}

    template <typename PointT> static pcl::PCLPointCloud2 pclToPointCloud2(const pcl::PointCloud<PointT>& p)
	{
		pcl::PCLPointCloud2 cloud;
		pcl::toPCLPointCloud2(p,cloud);
		return cloud;
	}	


    /***************************************************************************/
    //Other Data Structures to Point Cloud
    /***************************************************************************/
 
    static vector<vector<float> > pointCloud2ToVec(const pcl::PCLPointCloud2& p)
	{
        vector<vector<float> > v; 
        for(int i=0;i<p.row_step;i+=p.point_step)
        {
        	vector<float> t;
        	for(int j=0;j<3;j++)
        	{
         		if(p.fields[j].count==0)
        		{
        			continue;
        		}
    		    float x;
        		memcpy(&x,&p.data[i+p.fields[j].offset],sizeof(float));
        		t.push_back(x);
        	}
       		if(p.point_step>16)
       		{
       			float rgb;
       			memcpy(&rgb,&p.data[i+p.fields[3].offset],sizeof(float));
       			vector<int> c = splitRGBData(rgb);
       			for(int k=0;k<3;k++)
       				t.push_back(float(c[k]));
       		}        	
        	v.push_back(t);
        }
        return v;
	}	



	/*****************************************************************************/
	//Point Cloud File Handling Functions
	/****************************************************************************/	

	template <typename PointT> static void pclToCSV(const pcl::PointCloud<PointT>& p, std::string filename)
	{
		ofstream f;
		f.open(filename);
		MatrixXf v = pclToEigen<PointT>(p);
		for(int i=0;i<v.rows();i++)
		{
			for(int j=0;j<v.cols()-1;j++)
				f<<v(i,j)<<",";
			f<<v(i,v.cols()-1)<<"\n";
		}
		f.close();
	}

	template <typename PointT> static void pclToXYZ(const pcl::PointCloud<PointT>& p, std::string filename)
	{
		ofstream f;
		f.open(filename);
		MatrixXf v = pclToEigen<PointT>(p);
		for(int i=0;i<v.rows();i++)
		{
			for(int j=0;j<2;j++)
				f<<v(i,j)<<",";
			f<<v(i,2)<<"\n";
		}
		f.close();
	}

	static void xyzToPcd (const string &input_file, const string &output_file)
	{
		ifstream fs;
		fs.open (input_file.c_str (), ios::binary);
		if (!fs.is_open () || fs.fail ())
		{
			PCL_ERROR ("Could not open file '%s'! Error : %s\n", input_file.c_str (), strerror (errno)); 
			fs.close ();
			return ;
		}
		string line;
		vector<string> st;
		pcl::PointCloud<pcl::PointXYZ> cloud;
		while (!fs.eof ())
		{
			getline (fs, line);
			// Ignore empty lines
			if (line.empty())
				continue;
			boost::trim (line);
			boost::split (st, line, boost::is_any_of ("\t\r "), boost::token_compress_on);
			if (st.size () != 3)
				continue;
			cloud.push_back (PointXYZ (float (atof (st[0].c_str ())), float (atof (st[1].c_str ())), float (atof (st[2].c_str ()))));
		}
		fs.close ();
		cloud.width = uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
		// Convert to PCD and save
		PCDWriter w;
		w.writeBinaryCompressed (output_file, cloud);
	}

	pcl::PCLPointCloud2 pcdToPointCloud2 (const std::string &filename)
	{
		std::cout<<"Loading ";
		pcl::PCLPointCloud2 cloud;
		pcl::PLYReader reader;
		if (reader.read (filename, cloud) < 0)
			PCL_ERROR ("Unable to read the file. \n"); 
		return cloud;
	}
	void pointCloud2ToPly (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format)
	{
		std::cout<<"Saving \n"; printf ("%s ", filename.c_str ());
		pcl::PCDWriter writer;
		writer.write (filename, cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), format);
	}

	template <typename PointT> static pcl::PointCloud<PointT> PlyToPcl(std::string filename)
	{
		pcl::PointCloud<PointT> cloud;
        pcl::PLYReader Reader;
        Reader.read(filename, cloud);
    }

    template <typename PointT> static void PclToPcd(std::string filename,const pcl::PointCloud<PointT>& cloud)
    {
    	 pcl::io::savePCDFileASCII (filename, cloud);
    }
  

	template <typename PointT> static pcl::PointCloud<PointT> PcdToPcl(std::string filename)
    {
		pcl::PointCloud<PointT> cloud;
		if (pcl::io::loadPCDFile<PointT> (filename,cloud) == -1)
		{
			PCL_ERROR ("Couldn't read the inputted file. \n");
		}
		return cloud;
    } 

	/*******************************************************************/
	//Visualization Utilities
	/******************************************************************/

	template <typename PointT> static void visualizePointCloud(const pcl::PointCloud<PointT>& cloud)
	{
    	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		viewer->addPointCloud<PointT> (cloud.makeShared(), "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		while (!viewer->wasStopped ())
		{
		  	viewer->spinOnce (100);
		  	std::this_thread::sleep_for(100ms);
		}
	}

	static void visualizeMesh(pcl::PolygonMesh triangles)
	{
		pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		viewer->addPolygonMesh(triangles,"meshes",0);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "meshes");
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0.7, 0.7, 0,"meshes"); 
        viewer->setRepresentationToWireframeForAllActors(); 
		while (!viewer->wasStopped ())
		{
		    viewer->spinOnce (100);
		    std::this_thread::sleep_for(100ms);
		}
	}
};

#endif



