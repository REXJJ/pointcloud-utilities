#include <iostream>
#include "pcl_utilities.hpp"


template <typename PointT> vector<vector<double> > pcl_utilities::pclToVector(pcl::PointCloud<PointT> p)
{
	vector<vector<double> > v;
	for(size_t i=0;i<p.points.size();i++){
		vector<double> d;
		for(size_t j=0;j<4;j++)
		d.push_back(p.points[i].data[j]);
		v.push_back(d);
	}
	return v;
}


/*   vector<int> splitRGBData(float rgb)
   {
   	uint32_t data = *reinterpret_cast<int*>(rgb);
   	vector<int> d;
   	uint32_t l = data;
   	uint32_t p=1;
   	while(l)
   	{
   		d.push_back((data>>p) & 0x0000ff);
   		p=p*8;
   		l = data>>p;
   	}
   	return d;
   }*/