

class pcl_utilities
{  
public:
   template <typename PointT> static vector<vector<double> > pclToVector(pcl::PointCloud<PointT> p);

/*   vector<int> splitRGBData(float rgb)
   {
   	uint32_t data = *reinterpret_cat<int*>(rgb);
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
};




int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

   if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/rex/Desktop/GIT/Debugging_Utilities/test.pcd", *cloud) == -1) //* load the file
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

   vector<vector<double> > v = pcl_utilities::pclToVector<PointXYZRGB>(cloud_np);
   std::cout<<v.size()<<endl;

   for(auto x: v)
   	std::cout<<x[3]<<endl;
   return (0);
}