# PointCloudUtilities
This is a header file that provides some utility functions for operating on point clouds.

For easy reference, the functions provided with this file are:

- vector<vector<double> > pclToVector(const pcl::PointCloud<PointT>& p)
- MatrixXf pclToEigen(const pcl::PointCloud<PointT>& p)
- pcl::PCLPointCloud2 pclToPointCloud2(const pcl::PointCloud<PointT>& p)
- vector<vector<float> > pointCloud2ToVec(const pcl::PCLPointCloud2& p)
- void pclToCSV(const pcl::PointCloud<PointT>& p, std::string filename)
- void pclToXYZ(const pcl::PointCloud<PointT>& p, std::string filename)
- void xyzToPcd (const string &input_file, const string &output_file)
- pcl::PCLPointCloud2 pcdToPointCloud2 (const std::string &filename)
- void pointCloud2ToPly (const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool format)
- pcl::PointCloud<PointT> PlyToPcl(std::string filename)
- void PclToPcd(std::string filename,const pcl::PointCloud<PointT>& cloud)
- pcl::PointCloud<PointT> PcdToPcl(std::string filename)
- void visualizePointCloud(const pcl::PointCloud<PointT>& cloud)
- void visualizeMesh(pcl::PolygonMesh triangles)
