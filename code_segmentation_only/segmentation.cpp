#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <Eigen/Dense>
#include <string>
#include <sstream>
#include <cstdio>
#include <ctime>
#include <time.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int
main (int argc, char *argv[])
{

	PointCloudT::Ptr	cloud (new PointCloudT),
						cloud_inliers (new PointCloudT),
						cloud_outliers (new PointCloudT);
						
	for(int i = 1; i <= 20; i++){
		//initialize time counter
		std::clock_t start;
		double duration;


		// Load point cloud
		char numstr[21];
		std::string s;
		std::ostringstream oss;
		oss << "/home/vasco/Documents/Kinect_Images_Database/Blue_Canteen/" << i << ".pcd";
		if (pcl::io::loadPCDFile (oss.str(), *cloud) < 0) {
			PCL_ERROR ("Could not load PCD file !\n");
			continue;
		}

		// Start time counter
		start = std::clock();

		// Segment the ground
		pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr 		inliers_plane (new pcl::PointIndices);
		PointCloudT::Ptr 			cloud_plane (new PointCloudT);

		// Make room for a plane equation (ax+by+cz+d=0)
		plane->values.resize (4);

		pcl::SACSegmentation<PointT> seg;				// Create the segmentation object
		seg.setOptimizeCoefficients (true);				// Optional
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setDistanceThreshold (0.005f);
		seg.setInputCloud (cloud);
		seg.segment (*inliers_plane, *plane);

		if (inliers_plane->indices.size () == 0) {
			PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
			continue;
		}

		// Extract inliers
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (inliers_plane);
		//extract.setNegative (false);			// Extract the inliers
		//extract.filter (*cloud_inliers);		// cloud_inliers contains the plane

		// Extract outliers
		//extract.setInputCloud (cloud);		// Already done line 50
		//extract.setIndices (inliers);			// Already done line 51
		extract.setNegative (true);				// Extract the outliers
		extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

		//printf ("Plane segmentation equation [ax+by+cz+d]=0: [%3.4f | %3.4f | %3.4f | %3.4f]     \t\n", 
		//		plane->values[0], plane->values[1], plane->values[2] , plane->values[3]);

		// Visualization
		//pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

		//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_inliers_handler (cloud, 255, 20, 20); // Plane in RED
		//viewer.addPointCloud (cloud_inliers, cloud_inliers_handler, "cloud inliers");

		//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_outliers_handler (cloud, 200, 200, 200); // Everything else in GRAY
		//viewer.addPointCloud (cloud_outliers, cloud_outliers_handler, "cloud outliers");	

		duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
		std::cout << duration <<'\n';

		// Writing to disk
		std::ostringstream sss;
		sss << "/home/vasco/Documents/Kinect_Images_Database/Blue_Canteen/" << i << "_segmented.pcd";
		pcl::io::savePCDFileASCII (sss.str(), *cloud_outliers);


		//while (!viewer.wasStopped ()) {
		//	viewer.spinOnce ();
		//}
	}
	return (0);
}
