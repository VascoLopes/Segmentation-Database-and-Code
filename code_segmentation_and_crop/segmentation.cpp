#include <iostream>
#include <pcl/io/pcd_io.h>
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
#include <vector>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/min_cut_segmentation.h>
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

	int
main (int argc, char *argv[])
{
	double xyz = 0.0;

	PointCloudT::Ptr	cloud (new PointCloudT),
		cloud_inliers (new PointCloudT),
		cloud_outliers (new PointCloudT);

	std::string pastas[] = {"../../Blue_Canteen/","../../Cardboard_Box/","../../Cardboard_Cup/","../../Clay_Cup/","../../Gel_Tube/","../../Headphones/","../../Paper_Holder/","../../Water_Bottle/"};
	
	for(int currword = 0; currword <= 7; currword++)
	{

	std::cout << pastas[currword] << std::endl;
	for(int i = 1; i <= 20; i++){

		//initialize time counter
		std::clock_t start;
		double duration;


		// Load point cloud
		char numstr[21];
		std::string s;
		std::ostringstream oss;
		oss << pastas[currword] << i << ".pcd";
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
		extract.setNegative (true);				// Extract the outliers
		extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

		
		// Writing to disk
		std::ostringstream sss,ssss;

  		pcl::IndicesPtr indices (new std::vector <int>);

		sss << "./pcds/" << i << "_segmented.pcd";
		/* pcl::io::savePCDFileASCII (sss.str(), *cloud_outliers); */

		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud (cloud_outliers);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0, 1.0);
		pass.filter (*indices);

		pcl::MinCutSegmentation<pcl::PointXYZ> minseg;
		minseg.setInputCloud (cloud_outliers);
		minseg.setIndices (indices);

		pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointXYZ point;
		/*
		point.x = 68.97;
		point.y = -18.55;
		point.z = 0.57;
		foreground_points->points.push_back(point);
		minseg.setForegroundPoints (foreground_points);

		minseg.setSigma (0.25);
		minseg.setRadius (3.0433856);
		minseg.setNumberOfNeighbours (14);
		minseg.setSourceWeight (0.8);
		*/
		////
		//-0.053357, 0.027643, 0.675000]
		point.x = -0.054457;
		point.y = 0.027643;
		point.z = 0.675000;
		foreground_points->points.push_back(point);
		minseg.setForegroundPoints (foreground_points);

		minseg.setSigma (0.25);
		minseg.setRadius (0.3);
		minseg.setNumberOfNeighbours (10);
		minseg.setSourceWeight (0.8);
		////
		std::vector <pcl::PointIndices> clusters;
		minseg.extract (clusters);

  		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = minseg.getColoredCloud ();
		ssss << "./pcds/" << i << "_segmented_ivo.pcd";
		/* pcl::io::savePCDFileASCII (ssss.str(), *colored_cloud); */
		int j = 0;
		std::cout << "BeforeCycle"<< std::endl;
		for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
		{

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
			int aux;
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			{
				/* std::cout << "C"<< std::endl; */
				cloud_cluster->points.push_back (cloud_outliers->points[*pit]); //*
				aux++;
			}
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			std::stringstream ss;

			ss << pastas[currword] << i << "_croped_.pcd";
			//ss << "../../Blue_Canteen/" << i << ".pcd";
			//ss << "./pcds/" << i << "_segmented_ivo_cluster"<< j << ".pcd";
			/* ss << "cloud_cluster_" << j << ".pcd"; */
			/* writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); */
			pcl::io::savePCDFileASCII (ss.str(), *cloud_cluster);
			
			j++;
		}
		duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
		xyz = xyz + duration;
		std::cout << "Time:" << duration << std::endl;
		/* pcl::io::savePCDFileASCII (ssss.str(), *iout); */

		//while (!viewer.wasStopped ()) {
		//	viewer.spinOnce ();
		//}
	}
	}
	std::cout << "Total :"<< xyz << "\n Avg:"<< xyz/20 << std::endl;
	return (0);
}
