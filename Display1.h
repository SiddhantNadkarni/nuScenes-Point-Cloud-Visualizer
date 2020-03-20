#pragma once

/*C++ Headers*/
#include <iostream>
#include <thread>

/*PCL Headers*/
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

struct viewportPacker
{
	int v1;
	int v2;
	int v3;
	int v4;
};

template<typename PointT>
void createDisplay1(pcl::visualization::PCLVisualizer::Ptr& viewer,
	const typename pcl::PointCloud<PointT>::Ptr& cloud, 
	const std::string& cloudID,
	viewportPacker& vPacker)
{

    int v1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 0.5, vPacker.v1);
    viewer->setBackgroundColor (0, 0, 0, vPacker.v1);
    const std::string& cloudID1 = cloudID + std::to_string(1);
	
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> xCloud(cloud, "x");
    viewer->addPointCloud<PointT>(cloud, xCloud, cloudID1, vPacker.v1);
	viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
        0.5, cloudID1, vPacker.v1);
	viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 
        1.0, cloudID1, vPacker.v1);	
    viewer->setLookUpTableID(cloudID1);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID1, vPacker.v1);
    viewer->addText("Channel 1 Power-TE", 0, 0, 25.0, 1.0, 0, 0, "v1 text", vPacker.v1);
    viewer->createViewPortCamera(vPacker.v1);


	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1, 0.5, vPacker.v2);
	viewer->setBackgroundColor(0, 0, 0, vPacker.v2);
	const std::string& cloudID2 = cloudID + std::to_string(2);

	pcl::visualization::PointCloudColorHandlerGenericField<PointT> yCloud(cloud, "y");
	viewer->addPointCloud<PointT>(cloud, yCloud, cloudID2, vPacker.v2);
	viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
		0.5, cloudID2, vPacker.v2);
	viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 
		1.0, cloudID2, vPacker.v2);
	viewer->setLookUpTableID(cloudID2);
	viewer->addText("Channel 1 Power-TM", 0, 0, 25.0, 0, 1.0, 0, "v2 text", vPacker.v2);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID2, vPacker.v2);
	viewer->createViewPortCamera(vPacker.v2);



	int v3(0);
	viewer->createViewPort(0, 0.5, 0.5, 1, vPacker.v3);
	viewer->setBackgroundColor(0, 0, 0, vPacker.v3);
	const std::string& cloudID3 = cloudID + std::to_string(3);
	
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> zCloud(cloud, "z");
	viewer->addPointCloud<PointT>(cloud, zCloud, cloudID3, vPacker.v3);
	viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
		0.5, cloudID3, vPacker.v3);
	viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 
		1.0, cloudID3, vPacker.v3);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID3, vPacker.v3);
	viewer->setLookUpTableID(cloudID3);
	viewer->addText("Channel 1 Range", 0, 0, 25.0, 0, 0, 1.0, "v3 text", vPacker.v3);
	viewer->createViewPortCamera(vPacker.v3);



	int v4(0);
	viewer->createViewPort(0.5, 0.5, 1, 1, vPacker.v4);
	viewer->setBackgroundColor(0, 0, 0, vPacker.v4);
	const std::string& cloudID4 = cloudID + std::to_string(4);
	
	pcl::visualization::PointCloudColorHandlerGenericField<PointT> vCloud(cloud, "y");
	viewer->addPointCloud<PointT>(cloud, vCloud, cloudID4, vPacker.v4);
	viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
		0.5, cloudID4, vPacker.v4);
	viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 
		1.0, cloudID4, vPacker.v4);
	viewer->setLookUpTableID(cloudID4);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudID4, vPacker.v4);
	viewer->addText("Channel 1 Velocity", 0, 0, 25.0, 1.0, 1.0, 1.0, "v4 text", vPacker.v4);
	viewer->createViewPortCamera(vPacker.v4);

	
	viewer->spinOnce (500, true);

}


template<typename PointT>
void updateDisplay1(pcl::visualization::PCLVisualizer::Ptr& viewer,
	const typename pcl::PointCloud<PointT>::Ptr& cloud, 
	const std::string& cloudID,
	viewportPacker& vPacker)
{

	{
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> xCloud(cloud, "x"); /*8 to 10 us*/
		// std::cout << "Here 1" << '\n';
		if(!viewer->updatePointCloud<PointT>(cloud, xCloud, cloudID + std::to_string(1)))
		{
			viewer->addPointCloud<PointT>(cloud, xCloud, cloudID + std::to_string(1), vPacker.v1); /*1300 us to 1600 us*/
			viewer->updatePointCloud<PointT>(cloud, xCloud, cloudID + std::to_string(1));
			
		}

	}

	{
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> yCloud(cloud, "y"); /*8 to 10 us*/
		// std::cout << "Here 2" << '\n';
		if(!viewer->updatePointCloud<PointT>(cloud, yCloud, cloudID + std::to_string(2)))
		{
			viewer->addPointCloud<PointT>(cloud, yCloud, cloudID + std::to_string(2), vPacker.v2); /*1300 us to 1600 us*/
			viewer->updatePointCloud<PointT>(cloud, yCloud, cloudID + std::to_string(2));
			
		}

	}

	{
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> zCloud(cloud, "z"); /*8 to 10 us*/
		// std::cout << "Here 3" << '\n';
		if(!viewer->updatePointCloud<PointT>(cloud, zCloud, cloudID + std::to_string(3)))
		{
			viewer->addPointCloud<PointT>(cloud, zCloud, cloudID + std::to_string(3), vPacker.v3); /*1300 us to 1600 us*/
			viewer->updatePointCloud<PointT>(cloud, zCloud, cloudID + std::to_string(3));
		}

	}

	{
		pcl::visualization::PointCloudColorHandlerGenericField<PointT> velCloud(cloud, "intensity"); /*8 to 10 us*/
		// std::cout << "Here 4" << '\n';
		if(!viewer->updatePointCloud<PointT>(cloud, velCloud, cloudID + std::to_string(4)))
		{
			
			viewer->addPointCloud<PointT>(cloud, velCloud, cloudID + std::to_string(4), vPacker.v4); /*1300 us to 1600 us*/
			viewer->updatePointCloud<PointT>(cloud, velCloud, cloudID + std::to_string(4));
		}

	}
}