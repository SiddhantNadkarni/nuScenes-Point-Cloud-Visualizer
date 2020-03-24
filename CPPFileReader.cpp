#include <iostream>
#include <math.h>
#include <fstream>
#include <chrono>
#include <iterator>

#include "PCLUtilities.h"
#include "Display1.h"


int main(int argc, const char** argv) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->points.reserve(34688);

    float* f = new float [5];

    std::vector<boost::filesystem::path> stream = streamPcd<pcl::PointXYZI>("../../LIDAR_TOP/");
    auto iterator = stream.begin();

    std::size_t count{0};
    std::size_t itCount{0};

    viewportPacker vPacker{0, 0, 0, 0};
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    loadPcd<pcl::PointXYZI>(cloud, (*iterator).string(), f, count);
    createDisplay1<pcl::PointXYZI>(viewer, cloud, "cloud view", vPacker);
    std::cout << "itCount: " << ++itCount << ", path: " << *iterator << '\n';



    while (!viewer->wasStopped ())
    {
        auto startTime = std::chrono::steady_clock::now();
        viewer->removeAllPointClouds();

        loadPcd<pcl::PointXYZI>(cloud, (*iterator).string(), f, count);
        updateDisplay1<pcl::PointXYZI>(viewer, cloud, "cloud view", vPacker);
        viewer->spinOnce (1);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        iterator++;
        std::cout << "itCount: " << ++itCount << ", path: " << *iterator << '\n';

        auto endTime = std::chrono::steady_clock::now();

        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        std::cout << "elapsedTime: " << elapsedTime.count() << " milliseconds." << '\n';
        if(iterator == stream.end())
        {
            return 0;
        }


    } 

    delete [] f;
    
    return 0;
}