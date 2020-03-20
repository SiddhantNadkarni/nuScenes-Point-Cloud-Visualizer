#include <iostream>
#include <math.h>
#include <fstream>
#include <chrono>
#include <iterator>

// #include <pcl/common/common_headers.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/point_cloud_color_handlers.h>


#include "Display1.h"
// #include "Custom_Visualizer.h"

template<typename PointT>
std::vector<boost::filesystem::path> streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    std::sort(paths.begin(), paths.end());

    std::cout << "Number of paths: " << paths.size() << '\n';
 
    return paths;

}

template<typename PointT>
void loadPcd(typename pcl::PointCloud<PointT>::Ptr& cloud, 
const std::string& fileName, 
float*& f, 
std::size_t& count)
{
    std::fstream  myfile (fileName);

    std::cout << "load count: " << ++count << '\n';
    cloud->points.clear();
    while (myfile.read(reinterpret_cast<char*>(f), sizeof(float)*5))
    {
        pcl::PointXYZI p;
        p.x = f[0];
        p.y = f[1];
        p.z = f[2];
        p.intensity = f[3];

        // std::cout << "x: " << f[0] << '\n';
        // std::cout << "y: " << f[1] << '\n';
        // std::cout << "z: " << f[2] << '\n';
        // std::cout << "intensity: " << f[3] << '\n';

        cloud->points.push_back(p);
    }

}

int main(int argc, const char** argv) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    cloud->points.reserve(34688);
    float* f = new float [5];
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    std::vector<boost::filesystem::path> stream = streamPcd<pcl::PointXYZI>("../../LIDAR_TOP/");

    auto iterator = stream.begin();

    std::size_t count{0};
    std::size_t itCount{0};

    viewportPacker vPacker{0, 0, 0, 0};
    loadPcd<pcl::PointXYZI>(cloud, (*iterator).string(), f, count);
    createDisplay1<pcl::PointXYZI>(viewer, cloud, "cloud view", vPacker);
    // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud,"intensity");
  	// viewer->addPointCloud<pcl::PointXYZI>(cloud,  "cloud view");
    std::cout << "itCount: " << ++itCount << ", path: " << *iterator << '\n';



    while (!viewer->wasStopped ())
    {
        auto startTime = std::chrono::steady_clock::now();
        viewer->removeAllPointClouds();

        loadPcd<pcl::PointXYZI>(cloud, (*iterator).string(), f, count);
        updateDisplay1<pcl::PointXYZI>(viewer, cloud, "cloud view", vPacker);
        // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud,"y");
        // viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution,  "cloud view");
        viewer->spinOnce (1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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