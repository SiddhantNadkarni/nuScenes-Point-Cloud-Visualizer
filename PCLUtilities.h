#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

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
std::size_t& count = 0)
{
    std::fstream  myfile (fileName);

    std::cout << "load count: " << ++count << '\n';
    cloud->points.clear();
    while (myfile.read(reinterpret_cast<char*>(f), sizeof(float)*5))
    {
        PointT p;

        p.x = f[0];
        p.y = f[1];
        p.z = f[2];
        // p.intensity = f[3];

        // std::cout << "x: " << f[0] << '\n';
        // std::cout << "y: " << f[1] << '\n';
        // std::cout << "z: " << f[2] << '\n';
        // std::cout << "intensity: " << f[3] << '\n';

        cloud->points.push_back(p);
    }

}
