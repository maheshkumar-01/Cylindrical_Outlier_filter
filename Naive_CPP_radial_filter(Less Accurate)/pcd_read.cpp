#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>

using namespace std;

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,string WindowName){
    pcl::visualization::CloudViewer viewer (WindowName);
    viewer.showCloud (cloud);      
    while(!viewer.wasStopped()){}
}

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    if (argc<2){
        cout <<"Usage : ./pcd_read path_to_pcd_file.pcd" <<endl;
        return 1;
    }
    //* load the file
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) {
        PCL_ERROR ("Couldn't read file \n");
        return (-1);
    }
    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from "
            << argv[1]
            << std::endl;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.03);
    outrem.setMinNeighborsInRadius (3);
    outrem.filter (*cloud_filtered);
    pcl::io::savePCDFile( "Result/filtered.pcd", *cloud_filtered, true );
    visualize(cloud,"Input Point Cloud");
    visualize(cloud_filtered,"Filtered Point cloud");

    return (0);
}

