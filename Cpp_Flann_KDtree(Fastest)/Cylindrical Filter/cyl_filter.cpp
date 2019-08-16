#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#define PI 3.14159265
#include <chrono>
#include <time.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

bool SortByAzimuth( const vector<float>& v1, 
               const vector<float>& v2 ) { 

 return ((v1[6] < v2[6]) );
} 

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,string WindowName){
    pcl::visualization::CloudViewer viewer (WindowName);
    viewer.showCloud (cloud);      
    while(!viewer.wasStopped()){}
}

int main (int argc, char **argv)
{
    if(argc<5){
        std::cerr << "Usage: input bin file, radius constant(in cm), height constant (in deg) and Number of neighbors" << endl;
        return 1;
    }
    if(!strstr(argv[1],".bin")){
         std::cerr << "Please provide the input file in .bin format" << endl;
         std::cout << "To generate bin files, use velodyne capture" << endl;
         return 1;
    }
    // Get the bin file name, radius and height constants for the cylinder and the number of neighbors

    string filename = argv[1];
    float rad_c = atof(argv[2]);
    float ht_c = atof(argv[3]);
    int set_nn = atoi(argv[4]);
    
    // Initialize the input pointcloud and filtered point cloud 2D vectors
    vector<vector<float>> pointcloud;
    vector<vector<float>> filtered_pcd;

    // Enough length to hold the input points in 1 frame
    int32_t length = 1000000;
    float* values = (float*)malloc(length*sizeof(float));
    // Initialize pointers for x,y,z,intensity and scan_id
    float *x = values, *y = values+1, *z = values+2, *intensity = values+3, *id = values+4;

    FILE *fd = fopen(filename.c_str(), "rb");
    length = fread(values, sizeof(float), length, fd) / 5;
    for (int32_t i = 0; i < length; i++) {
        float dist = std::sqrt((*x)*(*x) + (*y)*(*y) + (*z)*(*z));
        float theta = std::atan2(*y, *x) * 180.0f / PI;
        pointcloud.push_back({*x, *y, *z, *intensity, *id, dist, theta});
        // Increment the pointers to point to the next line of the file
        x += 5, y += 5, z += 5, intensity += 5, id += 5;
    }
    fclose(fd);
    // Sort all the points in the point cloud based on the azimuth values
    sort(pointcloud.begin(), pointcloud.end(),SortByAzimuth); 

    
    int n = pointcloud.size();
    cout<<n<<endl;
    float ratio = n/16;
    rad_c = 360/ratio;
    
    // Converting Point cloud into a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
    cloud->width = n;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    for(int i=0;i<n;i++){
        cloud->points[i].x = pointcloud[i][0];
        cloud->points[i].y = pointcloud[i][1];
        cloud->points[i].z = pointcloud[i][2];
    }
    // Initializing the KD tree and setting the input point cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud); 
    // Set the number of neighbors to fetch from the KD-tree and the distance upto which the neighbors should be obtained

    std::vector<int> pointIdxNKNSearch(set_nn+2);
    std::vector<float> pointNKNSquaredDistance(set_nn+3);

    int filtered_pts_count = 0;
    // Setting the clock to measure the time the KD tree based search algorithm takes to filter all the points of the point cloud
    auto begin = std::chrono::high_resolution_clock::now(); 
    for(int i=0; i<n; i++){
        bool classified_pt_flag = 0;
        // For each point get the distance value in spherical coordinates
        float dist_from_pc = pointcloud[i][5];
        // Define the height of the dynamic cylinder for a point based on its distance from the point cloud and tan of the height constant
        float height = std::tan(ht_c* (3.14159265 / 180.))* dist_from_pc;
        float pt1[3] = {};
        float pt2[3] = {};
        float vec[3] = {};
        float pt_to_line_dist = 0.0;     
        pt1[0] = pointcloud[i][0];
        pt2[0] = pointcloud[i][0];
        // Define the axis of the cylinder as a line passing through the point at its mid point
        pt1[1] = pointcloud[i][1]+ float(height/2);
        pt2[1] = pointcloud[i][1]- float(height/2);
        pt1[2] = pointcloud[i][2] ;
        pt2[3] = pointcloud[i][2] ;
        // Define the radius of the dynamic cylinder as a function of distance from the pc and radius const,
        float radius = (dist_from_pc*rad_c);
        //  Define a vector between the plane of two circular facets of the cylinder
        vec[0] = (pt2[0] - pt1[0]);
        vec[1] = (pt2[1] - pt1[1]);
        vec[2] = (pt2[2] - pt1[2]);
        // Constant value to confirm that the point lies inside the curved surface of the cylinder (Point-line-distance)
        pt_to_line_dist = radius * sqrt(vec[2]*vec[2]);
        int count =0;
        bool is_inside = 0;
        pcl::PointXYZ testPoint;
        testPoint.x = pointcloud[i][0];
        testPoint.y = pointcloud[i][1];
        testPoint.z = pointcloud[i][2];
        // Obtain 20 nearest neighbors from our test point and check whether the number of set neighbors lie within the cylinder defined by the test point 
  	if ( kdtree.nearestKSearch (testPoint, set_nn+3, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j){
               float val[3] = {};
               // Get the x,y,z values of the neighbors
               val[0] = cloud->points[ pointIdxNKNSearch[j] ].x;
               val[1] = cloud->points[ pointIdxNKNSearch[j] ].y;
               val[2] = cloud->points[ pointIdxNKNSearch[j] ].z; 
               // Vector subtraction of our neighbor point with points in the circular facets
               float sub1[3] ={};
               sub1[0] =  val[0] - pt1[0];sub1[1] =  val[1] - pt1[1];sub1[2] =  val[2] - pt1[2];
               float sub2[3] = {};
               sub2[0] = val[0] - pt2[0]; sub2[1] = val[1] - pt2[1];sub2[2] = val[2] - pt2[2];
               // Dot product between the subtracted vectors and the vector between the plane of two circular facets of the cylinder
               float dot_prod1 = vec[2]*sub1[2];
               float dot_prod2 = vec[2]*sub2[2];
	       float cross_prod[3] = {};
               // Cross product between our vector and one of the subraction vector to confirm the point-line distance of our neighbor
               cross_prod[0] = sub1[1] * vec[2] - sub1[2] * vec[1]; 
               cross_prod[1] = sub1[0] * vec[2] - sub1[2] * vec[0]; 
               cross_prod[2] = sub1[0] * vec[1] - sub1[1] * vec[0]; 
               float mag = sqrt(cross_prod[0]*cross_prod[0] + cross_prod[1]*cross_prod[1] + cross_prod[2]*cross_prod[2]);

               /* Constraints for the test point to be a neighbor : Dot product 1 greater than or equal to 0 , Dot product 2 less than or equal to 0 and the magnitude of the cross product is less than or equal to our point-line distance */

               is_inside = (dot_prod1 >=0) && (dot_prod2 <= 0) && (mag <= pt_to_line_dist);
               if(is_inside==1){
                   ++count;
                   // If the count reaches the set neighbor count, push the test point to the filtered point cloud or else, leave the test point
                   if(count==set_nn){
                       filtered_pcd.push_back({pt1[0],pt1[1],pointcloud[i][2],pointcloud[i][3],pointcloud[i][4]});                              
                       classified_pt_flag = 1;
                       break;
                    }
                }
             }
        }
        if(classified_pt_flag != 1){  
            // Increment the number of filtered points in the points cloud          
            ++filtered_pts_count;
        }
    }           
    // Output the total time taken in nano seconds and the number of points filtered
    auto end = std::chrono::high_resolution_clock::now(); 
    auto total_time = (std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count())*pow(10,-9);
    cout << "Time taken by the algorithm for neighbors : " << set_nn << " and Number of points :" << n << " is : ";
    cout << total_time << " Seconds" << endl; 
    cout <<"The number of points filtered :" <<  filtered_pts_count <<endl;

    // Visualize the Input and filtered point cloud

    int m = filtered_pcd.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredcloud (new pcl::PointCloud<pcl::PointXYZ>);
    filteredcloud->width = m;
    filteredcloud->height = 1;
    filteredcloud->points.resize (cloud->width * cloud->height);

    for(int i=0;i<m;i++){
        filteredcloud->points[i].x = filtered_pcd[i][0];
        filteredcloud->points[i].y = filtered_pcd[i][1];
        filteredcloud->points[i].z = filtered_pcd[i][2];
    }
    visualize(cloud,"Input Point Cloud");
    visualize(filteredcloud,"Filtered Point cloud");
    
    return 0;
}

