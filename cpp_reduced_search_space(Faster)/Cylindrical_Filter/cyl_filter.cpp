#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#define PI 3.14159265
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <time.h>
#include <chrono>

using namespace std;

bool SortByAzimuth( const vector<float>& v1, 
               const vector<float>& v2 ) { 

 return ((v1[6] < v2[6]) );
} 
void visualize(const vector<vector<float>>& input_pcd,cv::viz::Viz3d viewer) {
    std::vector<cv::Vec3f> buffer(input_pcd.size());
    for (int i = 0; i < input_pcd.size(); i++) {
        buffer[i] = cv::Vec3f(input_pcd[i][0], input_pcd[i][1], input_pcd[i][2]);
    } 
    // Create Widget
    cv::Mat cloudMat = cv::Mat(static_cast<int>(buffer.size()), 1, CV_32FC3, &buffer[0]);
    cv::viz::WCloud cloud(cloudMat);
    // Show Point Cloud
    viewer.showWidget("Cloud", cloud);
    viewer.spinOnce();
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
    int ht_c = atof(argv[3]);
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

    // Set the number of vertical neighbors to scan based on the input height constant    
    int n = pointcloud.size();
    float ratio = 360/(n/16);
    //float horizontal_neighbors = ceil(rad_c/ratio);
    int vertical_neighbors = ceil(ht_c/2);
    vertical_neighbors = min(6,vertical_neighbors);
    
    // Initialize hash map for different scan IDs to hold the points
    unordered_map <int,vector < vector < float > > > scanID_points;
    unordered_map <int,int > scan_ID_count;

    // Initialize the 2D vector and dictionary to map the neighbors of a particular laser ID
    // 16 values for 16 lasers of VLP-16
    vector<vector<int> > neighbors_vec { { 2,4,6,8,10,12,14 }, {14,3,12,5,10,7,9  }, { 0,4,6,8,10,12,14 }, { 1,5,14,7,12,9,11 },
                                         { 2,6,0,8,10,12,14 }, { 3,7,1,9,14,11,13 }, {4,8,2,10,0,12,14 }, {5,9,3,11,1,13,15 },
                                         { 6,10,4,12,2,14,0 }, { 7,11,5,13,3,15,1 },{ 8,12,6,14,4,1,2 }, { 9,13,7,15,5,3,1 },
                                         { 10,14,8,1,6,3,4 } , { 11,15,9,7,5,3,1 },{ 12,1,10,3,8,5,6 },{ 13,11,9,7,5,3,1 }
                                       };
    unordered_map<int, vector<int> > neighbors_dict;

    // Insert the points ,mapping it to the corresponding laser_id, also map the neighbors for a scan_id with the corresponding vector
    for(int i=0; i<16; i++){
	scanID_points.insert(make_pair(i,vector < vector < float > >()));
	scan_ID_count.insert(make_pair(i,0));
        neighbors_dict.insert(make_pair(i,neighbors_vec[i]));
    }
    for(int j=0; j<n; j++){
        int idx = int(pointcloud[j][4]);
	scanID_points[idx].push_back(vector<float>());  
	for(int k=0; k<6; k++){      
	    scanID_points[idx][scan_ID_count[idx]].push_back(pointcloud[j][k]); //fulfill the last index regularly
	}
	    ++scan_ID_count[idx];
    }  
    int filtered_pts_count = 0;
    // Setting the clock to measure the time the reduced search space algorithm takes to filter all the points of the point cloud

    auto begin = std::chrono::high_resolution_clock::now(); 
    for(int i=0; i<16; i++){
	int test_pt_pos = 0;
	for(int j=0; j<scanID_points[i].size(); j++){
	      bool classified_pt_flag = 0;
              // For each point get the distance value in spherical coordinates
	      float dist_from_pc = scanID_points[i][j][5];
              // Define the height of the dynamic cylinder for a point based on its distance from the point cloud and tan of the height constant
	      float height = std::tan(ht_c* (3.14159265 / 180.))*dist_from_pc;
	      float pt1[3] = {};
	      float pt2[3] = {};
	      float vec[3] = {};
	      float pt_to_line_dist = 0.0;
              // Define the axis of the cylinder as a line passing through the point at its mid point
	      pt1[0] = scanID_points[i][j][0];
	      pt2[0] = scanID_points[i][j][0];
	      pt1[1] = scanID_points[i][j][1];
	      pt2[1] = scanID_points[i][j][1];
	      pt1[2] = scanID_points[i][j][2] + float(height/2);
	      pt1[2] = scanID_points[i][j][2] - float(height/2);
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
              // For each point, get the neighbors from atleast set_nn+5 apart from the test point
	      for(int r=test_pt_pos-set_nn-5;r<test_pt_pos+set_nn+5;r++){
	          if(r<scanID_points[i].size() && r>0 && r!=test_pt_pos){
	              float val[3] = {};
                      // Get the x,y,z values of the neighbors
	              val[0] = scanID_points[i][r][0]; val[1] = scanID_points[i][r][1];val[2] = scanID_points[i][r][2];
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
	                      filtered_pcd.push_back({pt1[0],pt1[1],scanID_points[i][j][2],scanID_points[i][j][3],scanID_points[i][j][4]});
	                      
	                      classified_pt_flag = 1;
	                      break;
	                   }
	               }
	            }
	        }
	        // Check if the vertical neighbors lie within the cylinder defined by the test point, if the horizontal search space was insufficient to classify the point
	        if(classified_pt_flag != 1){
                     for(int vert=0;vert<vertical_neighbors+1;vert++){
                         // Again, for each point , atleast set_nn +5 points of vertical neighbors from a scan ID is checked 
                         for(int r=test_pt_pos-set_nn-5;r<test_pt_pos+set_nn+5;r++){
                             if(r<scanID_points[i].size() && r>0 && r!=test_pt_pos){
                                  float val[3] = {};
                                          
                                  val[0] = scanID_points[neighbors_dict[i][vert]][r][0]; val[1] = scanID_points[neighbors_dict[i][vert]][r][1];val[2] = scanID_points[neighbors_dict[i][vert]][r][2];
                                  float temp[3] ={};
                                  temp[0] =  val[0] - pt1[0];temp[1] =  val[1] - pt1[1];temp[2] =  val[2] - pt1[2];
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

			          is_inside = (dot_prod1 >=0) && (dot_prod2 <= 0) && (mag <= pt_to_line_dist);
                                  if(is_inside==1){
	                              ++count;
	                              if(count==set_nn){
	                                  filtered_pcd.push_back({pt1[0],pt1[1],scanID_points[i][j][2],scanID_points[i][j][3],scanID_points[i][j][4]});
	                                  classified_pt_flag = 1;
	                                  break;
	                              }
	                          }
	                      }
                          }
                      }
                  }
                  // If the point is still not classified, do not add the test point to the filtered point cloud data
                  if(classified_pt_flag != 1){     
	              ++filtered_pts_count;
                  }
                  ++test_pt_pos;
	    }
	} 
        // Output the total time taken in nano seconds and the number of points filtered
        auto end = std::chrono::high_resolution_clock::now(); 
    	auto total_time = (std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count())*pow(10,-9);
    	cout << "Time taken by the algorithm for neighbors : " << set_nn << " and Number of points :" << n << " is : ";
    	cout << total_time << " Seconds" << endl; 
	cout <<"The number of points filtered :" <<  filtered_pts_count <<endl;
	cout<<filtered_pcd.size() << endl;
        // Visualize the Input and filtered point cloud

        cv::viz::Viz3d viewer( "Original Point cloud" );
	cv::viz::Viz3d viewer2( "Filtered point cloud" );
	// Register Keyboard Callback
	viewer.registerKeyboardCallback([]( const cv::viz::KeyboardEvent& event, void* cookie ){
	// Close Viewer
	if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
	    static_cast<cv::viz::Viz3d*>( cookie )->close();
	    }
	}
	, &viewer
	);
	viewer2.registerKeyboardCallback([]( const cv::viz::KeyboardEvent& event, void* cookie ){
	// Close Viewer
	if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
	    static_cast<cv::viz::Viz3d*>( cookie )->close();
	    }
	}
	, &viewer2
	);
	while(!viewer.wasStopped() && !viewer2.wasStopped()){
	visualize(pointcloud,viewer);
        visualize(filtered_pcd,viewer2);
	}

    return 0;
}

