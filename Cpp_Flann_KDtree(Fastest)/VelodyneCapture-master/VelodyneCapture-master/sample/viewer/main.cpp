#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <string>

// Include VelodyneCapture Header
#include "VelodyneCapture.h"
using namespace std;
int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from Sensor
    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.21" );
    const unsigned short port = 2368;
    velodyne::VLP16Capture capture( address, port );
    //velodyne::HDL32ECapture capture( address, port );

    
    // Open VelodyneCapture that retrieve from PCAP
    //const std::string filename = "sample.pcap";
    //velodyne::VLP16Capture capture( filename );
    //velodyne::HDL32ECapture capture( filename );
    

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    // Create Viewer
    cv::viz::Viz3d viewer( "Velodyne" );

    // Register Keyboard Callback
    viewer.registerKeyboardCallback(
        []( const cv::viz::KeyboardEvent& event, void* cookie ){
            // Close Viewer
            if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
                static_cast<cv::viz::Viz3d*>( cookie )->close();
            }
        }
        , &viewer
    );
    int i =0;
    while( capture.isRun() && !viewer.wasStopped() ){
        // Capture One Rotation Data
        string ext (".bin");
        string binFileName = "../../../data/point" + i++ + ext;
	std::ofstream binHandler (binFileName);
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){
            std::cout<<"Empty"<<endl;
            continue;
        }
	
        // Convert to 3-dimention Coordinates
        std::vector<cv::Vec3f> buffer( lasers.size() );
        for( const velodyne::Laser& laser : lasers ){
            float laser_mat[5];
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * CV_PI / 180.0;
            const double vertical = laser.vertical * CV_PI / 180.0;
            const double intensity = static_cast<float>(laser.intensity);
            float laser_id = static_cast<float>(laser.id);

            float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            float z = static_cast<float>( ( distance * std::sin( vertical ) ) );

            buffer.push_back( cv::Vec3f( x, y, z ) );
            laser_mat[0] = x;
            laser_mat[1] = y;
	    laser_mat[2] = z;
	    laser_mat[3] = azimuth;
            laser_mat[4] = laser_id;
            binHandler.write((char *)laser_mat,5*sizeof(float));
        }

        

    }


    return 0;
}
