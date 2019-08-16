cmake_minimum_required( VERSION 3.6 )

# Require C++11
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Create Project
project( Cyl_filter )
add_executable( Cyl_filter cyl_filter.cpp)


# Find Packages
#Find Package PCL 
find_package(PCL 1.2 REQUIRED)
# Find Package Thread
find_package( Threads REQUIRED )

# Find Package OpenCV
set( OpenCV_DIR "C:/Program Files/opencv/build" )
set( OpenCV_STATIC OFF )
find_package( OpenCV REQUIRED )

# Required Viz Module
if( OpenCV_FOUND )
  if( NOT "opencv_viz" IN_LIST OpenCV_LIBS )
    message( FATAL_ERROR "not found opencv_viz module." )
  endif()
endif()

if( OpenCV_STATIC AND MSVC )
  foreach( flag_var
      CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
      CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO )
    if( ${flag_var} MATCHES "/MD" )
      string( REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}" )
    endif()
  endforeach()
endif()

# Set Properties
if( OpenCV_FOUND)

  # Additional Include Directories
  include_directories(${PCL_INCLUDE_DIRS})
  include_directories( ${OpenCV_INCLUDE_DIRS} )



  # Additional Library Directories
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  link_directories( ${OpenCV_LIB_DIR} )



  # Additional Dependencies
  target_link_libraries( Cyl_filter ${CMAKE_THREAD_LIBS_INIT} )
  target_link_libraries (Cyl_filter ${PCL_LIBRARIES})
  target_link_libraries( Cyl_filter ${OpenCV_LIBS} )
  

else()
  message( WARNING "Cylindrical filter needs Opencv" )
endif()
