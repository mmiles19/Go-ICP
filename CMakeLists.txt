project(GoICP)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(GoICP
	jly_main.cpp
	jly_goicp.cpp
	jly_3ddt.cpp
	matrix.cpp
	ConfigMap.cpp
	StringTokenizer.cpp
	)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
	geometry_msgs
	message_filters
	pcl_ros
	pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp 
  )

include_directories(
    ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	.
    )

link_directories (${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})

add_executable(go_icp_node
	go_icp_node.cpp
	jly_goicp.cpp
	jly_3ddt.cpp
	matrix.cpp
	ConfigMap.cpp
	StringTokenizer.cpp
	)
target_link_libraries (go_icp_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

add_executable(point3d_publisher
	point3d_publisher.cpp
	)
target_link_libraries (point3d_publisher 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
