// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "jly_goicp.h"
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

int loadPointCloud(string , int & , POINT3D ** );

int main (int argc, char *argv[])
{
    if(argc < 2)
	{
		std::cerr << "Not enough arguments - " << argv[0] << " <input pcd> <input_frame_id>=cloud" << std::endl;
		return 1;
	}

	std::string input_file = argv[1];
	std::string input_frame_id;
	if(argc > 2){ 
		input_frame_id = argv[2]; 
	}
	else
	{
		input_frame_id = "cloud";
	}

	ROS_INFO("Initializing.");

    ros::init(argc, argv, "point3d_publisher");
    ros::NodeHandle n;
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1);

	ROS_INFO("Loading pointcloud from file.");

	POINT3D* cloud3d;
	int num_points;
	loadPointCloud (input_file, num_points, &cloud3d);

	ROS_INFO("Loaded with %d points.",num_points);
	ROS_INFO("Converting pointcloud to pcl.");

	pcl::PointCloud<pcl::PointXYZ> cloud;
	for (uint i=0; i<num_points; i++)
	{
		cloud.points.push_back(pcl::PointXYZ((cloud3d+i)->x,(cloud3d+i)->y,(cloud3d+i)->z));
	}

	ROS_INFO("Converting pointcloud to ros msg.");

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = input_frame_id;

	ROS_INFO("Publishing.");

    while(ros::ok())
    {
        cloud_msg.header.stamp = ros::Time::now();
        cloud_pub.publish(cloud_msg);
        ros::spinOnce();
        ros::Rate(5).sleep();
    }
}

int loadPointCloud(string FName, int & N, POINT3D ** p)
{
	int i;
	ifstream ifile;

	ifile.open(FName.c_str(), ifstream::in);
	if(!ifile.is_open())
	{
		cout << "Unable to open point file '" << FName << "'" << endl;
		exit(-1);
	}
	ifile >> N; // First line has number of points to follow
	*p = (POINT3D *)malloc(sizeof(POINT3D) * N);
	for(i = 0; i < N; i++)
	{
		ifile >> (*p)[i].x >> (*p)[i].y >> (*p)[i].z;
	}

	ifile.close();

	return 0;
}

