/********************************************************************
Main Function for point cloud registration with Go-ICP Algorithm
Last modified: Feb 13, 2014

"Go-ICP: Solving 3D Registration Efficiently and Globally Optimally"
Jiaolong Yang, Hongdong Li, Yunde Jia
International Conference on Computer Vision (ICCV), 2013

Copyright (C) 2013 Jiaolong Yang (BIT and ANU)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*********************************************************************/

#include <time.h>
#include <iostream>
#include <fstream>
using namespace std;

#include "jly_goicp.h"
#include "ConfigMap.hpp"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// #define DEFAULT_OUTPUT_FNAME "output.txt"
#define DEFAULT_CONFIG_FNAME "config.txt"
// #define DEFAULT_MODEL_FNAME "model.txt"
// #define DEFAULT_DATA_FNAME "data.txt"

// void parseInput(int argc, char **argv, string & modelFName, string & dataFName, int & NdDownsampled, string & configFName, string & outputFName);
void readConfig(string FName, GoICP & goicp);
// int loadPointCloud(string FName, int & N, POINT3D **  p);
void inputCB(const sensor_msgs::PointCloud2::ConstPtr& , const sensor_msgs::PointCloud2::ConstPtr& );

std::string config_filename_;
int nd_downsampled_;
ros::Publisher tf_pub_;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "go_icp_node");
    ros::NodeHandle nh("~");

	nh.param<std::string>("config_filename", config_filename_, std::string(DEFAULT_CONFIG_FNAME));
	nh.param<int>("nd_downsampled", nd_downsampled_, int(0));
	
    tf_pub_ = nh.advertise<geometry_msgs::TransformStamped>("transform",1);

	message_filters::Subscriber<sensor_msgs::PointCloud2> model_cloud_sub(nh, "model_cloud", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> data_cloud_sub(nh, "data_cloud", 1);
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>> sync(message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>(1), model_cloud_sub, data_cloud_sub);
	sync.registerCallback(boost::bind(&inputCB, _1, _2));

	ROS_INFO("GoICP initialized.");

    ros::spin();
	return 0;
}

void inputCB(const sensor_msgs::PointCloud2::ConstPtr& model_cloud_msg, const sensor_msgs::PointCloud2::ConstPtr& data_cloud_msg)
{
	ROS_INFO("Got set of clouds.\n");

	int Nm, Nd, NdDownsampled;
	clock_t  clockBegin, clockEnd;
	// string modelFName, dataFName, configFName, outputFname;
	POINT3D * pModel, * pData;
	GoICP goicp;

	float scModel, scData;
	Eigen::Vector3f trModel, trData;

	std::vector<POINT3D> model_pts;
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromROSMsg(*model_cloud_msg, cloud);

		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ( cloud )) );
		feature_extractor.compute ();
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);
		scModel = sqrt(pow(max_point_AABB.x-min_point_AABB.x,2)+pow(max_point_AABB.y-min_point_AABB.y,2)+pow(max_point_AABB.z-min_point_AABB.z,2))/sqrt(3);
		feature_extractor.getMassCenter (trModel);

		Nm = cloud.size();
		for( uint i = 0; i < cloud.size(); i++ )
		{
			POINT3D pt;
			pt.x = cloud.points[i].x;
			pt.y = cloud.points[i].y;
			pt.z = cloud.points[i].z;
			model_pts.push_back(pt);
		}
		pModel = &model_pts[0];
	}

	std::vector<POINT3D> data_pts;
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromROSMsg(*data_cloud_msg, cloud);

		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ( cloud )) );
		feature_extractor.compute ();
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		feature_extractor.getAABB (min_point_AABB, max_point_AABB);
		scData = sqrt(pow(max_point_AABB.x-min_point_AABB.x,2)+pow(max_point_AABB.y-min_point_AABB.y,2)+pow(max_point_AABB.z-min_point_AABB.z,2))/sqrt(3);
		feature_extractor.getMassCenter (trData);

		Nd = cloud.size();
		for( uint i = 0; i < cloud.size(); i++ )
		{
			POINT3D pt;
			pt.x = cloud.points[i].x;
			pt.y = cloud.points[i].y;
			pt.z = cloud.points[i].z;
			data_pts.push_back(pt);
		}
		pData = &data_pts[0];
	}

	NdDownsampled = nd_downsampled_;

	// parseInput(argc, argv, modelFName, dataFName, NdDownsampled, configFName, outputFname);
	readConfig(config_filename_, goicp);

	// Load model and data point clouds
	// loadPointCloud(modelFName, Nm, &pModel);
	// loadPointCloud(dataFName, Nd, &pData);
	
	goicp.pModel = pModel;
	goicp.Nm = Nm;
	goicp.pData = pData;
	goicp.Nd = Nd;

	// Build Distance Transform
	cout << "Building Distance Transform..." << flush;
	clockBegin = clock();
	goicp.BuildDT();
	clockEnd = clock();
	cout << (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC << "s (CPU)" << endl;

	// Run GO-ICP
	if(NdDownsampled > 0)
	{
		goicp.Nd = NdDownsampled; // Only use first NdDownsampled data points (assumes data points are randomly ordered)
	}
	// cout << "Model ID: " << modelFName << " (" << goicp.Nm << "), Data ID: " << dataFName << " (" << goicp.Nd << ")" << endl;
	cout << "Registering..." << endl;
	clockBegin = clock();
	goicp.Register();
	clockEnd = clock();
	double time = (double)(clockEnd - clockBegin)/CLOCKS_PER_SEC;
	cout << "Optimal Rotation Matrix:" << endl;
	cout << goicp.optR << endl;
	cout << "Optimal Translation Vector:" << endl;
	cout << goicp.optT << endl;
	cout << "Finished in " << time << endl;

	// ofstream ofile;
	// ofile.open(outputFname.c_str(), ofstream::out);
	// ofile << time << endl;
	// ofile << goicp.optR << endl;
	// ofile << goicp.optT << endl;
	// ofile.close();

	geometry_msgs::TransformStamped tf_msg;
	tf_msg.header.frame_id = model_cloud_msg->header.frame_id;
	tf_msg.child_frame_id = data_cloud_msg->header.frame_id;
	tf_msg.header.stamp = ros::Time::now();
	tf_msg.transform.translation.x = goicp.optT.val[0][0];
	tf_msg.transform.translation.y = goicp.optT.val[1][0];
	tf_msg.transform.translation.z = goicp.optT.val[2][0];
	tf_msg.transform.rotation.w = sqrt(1 + goicp.optR.val[0][0] + goicp.optR.val[1][1] + goicp.optR.val[2][2])/2.0;
	tf_msg.transform.rotation.x = (goicp.optR.val[2][1] - goicp.optR.val[1][2]) / (4*tf_msg.transform.rotation.w);
	tf_msg.transform.rotation.y = (goicp.optR.val[0][2] - goicp.optR.val[2][0]) / (4*tf_msg.transform.rotation.w);
	tf_msg.transform.rotation.z = (goicp.optR.val[1][0] - goicp.optR.val[0][1]) / (4*tf_msg.transform.rotation.w);
	tf_pub_.publish(tf_msg);
	ros::spinOnce();


	delete(pModel);
	delete(pData);

	return;
}

// void parseInput(int argc, char **argv, string & modelFName, string & dataFName, int & NdDownsampled, string & configFName, string & outputFName)
// {
// 	// Set default values
// 	modelFName = DEFAULT_MODEL_FNAME;
// 	dataFName = DEFAULT_DATA_FNAME;
// 	configFName = DEFAULT_CONFIG_FNAME;
// 	outputFName = DEFAULT_OUTPUT_FNAME;
// 	NdDownsampled = 0; // No downsampling

// 	//cout << endl;
// 	//cout << "USAGE:" << "./GOICP <MODEL FILENAME> <DATA FILENAME> <NUM DOWNSAMPLED DATA POINTS> <CONFIG FILENAME> <OUTPUT FILENAME>" << endl;
// 	//cout << endl;

// 	if(argc > 5)
// 	{
// 		outputFName = argv[5];
// 	}
// 	if(argc > 4)
// 	{
// 		configFName = argv[4];
// 	}
// 	if(argc > 3)
// 	{
// 		NdDownsampled = atoi(argv[3]);
// 	}
// 	if(argc > 2)
// 	{
// 		dataFName = argv[2];
// 	}
// 	if(argc > 1)
// 	{
// 		modelFName = argv[1];
// 	}

// 	cout << "INPUT:" << endl;
// 	cout << "(modelFName)->(" << modelFName << ")" << endl;
// 	cout << "(dataFName)->(" << dataFName << ")" << endl;
// 	cout << "(NdDownsampled)->(" << NdDownsampled << ")" << endl;
// 	cout << "(configFName)->(" << configFName << ")" << endl;
// 	cout << "(outputFName)->(" << outputFName << ")" << endl;
// 	cout << endl;
// }

void readConfig(string FName, GoICP & goicp)
{
	// Open and parse the associated config file
	ConfigMap config(FName.c_str());

	goicp.MSEThresh = config.getF("MSEThresh");
	goicp.initNodeRot.a = config.getF("rotMinX");
	goicp.initNodeRot.b = config.getF("rotMinY");
	goicp.initNodeRot.c = config.getF("rotMinZ");
	goicp.initNodeRot.w = config.getF("rotWidth");
	goicp.initNodeTrans.x = config.getF("transMinX");
	goicp.initNodeTrans.y = config.getF("transMinY");
	goicp.initNodeTrans.z = config.getF("transMinZ");
	goicp.initNodeTrans.w = config.getF("transWidth");
	goicp.trimFraction = config.getF("trimFraction");
	// If < 0.1% trimming specified, do no trimming
	if(goicp.trimFraction < 0.001)
	{
		goicp.doTrim = false;
	}
	goicp.dt.SIZE = config.getI("distTransSize");
	goicp.dt.expandFactor = config.getF("distTransExpandFactor");

	cout << "CONFIG:" << endl;
	config.print();
	//cout << "(doTrim)->(" << goicp.doTrim << ")" << endl;
	cout << endl;
}

// int loadPointCloud(string FName, int & N, POINT3D ** p)
// {
// 	int i;
// 	ifstream ifile;

// 	ifile.open(FName.c_str(), ifstream::in);
// 	if(!ifile.is_open())
// 	{
// 		cout << "Unable to open point file '" << FName << "'" << endl;
// 		exit(-1);
// 	}
// 	ifile >> N; // First line has number of points to follow
// 	*p = (POINT3D *)malloc(sizeof(POINT3D) * N);
// 	for(i = 0; i < N; i++)
// 	{
// 		ifile >> (*p)[i].x >> (*p)[i].y >> (*p)[i].z;
// 	}

// 	ifile.close();

// 	return 0;
// }
