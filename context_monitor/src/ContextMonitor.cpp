
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

// BattLev publisher and subscriber
ros::Publisher BattLev_pub;
ros::Subscriber BattLev_sub;

// SemLoc publisher and subscriber
ros::Publisher SemLoc_pub;
ros::Subscriber SemLoc_sub;

// ObstDist publisher and subscriber
ros::Publisher ObstDist_pub;
ros::Subscriber ObstDist_sub;

// ObjectMaterial publisher and subscriber
ros::Publisher ObjectMaterial_pub;
ros::Subscriber ObjectMaterial_sub;

void getClosestObstacle(const sensor_msgs::LaserScan::ConstPtr& input,
					 std_msgs::Float32& output){

	// Write your code between the following commnets
	/* protected region user implementation of function on begin */
	/* protected region user implementation of function end */

}

void BattLev_callback(const std_msgs::Int32::ConstPtr& input){

	std_msgs::Int32 output;
	output.data = input->data;
	BattLev_pub.publish(output);	

}

void SemLoc_callback(const std_msgs::String::ConstPtr& input){

	std_msgs::String output;
	output.data = input->data;
	SemLoc_pub.publish(output);	

}

void ObstDist_callback(const sensor_msgs::LaserScan::ConstPtr& input){

	std_msgs::Float32 output;
	getClosestObstacle(input, output);
	ObstDist_pub.publish(output);	

}




int main(int argc, char** argv){

	ros::init(argc, argv, "CaseStudyCDM");
	
	ros::NodeHandle nodeHandle;
	
	BattLev_pub = nodeHandle.advertise<std_msgs::Int32>("BattLev",1000);
	BattLev_sub = nodeHandle.subscribe("BattLev", 1000, BattLev_callback);
	SemLoc_pub = nodeHandle.advertise<std_msgs::String>("SemLoc",1000);
	SemLoc_sub = nodeHandle.subscribe("SemLocation", 1000, SemLoc_callback);
	ObstDist_pub = nodeHandle.advertise<std_msgs::Float32>("ObstDist",1000);
	ObstDist_sub = nodeHandle.subscribe("LaserScan", 1000, ObstDist_callback);
	
	ros::spin();
	
	return 0;

}
