#include <iostream>
#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <environment/Interpreter.hpp>
#include <libsvm/svmWrapper.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
ros::Publisher cloud_pub;
nav_msgs::Odometry odom_data;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//odom_data.pose.pose.position.x=
//odom_data.pose.pose.orientation.z=
std::string published_topic_name,subscribed_topic_name, subscribed_topic_name_odometry;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

double x=0,y=0;
double nx=0,ny=0;
int size=0;
double r=0,theta=0;
void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_set)
{
	cloud=point_set;
	size=(int)point_set->points.size();
}
void callBackOdom(const nav_msgs::Odometry& odom)
{
	PointCloud::Ptr cloud_msg(new PointCloud);
    cloud_msg->header.frame_id="base_link";
	cloud_msg->height=1;
	for(int i=0;i<size;i++)
	{
		x=cloud->points[i].x;
		y=cloud->points[i].y;
		nx=odom.pose.pose.position.x*0.01 + y * 2 * odom.pose.pose.orientation.z * odom.pose.pose.orientation.w + x * (1 - 2 * odom.pose.pose.orientation.z * odom.pose.pose.orientation.z);
		ny=odom.pose.pose.position.y*0.01 + y * (1 - 2 * odom.pose.pose.orientation.z * odom.pose.pose.orientation.z) - x * 2 * odom.pose.pose.orientation.z * odom.pose.pose.orientation.w;
		cloud_msg->points.push_back(pcl::PointXYZ((int)(nx),(int)(ny),0));
	}
	cloud_msg->width=cloud_msg->points.size();
	cloud_pub.publish(cloud_msg);
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "mapping");
	ros::NodeHandle handle;
	
	std::string node_name = std::string("/") + ros::this_node::getName();
	handle.getParam(node_name + "/published_topic",published_topic_name);
	handle.getParam(node_name + "/subscribed_topic",subscribed_topic_name);
	handle.getParam(node_name + "/odometry_topic",subscribed_topic_name_odometry);
	cloud_pub = handle.advertise<pcl::PointCloud<pcl::PointXYZ> >(published_topic_name, 1000);
	ros::Subscriber input_cloud=handle.subscribe(subscribed_topic_name,1000,callBackCloud);
	ros::Subscriber input_odom=handle.subscribe(subscribed_topic_name_odometry,1000,callBackOdom);
	ros::spin();
	return 0;
}