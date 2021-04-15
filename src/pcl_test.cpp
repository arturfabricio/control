#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback (const PointCloud::ConstPtr& msg)
{
    printf ("Cloud : width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/orb_slam2_mono/map_points",1,callback);
    ros::spin();
}