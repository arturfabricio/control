#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
double x_current = 0;
double y_current = 0;
double z_current = 0;
double ptx;
double pty;
double ptz;
double least_distance = 10000;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void measure_distance(std::vector<float> &x, std::vector<float> &y, std::vector<float> &z)
{
    std::vector<double> distance;
    least_distance = 10000;
    for (int i = 0; i < x.size(); i++)
    {
        distance.push_back(sqrt(pow((x_current - x[i]), 2) + (pow((y_current - y[i]), 2)) + (pow((z_current - z[i]), 2))));
        if (distance[i] < least_distance)
        {
            least_distance = distance[i];
        }
    }
    std::cout << "Least Distance: " << least_distance << "\n";
}

void tf_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // ROS_INFO_STREAM("Received pose: " << msg);
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
    z_current = msg->pose.position.z;
    // ROS_INFO_STREAM(y_current);
    pose.push_back(msg);
}

void xyz_callback(const PointCloud::ConstPtr &msg)
{
    std::vector<float> x_obstacle;
    std::vector<float> y_obstacle;
    std::vector<float> z_obstacle;
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ &pt, msg->points)
    {
        //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        x_obstacle.push_back(pt.x);
        y_obstacle.push_back(pt.y);
        z_obstacle.push_back(pt.z);
    }
    measure_distance(x_obstacle, y_obstacle, z_obstacle);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber subscribetf = nh.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscriverpc = nh.subscribe("/statisticalOutliers/output", 1, xyz_callback);
    ros::spin();
    return (0);
}