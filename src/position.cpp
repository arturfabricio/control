#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "controller.h"

struct goal_point{
    double x;
    double y;
    double z;
};

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
double least_distance = 10000;
struct point
{
    double x, y, z;
};
struct point drone_pos;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void measure_distance(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z)
{
    struct point least_distance_point;
    std::vector<double> distance;
    least_distance = 10000;
    for (int i = 0; i < x.size(); i++)
    {
        distance.push_back((sqrt(pow((drone_pos.x - x[i]), 2) + (pow((drone_pos.y - y[i]), 2)) + (pow((drone_pos.z - z[i]), 2)))) * 10);
        if (distance[i] < least_distance)
        {
            least_distance = distance[i];
            least_distance_point.x = x[i];
            least_distance_point.y = y[i];
            least_distance_point.z = z[i];
        }
    }
    //std::cout << "Least Distance: " << least_distance << "\n";
}

void tf_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // ROS_INFO_STREAM("Received pose: " << msg);
    drone_pos.x = msg->pose.position.x;
    drone_pos.y = msg->pose.position.y;
    drone_pos.z = msg->pose.position.z;
    // ROS_INFO_STREAM(drone_pos.y);
    pose.push_back(msg);
}

void xyz_callback(const PointCloud::ConstPtr &msg)
{
    std::vector<double> x_obstacle;
    std::vector<double> y_obstacle;
    std::vector<double> z_obstacle;
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZ &pt, msg->points)
    {
        printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
        x_obstacle.push_back(pt.x);
        y_obstacle.push_back(pt.y);
        z_obstacle.push_back(pt.z);
    }
    measure_distance(x_obstacle, y_obstacle, z_obstacle);
    std::cout << "x: " << x_obstacle[0] << "\n";
    std::cout << "y: " << y_obstacle[0] << "\n";
    std::cout << "z: " << z_obstacle[0] << "\n";


}

int main(int argc, char **argv)
{
    std::cout << "Initiated" << "\n";
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscriverpc = n.subscribe("/orb_slam2_mono/map_points", 1, xyz_callback);
    // controller(n,x_current,y_current,z_current, 0, 0, 0);
    ros::spin();
    return (0);
}