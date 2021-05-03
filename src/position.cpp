#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include "nav_msgs/Odometry.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "controller.h"
#include <tf/tf.h>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
double least_distance = 10000;
geometry_msgs::Twist twist;


struct point
{
    double x, y, z;
};

struct quaternion
{
    double x,y,z,w;
};

struct point drone_pos;
struct point drone_pos2;
struct quaternion orientation;
struct point goal_point;
double roll, pitch, yaw;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void linear_control(int x,int y){
    twist.linear.y = y; //l,r
    twist.linear.x = x; //f
}

void yaw_control(int s){
    twist.angular.z = s; //yaw
}

void getAngles(double x, double y, double z, double w){
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    std::cout << "Roll: " << roll << "\n";
    std::cout << "Pitch: " << pitch << "\n";
    std::cout << "Yaw: " << yaw << "\n";
}

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
    drone_pos.x = msg->pose.position.x;
    drone_pos.y = msg->pose.position.y;
    drone_pos.z = msg->pose.position.z;
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
        x_obstacle.push_back(pt.x);
        y_obstacle.push_back(pt.y);
        z_obstacle.push_back(pt.z);
    }
    measure_distance(x_obstacle, y_obstacle, z_obstacle);
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  drone_pos2.x = msg->pose.pose.position.x;
  drone_pos2.y = msg->pose.pose.position.y;
  drone_pos2.z = msg->pose.pose.position.z;
  orientation.x = msg->pose.pose.orientation.x;
  orientation.y = msg->pose.pose.orientation.y;
  orientation.z = msg->pose.pose.orientation.z;
  orientation.w = msg->pose.pose.orientation.w;
  getAngles(orientation.x, orientation.y, orientation.z, orientation.w);
}

int main(int argc, char **argv)
{
    goal_point.x = 0;
    goal_point.y = -10;
    goal_point.z = 0;

    std::cout << "Initiated" << "\n";
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle n;
    // ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscribe_state = n.subscribe("/ground_truth/state", 1000, chatterCallback);
    ros::Subscriber subscriverpc = n.subscribe("/orb_slam2_mono/map_points", 1, xyz_callback);
    ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
    ros::Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);  
    std_msgs::Empty msg;

    while (ros::ok()){
        // takeoff_pub.publish(msg);
        distance(drone_pos2.x,goal_point.x,drone_pos2.y,goal_point.y,drone_pos2.z,goal_point.z);
        ros::spinOnce();
        // pub_vel.publish(twist);

    }

    return (0);
}

