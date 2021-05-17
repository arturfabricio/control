#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <tf/tf.h>
#include <numeric>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <chrono>

const double speed = 100;

using namespace std;

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
geometry_msgs::Twist twist;
ros::Publisher land_pub;
ros::Publisher takeoff_pub;
std_msgs::Empty msg;

std::vector<double> x_position({15,  0,  -15,  0});
std::vector<double> y_position({ 0, -20,  0,   20});
std::vector<double> z_position({ 0,  0,   0,    0});

struct point
{
    double x, y, z;
};

struct quaternion
{
    double x, y, z, w;
};

struct point centroid_point;
struct point pose_vec;
struct point goal_vec;
struct point drone_pos;
struct point drone_pos2;
struct point alternative;
struct quaternion orientation;
struct quaternion pose_orientation;
struct point goal_point;
double angle_2points;
double dist;
double dis_obstacle = 1000;
double obstacle_distance = 1000;
long double dist_alternative = 1000;
double overall_distance = 1000;
bool at_goal = false;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void linear_control(double x, double y, double z)
{
    twist.linear.y = y; //l,r
    twist.linear.x = x; //f,b
    twist.linear.z = z; //u,d
}

void angular_control(double r, double p, double y)
{
    twist.angular.x = r; //roll
    twist.angular.y = p; //pitch
    twist.angular.z = y; //yaw
}

double distance(point point1, point point2)
{
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2)) * 10;
}

double angle_to_point(point pos, point goal, quaternion orientation)
{
    double roll, pitch, yaw;
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    struct point drone_vec
    {
        pos.x + cos(yaw), pos.y + sin(yaw)
    };
    double angle1 = atan2((drone_vec.y - pos.y), (drone_vec.x - pos.x));
    double angle2 = atan2((goal.y - pos.y), (goal.x - pos.x));
    return angle1 - angle2;
}

void tf_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    drone_pos.x = msg->pose.position.x;
    drone_pos.y = msg->pose.position.y;
    drone_pos.z = msg->pose.position.z;
    pose_orientation.x = msg->pose.orientation.x;
    pose_orientation.y = msg->pose.orientation.y;
    pose_orientation.z = msg->pose.orientation.z;
    pose_orientation.w = msg->pose.orientation.w;
    pose.push_back(msg);
}


void groundThruth_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    drone_pos2.x = msg->pose.pose.position.x;
    drone_pos2.y = msg->pose.pose.position.y;
    drone_pos2.z = msg->pose.pose.position.z;
    orientation.x = msg->pose.pose.orientation.x;
    orientation.y = msg->pose.pose.orientation.y;
    orientation.z = msg->pose.pose.orientation.z;
    orientation.w = msg->pose.pose.orientation.w;
}

void height_control(point *Drone)
{
    if (Drone->z < 0.9)
    {
        takeoff_pub.publish(msg);
    }
    else if (Drone->z > 0.9 && Drone->z < 1.1)
    {
        twist.linear.z = 1;
    }
    else
    {
        twist.linear.z = -1;
    }
}

void to_goal(point pos, point goal){
    
    struct point goal_vec
    {
        - (goal.x - pos.x), (goal.y - pos.y)
    };
    double dis_goal = distance(pos, goal);
    double angle = angle_to_point(pos, goal, orientation);
    double LowBound = -0.00872665 * dis_goal / 10;
    double UpBound = 0.00872665 * dis_goal / 10;
    if (angle > LowBound && angle < UpBound)
    {
        cout << "Moving to point: " << goal.x << " "<< goal.y << " " << goal.z <<endl;
        angular_control(0, 0, 0);
        linear_control(speed, 0, 0);
    }
    else if (dis_goal < 1)
    {
        // cout << "Arrived at temporary goal" << "\n";
        angular_control(0, 0, 0);
        linear_control(0, 0, 0);
        at_goal = true;
    }
    else
    {
        if (angle > 0)
        {
            angular_control(0, 0, -speed);
        }
        else
        {
            angular_control(0, 0, speed);
        }
    }
}

void calc()
{
    height_control(&drone_pos2);
    overall_distance = distance(&drone_pos2, &goal_point);
    to_goal(drone_pos2, goal_point);
}

void UpdateCSV(){
    ofstream GTData, DPose;
    chrono::time_point<chrono::system_clock> start_time;//, end_time;
    start_time = chrono::system_clock::now();
    GTData.open("GroundTruth.csv");
    GTData << "X" << "," << "Y" << "," << "Z" << "\n";
    DPose.open("DronePosition.csv");
    DPose << "X" << "," << "Y" << "," << "Z" << "\n";
    long secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start_time).count(); 
    if(secondsElapsed == 10){
        GTData << drone_pos2.x << "," << drone_pos2.y << "," << drone_pos2.z << endl;
        DPose << drone_pos.x << "," << drone_pos.y << "," << drone_pos.z << endl;
    }
}


int main(int argc, char **argv)
{
    int i = 0;
    
    goal_point.x = x_position[i];
    goal_point.y = y_position[i];
    goal_point.z = z_position[i];
    std::cout << "Initiated"
              << "\n";
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscribe_state = n.subscribe("/ground_truth/state", 1000, groundThruth_Callback);
    takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);

    
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        calc();
        UpdateCSV();
        pub_vel.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
        if(at_goal){
            cout << "At goal" << endl;
            i++;
            if(i > 3){
                i = 0;
            }
            at_goal = false;
        }
        goal_point.x = x_position[i];
        goal_point.y = y_position[i];
        goal_point.z = z_position[i];
        



    }

    return (0);
}
