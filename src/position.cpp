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
#include <iomanip>

const double speed = 100;
 
using namespace std;

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
geometry_msgs::Twist twist;
bool obstacle_bool = false;
bool avoidance_mode = false;
ros::Publisher land_pub;
ros::Publisher takeoff_pub;
std_msgs::Empty msg;

std::vector<double> x_obstacle;
std::vector<double> y_obstacle;
std::vector<double> z_obstacle;

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
struct point goal_vec1;
struct point goal_vec2;
struct point drone_pos;
struct point drone_pos2;
struct point alternative;
struct quaternion orientation;
struct quaternion pose_orientation;
struct point goal_point;
double angle_2points;
double slope_goal;
double slope_drone;
double dist;
double dis_obstacle = 1000;
double obstacle_distance = 1000;
long double dist_alternative = 1000;
double overall_distance = 1000;
bool new_goal = false;
bool calculate = true;
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

void centroid(vector<double> x, vector<double> y, vector<double> z)
{
    double x_total;
    double y_total;
    for (int i = 0; i < x.size(); i++){
        x_total += x[i];
        y_total += y[i];
    }
    dis_obstacle = distance(drone_pos, centroid_point);
    cout << "Distance centroid: " << dis_obstacle << endl;
    centroid_point.x = x_total / y.size();
    centroid_point.y = y_total / y.size();
    // cout << "Centroid x: " << centroid_point.x << " Centroid y: " << centroid_point.y << "\n";
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
    // cout << "Drone x: " << drone_pos.x << " Drone y: " << drone_pos.y << "\n";
}

void xyz_callback(const PointCloud::ConstPtr &msg)
{
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    x_obstacle.clear();
    y_obstacle.clear();
    z_obstacle.clear();
    BOOST_FOREACH (const pcl::PointXYZ &pt, msg->points)
    {
        x_obstacle.push_back(pt.x);
        y_obstacle.push_back(pt.y);
        z_obstacle.push_back(pt.z);
    }
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
    cout << "Dis_obstacle: " << dis_obstacle << "\n";
    //cout << "x_obstacle Size: " << x_obstacle.size() << endl;
    double threshold = 2;
    // cout << "Moving toward final goal" << endl;
    if (dis_obstacle < threshold && x_obstacle.size() > 20)
    {
        cout << "OBSTACLE!" << "\n";
        angular_control(0, 0, 0);
        linear_control(0, 0, 0);
        new_goal = true;
    }
    else if (dis_obstacle > threshold)
    {
        // cout << "To goal!" << endl;
        double angle = angle_to_point(pos, goal, orientation);
        double LowBound = -0.00872665 * dis_goal / 10;
        double UpBound = 0.00872665 * dis_goal / 10;
        // cout << "LowBound: "  << LowBound << "\n";
        // cout << "Upbound: "<< UpBound << "\n";

        if (((angle > LowBound && angle < UpBound) || angle == 0) && dis_obstacle > 1)
        {
            //cout << "Forward!" << "\n";
            angular_control(0, 0, 0);
            linear_control(speed, 0, 0);
        }
        else if (dis_obstacle < threshold)
        {
            // cout << "Arrived at temporary goal" << "\n";
            angular_control(0, 0, 0);
            linear_control(0, 0, 0);
            new_goal = false;
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
}

void avoid_obstacle(point pos, point obstacle){
    // double dis_goal = distance(&drone_pos2,&centroid_point);
        // cout << "Moving toward alternative" << endl;
        double angle = angle_to_point(pos, obstacle, orientation) + 1.57;
        double dis_obstacle = distance(pos, obstacle);
        double threshold = 2.2;

        //cout << "The angle: " << angle << endl;
        //cout << "The distance_obstacle: " << dis_obstacle << endl;

        if (dis_obstacle < threshold)
        {
            if (angle > 0 && angle < 1.15)
            {
                //cout << "rotating left" << endl;
                angular_control(0, 0, speed);
            }
            else if (angle < 0 && angle > -1.15)
            {
                //cout << "rotating right" << endl;
                angular_control(0, 0, -speed);
            }
            else
            {
                //cout << "Moving forward" << endl;
                angular_control(0,0,0);
                linear_control(speed, 0, 0);
            }
        }
        else
        {
            //cout << "back to normal" << endl;
            new_goal = false;
        }
        
}

void calc()
{
    height_control(&drone_pos2);
    overall_distance = distance(&drone_pos2, &goal_point);
    // obstacle_distance = distance(&drone_pos, &alternative);
    centroid(x_obstacle, y_obstacle, z_obstacle);
    if (new_goal == false)
    {
        cout << "Moving to Goal Point" << endl;
        to_goal(drone_pos2, goal_point);
    }
    else if (new_goal == true)
    {
        cout << "Moving to Alternative Point" << endl;
        avoid_obstacle(drone_pos, centroid_point);
    }
    if (overall_distance < 2)
        {
            cout << "Arrived at final goal!" << "\n";
            angular_control(0, 0, 0);
            linear_control(0, 0, 0);
            at_goal = true;
    }
}

int main(int argc, char **argv)
{
    goal_point.x = -5;
    goal_point.y = -16;
    goal_point.z = 0;

    std::cout << "Initiated"
              << "\n";
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscribe_state = n.subscribe("/ground_truth/state", 1000, groundThruth_Callback);
    ros::Subscriber subscriverpc = n.subscribe("/object_cluster", 1, xyz_callback);
    takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        calc();
        //pub_vel.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
        if(at_goal){
            // land_pub.publish(msg);
            break;
        } 
    }

    return (0);
}
