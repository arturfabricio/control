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
#include <fstream>
#include <string>
#include <chrono>
#include <time.h>




typedef std::chrono::high_resolution_clock Clock;
std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
geometry_msgs::Twist twist;
ros::Publisher takeoff_pub;
ros::Publisher land_pub;
std_msgs::Empty msg;
double least_distance = 10000;
bool new_point = false;
bool reached_point = false;

struct point
{
    double x, y, z;
};

struct quaternion
{
    double x, y, z, w;
};


struct point drone_vec;
struct point goal_vec;
struct point drone_pos;
struct point drone_pos2;
struct quaternion orientation;
struct point goal_point;
double roll, pitch, yaw;
double angle_2points;
double slope_goal;
double slope_drone;
double dist;

std::vector<double> x_position({0,-15, 0, 15, 0});
std::vector<double> y_position({20,0, -20, 0, 20});
std::vector<double> z_position({0, 0, 0, 0, 0});

clock_t timer_s;
clock_t timer_end;
clock_t current_time;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void linear_control(double x, double y, double z)
{
    twist.linear.y = y; //l,r
    twist.linear.x = x; //f
    twist.linear.z = z;
}

void angular_control(double r, double p, double y)
{
    twist.angular.x = r;
    twist.angular.y = p;
    twist.angular.z = y; //yaw
}

void distance(point *Drone, point *Goal){
    dist = ((sqrt(pow((Goal->x - Drone->x), 2) + (pow((Goal->y - Drone->y), 2)) * 10)));
    std::cout << "Distance: " << dist << "\n";
}

void getAngles(quaternion *orientation)
{
    tf::Quaternion q(orientation->x, orientation->y, orientation->z, orientation->w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    // std::cout << "Roll: " << roll << "\n";
    // std::cout << "Pitch: " << pitch << "\n";
    // std::cout << "Yaw: " << yaw << "\n";
}

void drone_vector(point *vec, double angle)
{
    drone_vec.x = vec->x + cos(angle);
    drone_vec.y = vec->y + sin(angle);
    //drone_vec.x = round( drone_vec.x * 100.0 ) / 100.0;
    //drone_vec.y = round( drone_vec.y * 100.0 ) / 100.0;
    //std::cout << "Drone vector x: " << drone_vec.x << "  Drone vector y: " << drone_vec.y << "\n";
}

void goal_vector(point *drone, point *goal)
{
    goal_vec.x = -(goal->x - drone->x);
    goal_vec.y = (goal->y - drone->y);
    //std::cout << "goalX: " << goal->x << " goalY: " << goal->y << " droneX: " << drone->x << " droneY: " << drone->y << endl;
    //std::cout << "Goal vector x: " << goal_vec.x << "  Goal vector y: " << goal_vec.y << "\n";
}

double theta(point *dronePos, point *droneVec, point *goalPoint)
{
    double angle1 = atan2((droneVec->y - dronePos->y),(droneVec->x - dronePos->x));
    double angle2 = atan2((goalPoint->y - dronePos->y),(goalPoint->x - dronePos->x));
    return angle1 - angle2;
}

void measure_distance(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z)
{
    struct point least_distance_point;
    std::vector<double> distance;
    least_distance = 10000;
    for (int i = 0; i < x.size(); i++)
    {
        distance.push_back((sqrt(pow((drone_pos.x - x[i]), 2) + (pow((drone_pos.y - y[i]), 2)))));
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

void chatterCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    drone_pos2.x = msg->pose.pose.position.x;
    drone_pos2.y = msg->pose.pose.position.y;
    drone_pos2.z = msg->pose.pose.position.z;
    orientation.x = msg->pose.pose.orientation.x;
    orientation.y = msg->pose.pose.orientation.y;
    orientation.z = msg->pose.pose.orientation.z;
    orientation.w = msg->pose.pose.orientation.w;
}

void height_control(point *Drone){
    std::cout << "Height: " << Drone->z << endl;
    if(Drone->z < 0.9){
        takeoff_pub.publish(msg);
    }
    else if(Drone->z > 0.9 && Drone->z < 1.1){
        twist.linear.z = 1;
    }
    else{
        twist.linear.z = -1;
    }
}

void calc()
{
    getAngles(&orientation);
    drone_vector(&drone_pos2, yaw);
    goal_vector(&drone_pos2, &goal_point);
    // cout << "Angle: " << theta(&drone_pos2, &drone_vec, &goal_point) << endl;
    distance(&drone_pos2, &goal_point);
    height_control(&drone_pos2);
    //storepose(&drone_pos2);
}

void align(){
    double angle = theta(&drone_pos2, &drone_vec, &goal_point);
    double LowBound = -0.00872665*dist/10;
    double UpBound = 0.00872665*dist/10;
    // cout << "LowBound: "  << LowBound << "\n";
    // cout << "Upbound: "<< UpBound << "\n";
    double i = 0;
    if (((angle > LowBound && angle < UpBound) || angle == 0) && dist > 1){
        std::cout << "Forward!" << "\n";
        angular_control(0,0,0);
        linear_control(500,0,0);
    }
    else if(dist < 1){
        std::cout << "Stop!" << "\n";
        reached_point = true;
        angular_control(0,0,0);
        linear_control(0,0,0);

        switch (reached_point = true && i)
        {
            case (0):
                goal_point.x = x_position[1]; 
                goal_point.y = y_position[1];
                goal_point.z = z_position[1];
                i++;
                reached_point = false;
                break;
            case (1):
                goal_point.x = x_position[2]; 
                goal_point.y = y_position[2];
                goal_point.z = z_position[2];
                i++;
                reached_point = false;
                break;
            case (2):
                goal_point.x = x_position[3]; 
                goal_point.y = y_position[3];
                goal_point.z = z_position[3];
                i++;
                reached_point = false;
                break;
            case (3):
                goal_point.x = x_position[4]; 
                goal_point.y = y_position[4];
                goal_point.z = z_position[4];
                i++;
                reached_point = false;
                break;
            default:
                reached_point = false;
                break;
        }
        
        

        // if(reached_point = true){
        //     for(int i = 0; i < x_position.size(); i++){
        //         goal_point.x = x_position[i]; 
        //         goal_point.y = y_position[i];
        //         goal_point.z = z_position[i];
        //         reached_point = false;
        //     }
                
        // } 
        


    }
    else{
        std::cout << "Rotating!" << "\n";
        if(angle > 0){
            angular_control(0,0,-1);

        }
        else{
            angular_control(0,0,1);
        }
        
    }
    
    std::cout << "i is valued: " << i << "\n";
    // if (dist > 1 && cond == false){
    //     linear_control(1,0,0);
    //     angular_control(0,0,0);
    //     cout << "Forward!" << "\n";
    // }
    //  if (dist < 1){
    //      linear_control(0,0,0);
    //      angular_control(0,0,0);
    //      cout << "Landing!" << "\n";
    //      land = true;
    // }
    //land_pub.publish(msg);

}

int main(int argc, char **argv)
{
    goal_point.x = x_position[0];
    goal_point.y = y_position[0];
    goal_point.z = z_position[0];

    std::cout << "Initiated" << "\n";
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle n;
    
    ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscribe_state = n.subscribe("/ground_truth/state", 1000, chatterCallback);
    //ros::Subscriber subscriverpc = n.subscribe("/orb_slam2_mono/map_points", 1, xyz_callback);
    takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);
    
    std::fstream poseData;
    poseData.open("poseData.csv", ios::out);
    poseData << "x" << "," << "y" << "," << "z" << "\n";
    ros::Rate loop_rate(30);
   
    while (ros::ok())
    {
        
        timer_s = clock();
        calc();
        align();
        timer_end = clock();
        //takeoff_pub.publish(msg);
        current_time =  timer_end - timer_s;
        std::cout << "timer_end - timer_s: " << current_time << "\n";
        if(current_time == 40){
            poseData << drone_pos2.x << "," << drone_pos2.y << "," << drone_pos2.z << "\n";
        }
        pub_vel.publish(twist);
        // if (land == true){
        //     land_pub.publish(msg);
        // }
        ros::spinOnce();
        loop_rate.sleep();
        
    }
    

    return (0);
}