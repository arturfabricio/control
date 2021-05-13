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
#include <numeric>

geometry_msgs::Twist twist;
bool land = false;
bool obstacle_bool = false;
bool avoidance_mode = false;
bool find_points = true;
bool ignore_var = true;
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
long double dist_obstacle = 1000;
long double dist_alternative = 1000;
double overall_distance = 1000;
bool new_goal = false;
bool calculate = true;

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
    double x_total = accumulate(x.begin(), x.end(), 0);
    double y_total = accumulate(y.begin(), y.end(), 0);

    if (x.size() < 20)
    {
        ignore_var = true;
    }
    else if (x.size() > 20)
    {
        dis_obstacle = distance(drone_pos, centroid_point);
    }
    centroid_point.x = x_total / y.size();
    centroid_point.y = y_total / y.size();
    //cout << "Centroid x: " << centroid_point.x << " Centroid y: " << centroid_point.y << "\n";
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
    //cout << "Drone x: " << drone_pos.x << " Drone y: " << drone_pos.y << "\n";
}

void xyz_callback(const PointCloud::ConstPtr &msg)
{
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
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

void align(point pos, point goal)
{

    if (new_goal == false)
    {
        struct point goal_vec
        {
            - (goal.x - pos.x), (goal.y - pos.y)
        };

        double dis_goal = distance(pos, goal);
        cout << "Dis_obstacle: " << dis_obstacle << "\n";
        // cout << "Moving toward final goal" << endl;
        if (dis_obstacle > 0.3)
        {
            if (dis_goal < 0.3 && ((centroid_point.x > 0 || centroid_point.x < 0) && (centroid_point.y > 0 || centroid_point.y < 0)) && ignore_var == false)
            {
                obstacle_bool = true;
            }
            else if (dis_goal > 0.3 && ((centroid_point.x > 0 || centroid_point.x < 0) && (centroid_point.y > 0 || centroid_point.y < 0)) && avoidance_mode == false)
            {
                obstacle_bool = false;
            }
            if (obstacle_bool == false)
            {
                // cout << "To goal!" << endl;
                double angle = angle_to_point(pos, goal, orientation);
                double LowBound = -0.00872665 * dis_goal / 10;
                double UpBound = 0.00872665 * dis_goal / 10;
                // cout << "LowBound: "  << LowBound << "\n";
                // cout << "Upbound: "<< UpBound << "\n";

                if (((angle > LowBound && angle < UpBound) || angle == 0) && dis_goal > 1)
                {
                    //cout << "Forward!" << "\n";
                    angular_control(0, 0, 0);
                    linear_control(100, 0, 0);
                }
                else if (dis_goal < 0.3 && overall_distance < 1)
                {
                    cout << "Arrived at final goal!"
                         << "\n";
                    angular_control(0, 0, 0);
                    linear_control(0, 0, 0);
                    calculate = false;
                }
                else if (dis_goal < 0.3)
                {
                    // cout << "Arrived at temporary goal" << "\n";
                    angular_control(0, 0, 0);
                    linear_control(0, 0, 0);
                    new_goal = false;
                    calculate = true;
                }
                else
                {
                    if (angle > 0)
                    {
                        angular_control(0, 0, -100);
                    }
                    else
                    {
                        angular_control(0, 0, 100);
                    }
                }
            }
        }
        else if (dis_obstacle < 0.3)
        {
            cout << "OBSTACLE!"
                 << "\n";
            angular_control(0, 0, 0);
            linear_control(0, 0, 0);
            new_goal = true;
        }
    }

    if (new_goal == true)
    {
        // double dis_goal = distance(&drone_pos2,&centroid_point);
        // cout << "Moving toward alternative" << endl;
        double angle = angle_to_point(pos, goal, orientation);
        cout << "The angle: " << angle << endl;
        cout << "The distance: " << dis_obstacle << endl;

        if (dis_obstacle < 1)
        {
            if (angle > 0 && angle < 1.57)
            {
                cout << "rotating right" << endl;
                angular_control(0, 0, 100);
            }
            else if (angle < 0 && angle > 1.57)
            {
                cout << "rotating left" << endl;
                angular_control(0, 0, -100);
            }
            else
            {
                linear_control(100, 0, 0);
            }
        }
        else if (dis_obstacle > 1)
        {
            cout << "back to normal" << endl;
            new_goal = false;
        }
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
        align(drone_pos2, goal_point);
    }
    else if (new_goal == true)
    {
        cout << "Moving to Alternative Point" << endl;
        align(drone_pos, centroid_point);
    }
}

int main(int argc, char **argv)
{
    goal_point.x = 0;
    goal_point.y = -10;
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

    ros::Rate loop_rate(45);
    while (ros::ok())
    {
        calc();
        pub_vel.publish(twist);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return (0);
}

// #include <ros/ros.h>
// #include <math.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <vector>
// #include <iostream>
// #include <sensor_msgs/PointCloud2.h>
// #include "nav_msgs/Odometry.h"
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>
// #include "controller.h"
// #include <tf/tf.h>

// std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
// double least_distance = 10000;
// geometry_msgs::Twist twist;
// bool land = false;
// bool obstacle_bool = false;
// ros::Publisher land_pub;
// ros::Publisher takeoff_pub;
// std_msgs::Empty msg;

// struct point
// {
//     double x, y, z;
// };

// struct quaternion
// {
//     double x, y, z, w;
// };

// struct point drone_vec;
// struct point drone_vec2;
// struct point goal_vec;
// struct point obstacle_vec;
// struct point drone_pos;
// struct point drone_pos2;
// struct point alternative;

// struct quaternion orientation;
// struct point goal_point;
// struct point centroid_point;
// double roll, pitch, yaw;
// double angle_2points;
// double slope_goal;
// double slope_drone;
// double dist = 1000;
// double dist_alternative;

// std::vector<double> x_obstacle;
// std::vector<double> y_obstacle;
// std::vector<double> z_obstacle;

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// void linear_control(double x, double y, double z)
// {
//     twist.linear.y = y; //l,r
//     twist.linear.x = x; //f
//     twist.linear.z = z;
// }

// void angular_control(double r, double p, double y)
// {
//     twist.angular.x = r;
//     twist.angular.y = p;
//     twist.angular.z = y; //yaw
// }

// void distance(point *Drone, point *Goal, double dist){
//     dist = ((sqrt(pow((Goal->x - Drone->x), 2) + (pow((Goal->y - Drone->y), 2)) * 10)));
//     //std::cout << "Distance: " << dist << "\n";
// }

// void getAngles(quaternion *orientation)
// {
//     tf::Quaternion q(orientation->x, orientation->y, orientation->z, orientation->w);
//     tf::Matrix3x3 m(q);
//     m.getRPY(roll, pitch, yaw);
//     // std::cout << "Roll: " << roll << "\n";
//     // std::cout << "Pitch: " << pitch << "\n";
//     // std::cout << "Yaw: " << yaw << "\n";
// }

// void drone_vector(point *vec, double angle, point *vec2)
// {
//     vec2->x = vec->x + cos(angle);
//     vec2->y = vec->y + sin(angle);
//     //drone_vec.x = round( drone_vec.x * 100.0 ) / 100.0;
//     //drone_vec.y = round( drone_vec.y * 100.0 ) / 100.0;
//     //std::cout << "Drone vector x: " << drone_vec.x << "  Drone vector y: " << drone_vec.y << "\n";
// }

// void goal_vector(point *drone, point *goal)
// {
//     goal_vec.x = -(goal->x - drone->x);
//     goal_vec.y = (goal->y - drone->y);
//     //std::cout << "goalX: " << goal->x << " goalY: " << goal->y << " droneX: " << drone->x << " droneY: " << drone->y << endl;
//     //std::cout << "Goal vector x: " << goal_vec.x << "  Goal vector y: " << goal_vec.y << "\n";
// }

// void obstacle_vector(point *drone, point *goal)
// {
//     obstacle_vec.x = -(goal->x - drone->x);
//     obstacle_vec.y = (goal->y - drone->y);
//     //std::cout << "goalX: " << goal->x << " goalY: " << goal->y << " droneX: " << drone->x << " droneY: " << drone->y << endl;
//     //std::cout << "Goal vector x: " << goal_vec.x << "  Goal vector y: " << goal_vec.y << "\n";
// }

// double theta(point *dronePos, point *droneVec, point *goalPoint)
// {
//     double angle1 = atan2((droneVec->y - dronePos->y),(droneVec->x - dronePos->x));
//     double angle2 = atan2((goalPoint->y - dronePos->y),(goalPoint->x - dronePos->x));
//     return angle1 - angle2;
// }

// void measure_distance(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z)
// {
//     struct point least_distance_point;
//     std::vector<double> distance;
//     least_distance = 10000;
//     for (int i = 0; i < x.size(); i++)
//     {
//         distance.push_back((sqrt(pow((drone_pos.x - x[i]), 2) + (pow((drone_pos.y - y[i]), 2)))));
//         if (distance[i] < least_distance)
//         {
//             least_distance = distance[i];
//             least_distance_point.x = x[i];
//             least_distance_point.y = y[i];
//             least_distance_point.z = z[i];
//         }
//     }
//     // std::cout << "Least Distance: " << least_distance*10 << "\n";
//     if (least_distance < 2){
//         obstacle_bool = true;
//         cout << "Going into obstacle avoidance!" << "\n";
//     }
// }

// void tf_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     drone_pos.x = msg->pose.position.x;
//     drone_pos.y = msg->pose.position.y;
//     drone_pos.z = msg->pose.position.z;
//     pose.push_back(msg);
//     //cout << "Drone x: " << drone_pos.x << " Drone y: " << drone_pos.y << "\n";
// }

// void centroid(vector<double> x, vector<double> y, vector<double> z){
//     double x_total;
//     double y_total;
//     double z_total;
//     for (int i = 0; i < x.size(); i++){
//         x_total += x[i];
//         z_total += y[i];
//         y_total += z[i];
//     }
//     centroid_point.x = x_total/x.size();
//     centroid_point.y = y_total/y.size();
//     centroid_point.z = z_total/z.size();
//     //cout << "Centroid x: " << centroid_point.x << " Centroid y: " << centroid_point.y << "\n";
// }

// void xyz_callback(const PointCloud::ConstPtr &msg)
// {
//     //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//     BOOST_FOREACH (const pcl::PointXYZ &pt, msg->points)
//     {
//         x_obstacle.push_back(pt.x);
//         y_obstacle.push_back(pt.y);
//         z_obstacle.push_back(pt.z);
//     }
// }

// void chatterCallback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     drone_pos2.x = msg->pose.pose.position.x;
//     drone_pos2.y = msg->pose.pose.position.y;
//     drone_pos2.z = msg->pose.pose.position.z;
//     orientation.x = msg->pose.pose.orientation.x;
//     orientation.y = msg->pose.pose.orientation.y;
//     orientation.z = msg->pose.pose.orientation.z;
//     orientation.w = msg->pose.pose.orientation.w;
//     //cout << "Drone2 x: " << drone_pos2.x << " Drone2 y: " << drone_pos2.y << "\n";
// }

// void height_control(point *Drone){
//     // std::cout << "Height: " << Drone->z << endl;
//     if(Drone->z < 0.9){
//         takeoff_pub.publish(msg);
//     }
//     else if(Drone->z > 0.9 && Drone->z < 1.1){
//         twist.linear.z = 1;
//     }
//     else{
//         twist.linear.z = -1;
//     }
// }

// double alternative_point(point *Drone,double x1, double x2, double y1, double y2)
// {
//     double angle1 = atan2((Drone->y - y1),(Drone->x - x1));
//     double angle2 = atan2((Drone->y - y2),(Drone->x - x2));
//     cout << "Angle 1: " << abs(angle1) << "\n";
//     cout << "Angle 2: " << abs(angle2) << "\n";

//     if (abs(angle1) > abs(angle2)){
//         alternative.x = x2;
//         alternative.y = y2;
//     }
//     else if(abs(angle1) < abs(angle2)){
//         alternative.x = x1;
//         alternative.y = y1;
//     }
// }

// void move_to_alternative(point *goal){
//     cout << "Moving to alternative" << "\n";
//     distance(&drone_pos, &centroid_point, dist_alternative);
//     double angle = theta(&drone_pos, &drone_vec2, goal);
//     double LowBound = -0.00872665*dist_alternative/10;
//     double UpBound = 0.00872665*dist_alternative/10;

//     if (((angle > LowBound && angle < UpBound) || angle == 0) && dist_alternative > 1){
//         angular_control(0,0,0);
//         linear_control(35,0,0);
//     }
//     else if(dist_alternative < 1){
//         cout << "At alternative!" << "\n";
//         angular_control(0,0,0);
//         linear_control(0,0,0);
//         obstacle_bool = false;
//     }
//     else{
//         if(angle > 0){
//             angular_control(0,0,-1);
//         }
//         else{
//             angular_control(0,0,1);
//         }
//     }

// }

// void avoidance_points(point *drone, point *obstacle){
//     if (obstacle_bool = true){
//         double slope = (obstacle->y - drone->y) / (obstacle->x - drone->x);
//         double b = obstacle->y/(-slope*obstacle->x);

//         double new_point1 = -slope*(obstacle->x + 2)+b;
//         double new_point2 = -slope*(obstacle->x - 2)+b;
//         std::cout << "New Point 1 x: " << obstacle->x << "  New Point 1 y: " << new_point1 << "\n";
//         std::cout << "New Point 2 x: " << obstacle->x << "  New Point 2 y: " << new_point2 << "\n";
//         alternative_point(&drone_pos, obstacle->x, obstacle->x, new_point1, new_point2);
//         cout << "Aternative Point x: " << alternative.x << " Alternative Point y: " << alternative.y << "\n";
//         move_to_alternative(&alternative);
//     }

// }

// void calc()
// {
//     height_control(&drone_pos2);
//     distance(&drone_pos2, &goal_point, dist);
//     //measure_distance(x_obstacle, y_obstacle, z_obstacle);
//     //centroid(x_obstacle, y_obstacle, z_obstacle);
//     getAngles(&orientation);
//     drone_vector(&drone_pos2, yaw, &drone_vec);
//     //drone_vector(&drone_pos, yaw, &drone_vec2);
//     goal_vector(&drone_pos2, &goal_point);
// }

// void align(){
//     if (obstacle_bool == false){
//         cout << "Moving to goal" << "\n";
//         cout << "Drone_pos2 x: " << drone_pos2.x << " Drone_pos2 y: " << drone_pos2.y << "\n";
//         double angle = theta(&drone_pos2, &drone_vec, &goal_point);
//         cout << "Angle: " << angle << "\n";
//         double LowBound = -0.00872665*dist/10;
//         double UpBound = 0.00872665*dist/10;

//         if (((angle > LowBound && angle < UpBound) || angle == 0) && dist > 1){
//             cout << "Forward!" << endl;
//             angular_control(0,0,0);
//             linear_control(35,0,0);
//         }
//         else if(dist < 1){
//             cout << "Stop!" << "\n";
//             angular_control(0,0,0);
//             linear_control(0,0,0);
//         }
//         else{
//             cout << "Here else>!" << endl;
//             if(angle > 0){
//                 cout << "Turning! -1" << "\n";
//                 angular_control(0,0,-20);
//             }
//             else if (angle > 0){
//                 angular_control(0,0,20);
//                 cout << "Turning! 1" << "\n";
//             }
//         }
//     }
//     // if (obstacle == true){
//     //     cout << "Obstacle Avoidance" << "\n";
//     //     angular_control(0,0,0);
//     //     linear_control(0,0,0);
//     //     avoidance_points(&drone_pos,&centroid_point);
//     // }
// }

// int main(int argc, char **argv)
// {
//     goal_point.x = 0;
//     goal_point.y = 0;
//     goal_point.z = 0;

//     std::cout << "Initiated" << "\n";
//     ros::init(argc, argv, "my_subscriber");
//     ros::NodeHandle n;
//     ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
//     ros::Subscriber subscribe_state = n.subscribe("/ground_truth/state", 1000, chatterCallback);
//     ros::Subscriber subscriverpc = n.subscribe("/object_cluster", 1, xyz_callback);
//     takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
//     ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
//     land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);

//     ros::Rate loop_rate(30);
//     while (ros::ok())
//     {
//         //takeoff_pub.publish(msg);
//         calc();
//         align();
//         pub_vel.publish(twist);
//         // if (land == true){
//         //     land_pub.publish(msg);
//         // }
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return (0);
// }
