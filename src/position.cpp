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
struct point drone_vec;
struct point pose_vec;
struct point goal_vec;
struct point goal_vec1;
struct point goal_vec2;
struct point drone_pos;
struct point drone_pos2;
struct point alternative;
struct point alternative1;
struct point alternative2;
struct quaternion orientation;
struct quaternion pose_orientation;
struct point goal_point;
double roll, pitch, yaw, roll2, pitch2, yaw2;
double angle_2points;
double slope_goal;
double slope_drone;
double dist;
double angle1;
double angle2;
long double dist_obstacle = 1000;
long double dist_alternative = 1000;


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
    //std::cout << "Distance: " << dist << "\n";
}

void distance2(point *Drone, point *Goal){
    dist_obstacle = ((sqrt(pow((Goal->x - Drone->x), 2) + (pow((Goal->y - Drone->y), 2)) * 10)));
    std::cout << "Distance Obstacle: " << dist_obstacle << "\n";
}

void distance3(point *Drone, point *Goal){
    dist_alternative = ((sqrt(pow((Goal->x - Drone->x), 2) + (pow((Goal->y - Drone->y), 2)) * 10)));
    std::cout << "Distance Alternative: " << dist_alternative << "\n";
}

void getAngles(quaternion *orientation)
{
    tf::Quaternion q(orientation->x, orientation->y, orientation->z, orientation->w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    //std::cout << "Yaw: " << yaw << "\n";
}

void getAngles2(quaternion *orientation)
{
    tf::Quaternion q(orientation->x, orientation->y, orientation->z, orientation->w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll2, pitch2, yaw2);
    // std::cout << "Roll: " << roll << "\n";
    // std::cout << "Pitch: " << pitch << "\n";
    //std::cout << "Yaw 2: " << yaw2 << "\n";
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

void centroid(vector<double> x, vector<double> y, vector<double> z){
    double x_total;
    double y_total;
    for (int i = 0; i < x.size(); i++){
        x_total += x[i];
        y_total += y[i];
    }
    if (x.size() < 250){
        ignore_var = true;
    }else if (x.size() > 250){
        ignore_var = false;
    }
    cout << "Size points: " << x.size() << endl;
    centroid_point.x = x_total/y.size();
    centroid_point.y = y_total/y.size();

    cout << "Centroid x: " << centroid_point.x << " Centroid y: " << centroid_point.y << "\n";
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
    measure_distance(x_obstacle, y_obstacle, z_obstacle);
    if (x_obstacle.size() < 200){
        cout << "nada" << endl;
    }else if (x_obstacle.size() > 200){
        distance2(&drone_pos, &centroid_point);
    }
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
    //std::cout << "Height: " << Drone->z << endl;
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
    getAngles2(&pose_orientation);
    drone_vector(&drone_pos2, yaw);
    goal_vector(&drone_pos2, &goal_point);
    height_control(&drone_pos2);
    centroid(x_obstacle,y_obstacle,z_obstacle);
}

void drone_vector_orb(point *vec, double angle)
{
    pose_vec.x = vec->x + cos(angle);
    pose_vec.y = vec->y + sin(angle);
    //std::cout << "Pose vector x: " << drone_vec.x << "  POse vector y: " << drone_vec.y << "\n";
}

void goal_vector1(point *drone, point *goal)
{
    goal_vec1.x = -(goal->x - drone->x);
    goal_vec1.y = (goal->y - drone->y);
    //std::cout << "goalX: " << goal->x << " goalY: " << goal->y << " droneX: " << drone->x << " droneY: " << drone->y << endl;
    //std::cout << "Goal vector x: " << goal_vec.x << "  Goal vector y: " << goal_vec.y << "\n";
}

void goal_vector2(point *drone, point *goal)
{
    goal_vec2.x = -(goal->x - drone->x);
    goal_vec2.y = (goal->y - drone->y);
    //std::cout << "goalX: " << goal->x << " goalY: " << goal->y << " droneX: " << drone->x << " droneY: " << drone->y << endl;
    //std::cout << "Goal vector x: " << goal_vec.x << "  Goal vector y: " << goal_vec.y << "\n";
}

double theta2(point *dronePos, point *droneVec, point *goalPoint)
{
    double angle1 = atan2((droneVec->y - dronePos->y),(droneVec->x - dronePos->x));
    double angle2 = atan2((goalPoint->y - dronePos->y),(goalPoint->x - dronePos->x));
    return angle1 - angle2;
}

void move_to_alternative(point *goal, double angle_new){
    cout << "Moving to alternative" << "\n";
    double LowBound = -0.00872665;
    double UpBound = 0.00872665;

    if (((angle_new > LowBound && angle_new < UpBound) || angle_new == 0) && dist_alternative > 1.5 && obstacle_bool == true){
        cout << "hererererererrere" << endl;
        angular_control(0,0,0);
        linear_control(100,0,0);
    }
    if(dist_alternative < 1.5){
        cout << "At alternative!" << "\n";
        avoidance_mode = false;
        obstacle_bool = false;
        angular_control(0,0,0);
        linear_control(0,0,0);
    }
    else{
        if(angle_new > 0 && obstacle_bool == true){
            cout << "turning 1" << endl;
            angular_control(100,0,-50);
        }
        else if(angle_new < 0 && obstacle_bool == true){
            angular_control(100,0,50);
            cout << "turning 2" << endl;
        }
    }
}

void avoidance_points(point *drone, point *obstacle){
    cout << "Avoidance function" << endl;
    drone_vector_orb(&drone_pos,yaw); //pose_vec
    goal_vector1(&drone_pos, &centroid_point); //goal_vec1
    angle1 = theta2(&drone_pos, &pose_vec, &goal_vec1);
    if(find_points == true){
    if (obstacle_bool == true){
        double t = 0.785398;
            angular_control(0,0,0);
            linear_control(0,0,0);
            if (angle1 < 0){
                alternative.x = (cos(t)*pose_vec.x - sin(t)*pose_vec.y);
                alternative.y = sin(t)*pose_vec.x + cos(t)*pose_vec.y;
            }
            if (angle1 > 0){
                alternative.x = cos(-t)*pose_vec.x - sin(-t)*pose_vec.y;
                alternative.y = sin(-t)*pose_vec.x + cos(-t)*pose_vec.y;
            find_points = false;
            }
        
    }
        std::cout << "New Point 1 x: " << alternative.x << "  New Point 1 y: " << alternative.y << "\n";
        angle2 = theta(&drone_pos, &pose_vec, &alternative);
        distance3(&drone_pos,&alternative);
        move_to_alternative(&alternative,angle2);
    }
}

void align(){
    distance(&drone_pos2, &goal_point);
    if (dist_obstacle < 0.4 && ((centroid_point.x > 0 || centroid_point.x < 0 ) && (centroid_point.y > 0 || centroid_point.y < 0 )) && ignore_var == false){
        //cout << "Bool_true" << endl;
        obstacle_bool = true;
    }else if(dist_obstacle > 0.4 && ((centroid_point.x > 0 || centroid_point.x < 0 ) && (centroid_point.y > 0 || centroid_point.y < 0 )) && avoidance_mode == false ){
        //cout << "Bool_false" << endl;
        obstacle_bool = false;
    }
    if (obstacle_bool == false && avoidance_mode == false){
        cout << "To goal!" << endl;
        double angle = theta(&drone_pos2, &drone_vec, &goal_point);
        double LowBound = -0.00872665*dist/10;
        double UpBound = 0.00872665*dist/10;
        // cout << "LowBound: "  << LowBound << "\n";
        // cout << "Upbound: "<< UpBound << "\n";

        if (((angle > LowBound && angle < UpBound) || angle == 0) && dist > 1){
            //cout << "Forward!" << "\n";
            angular_control(0,0,0);
            linear_control(50,0,0);
        }
        else if(dist < 1){
            cout << "Arrived at goal!" << "\n";
            angular_control(0,0,0);
            linear_control(0,0,0);
        }
        else{
            //cout << "Rotating!" << "\n";
            if(angle > 0){
                angular_control(0,0,-50);
            }
            else{
                angular_control(0,0,50);
            }
        }
    }
    if (obstacle_bool == true){
        cout << "Obstacle Detected!" << endl;
        avoidance_mode = true;
        angular_control(0,0,0);
        linear_control(0,0,0);
        avoidance_points(&drone_pos, &centroid_point);
    }
}

int main(int argc, char **argv)
{
    goal_point.x = 0;
    goal_point.y = -10;
    goal_point.z = 0;

    std::cout << "Initiated" << "\n";
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscribe_state = n.subscribe("/ground_truth/state", 1000, chatterCallback);
    ros::Subscriber subscriverpc = n.subscribe("/object_cluster", 1, xyz_callback);
    takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        calc();
        align();
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
