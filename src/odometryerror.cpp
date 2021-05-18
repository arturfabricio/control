#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <tf/tf.h>
#include <numeric>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

int i = 1;
const double speed = 1;
const double rspeed = 0.3;
const double threshold = 1.5;

std::vector<double> x_position({20,  31,  32,  28,  27,  8,   4});
std::vector<double> y_position({46,  35,  23,   9,   8, 16,  34});

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

geometry_msgs::Twist twist;
std_msgs::Empty msg;
ros::Publisher land_pub;
ros::Publisher takeoff_pub;

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

struct point drone_pos_vo;
struct point drone_pos_gt;     //Change name so it is easier to distinguish between ground truth and vo
struct quaternion orientation; //Same as above
struct quaternion pose_orientation;
struct point goal_point; //Revisit
double overall_distance = 1000;
bool new_goal = false;
bool at_goal = false;

// ofstream GTData, DPose;
// ofstream HeightCSV;

// void UpdateCSV(){
//     GTData << drone_pos2.x << "," << drone_pos2.y << "," << drone_pos2.z << endl;
//     DPose << drone_pos.x << "," << drone_pos.y << "," << drone_pos.z << endl; 
//     HeightCSV << drone_pos2.z << endl;
//     //cout << "Updating CSV files" << endl;
// }

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

double distance_points(point point1, point point2)
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

pair<point, double> pcl_center(vector<double> x, vector<double> y, vector<double> z)
{
    double x_total;
    double y_total;
    point center_point;
    for (int i = 0; i < x.size(); i++)
    {
        x_total += x[i];
        y_total += y[i];
    }
    center_point.x = x_total / x.size();
    center_point.y = y_total / y.size();
    double dis_obstacle = distance_points(drone_pos_vo, center_point);
    cout << "Distance centroid: " << dis_obstacle << endl;
    return make_pair(center_point, dis_obstacle);
}

void orb_slam_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    drone_pos_vo.x = msg->pose.position.x;
    drone_pos_vo.y = msg->pose.position.y;
    drone_pos_vo.z = msg->pose.position.z;
    pose_orientation.x = msg->pose.orientation.x;
    pose_orientation.y = msg->pose.orientation.y;
    pose_orientation.z = msg->pose.orientation.z;
    pose_orientation.w = msg->pose.orientation.w;
}

void obstacle_callback(const PointCloud::ConstPtr &msg)
{
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
    drone_pos_gt.x = msg->pose.pose.position.x;
    drone_pos_gt.y = msg->pose.pose.position.y;
    drone_pos_gt.z = msg->pose.pose.position.z;
    orientation.x = msg->pose.pose.orientation.x;
    orientation.y = msg->pose.pose.orientation.y;
    orientation.z = msg->pose.pose.orientation.z;
    orientation.w = msg->pose.pose.orientation.w;
}

void height_control(point drone)
{
    if (drone.z < 0.9)
    {
        takeoff_pub.publish(msg);
    }
    else if (drone.z > 0.9 && drone.z < 1.1)
    {
        twist.linear.z = 1;
    }
    else
    {
        twist.linear.z = -1;
    }
}

void avoid_obstacle(point pos, point obstacle, double distance_obstacle)
{
    double angle = angle_to_point(pos, obstacle, orientation) + 1.57;
    cout << "The angle: " << angle << endl;
    cout << "The distance_obstacle: " << distance_obstacle << endl;

    if (angle > 0 && angle < 1.3)
    {
        cout << "rotating left" << endl;
        linear_control(0, 0, 0);
        angular_control(0, 0, rspeed);
    }
    else if (angle < 0 && angle > -1.3)
    {
        cout << "rotating right" << endl;
        linear_control(0, 0, 0);
        angular_control(0, 0, -rspeed);
    }
    else
    {
        cout << "Moving forward" << endl;
        angular_control(0, 0, 0);
        linear_control(speed, 0, 0);
    }
}

void to_goal(point pos, point goal, double distance_obstacle)
{
    double angle = angle_to_point(pos, goal, orientation);
    double LowBound = -0.00872665 * overall_distance / 10;
    double UpBound = 0.00872665 * overall_distance / 10;

    if (((angle > LowBound && angle < UpBound) || angle == 0) && distance_obstacle > 1)
    {
        angular_control(0, 0, 0);
        linear_control(speed, 0, 0);
    }
    else
    {
        if (angle > 0)
        {
            angular_control(0, 0, -rspeed);
        }
        else
        {
            angular_control(0, 0, rspeed);
        }
    }
}

int main(int argc, char **argv)
{
    goal_point.x = 10;
    goal_point.y = 15;
    goal_point.z = 0;

    std::cout << "Initiated" << endl;
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 10, orb_slam_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscribe_state = n.subscribe("/ground_truth/state", 10, groundThruth_Callback);
    ros::Subscriber subscriverpc = n.subscribe("/object_cluster", 1, obstacle_callback);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
    land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);
    //HeightCSV.open("HeightTracking.csv");
    //GTData.open("GroundTruth.csv");
    //DPose.open("DronePosition.csv");



    ros::Rate loop_rate(30);

    while (ros::ok())
    {

        height_control(drone_pos_gt);
        overall_distance = distance_points(drone_pos_gt, goal_point);
        pair<point, double> center_point = pcl_center(x_obstacle, y_obstacle, z_obstacle);
        if (overall_distance < threshold)
        {
            cout << "Arrived at final goal!" << endl;
            i++;
            goal_point.x = x_position[i];
            goal_point.y = y_position[i];
            //angular_control(0, 0, 0);
            //linear_control(0, 0, 0);
            //land_pub.publish(msg);
            //break;
        }
        else if (center_point.second < threshold && x_obstacle.size() > 20)
        {
            cout << "OBSTACLE!" << endl;
            avoid_obstacle(drone_pos_vo, center_point.first, center_point.second);
        }
        else
        {
            cout << "Moving to Goal Point" << endl;
            to_goal(drone_pos_gt, goal_point, center_point.second);
        }
        pub_vel.publish(twist);
        cout <<  goal_point.x << "," <<  goal_point.y  << "\n";
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
// #include <nav_msgs/Odometry.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>
// #include <boost/foreach.hpp>
// #include <tf/tf.h>
// #include <numeric>
// #include <std_msgs/Empty.h>
// #include <geometry_msgs/Twist.h>
// #include <fstream>
// #include <time.h>

// const double speed = 5;
// const double rspeed = 0.3;
// int i = 0;
// using namespace std;

// std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
// geometry_msgs::Twist twist;
// ros::Publisher land_pub;
// ros::Publisher takeoff_pub;
// std_msgs::Empty msg;

// std::vector<double> x_position({10,  15,  10,      -10, -15, -10,   0});
// std::vector<double> y_position({15,   0, -15,      -15,   0,  15,  20});


// struct point
// {
//     double x, y, z;
// };

// struct quaternion
// {
//     double x, y, z, w;
// };

// struct point centroid_point;
// struct point pose_vec;
// struct point goal_vec;
// struct point drone_pos;
// struct point drone_pos2;
// struct point alternative;
// struct quaternion orientation;
// struct quaternion pose_orientation;
// struct point goal_point;
// double angle_2points;
// double dist;
// double dis_obstacle = 1000;
// double obstacle_distance = 1000;
// long double dist_alternative = 1000;
// double overall_distance = 1000;
// bool at_goal = false;

// ofstream GTData, DPose;
// ofstream HeightCSV;

// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// void linear_control(double x, double y, double z)
// {
//     twist.linear.y = y; //l,r
//     twist.linear.x = x; //f,b
//     twist.linear.z = z; //u,d
// }

// void angular_control(double r, double p, double y)
// {
//     twist.angular.x = r; //roll
//     twist.angular.y = p; //pitch
//     twist.angular.z = y; //yaw
// }

// double distance(point point1, point point2)
// {
//     return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2)) * 10;
// }

// double angle_to_point(point pos, point goal, quaternion orientation)
// {
//     double roll, pitch, yaw;
//     tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
//     tf::Matrix3x3 m(q);
//     m.getRPY(roll, pitch, yaw);

//     struct point drone_vec
//     {
//         pos.x + cos(yaw), pos.y + sin(yaw)
//     };
//     double angle1 = atan2((drone_vec.y - pos.y), (drone_vec.x - pos.x));
//     double angle2 = atan2((goal.y - pos.y), (goal.x - pos.x));
//     return angle1 - angle2;
// }

// void tf_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     drone_pos.x = msg->pose.position.x;
//     drone_pos.y = msg->pose.position.y;
//     drone_pos.z = msg->pose.position.z;
//     pose_orientation.x = msg->pose.orientation.x;
//     pose_orientation.y = msg->pose.orientation.y;
//     pose_orientation.z = msg->pose.orientation.z;
//     pose_orientation.w = msg->pose.orientation.w;
//     pose.push_back(msg);
// }


// void groundThruth_Callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     drone_pos2.x = msg->pose.pose.position.x;
//     drone_pos2.y = msg->pose.pose.position.y;
//     drone_pos2.z = msg->pose.pose.position.z;
//     orientation.x = msg->pose.pose.orientation.x;
//     orientation.y = msg->pose.pose.orientation.y;
//     orientation.z = msg->pose.pose.orientation.z;
//     orientation.w = msg->pose.pose.orientation.w;
// }

// void height_control(point *Drone)
// {
//     if (Drone->z < 0.9)
//     {
//         takeoff_pub.publish(msg);
//     }
//     else if (Drone->z > 0.9 && Drone->z < 1.1)
//     {
//         twist.linear.z = 1;
//     }
//     else
//     {
//         twist.linear.z = -1;
//     }
// }

// void to_goal(point pos, point goal){
    
//     struct point goal_vec
//     {
//         - (goal.x - pos.x), (goal.y - pos.y)
//     };
//     double dis_goal = distance(pos, goal);
//     double angle = angle_to_point(pos, goal, orientation);
//     double LowBound = -0.00872665 * dis_goal / 10;
//     double UpBound = 0.00872665 * dis_goal / 10;
//     if (angle > LowBound && angle < UpBound)
//     {
//         at_goal = false;
//         cout << "Moving to point: " << goal.x << " "<< goal.y << " " << goal.z <<endl;
//         cout << "Drone pos: " << drone_pos2.x << " , " << drone_pos2.y << endl;

//         angular_control(0, 0, 0);
//         linear_control(speed, 0, 0);
//     }
//     else if (dis_goal < 1 && at_goal == false)
//     {
//         i++;
//         goal_point.x = x_position[i];
//         goal_point.y = y_position[i];
//         cout << "Moving to point V2: " << goal.x << " "<< goal.y << " " << goal.z <<endl;
//         cout << "Drone pos V2: " << drone_pos2.x << " , " << drone_pos2.y << endl;
//         at_goal = true;
//     }
//     else
//     {
//         at_goal = false;
//         if (angle > 0)
//         {
//             angular_control(0, 0, -rspeed);
//         }
//         else
//         {
//             angular_control(0, 0, rspeed);
//         }
//     }
// }

// void calc()
// {
//     height_control(&drone_pos2);
//     overall_distance = distance(&drone_pos2, &goal_point);
//     to_goal(drone_pos2, goal_point);
// }

// void UpdateCSV(){
//     GTData << drone_pos2.x << "," << drone_pos2.y << "," << drone_pos2.z << endl;
//     DPose << drone_pos.x << "," << drone_pos.y << "," << drone_pos.z << endl; 
//     HeightCSV << drone_pos2.z << endl;
//     //cout << "Updating CSV files" << endl;
// }


// int main(int argc, char **argv)
// {
//     int ms = 5;

//     goal_point.x = x_position[i];
//     goal_point.y = y_position[i];
//     goal_point.z = 0;
//     std::cout << "Initiated"
//               << "\n";
//     ros::init(argc, argv, "my_subscriber");
//     ros::NodeHandle n;
//     ros::Subscriber subscribetf = n.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
//     ros::Subscriber subscribe_state = n.subscribe("/ground_truth/state", 1000, groundThruth_Callback);
//     takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
//     ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
//     land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);

//     HeightCSV.open("HeightTracking.csv");
//     GTData.open("GroundTruth.csv");
//     DPose.open("DronePosition.csv");
//     clock_t timer_start, timer_end;
    
//     // timer_start = time(0);

//     ros::Rate loop_rate(60);
//     while (ros::ok())
//     {  

//         calc();
//         pub_vel.publish(twist);
//         // timer_end = time(0);
//         // if(at_goal){
//         //     cout << "At goal" << endl;
//         //     if(i > 7){
//         //         i = 0;
//         //         //cout << "Finished Lap, proceeding to next" << endl; 
//         //     }
//         //     at_goal = false;
//         // }
//         // goal_point.x = x_position[i];
//         // goal_point.y = y_position[i];
        
//         //cout << "milliseconds: "<< timer_end - timer_start << endl;
//         // if((timer_end - timer_start ) == ms){
//         //     UpdateCSV();
//         //     timer_start = timer_start+ms;
//         // }

//         ros::spinOnce();
//         loop_rate.sleep();

//     }

//     return (0);
// }