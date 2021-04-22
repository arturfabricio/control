#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>

using namespace std;
using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv, "controller");
    NodeHandle n;
    Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
    Publisher land_pub = n.advertise<std_msgs::Empty>("ardrone/land", 10);
    Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    std_msgs::Empty msg;
    geometry_msgs::Twist vel;
    string choice;
    while (ros::ok())
    {
        cin >> choice;
        if (choice == "t")
        {
            takeoff_pub.publish(msg);
        }
        else if (choice == "l")
        {
            land_pub.publish(msg);
        }
        else
        {
            vel.linear.x = stoi(choice);
            vel_pub.publish(vel);
        }
        spinOnce();
    }
    return 0;
}
