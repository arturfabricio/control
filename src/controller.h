#ifndef CONTROLLER_H // include guard
#define CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>

using namespace std;
using namespace ros;

void controller(NodeHandle n, double init_x, double init_y, double init_z, double goal_x, double goal_y, double goal_z){
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
}

#endif