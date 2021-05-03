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

void distance(double x1, double x2, double y1, double y2, double z1, double z2){
    double dist = ((sqrt(pow((x2 - x1), 2) + (pow((y2 - y1), 2)) + (pow((z2 - z1), 2)))) * 10);
    // std::cout << "Distance: " << dist << "\n";
}

#endif