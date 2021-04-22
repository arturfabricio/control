#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

using namespace ros;

int main(int argc, char **argv)
{
    init(argc, argv, "controller");
    NodeHandle n;
    Publisher takeoff_pub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
    std_msgs::Empty msg;
    while (ros::ok())
    {
        takeoff_pub.publish(msg);
        spinOnce();
    }
    return 0;
}
