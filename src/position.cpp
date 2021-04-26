#include <ros/ros.h>
#include <math.h>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
double x_current = 0;
double y_current = 0;
double z_current = 0;
double ptx;
double pty;
double ptz;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // ROS_INFO_STREAM("Received pose: " << msg);
    x_current = msg->pose.position.x;
    y_current = msg->pose.position.y;
    z_current = msg->pose.position.z;
    // ROS_INFO_STREAM(y_current);
    pose.push_back(msg);
}

void xyz_callback(const PointCloud::ConstPtr& msg){
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH(const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

// void measure_distance (float x1, float y1, float z1, float x2, float y2, float z2){
//     distance = sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1))+((z2-z1)*(z2-z1)));
// }

// void pcl_to_octomap(){
//     octomap::point3d sensorOrigin(x,y,z);
//     for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pt.begin(); it != pt.end(); ++it){
//         cloud.push_back(it->x,it->y,it->z);
//     }
//     tree.insertPointCloud(cloud,sensorOrigin);
//     tree.writeBinary("simple_tree.bt");
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber subscribetf = nh.subscribe("/orb_slam2_mono/pose", 1000, tf_callback); //Topic_name, queue size and callback function.
    ros::Subscriber subscriverpc = nh.subscribe("/octomap_point_cloud_centers", 1, xyz_callback);
    // ros::Publisher point_pub = nh.advertise<pcl::PointXYZ>("/points", 1000);
    
    // while(ros::ok()){
    // point_pub.publish(pt);
    ros::spin();
    // }
    return(0);
}