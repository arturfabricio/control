#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/crop_box.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <tf_conversions/tf_eigen.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.h>

int main(int argc, char *argv[])
{
    // Initialzie ROS Node
    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh_("~");

    // Set up parameters
    std::string cloud_topic, world_frame, camera_frame;
    world_frame = "map";
    camera_frame = "camera";
    cloud_topic = "orb_slam2_mono/map_points";

    // Set up publishers
    ros::Publisher object_pub, cluster_pub, pose_pub;
    object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);
    cluster_pub = nh.advertise<sensor_msgs::PointCloud2>("primary_cluster", 1);
    while (ros::ok())
    {
        // Listen for PointCloud
        std::string topic = nh.resolveName(cloud_topic);
        ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);
        sensor_msgs::PointCloud2::ConstPtr recent_cloud =
            ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, nh);

        // Transform PointCloud from Camera Frame to World Frame
        tf::TransformListener listener;
        tf::StampedTransform stransform;
        try
        {
            listener.waitForTransform(world_frame, recent_cloud->header.frame_id, ros::Time::now(), ros::Duration(6.0));
            listener.lookupTransform(world_frame, recent_cloud->header.frame_id, ros::Time(0), stransform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        sensor_msgs::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

        // Convert PointCloud from ROS to PCL
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(transformed_cloud, cloud);

        //Voxel Grid Filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_ptr);
        voxel_filter.setLeafSize(float(0.002), float(0.002), float(0.002));
        voxel_filter.filter(*cloud_voxel_filtered);

        // Conver PointCloud from PCL to ROS
        sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_ptr, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp = ros::Time::now();
        object_pub.publish(pc2_cloud);
    }
    return 0;
}