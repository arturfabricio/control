#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro
#include <geometry_msgs/PoseStamped.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;
struct point
{
    double x, y, z;
};
struct point drone_pos;

void drone_position(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //ros::Publisher dronePose;
    // ROS_INFO_STREAM("Received pose: " << msg);
    drone_pos.x = msg->pose.position.x;
    drone_pos.y = msg->pose.position.y;
    drone_pos.z = msg->pose.position.z;
    // ROS_INFO_STREAM(drone_pos.y);
    pose.push_back(msg);
    //dronePose.publish(pose);
}

int main(int argc, char *argv[])
{
    // Initialzie ROS Node

    ros::init(argc, argv, "perception_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh_("~");

    // Set up parameters
    std::string cloud_topic, world_frame, camera_frame, pose_topic;
    world_frame = "map";
    camera_frame = "front_link";
    cloud_topic = "orb_slam2_mono/map_points";
    pose_topic = "orb_slam2_mono/pose";

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

        // Listen for drone position
        std::string poseDrone = nh.resolveName(pose_topic);
        //ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topic " << topic);
        geometry_msgs::PoseStamped::ConstPtr drone_pose =
            ros::topic::waitForMessage<geometry_msgs::PoseStamped>(poseDrone, nh);

        //Subtract drone position from pointcloud
        typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointXYZ newpoint;
        pcl::PointXYZ dronePoint;
        dronePoint.x = drone_pose->pose.position.x;
        dronePoint.y = drone_pose->pose.position.y;
        dronePoint.z = drone_pose->pose.position.z;
        pcl::fromROSMsg(*recent_cloud, cloud);
        //Subtract drone pose from pointcloud, to get the clusters relative to the drone.
        pcl::PointCloud<pcl::PointXYZ> update_cloud;
        for (int i = 0; i < cloud.points.size(); i++)
        {
            newpoint.x = cloud.points[i].x - dronePoint.x;
            newpoint.y = cloud.points[i].y - dronePoint.y;
            newpoint.z = cloud.points[i].z - dronePoint.z;
            update_cloud.push_back(newpoint);
        }

        //Pass-through filters
        pcl::PointCloud<pcl::PointXYZ>::Ptr update_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(update_cloud));
        //x-direction
        pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(update_cloud_ptr);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-0.5, 0.5);
        pass_x.filter(xf_cloud);

        //y-direction
        pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(xf_cloud_ptr);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-0.5, 0.5);
        pass_y.filter(yf_cloud);

        // Add drone position to pointcloud
        pcl::PointCloud<pcl::PointXYZ> new_cloud;
        for (int i = 0; i < yf_cloud.points.size(); i++)
        {
            newpoint.x = yf_cloud.points[i].x + dronePoint.x;
            newpoint.y = yf_cloud.points[i].y + dronePoint.y;
            newpoint.z = yf_cloud.points[i].z + dronePoint.z;
            new_cloud.push_back(newpoint);
        }

        // // Transform PointCloud from Camera Frame to World Frame
        // tf::TransformListener listener;
        // tf::StampedTransform stransform;
        // try
        // {
        //     listener.waitForTransform(world_frame, recent_cloud->header.frame_id, ros::Time::now(), ros::Duration(6.0));
        //     listener.lookupTransform(world_frame, recent_cloud->header.frame_id, ros::Time(0), stransform);
        // }
        // catch (tf::TransformException ex)
        // {
        //     ROS_ERROR("%s", ex.what());
        // }
        // sensor_msgs::PointCloud2 transformed_cloud;
        // pcl_ros::transformPointCloud(world_frame, stransform, *recent_cloud, transformed_cloud);

        // // Convert PointCloud from ROS to PCL
        // pcl::PointCloud<pcl::PointXYZ> cloud;
        // pcl::fromROSMsg(transformed_cloud, cloud);

        //Voxel Grid Filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(new_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_ptr);
        voxel_filter.setLeafSize(float(0.01), float(0.01), float(0.01));
        voxel_filter.filter(*cloud_voxel_filtered);

        // // Crop-Box Filtering
        // pcl::PointCloud<pcl::PointXYZ> xyz_filtered_cloud;
        // pcl::CropBox<pcl::PointXYZ> crop;
        // crop.setInputCloud(cloud_voxel_filtered);
        // Eigen::Vector4f min_point = Eigen::Vector4f(-1.5, -1.5, -1.5, 0);
        // Eigen::Vector4f max_point = Eigen::Vector4f(1.5, 1.5, 1.5, 0);
        // crop.setMin(min_point);
        // crop.setMax(max_point);
        // crop.filter(xyz_filtered_cloud);

        //Plane Segmentation
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud_voxel_filtered));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.010);

        // Segment the largest planar component from the cropped cloud
        seg.setInputCloud(cropped_cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            ROS_WARN_STREAM("Could not estimate a planar model for the given dataset.");
            //break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cropped_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size() << " data points.");

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);

        //Euclidean Cluster Extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        *cloud_filtered = *cloud_f;
        tree->setInputCloud(cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.1); // 2cm
        ec.setMinClusterSize(10);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
            clusters.push_back(cloud_cluster);
            sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
            pc2_clusters.push_back(tempROSMsg);
        }

        // Statistical Outlier Removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud_ptr = clusters.at(0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cluster_cloud_ptr);
        sor.setMeanK(50);
        sor.setStddevMulThresh(0.5);

        sor.filter(*sor_cloud_filtered);

        // Convert PointCloud from PCL to ROS
        sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*(clusters.at(0)), *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp = ros::Time::now();
        object_pub.publish(pc2_cloud);
    }
    return 0;
}