#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h> //hydro

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h> //hydro
#include "pcl_ros/transforms.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
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
    camera_frame = "ardrone_base_frontcam";
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
        sensor_msgs::PointCloud2 transformed_clonamespace "pcl" has no member "SACSegmentation"C/C++(135)

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(transformed_cloud, cloud);

        //Voxel Grid Filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud_ptr);
        voxel_filter.setLeafSize(float(0.01), float(0.01), float(0.01));
        voxel_filter.filter(*cloud_voxel_filtered);

        //Pass-through filters//////////////////////////////////////////
            //x-direction
        pcl::PointCloud<pcl::PointXYZ> xf_cloud, yf_cloud, zf_cloud;
        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_voxel_filtered);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-1.5,1.5);
        pass_x.filter(xf_cloud);

            //y-direction
        pcl::PointCloud<pcl::PointXYZ>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(xf_cloud));
        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(xf_cloud_ptr);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-1.5, 1.5);
        pass_y.filter(yf_cloud);

            //z-direction
        pcl::PointCloud<pcl::PointXYZ>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(yf_cloud));
        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(yf_cloud_ptr);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-1.5, 1.5);
        pass_z.filter(zf_cloud);

        //Plane Segmentation///////////////////////////////////////////
        pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>(zf_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
            
            // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (200);
        seg.setDistanceThreshold (0.004);
            
            // Segment the largest planar component from the cropped cloud
        seg.setInputCloud (cropped_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        ROS_WARN_STREAM ("Could not estimate a planar model for the given dataset.") ;
        //break;
        }

            // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cropped_cloud);
        extract.setIndices(inliers);
        extract.setNegative (false);

            // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." );

            // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);


        // //Euclidean Cluster Extraction
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        // *cloud_filtered = *cloud_f;
        // tree->setInputCloud (cloud_filtered);

        // std::vector<pcl::PointIndices> cluster_indices;
        // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        // ec.setClusterTolerance (0.01); // 2cm
        // ec.setMinClusterSize (300);
        // ec.setMaxClusterSize (10000);
        // ec.setSearchMethod (tree);
        // ec.setInputCloud (cloud_filtered);
        // ec.extract (cluster_indices);

        // std::vector<sensor_msgs::PointCloud2::Ptr> pc2_clusters;
        // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
        // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        // {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        // for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        //     cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
        // cloud_cluster->width = cloud_cluster->points.size ();
        // cloud_cluster->height = 1;
        // cloud_cluster->is_dense = true;
        // std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
        // clusters.push_back(cloud_cluster);
        // sensor_msgs::PointCloud2::Ptr tempROSMsg(new sensor_msgs::PointCloud2);
        // pcl::toROSMsg(*cloud_cluster, *tempROSMsg);
        // pc2_clusters.push_back(tempROSMsg);
        // }


        // Convert PointCloud from PCL to ROS
        sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(zf_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp = ros::Time::now();
        object_pub.publish(pc2_cloud);
    }
    return 0;
}