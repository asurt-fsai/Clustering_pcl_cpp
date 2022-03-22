#include <ros/ros.h>
#include <time.h>
#include <vector>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>

#include <asurt_msgs/CloudArray.h>

ros::Publisher pub;

typedef pcl::PointXYZ PointT;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  ros::Time begin = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg (*input, cloud);
  std::cout<<"ana ast2blt mn lidar 1"<< std::endl;

  // Data containers used
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud.makeShared());
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);
  pcl::PassThrough<pcl::PointXYZ> pass;
  /*
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(2,5);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-1,2);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-1.0,5);
  pass.filter(*cloud_filtered);
  */

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits(0, 10);
  pass.filter(*cloud_filtered);
  
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-2.3,2.3);
  pass.filter(*cloud_filtered);
  
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.5, 2.3);
  pass.filter(*cloud_filtered);
  //pass.setFilterFieldName("z");
  //pass.setFilterLimits(-0.3,0.3);

  ec.setClusterTolerance (0.5); // 4cm
  ec.setMinClusterSize (8);
  ec.setMaxClusterSize (170);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
 

  std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;
 
  asurt_msgs::CloudArray cloud_array = asurt_msgs::CloudArray();
  std::vector<pcl::PointCloud<pcl::PointXYZI>> cluster_pointclouds(cluster_indices.size()); 
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    cluster_pointclouds[j] = pcl::PointCloud<pcl::PointXYZI>();
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
        pcl::PointXYZ pt = cloud_filtered->points[*pit];
            pcl::PointXYZI pt2;
            pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
            pt2.intensity = (float)(j + 1);

            // Add the pooint to its respective cluster's pointcloud.
            cluster_pointclouds[j].push_back(pt2);
    }

    // Convert To ROS data type 
    pcl::PCLPointCloud2 cloud_p;
    pcl::toPCLPointCloud2(cluster_pointclouds[j], cloud_p);
    
    sensor_msgs::PointCloud2 output; 
    pcl_conversions::fromPCL(cloud_p, output);
    output.header.frame_id = "velodyne";

    cloud_array.CloudArray.push_back(output);

    j++;
  }
  
  pub.publish(cloud_array); 
  ros::Time end = ros::Time::now();
  std::cout<<"time of cluster:"<< (end-begin)*1000 <<std::endl;
  
 
  ROS_INFO("published it."); 
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/perception/filtered", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("pclplaneoutput", 1);
  pub = nh.advertise<asurt_msgs::CloudArray> ("/clusters_pcs", 1);
 
  // Spin
  ros::spin ();
}
