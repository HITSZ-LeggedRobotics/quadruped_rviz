#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <ros/ros.h>
#include <iostream>
#include <string.h>

using namespace std;
std::string map_file;
double voxel_filter_;
double filter_point_num;


int main (int argc, char **argv)
{

  ros::init (argc, argv, "load_raw_pcd");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/3d_map", 1);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ>  filter_cloud;


  sensor_msgs::PointCloud2 output;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("global_map_file",map_file);
  private_nh.getParam("voxel_grid_filter",voxel_filter_);
  private_nh.getParam("filter_point_num",filter_point_num);//100000
  pcl::io::loadPCDFile (map_file, cloud);
  double init_points_num = cloud.size();

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  double scan_points_num = 0.0;
  if(init_points_num > filter_point_num){
      voxel_grid_filter.setLeafSize(voxel_filter_,voxel_filter_,voxel_filter_);
      voxel_grid_filter.setInputCloud(cloud.makeShared());
      voxel_grid_filter.filter(filter_cloud);
      scan_points_num = filter_cloud.size();
  }else{
      filter_cloud = cloud;
  }
  cout << "init_points_num" << init_points_num << "  scan_points_num: " <<  scan_points_num<< endl;

  //removeNaN
  vector<int> target_indices;
  pcl::removeNaNFromPointCloud(filter_cloud,filter_cloud,target_indices);
  size_t scan_points_num_removeNAN = filter_cloud.size();
  cout << "  scan_points_num_removeNAN: " <<  scan_points_num_removeNAN<< endl;

  //Convert the cloud to ROS message
  pcl::toROSMsg(filter_cloud, output);
  cout<< "pcd-> rosmsg "<<endl;
  output.header.frame_id = "map";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
  output.header.stamp = ros::Time::now();
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    pcl_pub.publish(output);

    // ROS_ERROR("publish pcl_load");
    ros::spinOnce();
    loop_rate.sleep();
  }
  cout <<"publish pcl_load" <<endl;
  return 0;

}
