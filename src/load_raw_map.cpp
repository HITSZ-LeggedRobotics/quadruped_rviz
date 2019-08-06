#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>
using namespace std;
std::string map_file;


int main (int argc, char **argv)
{
  ros::init (argc, argv, "load_raw_map");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/3d_map/raw", 10);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("global_map_file",map_file);
  pcl::io::loadPCDFile (map_file, cloud);
  double init_points_num = cloud.size();
  cout << "init_points_num" << init_points_num << endl;

  //Convert the cloud to ROS message
  pcl::toROSMsg(cloud, output);
  cout<< "pcd-> rosmsg "<<endl;
  output.header.frame_id = "map";//this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
  output.header.stamp = ros::Time::now();
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
