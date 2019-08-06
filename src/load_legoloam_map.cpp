#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include<pcl/io/pcd_io.h>

std::string map_file;

int main (int argc, char **argv)
{
  ros::init (argc, argv, "legomap");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("/legoloam/3d_map", 10);

  pcl::PointCloud<pcl::PointXYZ> cloud1,cloud2;
  sensor_msgs::PointCloud2 output;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("global_map_file",map_file);
  pcl::io::loadPCDFile (map_file, cloud1);
//   pcl::io::loadPCDFile ("/home/xavier/test_ws/last.pcd", cloud1);

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();


  //绕x轴旋转一个theta角
  transform_2.rotate(Eigen::AngleAxisf(1.570795, Eigen::Vector3f::UnitX()));

  //执行变换
  //pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudOut(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(cloud1, cloud2, transform_2);


  pcl::toROSMsg(cloud2,output);// 转换成ROS下的数据类型 最终通过topic发布

  output.header.stamp=ros::Time::now();
  output.header.frame_id  ="/map";

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    pcl_pub.publish(output);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
