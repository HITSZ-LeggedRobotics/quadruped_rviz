#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>

//just publish every path
using namespace std;

class starthit_utils{

public:

    void connect(const ros::NodeHandle nh){
        n = nh;
        subscribe();
        advertise();
    }

private:
    ros::NodeHandle n;
    ros::Subscriber leg_sub, ekf_legodom_rl_sub,ekf_legodom_rpe_sub,legodom_odom_sub,gazebo_pose_sub,ndt_pose_sub,mrobot_sub, ekf_ndt_sub,predict_pose_sub;
    ros::Publisher leg_odom_path_pub,legodom_map_path_pub,ground_truth_odom_path_pub,ground_truth_odom_pose, ekf_legodom_rl_path_pub,ekf_legodom_rpe_path_pub,ndt_path_pub,ekf_ndt_pub,predict_pose_pub,ekf_legodom_pub;

    nav_msgs::Path leg_path_msg,leg_map_path_msg, ground_truth_odom_path_msg, ekf_rl_leg_path_msg,ekf_rpe_leg_path_msg, leg1_path_msg, ndt_path_msg, ekf_ndt_path_msg,predict_pose_msg;
    geometry_msgs::PoseStamped tmp_pose,tmp_pose_gazebo,tmp_pose_rl,tmp_pose_rpe,tmp_pose_legodom,tmp_pose_ndt,tmp_ekf_ndt_pose, tmp_predict_pose;
    geometry_msgs::PoseWithCovarianceStamped legodom_pose;

    void subscribe()
    {
        gazebo_pose_sub = n.subscribe("/gazebo/odom", 10, &starthit_utils::gazebocb, this);
        legodom_odom_sub = n.subscribe("/legodom_map", 10, &starthit_utils::legodomCb, this);

        //lidar
//        ndt_pose_sub = n.subscribe("/current_pose",10,&starthit_utils::NDTcallback,this);
//        predict_pose_sub = n.subscribe("/predict_pose", 10, &starthit_utils::NdtPredictCb, this);

        //robot_localization
         ekf_legodom_rl_sub = n.subscribe("/robot_localization/odom_3d",10, &starthit_utils::ekflegRLCb, this);
        //robot_pose_ekf
         ekf_legodom_rpe_sub = n.subscribe("/robot_pose_ekf/odom_3d",10, &starthit_utils::ekflegRPECb, this);

        //slam
//        ekf_ndt_sub = n.subscribe("/ekf_pose_with_covariance", 10, &starthit_utils::ekfndtCb, this);

    }

    void advertise()
    {

        leg_path_msg.poses.resize(2);
        leg_map_path_msg.poses.resize(2);
        ekf_rl_leg_path_msg.poses.resize(2);
        ekf_rpe_leg_path_msg.poses.resize(2);
        ndt_path_msg.poses.resize(2);

        //gazebo path
        ground_truth_odom_path_pub = n.advertise<nav_msgs::Path>("/gazebo_ground_truth/path",10);
        ground_truth_odom_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/starthit/robot_state/pose",10);
        //legodom
        legodom_map_path_pub = n.advertise<nav_msgs::Path>("/legodom_map/path",10);
        //ekf-legodom
        ekf_legodom_rl_path_pub = n.advertise<nav_msgs::Path>("/ekf_rl_legodom_path",10);
        ekf_legodom_rpe_path_pub = n.advertise<nav_msgs::Path>("/ekf_rpe_legodom_path",10);

        //lidar
//        ndt_path_pub = n.advertise<nav_msgs::Path>("/ndt_pose/path",10);

//        ekf_ndt_pub = n.advertise<nav_msgs::Path>("/ekf_localizer/path",10);


    }
    void gazebocb(const nav_msgs::Odometry::ConstPtr& msg){
        ground_truth_odom_path_msg.header.frame_id = "map";
        ground_truth_odom_path_msg.header.stamp = msg->header.stamp;
        ground_truth_odom_path_msg.header.seq = msg->header.seq;
        tmp_pose_gazebo.header.frame_id = "map";
        tmp_pose_gazebo.header.stamp = msg->header.stamp;
        tmp_pose_gazebo.header.seq = msg->header.seq;
        tmp_pose_gazebo.pose = msg->pose.pose;
        ground_truth_odom_path_msg.poses.push_back(tmp_pose_gazebo);
        ground_truth_odom_path_pub.publish(ground_truth_odom_path_msg);
        //pose
        legodom_pose.header = msg->header;
        legodom_pose.pose = msg->pose;
        ground_truth_odom_pose.publish(legodom_pose);
    }

    void legodomCb(const nav_msgs::Odometry::ConstPtr& msg){

        leg_map_path_msg.header.frame_id = "map";
        leg_map_path_msg.header.stamp = msg->header.stamp;
        leg_map_path_msg.header.seq = msg->header.seq;
        tmp_pose_legodom.header.frame_id = "map";
        tmp_pose_legodom.header.stamp = msg->header.stamp;
        tmp_pose_legodom.header.seq = msg->header.seq;
        tmp_pose_legodom.pose = msg->pose.pose;
        leg_map_path_msg.poses.push_back(tmp_pose_legodom);
        legodom_map_path_pub.publish(leg_map_path_msg);

    }


    void ekflegRLCb(const nav_msgs::Odometry::ConstPtr& msg){
        ekf_rl_leg_path_msg.header.frame_id = "map";
        ekf_rl_leg_path_msg.header.stamp = msg->header.stamp;
        ekf_rl_leg_path_msg.header.seq = msg->header.seq;
        tmp_pose_rl.header.frame_id = "map";
        tmp_pose_rl.header.stamp = msg->header.stamp;
        tmp_pose_rl.header.seq = msg->header.seq;
        tmp_pose_rl.pose = msg->pose.pose;
        ekf_rl_leg_path_msg.poses.push_back(tmp_pose_rl);
        ekf_legodom_rl_path_pub.publish(ekf_rl_leg_path_msg);


    }

    void ekflegRPECb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
        ekf_rpe_leg_path_msg.header.frame_id = "map";
        ekf_rpe_leg_path_msg.header.stamp = msg->header.stamp;
        ekf_rpe_leg_path_msg.header.seq = msg->header.seq;
        tmp_pose_rpe.header.frame_id = "map";
        tmp_pose_rpe.header.stamp = msg->header.stamp;
        tmp_pose_rpe.header.seq = msg->header.seq;
        tmp_pose_rpe.pose = msg->pose.pose;
        ekf_rpe_leg_path_msg.poses.push_back(tmp_pose_rpe);
        ekf_legodom_rpe_path_pub.publish(ekf_rpe_leg_path_msg);
     }


//    void NDTcallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
//        ndt_path_msg.header.frame_id = "map";
//        ndt_path_msg.header.stamp = msg->header.stamp;
//        ndt_path_msg.header.seq = msg->header.seq;
//        tmp_pose_ndt.header.frame_id = "map";
//        tmp_pose_ndt.header.stamp = msg->header.stamp;
//        tmp_pose_ndt.header.seq = msg->header.seq;
//        tmp_pose_ndt.pose = msg->pose.pose;
//        ndt_path_msg.poses.push_back(tmp_pose_ndt);
//        ndt_path_pub.publish(ndt_path_msg);
//    }
//    void NdtPredictCb(const geometry_msgs::PoseStamped::ConstPtr& msg){

//        predict_pose_msg.header.frame_id = "map";
//        predict_pose_msg.header.stamp = msg->header.stamp;
//        predict_pose_msg.header.seq = msg->header.seq;
//        tmp_predict_pose.pose = msg->pose;
//        predict_pose_msg.poses.push_back(tmp_predict_pose);
//        predict_pose_pub.publish(predict_pose_msg);

//    }

//    void ekfndtCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
//        ekf_ndt_path_msg.header.frame_id = "map";
//        ekf_ndt_path_msg.header.stamp = msg->header.stamp;
//        ekf_ndt_path_msg.header.seq = msg->header.seq;
//        tmp_ekf_ndt_pose.header.frame_id = "map";
//        tmp_ekf_ndt_pose.header.stamp = msg->header.stamp;
//        tmp_ekf_ndt_pose.header.seq = msg->header.seq;
//        tmp_ekf_ndt_pose.pose = msg->pose.pose;
//        ekf_ndt_path_msg.poses.push_back(tmp_ekf_ndt_pose);
//        ekf_ndt_pub.publish(ekf_ndt_path_msg);
//    }

};


int main( int argc, char** argv )
{
       ros::init(argc, argv, "leg_utils");
       ros::NodeHandle n;
       starthit_utils* su = new starthit_utils();
       su->connect(n);
       std::cout << "path show in rviz " << std::endl;

       ros::spin();
//       ros::Rate loop_rate(10);
//       while(ros::ok()){
//           ros::spinOnce();

//           loop_rate.sleep();
//       }

       return 0;
}
