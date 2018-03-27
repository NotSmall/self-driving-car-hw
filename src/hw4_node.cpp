#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Header.h>
#include <stdlib.h> 
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

//using namespace Eigen;

class SubscribeAndPublish {
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_orig_pointer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_orig_pointer;
  pcl::PointCloud<pcl::PointXYZ> scene_orig;
  pcl::PointCloud<pcl::PointXYZ> target_orig;
  SubscribeAndPublish(): scene_orig_pointer(new pcl::PointCloud<pcl::PointXYZ>), 
  target_orig_pointer(new pcl::PointCloud<pcl::PointXYZ>), scene_orig(), target_orig(){
    // pub and sub initial
    pub = nh.advertise<sensor_msgs::PointCloud2>("hw4output", 1);
    // do the launch later
    sub_scene = nh.subscribe<sensor_msgs::PointCloud2>("/scene/cloud_pcd", 1, &SubscribeAndPublish::scene_callback, this);
    sub_target = nh.subscribe<sensor_msgs::PointCloud2>("/target/cloud_pcd", 1, &SubscribeAndPublish::target_callback, this);
  }
  void scene_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::PCLPointCloud2 temp_pclpc2;
    pcl_conversions::toPCL(*msg, temp_pclpc2);
    fromPCLPointCloud2 (temp_pclpc2, scene_orig);
    *scene_orig_pointer.get() = scene_orig;
  }
  void target_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    pcl::PCLPointCloud2 temp_pclpc2;
    pcl_conversions::toPCL(*msg, temp_pclpc2);
    fromPCLPointCloud2 (temp_pclpc2, target_orig);
    *target_orig_pointer.get() = target_orig;
    // output setting
    sensor_msgs::PointCloud2 output = *msg;
    output.header.frame_id = "/map";

    // mapping
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(scene_orig_pointer);
    icp.setInputTarget(target_orig_pointer);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    pcl::toPCLPointCloud2(Final, temp_pclpc2);
    //pcl_conversions::fromPCL(temp_pclpc2, output);

    // publish to rviz
    while (pub.getNumSubscribers() < 1) // rviz subscribe check
    {
      if (!ros::ok())
      {
        break;
      }
      ROS_WARN_ONCE("Please create a subscriber");
      sleep(1);
    }
    pub.publish(output);
    
  }

private:
  ros::NodeHandle nh; 
  ros::Publisher pub;
  ros::Subscriber sub_scene;
  ros::Subscriber sub_target;
};//End of class SubscribeAndPublish

int main(int argc, char **argv) {
  ros::init(argc, argv, "hw4_node");
  SubscribeAndPublish SAPObject;
  ros::spin();
  return 0;
}
