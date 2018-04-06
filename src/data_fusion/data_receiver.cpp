/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 订阅毫米波雷达和激光雷达话题，并同步消息，调用一个回调函数进行可视化
* References   :
======================================================================*/
//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Project headers
#include "frontal_delphi_radar/RadarPoint.h"
#include "frontal_delphi_radar/RadarData.h"
#include "velodyne/HDL32Structure.h"

using namespace message_filters;
void callback(const sensor_msgs::PointCloud2ConstPtr& msg_pc,
              const frontal_delphi_radar::RadarDataConstPtr& msg_radar)
{
  ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
  ROS_INFO_STREAM("MMW radar received at " << msg_radar->header.stamp.toSec());

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_transform");

  ros::NodeHandle n;
  ROS_INFO_STREAM("Reading MMW Radar and lidar info");

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n,"lidar_cloud",1);
  message_filters::Subscriber<frontal_delphi_radar::RadarData> radar_sub(n,"radardata",1);
  ROS_INFO_STREAM("subscribing data topics!");
  //data sync
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, frontal_delphi_radar::RadarData> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),cloud_sub, radar_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();


}
