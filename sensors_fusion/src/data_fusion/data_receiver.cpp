/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 订阅毫米波雷达和激光雷达话题，并同步消息，调用一个回调函数进行可视化
* References   :
======================================================================*/
#include <iostream>
#include <string>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//Project headers
#include "frontal_delphi_radar/RadarPoint.h"
#include "frontal_delphi_radar/RadarData.h"
#include "velodyne/HDL32Structure.h"

using namespace message_filters;
using std::string;
FILE* radar_data_save;
void callback(const sensor_msgs::PointCloud2ConstPtr& msg_pc,
              const frontal_delphi_radar::RadarDataConstPtr& msg_radar)
{
  ROS_INFO("Sync_Callback");
  ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
  ROS_INFO_STREAM("MMW radar received at " << msg_radar->header.stamp.toSec());

  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg_pc,*temp_cloud);
  /******save pcd data*******/
  static int sequence = 0;
  string suffix = ".pcd";
  string pcd_filename = "./"+to_string(sequence)+suffix;
  pcl::io::savePCDFileASCII(pcd_filename,*temp_cloud);

  /******save pcd data*******/

  /******save MMW radar data*******/
  string radar_filename = "./"+to_string(sequence)+".txt";
  radar_data_save = fopen(radar_filename.c_str(),"w");
  //每帧毫米波数据一个文件，每个文件中一个目标数据占一行，共64行数据，方便matlab读取形成一个数据矩阵
  for(int i=0;i<64;++i)
  {
    fprintf(radar_data_save,"%d %.6f %.6f %.6f %.6f\n",
        msg_radar->delphi_detection_array[i].target_ID,
        msg_radar->delphi_detection_array[i].range,
        msg_radar->delphi_detection_array[i].angle,
        msg_radar->delphi_detection_array[i].x,
        msg_radar->delphi_detection_array[i].y);
  }
  fclose(radar_data_save);
  /******save MMW radar data*******/
  ++sequence;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "radar_lidar_receiver");

  ros::NodeHandle n;
  ROS_INFO_STREAM("Reading MMW Radar and lidar info");

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n,"lidar_cloud",10);
  message_filters::Subscriber<frontal_delphi_radar::RadarData> radar_sub(n,"radardata",10);
  ROS_INFO_STREAM("subscribing data topics!");
  //data sync
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, frontal_delphi_radar::RadarData> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100),cloud_sub, radar_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();


}
