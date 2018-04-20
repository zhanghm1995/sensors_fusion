/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 订阅相机图像和激光雷达点云话题，并同步消息，调用一个回调函数进行可视化
* References   :
======================================================================*/
//C/C++
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <string>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
//OpenCV
#include <opencv2/opencv.hpp>
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include "boost/asio.hpp"

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h> //image handler
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "frontal_delphi_radar/RadarData.h"
#include "velodyne/HDL32Structure.h"
#include <glog/logging.h>

class DataSync
{
public:
  DataSync(ros::NodeHandle& nodehandle):nodehandle_(nodehandle)
  ,processthread_(NULL)
  ,processthreadfinished_ (false),
  subCameraImage_(nodehandle_,"/image_raw",100),
  subRadarData_(nodehandle_,"/radardata",100),
  sync(MySyncPolicy(100), subCameraImage_, subRadarData_)
{
    //发布同步后的数据
    pubImage_ = nodehandle_.advertise<sensor_msgs::Image>("/synchronized/image",100);
    pubRadar_ = nodehandle_.advertise<sensor_msgs::PointCloud2>("/synchronized/radar_data",100);
    ROS_INFO("radar and camera data synchronizing started");
    sync.registerCallback(boost::bind(&DataSync::callback, this,_1, _2));
}
~DataSync()
{
  processthread_->join();
}

void callback(const sensor_msgs::ImageConstPtr& image_msg,const frontal_delphi_radar::RadarDataConstPtr& radar_msg)
{
  ROS_INFO("Sync_Callback");
  pubImage_.publish(*image_msg);
  pubRadar_.publish(*radar_msg);
}



private:
  ros::NodeHandle& nodehandle_;
  //消息过滤器订阅相机和激光雷达点云话题
  typedef message_filters::Subscriber<sensor_msgs::Image> subCameraImage;
  subCameraImage subCameraImage_;//订阅图像消息
  typedef message_filters::Subscriber<frontal_delphi_radar::RadarData> subRadarData;
  subRadarData subRadarData_;//订阅毫米波雷达消息
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,frontal_delphi_radar::RadarData> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync;

  //发布同步后的数据
  ros::Publisher pubImage_;
  ros::Publisher pubRadar_;


  bool processthreadfinished_;
  boost::thread* processthread_;

};


int main(int argc,char** argv)
{
  ros::init(argc, argv, "cam_radar_sync");
  ros::NodeHandle nh;

  DataSync data_sync(nh);
  ros::spin();
  return 0;
}
