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


#include "velodyne/HDL32Structure.h"
#include <glog/logging.h>
#include "common/blocking_queue.h"
#include "common/make_unique.h"

class DataSync
{
public:
  DataSync(ros::NodeHandle& nodehandle):nodehandle_(nodehandle)
  ,processthread_(NULL)
  ,processthreadfinished_ (false),
  subCameraImage_(nodehandle_,"camera_image",10),
  subLidarData_(nodehandle_,"lidar_cloud",10),
  sync(MySyncPolicy(100), subCameraImage_, subLidarData_)
{
    sync.registerCallback(boost::bind(&DataSync::callback, this,_1, _2));
    init();
}
~DataSync()
{
  processthread_->join();
}
void init()
{

}

void callback(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
{

}



private:
  ros::NodeHandle& nodehandle_;
  //消息过滤器订阅相机和激光雷达点云话题
  typedef message_filters::Subscriber<sensor_msgs::Image> subCameraImage;
  subCameraImage subCameraImage_;//订阅图像消息
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> subLidarData;
  subLidarData subLidarData_;//订阅激光雷达消息
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer< MySyncPolicy > sync;


  bool processthreadfinished_;
  boost::thread* processthread_;
  common::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> lidarCloudMsgs_;

};


int main(int argc,char** argv)
{
  ros::init(argc, argv, "data_sync");
  ros::NodeHandle nh;

  DataSync data_sync(nh);
  ros::spin();
  return 0;
}
