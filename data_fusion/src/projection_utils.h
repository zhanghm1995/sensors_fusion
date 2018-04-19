/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 传感器数据投影关系功能函数
* 				 1、激光雷达向相机画面投影
* 				 2、毫米波雷达向相机画面投影
* 				 3、毫米波雷达向激光雷达投影
* References   :
======================================================================*/

#ifndef SRC_SENSORS_FUSION_SRC_DATA_FUSION_PROJECTION_UTILS_H_
#define SRC_SENSORS_FUSION_SRC_DATA_FUSION_PROJECTION_UTILS_H_
// C++

//OpenCV
#include <opencv2/opencv.hpp>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//Eigen
#include <Eigen/Dense>

namespace data_fusion{
//! @brief given rgb image, and point cloud, projecting all the points to the image and draw the circle on image
//! 	   by given matrix Rv、Tv、and camera intrinsic parameters matrix
bool CloudProject2Image(cv::Mat& img_src,pcl::PointCloud<pcl::PointXYZI>::Ptr veloCloudPtr,Eigen::MatrixXd Rv,
				   Eigen::MatrixXd Tv,
				   Eigen::MatrixXd intrinsicMat);

//! @brief given single point, project all the points to the image and return the projection points image coordnate
//! 	   by given matrix Rv、Tv、and camera intrinsic parameters matrix
const cv::Point PointProject2Image(const Eigen::Vector3d& point3d,Eigen::MatrixXd Rv,
		   Eigen::MatrixXd Tv,
		   Eigen::MatrixXd intrinsicMat,
		   int xmin,int xmax,int ymin,int ymax);
}



#endif /* SRC_SENSORS_FUSION_SRC_DATA_FUSION_PROJECTION_UTILS_H_ */
