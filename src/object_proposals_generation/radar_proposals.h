//======================================================================
// Author   : Haiming Zhang
// Email    : zhanghm_1995@qq.com
// Version  :
// Copyright    :
// Descriptoin  : 利用毫米波雷达形成候选检测目标
// References   :
//======================================================================

#ifndef SRC_SENSORS_FUSION_SRC_OBJECT_PROPOSALS_GENERATION_RADAR_PROPOSALS_H_
#define SRC_SENSORS_FUSION_SRC_OBJECT_PROPOSALS_GENERATION_RADAR_PROPOSALS_H_
#include <iostream>
#include <string>
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
//project headers
#include "utils/TypeDef.h"
#include "utils/bounding_box.h"

#define PLANE_WIDTH 401
#define PLANE_HEIGHT 501
#define METER2PIXEL 5.0
#define RADAR2CAR 3.50 //毫米波雷达到后轴的距离
#define OBJECT_WIDTH 1.8 //默认车宽1.8
class RadarProposals {
public:
  RadarProposals(const cv::Mat& img,const delphi_radar_target& radar_points,const std::string& xml_file_path);
  virtual ~RadarProposals();
  std::vector<sensors_fusion::BoundingBox> GenerateProposals();
private:
  //project one radar point to image to get corresponding image coord, x, y Unit: m
  cv::Point ProjectPoint2Image(float x, float y);

  void sg_pt_warp(float* src_pt, CvMat* pers, float* dst_pt);//车体坐标系转图像坐标系
private:
  cv::Mat img_src_;//src image
  delphi_radar_target radar_points_; //all radar points
  //TODO: 存储xml变换矩阵用的是opencv1的类型,应该统一用opencv2的
  CvMat* swarp_plane2image_;//将毫米波雷达点转换成为相机画面点的透视变换矩阵



};

#endif /* SRC_SENSORS_FUSION_SRC_OBJECT_PROPOSALS_GENERATION_RADAR_PROPOSALS_H_ */
