/*
 * radar_proposals.cpp
 *
 *  Created on: 2018年3月26日
 *      Author: zhanghm
 */

#include "radar_proposals.h"

RadarProposals::RadarProposals(const cv::Mat& img,const delphi_radar_target& radar_points,
    const std::string& xml_file_path)
{
  img_src_ = img.clone();
  radar_points_ = radar_points;
  swarp_plane2image_ = cvCreateMat(3,3,CV_32FC1);
  swarp_plane2image_ = (CvMat*)cvLoad(xml_file_path.c_str());


}

RadarProposals::~RadarProposals() {
  // TODO Auto-generated destructor stub
}
std::vector<sensors_fusion::BoundingBox> RadarProposals::GenerateProposals()
{
  std::vector<sensors_fusion::BoundingBox> res;
  for(int i = 0; i < 64; ++i) //遍历雷达点
  {
    float radar_pt_x = radar_points_.delphi_detection_array[i].x;
    float radar_pt_y = radar_points_.delphi_detection_array[i].y;
    //1) 将雷达点投影到图像上
    cv::Point radar_proj = ProjectPoint2Image(radar_pt_x,radar_pt_y);
    if(radar_proj.x!=0|radar_proj.y!=0)
    {
    //2) 求出实际车宽在图像上像素车宽
      float left_pt_x = radar_pt_x - OBJECT_WIDTH/2;
      float left_pt_y = radar_pt_y;
      float right_pt_x = radar_pt_x + OBJECT_WIDTH/2;
      float rigth_pt_y = radar_pt_y;
      cv::Point left_pt_img = ProjectPoint2Image(left_pt_x, left_pt_y);
      cv::Point right_pt_img = ProjectPoint2Image(right_pt_x, rigth_pt_y);
      int pixel_distance = right_pt_img.x - left_pt_img.x;

      //3)确定图像中ROI
      sensors_fusion::PixelCoord top_left(left_pt_img.x - pixel_distance/2,radar_proj.y - pixel_distance);
      sensors_fusion::PixelCoord bottom_right(right_pt_img.x + pixel_distance/2,radar_proj.y + pixel_distance);
      sensors_fusion::BoundingBox temp(top_left,bottom_right);
      //4)越界判断
      if(temp.beg_x()<0)
      {
        temp.beg_x() = 0;
      }
      if(temp.end_x()>img_src_.cols)
      {
        temp.end_x() = img_src_.cols - 1;
      }
      if(temp.beg_y() < 0)
      {
        temp.beg_y() = 0;
      }
      if(temp.end_y()>img_src_.rows)
      {
        temp.end_y() = img_src_.rows - 1;
      }
      if(temp.width()>30&&temp.height()>30&&((float)(temp.height()/temp.width())<2.0))
      {
        res.push_back(temp);
      }
    }//end radar_proj.x!=0|radar_proj.y!=0
  }//end for(int i = 0; i < 64; ++i)
  return res;

}

cv::Point RadarProposals::ProjectPoint2Image(float x, float y)
{
  cv::Point res(0,0);
  float* Delphi_input = new float[2];//雷达点在鸟瞰图上的位置
  float* image_output = new float[2]; //在图像上的对应位置
  Delphi_input[0] = (float)((PLANE_WIDTH-1)/2 + x*METER2PIXEL);
  Delphi_input[1] = (float)(PLANE_HEIGHT - (y + RADAR2CAR)*METER2PIXEL);
  sg_pt_warp(Delphi_input,swarp_plane2image_,image_output);
  //check valid or not
  if(image_output[0]>0&&image_output[0]<img_src_.cols&&image_output[1]>0&&image_output[1]<img_src_.rows)//valid point
  {
    res.x = image_output[0];
    res.y = image_output[1];
  }
  return res;
}

void RadarProposals::sg_pt_warp(float* src_pt, CvMat* pers, float* dst_pt)
{
  float v1,v2,v3;
  v1 = cvmGet(pers,0,0)*src_pt[0] + cvmGet(pers,0,1)*src_pt[1] + cvmGet(pers,0,2);
  v2 = cvmGet(pers,1,0)*src_pt[0] + cvmGet(pers,1,1)*src_pt[1] + cvmGet(pers,1,2);
  v3 = cvmGet(pers,2,0)*src_pt[0] + cvmGet(pers,2,1)*src_pt[1] + cvmGet(pers,2,2);

  dst_pt[0] = v1/v3;
  dst_pt[1] = v2/v3;
}

