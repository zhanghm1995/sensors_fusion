/*
 * haar_detection.cpp
 *
 *  Created on: 2018年3月27日
 *      Author: zhanghm
 */

#include "haar_detection.h"

HaarDetection::HaarDetection() {
  // TODO Auto-generated constructor stub

}

HaarDetection::~HaarDetection() {
  // TODO Auto-generated destructor stub
}

void HaarDetection::LoadDetector(const char* cascade_file_path){
  cascade_xml_file_ = (CvHaarClassifierCascade*)cvLoad(cascade_file_path);
}

sensors_fusion::BoundingBox HaarDetection::ProcessDetection(IplImage* src){
  //generate gray image
  IplImage* gray_img = cvCreateImage(cvGetSize(src),src->depth,1);
  if(src->nChannels > 1)
  {
    cvCvtColor(src,gray_img,CV_BGR2GRAY);
  }
  else
  {
    cvCopy(src,gray_img);
  }

  //image pyramid
  int pyramid_scale = 0;
  int area = gray_img->height*gray_img->width;
  if(area > 40000){
    pyramid_scale = 4;
  }
  else if(area > 22500){
    pyramid_scale = 3;
  }
  else if(area > 10000){
    pyramid_scale = 2;
  }

  if(DetectObject(gray_img,pyramid_scale))
  {
    return object_result_;
  }
  else
  {
    sensors_fusion::BoundingBox empty;
    return empty;
  }


}

bool HaarDetection::DetectObject(IplImage* image,int do_pyramid){
  IplImage* small_image = image;
  CvMemStorage* storage = cvCreateMemStorage(0);
  CvSeq* car_box; //检测到的车辆包围盒
  int scale;
  if(do_pyramid)
  {
    small_image = cvCreateImage(cvSize(image->width/do_pyramid,image->height/do_pyramid),IPL_DEPTH_8U,1);
    cvResize(image,small_image);
    scale = do_pyramid;
  }
  //important! finish Haar detection
  car_box = cvHaarDetectObjects(small_image,cascade_xml_file_,storage,120.0/100.0,
      5,CV_HAAR_DO_CANNY_PRUNING+CV_HAAR_FIND_BIGGEST_OBJECT,cvSize(small_image->width/3,small_image->height/3),
      cvSize(small_image->width*2/3,small_image->height*2/3));
  //find the detection result
  bool car_flag = false;
  for(int i = 0;i < car_box->total; ++i)
  {
    CvAvgComp car_rect = *(CvAvgComp*)cvGetSeqElem(car_box,i);
    sensors_fusion::PixelCoord top_left(car_rect.rect.x*scale,car_rect.rect.y*scale);
    sensors_fusion::PixelCoord bottom_right((car_rect.rect.x+car_rect.rect.width)*scale,(car_rect.rect.y+car_rect.rect.height)*scale);
    sensors_fusion::BoundingBox object_result_(top_left,bottom_right);

    if((object_result_.width()*object_result_.height())>(image->width/2)*(image->height/1.5)*1.2) //not OK
    {
      continue;
    }
    car_flag = true;
    break;
  }
  if(small_image != image)
    cvReleaseImage(&small_image);
  cvReleaseMemStorage(&storage);
  return car_flag;
}
