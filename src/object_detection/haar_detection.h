//======================================================================
// Author   : Haiming Zhang
// Email    : zhanghm_1995@qq.com
// Version  : V2018-03-27
// Copyright    :
// Descriptoin  : 基于Haar特征的AdaBoost分类器用于目标检测
//                输入: ROI
//                输入：Bounding box，只针对一张图片输出，多张图片，需要循环调用相应函数
// References   :
//======================================================================
#ifndef SRC_SENSORS_FUSION_SRC_OBJECT_DETECTION_HAAR_DETECTION_H_
#define SRC_SENSORS_FUSION_SRC_OBJECT_DETECTION_HAAR_DETECTION_H_
//OpenCV
#include <opencv2/opencv.hpp>
//Project headers
#include "utils/bounding_box.h"
class HaarDetection {
public:
  HaarDetection();
  virtual ~HaarDetection();
  //main function, implement detection
  sensors_fusion::BoundingBox ProcessDetection(IplImage* src);
  void LoadDetector(const char* cascade_file_path);
private:
  //called by ProcessDetection() function, to do haar detection
  bool DetectObject(IplImage* image,int do_pyramid = 0);
private:
  CvHaarClassifierCascade* cascade_xml_file_; //the trained classifier files
  sensors_fusion::BoundingBox object_result_; //the final detection result
};

#endif /* SRC_SENSORS_FUSION_SRC_OBJECT_DETECTION_HAAR_DETECTION_H_ */
