//======================================================================
// Author   : Haiming Zhang
// Email    : zhanghm_1995@qq.com
// Version  :
// Copyright    :
// Descriptoin  : 基于HOG特征的SVM分类器
// References   :
//======================================================================

#ifndef SRC_SENSORS_FUSION_SRC_OBJECT_DETECTION_HOG_SVM_H_
#define SRC_SENSORS_FUSION_SRC_OBJECT_DETECTION_HOG_SVM_H_
//C++
#include <vector>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#define HOG_SIZE_WIDTH 32
#define HOG_SIZE_HEIGHT 32
class HOG_SVM {
public:
  HOG_SVM();
  virtual ~HOG_SVM();
  //specify the svm classifier file
  void Init(char* filename);
  int SVMPredict(IplImage* src);
private:
  IplImage* train_img_;
  cv::HOGDescriptor* hog_;
  CvSVM svm_;
  CvFont cf_;
  std::vector<float> descriptors_; //结果数组
  CvMat* SVM_train_Mat_;
  char* text_;
  IplImage* copysrc_;
  IplImage* train_gray_;
};

#endif /* SRC_SENSORS_FUSION_SRC_OBJECT_DETECTION_HOG_SVM_H_ */
