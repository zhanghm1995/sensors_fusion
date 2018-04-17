/*
 * hog_svm.cpp
 *
 *  Created on: 2018年3月27日
 *      Author: zhanghm
 */

#include "hog_svm.h"
using std::vector;
HOG_SVM::HOG_SVM() {
  // TODO Auto-generated constructor stub
  train_img_ = cvCreateImage(cvSize(HOG_SIZE_WIDTH,HOG_SIZE_HEIGHT),8,3);
  train_gray_ = cvCreateImage(cvSize(HOG_SIZE_WIDTH,HOG_SIZE_HEIGHT),8,1);
  cvZero(train_img_);
  cvZero(train_gray_);
  hog_ = new cv::HOGDescriptor(cvSize(HOG_SIZE_WIDTH,HOG_SIZE_HEIGHT),
      cvSize(HOG_SIZE_WIDTH/2,HOG_SIZE_HEIGHT/2),
      cvSize(HOG_SIZE_WIDTH/4,HOG_SIZE_HEIGHT/4),
      cvSize(HOG_SIZE_WIDTH/4,HOG_SIZE_HEIGHT/4),
      9);
  cvInitFont(&cf_,CV_FONT_HERSHEY_COMPLEX_SMALL,0.8,0.8);
  SVM_train_Mat_ = cvCreateMat(1,324,CV_32FC1);
  text_ = new char[260];
}

HOG_SVM::~HOG_SVM() {
  // TODO Auto-generated destructor stub
}

void HOG_SVM::Init(char* filename) {
  //load classifier file
  svm_.load(filename);

}

int HOG_SVM::SVMPredict(IplImage* src){
  if(src->nChannels > 1)
  {
    cvResize(src,train_img_);
    //计算HOG特征
    hog_->compute(train_img_, descriptors_, cv::Size(1,1), cv::Size(0,0));
  }
  else
  {
    cvResize(src,train_gray_);
    //计算HOG特征
    hog_->compute(train_gray_, descriptors_, cv::Size(1,1), cv::Size(0,0));
  }
  int n = 0;
  //写入特征到SVM要求的矩阵格式
  for(std::vector<float>::iterator iter = descriptors_.begin();iter!=descriptors_.end();++iter)
  {
    cvmSet(SVM_train_Mat_,0,n,*iter);
    ++n;
  }
  //使用SVM分类器判断分类结果
  int result = svm_.predict(SVM_train_Mat_);
  return result;
}
