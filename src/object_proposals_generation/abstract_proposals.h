//======================================================================
// Author   : Haiming Zhang
// Email    : zhanghm_1995@qq.com
// Version  :
// Copyright    :
// Descriptoin  : 抽象类，用于生成目标检测用的候选物体，目前是生成是图像上bounding boxes候选物体
//                输入：
//                输出：
// References   :
//======================================================================
#ifndef SRC_SENSORS_FUSION_SRC_ABSTRACT_PROPOSALS_H_
#define SRC_SENSORS_FUSION_SRC_ABSTRACT_PROPOSALS_H_
#include <vector>
#include "utils/bounding_box.h"

class AbstractProposals
{
public:
  AbstractProposals(){}
  virtual ~AbstractProposals(){}
  //不管用什么方法产生候选物体，输出都为这些物体在图像上的二维包围盒
  virtual std::vector<sensors_fusion::BoundingBox> GenerateProposals() = 0;

};
#endif  // SRC_SENSORS_FUSION_SRC_ABSTRACT_PROPOSALS_H_
