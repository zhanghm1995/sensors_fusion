/*
 * abstract_clusterer.h
 *
 *  Created on: 2018年3月28日
 *      Author: zhanghm
 */

#ifndef SRC_SENSORS_FUSION_SRC_CLUSTERERS_ABSTRACT_CLUSTERER_H_
#define SRC_SENSORS_FUSION_SRC_CLUSTERERS_ABSTRACT_CLUSTERER_H_
#include <unordered_map>
#include <vector>
//PCL
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/time.h>

#include "utils/cloud.h"

namespace depth_clustering {

/**
 * @brief      Class for abstract clusterer.
 */
class AbstractClusterer{
 public:
  using PointT = pcl::PointXYZ;
  /**
   * @brief      Construct a clusterer.
   *
   * @param[in]  cluster_tollerance  The cluster tollerance
   * @param[in]  min_cluster_size    The minimum cluster size
   * @param[in]  max_cluster_size    The maximum cluster size
   * @param[in]  skip                Only cluster every skip cloud
   */
  explicit AbstractClusterer(double cluster_tollerance = 0.2,
                             uint16_t min_cluster_size = 100,
                             uint16_t max_cluster_size = 25000,
                             uint16_t skip = 10)
      :  _cluster_tollerance(cluster_tollerance),
        _min_cluster_size(min_cluster_size),
        _max_cluster_size(max_cluster_size),
        _skip(skip),
        _counter(0) {}
  virtual ~AbstractClusterer() {}

  virtual void OnNewObjectReceived(const PointT& object) = 0;

 protected:
  double _cluster_tollerance;
  uint16_t _min_cluster_size;
  uint16_t _max_cluster_size;
  uint16_t _skip;
  uint32_t _counter;
};

}  // namespace depth_clustering


#endif /* SRC_SENSORS_FUSION_SRC_CLUSTERERS_ABSTRACT_CLUSTERER_H_ */
