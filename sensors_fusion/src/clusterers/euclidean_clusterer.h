/*
 * euclidean_clusterer.h
 *
 *  Created on: 2018年3月28日
 *      Author: zhanghm
 */

#ifndef SRC_SENSORS_FUSION_SRC_CLUSTERERS_EUCLIDEAN_CLUSTERER_H_
#define SRC_SENSORS_FUSION_SRC_CLUSTERERS_EUCLIDEAN_CLUSTERER_H_
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_map>
//Project headers
#include "clusterers/abstract_clusterer.h"
namespace depth_clustering {

/**
 * @brief      Class for euclidean clustering.
 */
class EuclideanClusterer : public AbstractClusterer {
 public:
//  using PointT = pcl::PointXYZL;
  explicit EuclideanClusterer(double cluster_tollerance = 0.2,
                              uint16_t min_cluster_size = 100,
                              uint16_t max_cluster_size = 25000,
                              uint16_t skip = 10)
      : AbstractClusterer(cluster_tollerance, min_cluster_size,
                          max_cluster_size, skip) {}
  virtual ~EuclideanClusterer() {}

  /**
   * @brief      Gets called when somebody sends this client an object.
   *
   * @param[in]  cloud      The cloud to cluster
   * @param[in]  sender_id  The sender identifier
   */
  void OnNewObjectReceived(const PointT& cloud) override {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr = new pcl::PointCloud<pcl::PointXYZ>(cloud);
    std::unordered_map<uint16_t, PointT> clusters;
    if (this->_counter++ % this->_skip != 0) {
      // share empty clusters
      return;
    }
    // 创建用于提取搜索方法的kdtree树对象
    typename pcl::search::KdTree<PointT>::Ptr tree(
        new pcl::search::KdTree<PointT>);
    tree->setInputCloud(pcl_cloud_ptr);
    std::vector<pcl::PointIndices> cluster_indices;//被分割出来的点云团（标号队列）
    pcl::EuclideanClusterExtraction<PointT> clusterer; //欧式聚类对象
    clusterer.setClusterTolerance(this->_cluster_tollerance);// 设置近邻搜索的搜索半径
    clusterer.setMinClusterSize(this->_min_cluster_size);//设置一个聚类需要的最少的点数目
    clusterer.setMaxClusterSize(this->_max_cluster_size);//设置一个聚类需要的最大点数目
    //搜索策略树
    clusterer.setSearchMethod(tree);//设置点云的搜索机制
    clusterer.setInputCloud(pcl_cloud_ptr);
    //cluster_indices[0] contains all indices of the first cluster in our point cloud
    clusterer.extract(cluster_indices);//从点云中提取聚类，并将点云索引保存在cluster_indices中
    //迭代访问点云索引cluster_indices,直到分割出所有聚类
    for (auto cluster_iter = cluster_indices.begin();cluster_iter != cluster_indices.end(); ++cluster_iter)
    {
      int idx = std::distance(cluster_indices.begin(), cluster_iter);
      //迭代容器中的点云的索引，并且分开保存索引的点云
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      for (auto point_iter = cluster_iter->indices.begin();point_iter != cluster_iter->indices.end(); ++point_iter)
      {
        //设置保存点云的属性问题
        cloud_cluster->points.push_back (pcl_cloud_ptr->points[*cluster_iter]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
      }
      clusters[idx] = cloud_cluster;
    }
  }
};

}  // namespace depth_clustering



#endif /* SRC_SENSORS_FUSION_SRC_CLUSTERERS_EUCLIDEAN_CLUSTERER_H_ */
