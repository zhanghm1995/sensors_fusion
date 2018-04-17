//
// Created by zhanghm on 17-12-27.
//
#include "./utils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "./velodyne_utils.h"

using depth_clustering::Cloud;
using depth_clustering::ProjectionParams;
using depth_clustering::MatFromDepthPng;
using depth_clustering::ReadKittiCloudTxt;
using depth_clustering::ReadKittiCloud;

Cloud::Ptr CloudFromFile(const std::string &file_name,const std::string& file_type,
                         const ProjectionParams &proj_params) {
    Cloud::Ptr cloud = nullptr;
    if (file_type == ".pcd") {
        pcl::PointCloud<pcl::PointXYZL> pcl_cloud;
        pcl::io::loadPCDFile(file_name, pcl_cloud);
        cloud = Cloud::FromPcl<pcl::PointXYZL>(pcl_cloud);
        cloud->InitProjection(proj_params);
    } else if ((file_type ==".png") || (file_type ==".exr")) {
        cloud = Cloud::FromImage(MatFromDepthPng(file_name), proj_params);
    } else if (file_type ==".txt") {
        cloud = ReadKittiCloudTxt(file_name);
        cloud->InitProjection(proj_params);
    } else if (file_type ==".bin") {
        cloud = ReadKittiCloud(file_name);
        cloud->InitProjection(proj_params);
    }
    // if (cloud) {
    //   // if the cloud was actually set, then set the name in the gui
    //   ui->lbl_cloud_name->setText(name);
    // }
    return cloud;
}
