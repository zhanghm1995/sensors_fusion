/*
 * projection_utils.cpp
 *
 *  Created on: 2018年4月17日
 *      Author: zhanghm
 */

#include "projection_utils.h"
//C++
#include <iostream>
namespace data_fusion{

bool CloudProject2Image(cv::Mat& img_src,pcl::PointCloud<pcl::PointXYZI>::Ptr veloCloudPtr,Eigen::MatrixXd Rv,
				   Eigen::MatrixXd Tv,
				   Eigen::MatrixXd intrinsicMat)
{
	if(!img_src,data||!veloCloudPtr)
	{
		printf("[ERROR] No valid rgbImage or valid point cloud data!");
		return false;
	}
	printf("[INFO] start project point cloud and draw points in image...\n");

	for(size_t i=0;i<veloCloudPtr->points.size();++i)
	{
		Eigen::Vector3d point_3d;
		Eigen::Vector3d point_t3d;
		Eigen::Vector3d imgPoint2D;
		point_3d << veloCloudPtr->at(i).x,
				veloCloudPtr->at(i).y,
				veloCloudPtr->at(i).z;
		double ix = point_3d[0], iy = point_3d[1], iz = point_3d[2];
		//right->x, forward->y, up->z
		/*Important!! eliminate the point behind camera
		otherwise, it would project redundant points*/
		if(iy<0)
			continue;

		point_t3d = Rv*point_3d+Tv;//rigid body transformation
		imgPoint2D = intrinsicMat*point_t3d; //perspective transformation

		if( imgPoint2D.[2] == 0 )
		{
			fprintf(stderr,"the calculated 2D image points are wrong!\n");
			exit(0);
		}
		//obtain the projection point in image
		double x = (imgPoint2D[0]/imgPoint2D[2]+2+15.0802)/0.6830;
		double y = (imgPoint2D[1]/imgPoint2D[2]+2+11.3102)/0.6830;

		if(x<0||x>img_src.cols||y<0||y>img_src.rows)
		{
			continue;
		}
		cv::circle(img_src,cv::Point((int)x,(int)y),1,cv::Scalar(0,0,255)); //red points by default


	}
	return true;
}


const cv::Point PointProject2Image(const Eigen::Vector3d& point3d,Eigen::MatrixXd Rv,
		   Eigen::MatrixXd Tv,
		   Eigen::MatrixXd intrinsicMat,
		   int xmin,int xmax,int ymin,int ymax)
{
	cv::Point res(0,0);
	if(point3d[1]<0)
	{
		printf("[INFO] 3D point is in the back of camera\n");
		return res;
	}

	Eigen::Vector3d point_t3d;
	Eigen::Vector3d imgPoint2D;
	point_t3d = Rv*point_3d+Tv;//rigid body transformation
	imgPoint2D = intrinsicMat*point_t3d; //perspective transformation

	if( imgPoint2D.[2] == 0 )
	{
		fprintf(stderr,"the calculated 2D image points are wrong!\n");
		exit(0);
	}
	//obtain the projection point in image
	double x = (imgPoint2D[0]/imgPoint2D[2]+2+15.0802)/0.6830;
	double y = (imgPoint2D[1]/imgPoint2D[2]+2+11.3102)/0.6830;

	if(x<xmin||x>xmax||y<ymin||y>ymax)
	{
		printf("[INFO] 3D point not in the limits of camera view\n");
		return res;
	}

	res.x = x;
	res.y = y;
	return res;

}


}
