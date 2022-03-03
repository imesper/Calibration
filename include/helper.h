#ifndef HELPER_H
#define HELPER_H


#include <QString>
#include <iostream>
#include <string>
#include <omp.h>
#include <map>
#include <vector>

#include <librealsense2/rs.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/common/pca.h>
#include <pcl/console/parse.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "robpoint.h"

class ToolMat
{
public:
  ToolMat(float x, float y, float z, float rx, float ry, float rz)
    : x(x)
    , y(y)
    , z(z)
    , rx(rx)
    , ry(ry)
    , rz(rz)

  {}
  float x;
  float y;
  float z;
  float rx;
  float ry;
  float rz;
  void setX(float value);
  void setRz(float value);
  void setRy(float value);
  void setRx(float value);
  void setZ(float value);
  void setY(float value);
};

class Helper
{
public:
  Helper();
  static Eigen::Affine3d RobTarget2Affine3d(RobPoint target, ToolMat tool);

  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr RS2toPCL(cv::Mat depth,
    cv::Mat& color,
    rs2_intrinsics intrinsic,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr points_pc,
    std::map<int, std::vector<cv::Point2f>>& m_markerCorners,
    int p_color = 3,
    double *m_depth = 0);

  static void detectCharucoBoardWithCalibrationPose(
    cv::Mat image,
    cv::Mat cameraMatrix,
    Eigen::Affine3d& transformation,
    std::map<int, std::vector<cv::Point2f>>& m_markerCorners);

  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr loadBag(
    std::string filename,
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointsPRef,
    Eigen::Affine3d& transformation,
    bool generateCube);
  static Eigen::Affine3d RobTarget2Affine3d(RobPoint target);
};

#endif // HELPER_H
