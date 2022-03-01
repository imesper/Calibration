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

class RobPoint
{
public:
  RobPoint(float x, float y, float z, float q1, float q2, float q3, float q4)
    : x(x)
    , y(y)
    , z(z)
    , q1(q1)
    , q2(q2)
    , q3(q3)
    , q4(q4)
  {}
  float x;
  float y;
  float z;
  float q1;
  float q2;
  float q3;
  float q4;
};

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
