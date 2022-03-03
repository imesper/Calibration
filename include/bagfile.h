#ifndef BAGFILE_H
#define BAGFILE_H

#include "helper.h"

#include <QString>
#include <vector>

class BagFile
{

public:
  explicit BagFile();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectPointCloud(
    ToolMat tool,
    int frame = 10,
    double threshold = 1.7,
    bool filter = true,
    bool genMarkerPointPC = false);
  QString getSerial() const;
  cv::Mat getCVColorFrame();

  int getCameraPos() const;
  void setCameraPos(int cameraPos);

  void init(QString filename, QString matrix = "");

  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud() const;

  bool isFiltered() const;

  bool hasMarkerPC() const;

  const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& markerPointcloud() const;

  const Eigen::Affine3d &charucoTransformation() const;

private:
  QString _serial;
  QString _filename;
  QString _matrix;
  Eigen::Affine3d m_charucoTransformation;
  int _cameraPos;
  bool m_isFiltered;
  bool m_hasMarkerPC;
  std::vector<rs2::depth_frame> _depths;
  std::vector<rs2::video_frame> _colors;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
  //  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _labeledPointcloud;
  std::map<int, std::vector<cv::Point2f>> m_markerCorners;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr m_markerPointcloud;
  rs2_intrinsics _intrinsics;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointsPRef;
};

#endif // BAGFILE_H
