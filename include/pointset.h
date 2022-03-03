#ifndef POINTSET_H
#define POINTSET_H

#include "bagfile.h"
#include "robpoint.h"

#include <QString>
#include <QVector>
class PointSet
{
public:
  PointSet();

  void generateTranformations();

  QVector<RobPoint> rps;
  QVector<BagFile> bfs;
  QVector<Eigen::Affine3d> transformations;

  QVector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr>
    _transformedLabeledPointcloud;
  QVector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> m_pointsPRef;
  ToolMat tool;

  double rmse;
  double mea;
  double smea;
  double c_rmse;
  double c_mea;
  double c_smea;
};

#endif // POINTSET_H
