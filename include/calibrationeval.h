#ifndef CALIBRATIONEVAL_H
#define CALIBRATIONEVAL_H

#include "bagfile.h"
#include "helper.h"
#include "pointset.h"
#include "robpoint.h"

#include <memory>

class CalibrationEval
{

public:
  explicit CalibrationEval();

  void extractPosInfo(QString filename, RobPoint& point);
  void CalcError();

  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& transformedCloud() const;

  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& transformedCharucoCloud() const;

  double getRmse() const;

  double getMea() const;

  double getSmea() const;

  double getC_rmse() const;

  double getC_mea() const;

  double getC_smea() const;

  void init(QString directory);

private:
  QStringList m_bagFiles;
  QStringList m_posFiles;

  QVector<PointSet> m_pointsets;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _transformedCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _transformedCharucoCloud;
  double rmse;
  double mea;
  double smea;
  double c_rmse;
  double c_mea;
  double c_smea;
};

#endif // CALIBRATIONEVAL_H
