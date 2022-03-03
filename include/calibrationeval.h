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
  explicit CalibrationEval(QString directory);

  void extractPosInfo(QString filename, RobPoint& point);
  void CalcError();

private:
  QStringList m_bagFiles;
  QStringList m_posFiles;

  QVector<PointSet> m_pointsets;

  double rmse;
  double mea;
  double smea;
  double c_rmse;
  double c_mea;
  double c_smea;
};

#endif // CALIBRATIONEVAL_H
