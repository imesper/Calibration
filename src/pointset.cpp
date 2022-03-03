#include "../include/pointset.h"

PointSet::PointSet()
  : rmse(0.0)
  , mea(0.0)
  , smea(0.0)
  , c_rmse(0.0)
  , c_mea(0.0)
  , c_smea(0.0)
  //  , tool(-0.05, 0.073, 0.0396, 0.2300, 0, 0)
//  , tool(-0.04689, 0.07864, -0.02807, 90.8897104, 0.8983838, 0.7490231)
  , tool(-0.050, -0.04418107, 0.078, 90.917997, 1.17740464, 0.485910)
  , rps(QVector<RobPoint>(6))
  , bfs(QVector<BagFile>(6))
  , m_pointsPRef(QVector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr>(3))
{
  for (auto& refCloud : m_pointsPRef) {
    refCloud.reset(new pcl::PointCloud<pcl::PointXYZRGBL>());
  }
}

void
PointSet::generateTranformations()
{
  for (auto robPoint : rps) {
    transformations.append(Helper::RobTarget2Affine3d(robPoint, tool));
  }
}
