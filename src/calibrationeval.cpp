#include "../include/calibrationeval.h"

#include <QDebug>
#include <QDir>
#include <QFile>

CalibrationEval::CalibrationEval()
  : rmse(0.0)
  , mea(0.0)
  , smea(0.0)
  , c_rmse(0.0)
  , c_mea(0.0)
  , c_smea(0.0)
  , _transformedCharucoCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
  , _transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
{}

void
CalibrationEval::init(QString directory)
{
  QDir dir(directory);

  m_bagFiles = dir.entryList(QStringList() << "*.bag"
                                           << "*.BAG",
                             QDir::Files);
  m_posFiles = dir.entryList(QStringList() << "*.txt"
                                           << "*.TXT",
                             QDir::Files);

  qDebug() << m_posFiles.size() << " Sets: " << m_posFiles.size() / 6;
  qDebug() << m_bagFiles.size() << " Sets: " << m_bagFiles.size() / 6;
  m_pointsets.resize(m_bagFiles.size() / 6);

  for (int i = 0; i < m_posFiles.size(); i++) {
    QString bgFilename = directory + "/" + m_bagFiles[i];
    QString filename = directory + "/" + m_posFiles[i];

    qDebug() << bgFilename;
    qDebug() << filename;
    int index =
      bgFilename
        .mid(bgFilename.lastIndexOf('_') + 1,
             bgFilename.indexOf('.') - bgFilename.lastIndexOf('_') - 1)
        .toInt() -
      1;

    int cam = bgFilename
                .mid(bgFilename.indexOf('_') + 1,
                     bgFilename.lastIndexOf('_') - bgFilename.indexOf('_') - 1)
                .toInt();

    m_pointsets[index].bfs[cam].init(bgFilename);

    extractPosInfo(filename, m_pointsets[index].rps[cam]);
    m_pointsets[index].bfs[cam].projectPointCloud(
      m_pointsets[index].tool, 4, 1.2, false);
  }
}

void
CalibrationEval::extractPosInfo(QString filename, RobPoint& point)
{
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    return;

  while (!file.atEnd()) {
    QByteArray line = file.readLine();
    auto data = line.mid(2).split(',');
    int index = 0;

    for (auto dt : data) {
      if (dt.contains('['))
        dt = dt.mid(1);
      else if (dt.contains(']'))
        dt = dt.left(dt.size() - 1);

      //        qDebug() << dt;
      switch (index) {
        case 0:
          point.x = dt.toDouble();
          break;
        case 1:
          point.y = dt.toDouble();
          break;
        case 2:
          point.z = dt.toDouble();
          break;
        case 3:
          point.q1 = dt.toDouble();
          break;
        case 4:
          point.q2 = dt.toDouble();
          break;
        case 5:
          point.q3 = dt.toDouble();
          break;
        case 6:
          point.q4 = dt.toDouble();
          break;
        default:
          break;
      }
      //        m_poss.append(point);
      if (index == 6)
        break;
      index++;
    }
  }
}

void
CalibrationEval::CalcError()
{
  int meas_count = 0;
  int c_meas_count = 0;

  for (int j = 0; j < m_pointsets.size(); j++) {
    auto pset = m_pointsets[j];
    pset.generateTranformations();
    pcl::transformPointCloud(*(pset.bfs[0].markerPointcloud()),
                             *(pset.m_pointsPRef[0]),
                             pset.transformations[0]);

    Eigen::Affine3d A =
      pset.transformations[0] * pset.bfs[0].charucoTransformation() *
      Eigen::Translation3d(Eigen::Vector3d(0.280, 0.187, -0.19)) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
      pset.bfs[0].charucoTransformation().inverse();

    pcl::transformPointCloud(
      *(pset.bfs[0].markerPointcloud()), *(pset.m_pointsPRef[1]), A);

    Eigen::Affine3d B =
      pset.transformations[0] * pset.bfs[0].charucoTransformation() *
      Eigen::Translation3d(Eigen::Vector3d(0.280, 0.0, -0.20)) *
      Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
      pset.bfs[0].charucoTransformation().inverse();

    pcl::transformPointCloud(
      *(pset.bfs[0].markerPointcloud()), *(pset.m_pointsPRef[2]), B);

    for (int i = 0; i < pset.bfs.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZRGBL>::Ptr points_pc(
        new pcl::PointCloud<pcl::PointXYZRGBL>());
      *points_pc = *pset.bfs[i].markerPointcloud();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());
      *cloud = *pset.bfs[i].cloud();
      pcl::transformPointCloud(
        *(points_pc), *(points_pc), pset.transformations[i]);
      pcl::transformPointCloud(*(cloud), *(cloud), pset.transformations[i]);
      if (j == m_pointsets.size() - 1)
        *_transformedCloud += *cloud;

      int idx = i / 2;
      for (size_t k = 0; k < points_pc->points.size(); k++) {
        auto p1 = points_pc->points.at(k);
        std::vector<pcl::PointXYZRGBL,
                    Eigen::aligned_allocator<pcl::PointXYZRGBL>>::iterator it =
          std::find_if(
            pset.m_pointsPRef[idx]->points.begin(),
            pset.m_pointsPRef[idx]->points.end(),
            [p1](pcl::PointXYZRGBL const& p) { return p1.label == p.label; });

        if (it != pset.m_pointsPRef[idx]->points.end()) {
          int index = std::distance(pset.m_pointsPRef[idx]->points.begin(), it);
          auto p2 = pset.m_pointsPRef[idx]->points.at(index);

          double dis = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) +
                            pow((p1.z - p2.z), 2));
          if (!dis)
            continue;
          meas_count++;
          pset.mea += dis;
          pset.smea += pow(dis, 2);
        }
      }
    }

    for (int i = 0; i < pset.bfs.size(); ++i) {
      pcl::PointCloud<pcl::PointXYZRGBL>::Ptr points_pc(
        new pcl::PointCloud<pcl::PointXYZRGBL>());
      *points_pc = *pset.bfs[i].markerPointcloud();

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());
      *cloud = *pset.bfs[i].cloud();

      if (i == 0) {
        pcl::transformPointCloud(*(cloud), *(cloud), pset.transformations[i]);
        pcl::transformPointCloud(
          *(points_pc), *(points_pc), pset.transformations[i]);
      }
      if (i == 1) {
        pcl::transformPointCloud(
          *(cloud),
          *(cloud),
          pset.transformations[0] * pset.bfs[0].charucoTransformation() *
            pset.bfs[i].charucoTransformation().inverse());
        pcl::transformPointCloud(
          *(points_pc),
          *(points_pc),
          pset.transformations[0] * pset.bfs[0].charucoTransformation() *
            pset.bfs[i].charucoTransformation().inverse());
      }
      if (i == 2 || i == 3) {
        Eigen::Affine3d A_C =
          pset.transformations[0] * pset.bfs[0].charucoTransformation() *
          Eigen::Translation3d(Eigen::Vector3d(0.280, 0.187, -0.19)) *
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
          pset.bfs[i].charucoTransformation().inverse();
        pcl::transformPointCloud(*(cloud), *(cloud), A_C);
        pcl::transformPointCloud(*(points_pc), *(points_pc), A_C);
      }
      if (i == 4 || i == 5) {
        Eigen::Affine3d B_C =
          pset.transformations[0] * pset.bfs[0].charucoTransformation() *
          Eigen::Translation3d(Eigen::Vector3d(0.280, 0.0, -0.20)) *
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
          pset.bfs[i].charucoTransformation().inverse();
        pcl::transformPointCloud(*(cloud), *(cloud), B_C);
        pcl::transformPointCloud(*(points_pc), *(points_pc), B_C);
      }
      if (j == m_pointsets.size() - 1)
        *_transformedCharucoCloud += *cloud;

      int idx = i / 2;
      for (size_t k = 0; k < points_pc->points.size(); k++) {
        auto p1 = points_pc->points.at(k);
        std::vector<pcl::PointXYZRGBL,
                    Eigen::aligned_allocator<pcl::PointXYZRGBL>>::iterator it =
          std::find_if(
            pset.m_pointsPRef[idx]->points.begin(),
            pset.m_pointsPRef[idx]->points.end(),
            [p1](pcl::PointXYZRGBL const& p) { return p1.label == p.label; });

        if (it != pset.m_pointsPRef[idx]->points.end()) {
          int index = std::distance(pset.m_pointsPRef[idx]->points.begin(), it);
          auto p2 = pset.m_pointsPRef[idx]->points.at(index);

          double dis = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) +
                            pow((p1.z - p2.z), 2));
          if (!dis)
            continue;

          c_meas_count++;
          pset.c_mea += dis;
          pset.c_smea += pow(dis, 2);
        }
      }
    }

    //        viewer.spin();
    pset.rmse = sqrt(pset.smea / meas_count);
    pset.mea = pset.mea / meas_count;
    pset.smea = pset.smea / meas_count;

    pset.c_rmse = sqrt(pset.c_smea / c_meas_count);
    pset.c_mea = pset.c_mea / c_meas_count;
    pset.c_smea = pset.c_smea / c_meas_count;
    //    ui->rmse->display(rmse);
    std::cout << "Mean Error: " << pset.mea << " m" << endl;
    std::cout << "Squared Mean Error: " << pset.smea << " m" << endl;
    std::cout << "RMSE: " << pset.rmse << " for " << meas_count << " Points."
              << endl;
    std::cout << "Charuco Mean Error: " << pset.c_mea << " m" << endl;
    std::cout << "Charuco Squared Mean Error: " << pset.c_smea << " m" << endl;
    std::cout << "Charuco RMSE: " << pset.c_rmse << " for " << c_meas_count
              << " Points." << endl;
    rmse += pset.rmse;
    mea += pset.mea;
    smea += pset.smea;
    c_rmse += pset.c_rmse;
    c_mea += pset.c_mea;
    c_smea += pset.c_smea;
  }
  rmse /= m_pointsets.size();
  smea /= m_pointsets.size();
  mea /= m_pointsets.size();
  c_rmse /= m_pointsets.size();
  c_smea /= m_pointsets.size();
  c_mea /= m_pointsets.size();
  std::cout << "Mean Error: " << mea << " m" << endl;
  std::cout << "Squared Mean Error: " << smea << " m" << endl;
  std::cout << "RMSE: " << rmse << " for " << meas_count << " Points." << endl;
  std::cout << "Total charuco Mean Error: " << c_mea << " m" << endl;
  std::cout << "Total charuco Squared Mean Error: " << c_smea << " m" << endl;
  std::cout << "Total charuco RMSE: " << c_rmse << " for " << c_meas_count
            << " Points." << endl;
}

const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&
CalibrationEval::transformedCloud() const
{
  return _transformedCloud;
}

const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&
CalibrationEval::transformedCharucoCloud() const
{
  return _transformedCharucoCloud;
}

double
CalibrationEval::getRmse() const
{
  return rmse;
}

double
CalibrationEval::getMea() const
{
  return mea;
}

double
CalibrationEval::getSmea() const
{
  return smea;
}

double
CalibrationEval::getC_rmse() const
{
  return c_rmse;
}

double
CalibrationEval::getC_mea() const
{
  return c_mea;
}

double
CalibrationEval::getC_smea() const
{
  return c_smea;
}
