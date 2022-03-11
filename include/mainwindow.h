#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QFileDialog>
#include <QMainWindow>
#include <QVector>
#include <vtkActor.h>
#include <vtkCamera.h>

#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
//#include <vtkOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>

#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "calibrationeval.h"
#include "helper.h"
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

  void CalcError();
private slots:

  void on_doubleSpinBox_x_valueChanged(double arg1);

  void on_doubleSpinBox_rx_valueChanged(double arg1);

  void on_doubleSpinBox_ry_valueChanged(double arg1);

  void on_doubleSpinBox_rz_valueChanged(double arg1);

  void on_doubleSpinBox_y_valueChanged(double arg1);

  void on_doubleSpinBox_z_valueChanged(double arg1);

  void on_horizontalSliderPSize_valueChanged(int value);

  void on_pushButton_clicked();

private:
  Ui::MainWindow* ui;

  QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _clouds;
  QVector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> _pointsPRef;

  QVector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> _pointsclouds;
  QVector<pcl::PointCloud<pcl::PointXYZRGBL>::Ptr> _pointscloudsT;
  QVector<Eigen::Affine3d> transformations;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _refCloud;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _refLeftCloud;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _refUpCloud;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _refRightCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _charucoCloud;
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _pointsCloud;
  pcl::visualization::PCLVisualizer::Ptr _vis;
  pcl::visualization::PCLVisualizer::Ptr c_vis;
  std::map<int, std::vector<cv::Point2f>> m_markerCorners;

  Eigen::Affine3d transformation;
  Eigen::Affine3d transformation2;
  RobPoint* mP6;
  RobPoint* mP5;
  RobPoint* mP4;
  RobPoint* mP3;
  RobPoint* mP2;
  RobPoint* mP1;

  ToolMat* toolMat;

  double mea;
  double smea;
  double rmse;
  void updateCloud();
  CalibrationEval calibrationEval;
  bool calibrationEvaluated;
};
#endif // MAINWINDOW_H
