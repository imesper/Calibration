#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent)
  , _pointsCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , _refLeftCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , _refUpCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , _refRightCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , _refCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , ui(new Ui::MainWindow)
{

  ui->setupUi(this);
    vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkNew<vtkRenderer> render;
   renderWindow->AddRenderer(render);
  _vis.reset(new pcl::visualization::PCLVisualizer(render, renderWindow, "Main", false));

  ui->qvtkWidget->setRenderWindow(_vis->getRenderWindow());

  _vis->setBackgroundColor(0.3, 0.3, 0.3);

  _vis->setupInteractor(ui->qvtkWidget->interactor(),
                        ui->qvtkWidget->renderWindow());



//  ui->qvtkWidget->setRenderWindow(renderWindow.Get());
//
//  ui->qvtkWidget->update();
//  ui->qvtkWidget->renderWindow()->Render();

  //  toolMat = new ToolMat(-0.228, 0.0239, 0.0196, 0.0880, -0.98, 0.31);
  // toolMat = new ToolMat(-0.0424, 0.06428, 0.07375, 0, 0, 0);
  //  toolMat = new ToolMat(-0.050, 0.073, 0.0396, 0, 0, 0);
//  toolMat =
//    new ToolMat(-0.048, -0.04107, 0.078, 90.7997, 1.0464, 0.5910);
  toolMat =
    new ToolMat(-0.050, -0.04418107, 0.078, 90.917997, 1.17240464, 0.485910);
  //        new ToolMat(-0.04689, -0.0411, 0.07864, 90.8897104, 0.8983838, 0.490231);
  mP6 = new RobPoint(1314.19,
                     -1063.44,
                     230.183,
                     3.96905e-05,
                     0.999989,
                     -0.00477632,
                     5.51653e-05);

  mP5 = new RobPoint(2130.96,
                     -1063.1,
                     229.973,
                     1.07587e-05,
                     -0.999989,
                     0.00466673,
                     -1.88101e-05);

  mP4 = new RobPoint(
    2095.06, -43.7679, 1124.66, 0.00441605, 0.00401656, -0.707075, -0.70711);

  mP3 = new RobPoint(
    1238.07, -43.5493, 1124.65, 0.00498674, 0.00396717, -0.707169, -0.707016);

  mP2 = new RobPoint(
    1240.28, 1076.08, 230.584, 6.12919e-05, 0.0217185, -0.999764, 4.54116e-05);

  mP1 = new RobPoint(
    2045.71, 1076.15, 230.597, 3.05979e-05, 0.0216769, -0.999765, 1.79927e-05);

  transformations.append(Helper::RobTarget2Affine3d(*mP1, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP2, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP3, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP4, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP5, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP6, *toolMat));

  //  _cloud = Helper::loadBag(
  //    "calib/Cam_0_1.bag", _pointsCloud, m_markerCorners, transformation,
  //    true);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tempCloud(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp2Cloud(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp3Cloud(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp4Cloud(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp5Cloud(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp6Cloud(
    new pcl::PointCloud<pcl::PointXYZRGBL>);

  std::string folder =
    "/Users/ianesper/Development/Test/";
  _clouds.append(
    Helper::loadBag(folder + "Cam_0_2.bag", _refCloud, transformation, true));
  *tempCloud = *_refCloud;
  _pointsclouds.append(tempCloud);
  _clouds.append(
    Helper::loadBag(folder + "Cam_1_2.bag", temp2Cloud, transformation2, true));
  _pointsclouds.append(temp2Cloud);
  _clouds.append(
    Helper::loadBag(folder + "Cam_2_2.bag", temp3Cloud, transformation, false));
  _pointsclouds.append(temp3Cloud);
  _clouds.append(
    Helper::loadBag(folder + "Cam_3_2.bag", temp4Cloud, transformation, false));
  _pointsclouds.append(temp4Cloud);
  _clouds.append(
    Helper::loadBag(folder + "Cam_4_2.bag", temp5Cloud, transformation, false));
  _pointsclouds.append(temp5Cloud);
  _clouds.append(
    Helper::loadBag(folder + "Cam_5_2.bag", temp6Cloud, transformation, false));
  _pointsclouds.append(temp6Cloud);

  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr tempCloudT(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp2CloudT(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp3CloudT(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp4CloudT(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp5CloudT(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr temp6CloudT(
    new pcl::PointCloud<pcl::PointXYZRGBL>);
  _pointscloudsT.append(tempCloudT);
  _pointscloudsT.append(temp2CloudT);
  _pointscloudsT.append(temp3CloudT);
  _pointscloudsT.append(temp4CloudT);
  _pointscloudsT.append(temp5CloudT);
  _pointscloudsT.append(temp6CloudT);

  updateCloud();

  ui->doubleSpinBox_x->setValue(toolMat->x);
  ui->doubleSpinBox_y->setValue(toolMat->y);
  ui->doubleSpinBox_z->setValue(toolMat->z);
  ui->doubleSpinBox_rx->setValue(toolMat->rx);
  ui->doubleSpinBox_ry->setValue(toolMat->ry);
  ui->doubleSpinBox_rz->setValue(toolMat->rz);

  /******************************************************************
   *
   * Calculate Camrera / TCP
   *
   * ***************************************************************/
  Eigen::Affine3d transformation_inv = transformation.inverse();
  Eigen::Affine3d transformation2_inv = transformation2.inverse();
  Eigen::Affine3d transformation_camPos1 = Helper::RobTarget2Affine3d(*mP1);
  Eigen::Affine3d transformation_camPos2 = Helper::RobTarget2Affine3d(*mP2);

  std::cout << (transformation_camPos1 * transformation_inv).matrix()
            << std::endl;
  std::cout << (transformation_camPos2 * transformation2_inv).matrix()
            << std::endl;
  //  std::cout << transformation_cam1.matrix() << std::endl;
}

MainWindow::~MainWindow()
{
  delete ui;
}

void
MainWindow::updateCloud()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZRGB>);

  for (int i = 0; i < _clouds.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

    *tempCloud = *_clouds[i];
    pcl::transformPointCloud(*tempCloud, *tempCloud, transformations[i]);

    pcl::transformPointCloud(
      *_pointsclouds[i], *_pointscloudsT[i], transformations[i]);

    std::string name = "CloudPC" + std::to_string(i);
    if (!_vis->updatePointCloud<pcl::PointXYZRGBL>(_pointscloudsT[i],
                                                   name.c_str()))
      _vis->addPointCloud<pcl::PointXYZRGBL>(_pointscloudsT[i], name.c_str());

    *cloud += *tempCloud;
  }

  pcl::transformPointCloud(*_refCloud, *_refLeftCloud, transformations[0] );

  Eigen::Affine3d A =
    transformations[0] * transformation *
    Eigen::Translation3d(Eigen::Vector3d(0.280, 0.187, -0.190)) *
//          Eigen::Translation3d(Eigen::Vector3d(0.280, 0.191, -0.0163)) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
     transformation.inverse();
  Eigen::Affine3d C =
transformations[0] *transformation *
    Eigen::Translation3d(Eigen::Vector3d(0.280, 0.187, -0.19)) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())
    ;
  _vis->addCoordinateSystem(0.05, static_cast<Eigen::Affine3f>(transformations[0] * transformation), "T1");
_vis->addCoordinateSystem(0.05, static_cast<Eigen::Affine3f>(C), "T2");
  pcl::transformPointCloud(*_refCloud, *_refUpCloud, A);

  Eigen::Affine3d B =
    transformations[0] * transformation *
    Eigen::Translation3d(Eigen::Vector3d(0.280, 0.0, -0.20)) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()) *
    transformation.inverse();
  Eigen::Affine3d B1 =
    transformations[0] * transformation *
    Eigen::Translation3d(Eigen::Vector3d(0.280, 0.0, -0.20)) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())
    ;
_vis->addCoordinateSystem(0.05, static_cast<Eigen::Affine3f>(B1), "T3");
  pcl::transformPointCloud(*_refCloud, *_refRightCloud, B);

  _pointsPRef.clear();
  _pointsPRef.append(_refLeftCloud);
  _pointsPRef.append(_refUpCloud);
  _pointsPRef.append(_refRightCloud);

  if (!_vis->updatePointCloud(cloud, "cloud")) {
    _vis->addPointCloud(cloud, "cloud");
    _vis->addCoordinateSystem(0.5);
  }

  if (!_vis->updatePointCloud<pcl::PointXYZRGBL>(_refLeftCloud, "refLeft")) {

    _vis->addPointCloud<pcl::PointXYZRGBL>(_refLeftCloud, "refLeft");
  }
  if (!_vis->updatePointCloud<pcl::PointXYZRGBL>(_refUpCloud, "refUp")) {

    _vis->addPointCloud<pcl::PointXYZRGBL>(_refUpCloud, "refUp");
  }
  if (!_vis->updatePointCloud<pcl::PointXYZRGBL>(_refRightCloud, "refRight")) {

    _vis->addPointCloud<pcl::PointXYZRGBL>(_refRightCloud, "refRight");
  }

//  ui->qvtkWidget->update();
  ui->qvtkWidget->renderWindow()->Render();
  CalcError();
}
void
MainWindow::CalcError()
{
  rmse = 0.0;
  mea = 0.0;
  smea = 0.0;

  int meas_count = 0;

  for (int i = 1; i < _pointscloudsT.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr points_pc = _pointscloudsT[i];
    int idx = i / 2;
    for (size_t k = 0; k < points_pc->points.size(); k++) {
      auto p1 = points_pc->points.at(k);
      std::vector<pcl::PointXYZRGBL,
                  Eigen::aligned_allocator<pcl::PointXYZRGBL>>::iterator it =
        std::find_if(
          _pointsPRef[idx]->points.begin(),
          _pointsPRef[idx]->points.end(),
          [p1](pcl::PointXYZRGBL const& p) { return p1.label == p.label; });

      if (it != _pointsPRef[idx]->points.end()) {
        int index = std::distance(_pointsPRef[idx]->points.begin(), it);
        auto p2 = _pointsPRef[idx]->points.at(index);

        double dis = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) +
                          pow((p1.z - p2.z), 2));
        if (!dis)
          continue;
        meas_count++;

        //        std::cout << "Euc Dist: " << dis << std::endl;
//        if(meas_count %10)
//                _vis->addArrow(
//                  p1, p2, 255, 0, 0, false, std::to_string(p1.label +
//                  rand()));
        mea += dis;

        smea += pow(dis, 2);
        // cout << "Sum Euc Dist: " << sum_dist << endl;
      }
    }
  }
  rmse = sqrt(smea / meas_count);
  ui->rmse->display(rmse);
  std::cout << "Mean Error: " << mea / meas_count << " m" << endl;
  std::cout << "Squared Mean Error: " << smea / meas_count << " m" << endl;
  std::cout << "RMSE: " << rmse << " for " << meas_count << " Points." << endl;
}
void
MainWindow::on_doubleSpinBox_x_valueChanged(double arg1)
{
  toolMat->setX(static_cast<float>(arg1));
  transformations.clear();
  transformations.append(Helper::RobTarget2Affine3d(*mP1, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP2, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP3, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP4, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP5, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP6, *toolMat));
  updateCloud();
}

void
MainWindow::on_doubleSpinBox_rx_valueChanged(double arg1)
{
  toolMat->setRx(static_cast<float>(arg1));
  transformations.clear();
  transformations.append(Helper::RobTarget2Affine3d(*mP1, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP2, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP3, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP4, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP5, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP6, *toolMat));
  updateCloud();
}

void
MainWindow::on_doubleSpinBox_ry_valueChanged(double arg1)
{
  toolMat->setRy(static_cast<float>(arg1));
  transformations.clear();
  transformations.append(Helper::RobTarget2Affine3d(*mP1, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP2, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP3, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP4, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP5, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP6, *toolMat));
  updateCloud();
}

void
MainWindow::on_doubleSpinBox_rz_valueChanged(double arg1)
{
  toolMat->setRz(static_cast<float>(arg1));
  transformations.clear();
  transformations.append(Helper::RobTarget2Affine3d(*mP1, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP2, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP3, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP4, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP5, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP6, *toolMat));
  updateCloud();
}

void
MainWindow::on_doubleSpinBox_y_valueChanged(double arg1)
{
  toolMat->setY(static_cast<float>(arg1));
  transformations.clear();
  transformations.append(Helper::RobTarget2Affine3d(*mP1, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP2, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP3, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP4, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP5, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP6, *toolMat));
  updateCloud();
}

void
MainWindow::on_doubleSpinBox_z_valueChanged(double arg1)
{
  toolMat->setZ(static_cast<float>(arg1));
  transformations.clear();
  transformations.append(Helper::RobTarget2Affine3d(*mP1, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP2, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP3, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP4, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP5, *toolMat));
  transformations.append(Helper::RobTarget2Affine3d(*mP6, *toolMat));
  updateCloud();
}

void
MainWindow::on_horizontalSliderPSize_valueChanged(int value)
{
  _vis->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "refLeft");
  _vis->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "refUp");
  _vis->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "refRight");
  _vis->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget->update();
}
