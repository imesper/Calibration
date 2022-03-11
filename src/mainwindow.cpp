#include "../include/mainwindow.h"
#include "./ui_mainwindow.h"
#include <QDebug>
MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent)
  , _pointsCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , _refLeftCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , _refUpCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , _refRightCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , _refCloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , calibrationEvaluated(false)
  , ui(new Ui::MainWindow)
{

  ui->setupUi(this);
  vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
  vtkNew<vtkRenderer> render;
  renderWindow->AddRenderer(render);

  _vis.reset(
    new pcl::visualization::PCLVisualizer(render, renderWindow, "Main", false));
  ui->qvtkWidget->setRenderWindow(_vis->getRenderWindow());

  _vis->setBackgroundColor(0.3, 0.3, 0.3);

  _vis->setupInteractor(ui->qvtkWidget->interactor(),
                        ui->qvtkWidget->renderWindow());

  vtkNew<vtkGenericOpenGLRenderWindow> c_renderWindow;
  vtkNew<vtkRenderer> c_render;
  c_renderWindow->AddRenderer(c_render);
  c_vis.reset(new pcl::visualization::PCLVisualizer(
    c_render, c_renderWindow, "Main Charuco", false));

  ui->qvtkWidget_2->setRenderWindow(c_vis->getRenderWindow());

  c_vis->setBackgroundColor(0.3, 0.3, 0.3);

  c_vis->setupInteractor(ui->qvtkWidget_2->interactor(),
                         ui->qvtkWidget_2->renderWindow());
}

MainWindow::~MainWindow()
{
  delete ui;
}

void
MainWindow::updateCloud()
{

  if (!_vis->updatePointCloud(_cloud, "cloud")) {
    _vis->addPointCloud(_cloud, "cloud");
    _vis->addCoordinateSystem(0.5);
  }

  if (!c_vis->updatePointCloud(_charucoCloud, "Charuco Cloud")) {
    c_vis->addPointCloud(_charucoCloud, "Charuco Cloud");
    c_vis->addCoordinateSystem(0.5);
  }

  //  ui->qvtkWidget->update();
  ui->qvtkWidget->renderWindow()->Render();
  ui->qvtkWidget_2->renderWindow()->Render();
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
  c_vis->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "Charuco Cloud");
  ui->qvtkWidget_2->update();
}

void
MainWindow::on_pushButton_clicked()
{
  QString directory = QFileDialog::getExistingDirectory(
    this,
    tr("Open Directory"),
    "~/",
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  calibrationEval.init(directory);
  calibrationEvaluated = true;
  calibrationEval.CalcError();
  _cloud = calibrationEval.transformedCloud();
  _charucoCloud = calibrationEval.transformedCharucoCloud();
  ui->rmse->display(calibrationEval.getRmse());
  ui->sme->display(calibrationEval.getSmea());
  ui->mea->display(calibrationEval.getMea());
  ui->rmse_2->display(calibrationEval.getC_rmse());
  ui->sme_2->display(calibrationEval.getC_smea());
  ui->mea_2->display(calibrationEval.getC_mea());
  updateCloud();
}
