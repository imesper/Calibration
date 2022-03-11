#include "../include/bagfile.h"
#include <QDebug>
#include <librealsense2/rs.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

BagFile::BagFile()
  : _cameraPos(0)
  , m_markerPointcloud(new pcl::PointCloud<pcl::PointXYZRGBL>())
  , pointsPRef(new pcl::PointCloud<pcl::PointXYZRGBL>())
{}

void
BagFile::init(QString filename, QString matrix)
{
  this->_filename = filename;
  this->_matrix = matrix;

  rs2::pipeline pipe;
  rs2::config cfg;

  std::cout << filename.toStdString() << std::endl;
  cfg.enable_device_from_file(filename.toStdString());

  std::cout << "Pipe -> First" << std::endl;

  pipe.start(cfg);
  auto device = pipe.get_active_profile().get_device();

  rs2::frameset fs, aligned_frames;
  std::cout << "Frameset -> First entering wait for frame" << std::endl;

  // First Frame
  fs = pipe.wait_for_frames();
  std::cout << fs.size() << std::endl;
  auto intrinsic = fs.get_depth_frame()
                     .get_profile()
                     .as<rs2::video_stream_profile>()
                     .get_intrinsics();

  auto color_intrinsic = fs.get_color_frame()
                           .get_profile()
                           .as<rs2::video_stream_profile>()
                           .get_intrinsics();

  cv::Mat depth;

  rs2::temporal_filter temp_filter(0.1f, 20, 3);
  rs2::spatial_filter spatial_filter(0.4f, 4, 3, 1);
  for (size_t j = 0; j < 5; j++) {
    fs = pipe.wait_for_frames();
    //         fs = spatial_filter.process(fs);
    //         fs = temp_filter.process(fs);

    rs2::align align(RS2_STREAM_COLOR);
    try {
      fs = align.process(fs);
    } catch (std::exception& e) {
    }
  }
  rs2::depth_frame depth_frame = fs.get_depth_frame();
  _depths.emplace_back(depth_frame);
  _colors.emplace_back(fs.get_color_frame());

  _serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  pipe.stop();
}

const pcl::PointCloud<pcl::PointXYZRGB>::Ptr&
BagFile::cloud() const
{
  return _cloud;
}

bool
BagFile::isFiltered() const
{
  return m_isFiltered;
}

bool
BagFile::hasMarkerPC() const
{
  return m_hasMarkerPC;
}

const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr&
BagFile::markerPointcloud() const
{
  return m_markerPointcloud;
}

const Eigen::Affine3d&
BagFile::charucoTransformation() const
{
  return m_charucoTransformation;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
BagFile::projectPointCloud(ToolMat tool,
                           int frame,
                           double threshold,
                           bool filter,
                           bool genMarkerPointPC)
{

  if (_depths.size() <= frame)
    frame = _depths.size() - 1;

  auto color = cv::Mat(_colors[frame].as<rs2::video_frame>().get_height(),
                       _colors[frame].as<rs2::video_frame>().get_width(),
                       CV_8UC3,
                       const_cast<void*>(_colors[frame].get_data()),
                       cv::Mat::AUTO_STEP);

  auto temp = cv::Mat(_depths[frame].get_height(),
                      _depths[frame].get_width(),
                      CV_16UC1,
                      const_cast<void*>(_depths[frame].get_data()),
                      cv::Mat::AUTO_STEP);
  cv::Mat depth;
  temp.convertTo(depth, CV_64F);

  std::cout << "Color frame: " << depth.size() << std::endl;

  auto color_intrinsic = _colors[frame]
                           .get_profile()
                           .as<rs2::video_stream_profile>()
                           .get_intrinsics();

  cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);

  cameraMatrix.at<float>(0, 0) = color_intrinsic.fx;
  cameraMatrix.at<float>(0, 2) = color_intrinsic.ppx;
  cameraMatrix.at<float>(1, 1) = color_intrinsic.fy;
  cameraMatrix.at<float>(1, 2) = color_intrinsic.ppy;
  cameraMatrix.at<float>(2, 2) = 1;

  Helper::detectCharucoBoardWithCalibrationPose(
    color, cameraMatrix, m_charucoTransformation, m_markerCorners);
  double tansl[3];
  tansl[0] = m_charucoTransformation.translation()[0];
  tansl[1] = m_charucoTransformation.translation()[1];
  tansl[2] = m_charucoTransformation.translation()[2];

  auto cloud = Helper::RS2toPCL(depth,
                                color,
                                color_intrinsic,
                                m_markerPointcloud,
                                m_markerCorners,
                                0,
                                false ? tansl : nullptr);

  if (m_markerPointcloud->width == 0 && m_markerPointcloud->points.size() > 0) {

    m_markerPointcloud->width = m_markerPointcloud->points.size();
    m_markerPointcloud->height = 1;
    m_hasMarkerPC = true;
  }

  qDebug() << "Cloud Size: " << cloud->size();

  pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter;
  Cloud_Filter.setFilterFieldName("z");
  Cloud_Filter.setFilterLimits(0.5, threshold);
  Cloud_Filter.setInputCloud(cloud);
  Cloud_Filter.filter(*cloud);

  if (!_matrix.isEmpty()) {
    std::cout << "Reading Transformation Matrix." << std::endl;
    //    Eigen::Affine3d t = Helper::readTransformation(_matrix.toStdString());
    // std::cout << "Eigen Transformation Matrix: " << t.matrix() << std::endl;
    // pcl::transformPointCloud(*cloud, *cloud, t);
  }

  //    std::cout << "Eigen Transformation Matrix: " << t.matrix() << std::endl;
  //    pcl::transformPointCloud(*cloud, *cloud, t);

  qDebug() << cloud->size();

  // py::finalize_interpreter();
  _cloud = cloud;
  return cloud;
}

QString
BagFile::getSerial() const
{
  return _serial;
}

cv::Mat
BagFile::getCVColorFrame()
{

  auto frame = _depths.size() - 1;

  auto color = cv::Mat(_colors[frame].as<rs2::video_frame>().get_height(),
                       _colors[frame].as<rs2::video_frame>().get_width(),
                       CV_8UC3,
                       const_cast<void*>(_colors[frame].get_data()),
                       cv::Mat::AUTO_STEP);
  return color;
}

int
BagFile::getCameraPos() const
{
  return _cameraPos;
}

void
BagFile::setCameraPos(int cameraPos)
{
  _cameraPos = cameraPos;
}
