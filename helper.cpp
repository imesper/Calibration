#include "helper.h"

Helper::Helper() {}

Eigen::Affine3d
Helper::RobTarget2Affine3d(RobPoint target, ToolMat tool)
{
  std::cout << "Starting RobTarget2Affine3d" << std::endl;

  Eigen::Vector3d y_rot = Eigen::Vector3d::UnitY();
  Eigen::Vector3d x_rot = Eigen::Vector3d::UnitX();
  Eigen::Vector3d z_rot = Eigen::Vector3d::UnitZ();

  Eigen::Affine3d transformation1 = Eigen::Affine3d::Identity();

  transformation1 =
    transformation1 *
    //    Eigen::AngleAxisd(M_PI, y_rot) *
//    Eigen::AngleAxisd(90 * 0.0174533, x_rot) *

    Eigen::Translation3d(Eigen::Vector3d(tool.x, tool.y, tool.z))
          * Eigen::AngleAxisd(tool.rx * 0.0174533, x_rot)
          * Eigen::AngleAxisd(tool.ry * 0.0174533, y_rot)
          * Eigen::AngleAxisd(tool.rz * 0.0174533, z_rot);

  //  transformation1 = Eigen::Affine3d::Identity();
  //  transformation1 =
  //    transformation1 * Eigen::AngleAxisd(-M_PI_2, y_rot) *
  //    Eigen::AngleAxisd(M_PI, z_rot) *
  //    Eigen::Translation3d(Eigen::Vector3d(tool.x, tool.y, tool.z)) *
  //    Eigen::AngleAxisd(tool.rx * 0.0174533, x_rot) *
  //    Eigen::AngleAxisd(tool.ry * 0.0174533, y_rot) *
  //    Eigen::AngleAxisd(tool.rz * 0.0174533, z_rot);

  /***************************************************************************
   * Camera Tool Geometry in relation to TCP
   * ************************************************************************/
  // if (target.x == 1987.52 || target.x == 1102.77)
  //   transformation1 = transformation1 *bbb
  //                     Eigen::AngleAxisd(0 * 0.0174533, z_rot) *
  //                     // Eigen::AngleAxisd(0.0174533, x_rot) *
  //                     Eigen::AngleAxisd(0 * 0.0174533, y_rot);
  // else
  //   transformation1 = transformation1 *
  //                     Eigen::AngleAxisd(0 * -0.0174533, z_rot) *
  //                     // Eigen::AngleAxisd(0.0174533, x_rot) *
  //                     Eigen::AngleAxisd(0 * -0.0174533, y_rot);
  /**************************************************************************/

  /**************************************************************************
   * Quaternion (W, X, Y, Z)
   * ABB Robot (W, X, Y, Z)
   * ************************************************************************/
  Eigen::Quaterniond q(target.q1, target.q2, target.q3, target.q4);

  std::cout << "Q: " << q.matrix() << std::endl;

  float x = target.x / 1000.0;
  float y = target.y / 1000.0;
  float z = target.z / 1000.0;

  Eigen::Vector3d translation(x, y, z);

  Eigen::Matrix3d R;

  q.normalize();
  std::cout << "Q: " << q.matrix() << std::endl;
  transformation1 = Eigen::Translation3d(translation) * q * transformation1;

  //  translation[0] += transformation1.translation()[0];
  //  translation[1] += transformation1.translation()[1];
  //  translation[2] += transformation1.translation()[2];

  //  transformation1.translation()[0] += translation[0];
  //  transformation1.translation()[1] += translation[1];
  //  transformation1.translation()[2] += translation[2];
  //  // cv::eigen2cv(transformation1.rotation(), R);

  //  std::cout << "Eigen Affine:" << transformation1.matrix() << std::endl;
  //  std::cout << "Translation:" << transformation1.translation() << std::endl;
  //  std::cout << "Translation:" << translation << std::endl;
  //  Eigen::Affine3d transformation2;
  //  transformation2.rotate(R);
  //  transformation2.translate(translation);

  // std::cout << "Transformation:" << transformation2.matrix() << std::endl;

  return transformation1;
}

Eigen::Affine3d
Helper::RobTarget2Affine3d(RobPoint target)
{
  std::cout << "Starting RobTarget2Affine3d" << std::endl;

  Eigen::Vector3d y_rot = Eigen::Vector3d::UnitY();
  Eigen::Vector3d x_rot = Eigen::Vector3d::UnitX();
  Eigen::Vector3d z_rot = Eigen::Vector3d::UnitZ();

  Eigen::Affine3d transformation1 = Eigen::Affine3d::Identity();

  /**************************************************************************
   * Quaternion (W, X, Y, Z)
   * ABB Robot (W, X, Y, Z)
   * ************************************************************************/
  Eigen::Quaterniond q(target.q1, target.q2, target.q3, target.q4);

  std::cout << "Q: " << q.matrix() << std::endl;

  float x = target.x / 1000.0;
  float y = target.y / 1000.0;
  float z = target.z / 1000.0;

  Eigen::Vector3d translation(x, y, z);

  Eigen::Matrix3d R;

  q.normalize();
  std::cout << "Q: " << q.matrix() << std::endl;
  transformation1 = Eigen::Translation3d(translation) * q * transformation1;

  return transformation1;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Helper::RS2toPCL(cv::Mat depth,
                 cv::Mat& color,
                 rs2_intrinsics intrinsic,
                 pcl::PointCloud<pcl::PointXYZRGBL>::Ptr points_pc,
                 std::map<int, std::vector<cv::Point2f>>& m_markerCorners,
                 int p_color,
                 double *m_depth)
{
#ifdef PRINT_TIME
  pcl::ScopeTime t("Processing Cloud Direct");
#endif

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  float fx = intrinsic.fx;
  float fy = intrinsic.fy;
  float cx = intrinsic.ppx;
  float cy = intrinsic.ppy;
  std::cout << fx << " " << fy << " " << cx << " " << cy << " " << depth.rows
            << " " << depth.cols << std::endl;

  if (!depth.data) {
    std::cerr << "No depth data!!!" << std::endl;
    return nullptr;
  }

  pointcloud->width = static_cast<uint>(depth.cols);
  pointcloud->height = static_cast<uint>(depth.rows);
  pointcloud->reserve(pointcloud->width * pointcloud->height);
  std::cout << depth.rows << " : " << color.rows << std::endl;
  std::cout << depth.cols << " : " << color.cols << std::endl;
  {
    float Z;
    const double f_scale = 0.001;

    double* image = reinterpret_cast<double*>(depth.data);
    std::cout << "PCL Loop -> Entering" << std::endl;
#ifndef PRINT_TIME
#define PRINT_TIME
#endif
#ifdef PRINT_TIME
    pcl::ScopeTime t("Processing Cloud Direct - For loop");
#endif
    uchar* rgb = color.data;
//#pragma omp parallel for
    for (int y = 0; y < depth.rows; y++) {
      for (int x = 0; x < depth.cols; x++) {
        cv::Point2f point(x, y);

        int in = (y * depth.cols) + x;

        double d = image[in];
        Z = (f_scale * d);

        if (Z > 0.400 && Z < 1.5) {
          pcl::PointXYZRGB p;

          p.z = Z;
          p.x = ((x - cx) / fx) * Z;
          p.y = ((y - cy) / fy) * Z;

          int index = (color.step[0] * y) + (3 * x);
          p.b = rgb[index];     // Reference tuple<2>
          p.g = rgb[index + 1]; // Reference tuple<1>
          p.r = rgb[index + 2]; // Reference tuple<0>

          pointcloud->points.emplace_back(p);
          for (auto marker : m_markerCorners) {

            std::vector<cv::Point2f> markerCorners = marker.second;
            std::vector<cv::Point2f>::iterator it =
              std::find(markerCorners.begin(), markerCorners.end(), point);

            if (it != markerCorners.end()) {
              pcl::PointXYZRGBL pl;
              int index = std::distance(markerCorners.begin(), it);
              // std::cout << "Index: " << index << std::endl;
              // std::cout << "First: " << marker.first << std::endl;
              if (m_depth){
                pl.x = m_depth[0];
              pl.y = m_depth[1];
              pl.z = m_depth[2];
                }
              else
                pl.z = Z;
              pl.x = ((x - cx) / fx) * pl.z;
              pl.y = ((y - cy) / fy) * pl.z;

              pl.label = (marker.first * 4) + index;
              // std::cout << "Label: " << pl.label << std::endl;
              switch (p_color) {
                case 0:
                  pl.r = 255;
                  pl.g = 0;
                  pl.b = 0;
                  break;
                case 1:
                  pl.r = 0;
                  pl.g = 255;
                  pl.b = 0;
                  break;
                case 2:
                  pl.r = 0;
                  pl.g = 0;
                  pl.b = 255;
                  break;
                default:
                  pl.b = 0;     // Reference tuple<2>
                  pl.g = 0; // Reference tuple<1>
                  pl.r = 255; // Reference tuple<0>
                  break;
              }
              points_pc->points.emplace_back(pl);
            }
          }
        }
      }
    }
  }
  return pointcloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Helper::loadBag(std::string filename,
                pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pointsPRef,
                Eigen::Affine3d& transformation,
                bool generateCube)
{
  std::map<int, std::vector<cv::Point2f>> m_markerCorners;
  rs2::pipeline pipe;
  rs2::config cfg;

  cfg.enable_device_from_file(filename);

  std::cout << "Pipe -> First" << std::endl;

  pipe.start(cfg);
  auto device = pipe.get_active_profile().get_device();

  rs2::frameset fs, aligned_frames;
  std::cout << "Frameset -> First entering wait for frame" << std::endl;

  // First Frame
  fs = pipe.wait_for_frames();

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
           fs = spatial_filter.process(fs);
           fs = temp_filter.process(fs);
          std::cout << fs.size() << std::endl;
        rs2::align align(RS2_STREAM_COLOR);

    aligned_frames = align.process(fs);

    fs = pipe.wait_for_frames();
  }
  rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
  rs2::depth_frame filtered = temp_filter.process(depth_frame);
  // filtered = spat_filter.process(filtered);

  depth = cv::Mat(filtered.get_height(),
                  filtered.get_width(),
                  CV_16UC1,
                  const_cast<void*>(filtered.get_data()),
                  cv::Mat::AUTO_STEP);

  cv::Mat temp;

  depth.convertTo(temp, CV_64F);
  depth = temp;
  rs2::video_frame rgb = aligned_frames.get_color_frame();
  cv::Mat color = cv::Mat(rgb.get_height(),
                          rgb.get_width(),
                          CV_8UC3,
                          const_cast<void*>(rgb.get_data()),
                          cv::Mat::AUTO_STEP);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZRGB>());

  cv::cvtColor(color, color, cv::COLOR_BGR2RGB);

  cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_32F);

  cameraMatrix.at<float>(0, 0) = color_intrinsic.fx;
  cameraMatrix.at<float>(0, 2) = color_intrinsic.ppx;
  cameraMatrix.at<float>(1, 1) = color_intrinsic.fy;
  cameraMatrix.at<float>(1, 2) = color_intrinsic.ppy;
  cameraMatrix.at<float>(2, 2) = 1;

  Eigen::Affine3d trans;
  detectCharucoBoardWithCalibrationPose(
    color, cameraMatrix, trans, m_markerCorners);

  std::vector<cv::Point2f> points;
  std::vector<cv::Point2f> ids;
  for (auto item : m_markerCorners) {
    ids.emplace_back(item.first);
    for (auto p : item.second) {
      points.emplace_back(p);
    }
  }
  double tansl[3];
  tansl[0] =trans.translation()[0];
          tansl[1]=trans.translation()[1];
          tansl[2]=trans.translation()[2];

  *cloud += *RS2toPCL(depth,
                      color,
                      color_intrinsic,
                      pointsPRef,
                      m_markerCorners,
                      0,
                      false ? tansl : nullptr);
  if (generateCube) {
    transformation = trans;
  }

  return cloud;
}
void
Helper::detectCharucoBoardWithCalibrationPose(
  cv::Mat image,
  cv::Mat cameraMatrix,
  Eigen::Affine3d& transformation,
  std::map<int, std::vector<cv::Point2f>>& m_markerCorners)
{

  cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_32F);
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
  cv::Ptr<cv::aruco::CharucoBoard> board =
    cv::aruco::CharucoBoard::create(14, 9, 0.02025f, 0.015f, dictionary);
  cv::Ptr<cv::aruco::DetectorParameters> params =
    cv::aruco::DetectorParameters::create();

  cv::Mat imageCopy, imageShow, camMatrix;
  cameraMatrix.copyTo(camMatrix);

  // camMatrix.at<float>(0, 0) *= scale;
  // camMatrix.at<float>(0, 2) *= scale;
  // camMatrix.at<float>(1, 1) *= scale;
  // camMatrix.at<float>(1, 2) *= scale;

  // cv::resize(image,
  //            imageCopy,
  //            cv::Size(scale * image.cols, scale * image.rows),
  //            0,
  //            0,
  //            cv::INTER_CUBIC);

  image.copyTo(imageCopy);

  // cv::cvtColor(image, imageCopy, cv::COLOR_BGR2RGB);
  cv::Mat sharp;
  cv::Mat sharpening_kernel =
    (cv::Mat_<double>(3, 3) << -1, -1, -1, -1, 9, -1, -1, -1, -1);
  cv::filter2D(imageCopy, imageCopy, -1, sharpening_kernel);

  imageCopy.copyTo(imageShow);

  std::vector<int> markerIds;

  std::vector<std::vector<cv::Point2f>> rejected;

  cv::Mat array = camMatrix.reshape(1, camMatrix.rows * camMatrix.cols);

  cv::aruco::detectMarkers(imageCopy,
                           board->dictionary,
                           markerCorners,
                           markerIds,
                           params,
                           rejected,
                           cameraMatrix);

  // if at least one marker detected
  if (markerIds.size() != 63) {
    std::cout << "Markers Before: " << markerIds.size() << std::endl;
    cv::aruco::refineDetectedMarkers(
      imageCopy, board, markerCorners, markerIds, rejected, camMatrix);
    std::cout << "Markers After: " << markerIds.size() << std::endl;
  } else {
    std::cout << "Found All" << std::endl;
  }
  if (markerIds.size() > 0) {
    cv::aruco::drawDetectedMarkers(imageShow, markerCorners, markerIds);
    std::vector<cv::Point2f> charucoCorners;
    std::vector<int> charucoIds;
    cv::aruco::interpolateCornersCharuco(markerCorners,
                                         markerIds,
                                         imageCopy,
                                         board,
                                         charucoCorners,
                                         charucoIds,
                                         camMatrix,
                                         distCoeffs);
    // if at least one charuco corner detected
    if (charucoIds.size() > 0) {
      cv::Scalar color = cv::Scalar(255, 0, 0);
      cv::aruco::drawDetectedCornersCharuco(
        imageShow, charucoCorners, charucoIds, color);

      std::vector<cv::Vec3d> rvecs, tvecs;
      // cv::aruco::estimatePoseSingleMarkers(
      //   markerCorners, 0.0186f, camMatrix, distCoeffs, rvecs, tvecs);
      // // draw axis for each marker
      for (int i = 0; i < markerIds.size(); i++) {
        m_markerCorners.emplace(markerIds[i], markerCorners[i]);

        //   // cv::aruco::drawAxis(
        //   //   imageShow, camMatrix, distCoeffs, rvecs[i], tvecs[i], 0.018f);
      }
      cv::Vec3f rvec, tvec;
      bool valid = cv::aruco::estimatePoseCharucoBoard(
        charucoCorners, charucoIds, board, camMatrix, distCoeffs, rvec, tvec);
      // if charuco pose is valid
      if (valid) {
        cv::aruco::drawAxis(imageShow, camMatrix, distCoeffs, rvec, tvec, 0.3f);
        cv::Affine3d aff(rvec, tvec);

        // cv::Mat rot_mat = cv::Mat::zeros(3, 3, CV_64F);

        // cv::Rodrigues(rvec, rot_mat);

        // std::cout << "Rvec: " << rvec << std::endl;
        // std::cout << "Tvec: " << tvec << std::endl;
        // std::cout << "Mat: " << rot_mat << std::endl;
        // Eigen::Map<Eigen::Matrix3f> eigenT();
        // std::cout << "Eigen: " << eigenT << std::endl;
        // aff.rotation(rvec);

        transformation = aff;

        std::cout << "Trans Eigen: " << transformation.matrix() << std::endl;
      }
    }
  }
//  cv::imshow("out" + std::to_string(transformation.translation()[0]),
//             imageShow);
//  cv::waitKey();
}

void
ToolMat::setRz(float value)
{
  rz = value;
}

void
ToolMat::setRy(float value)
{
  ry = value;
}

void
ToolMat::setRx(float value)
{
  rx = value;
}

void
ToolMat::setZ(float value)
{
  z = value;
}

void
ToolMat::setY(float value)
{
  y = value;
}

void
ToolMat::setX(float value)
{
  x = value;
}
