/**
 * \file  calibrator.cc
 * \brief  Gathers data from the PointCloud to improve the
 *         pitch calibration of eacj laser
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, UT Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 03/05/2012 02:40:43 PM piyushk $
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>

#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

#include <velodyne_image/overhead_image_generator.h>
#include <velodyne_pointcloud/rawdata.h>

#include <boost/thread/mutex.hpp>

#define NODE "velodyne_calibrator"
#define NUM_ROUNDS 100

namespace {
  boost::shared_ptr<velodyne_rawdata::RawData> data_;
  boost::shared_ptr<velodyne_rawdata::RawData> calibrated_data_;
  velodyne_image::OverheadImageGenerator generator_;

  ros::Subscriber velodyne_scan_;

  // Draw and store rectangle coordinates
  bool rectangle_available_;
  bool rectangle_in_process_;
  int rect_x1_;
  int rect_x2_;
  int rect_y1_;
  int rect_y2_;

  // Used to convert pixels to 2D points
  int image_size_;
  float distance_per_pixel_;
  float ground_height_;

  // Use for calibration
  bool calibrate_ = false;
  boost::mutex m_calib;

  velodyne_pointcloud::Calibration default_calibration_;
  velodyne_pointcloud::Calibration current_calibration_;
  std::string output_file_ = "out.yaml";
}

void performCalibration(const velodyne_rawdata::VPointCloud& cloud,
    const velodyne_msgs::VelodyneScan::ConstPtr &scan_msg) {

  // Compute real world x and y boundaries
  int x1, x2, y1, y2;
  if (rect_x1_ < rect_x2_) {
    x1 = rect_x1_;
    x2 = rect_x2_;
  } else {
    x2 = rect_x1_;
    x1 = rect_x2_;
  }
  if (rect_y1_ < rect_y2_) {
    y1 = rect_y1_;
    y2 = rect_y2_;
  } else {
    y2 = rect_y1_;
    y1 = rect_y2_;
  }
  float x_high = distance_per_pixel_ * (image_size_ / 2 - y1); 
  float x_low = distance_per_pixel_ * (image_size_ / 2 - y2); 
  float y_high = distance_per_pixel_ * (image_size_ / 2 - x1); 
  float y_low = distance_per_pixel_ * (image_size_ / 2 - x2);

  ROS_INFO_STREAM(x_high << "," << x_low << "," << y_high << "," << y_low);

  // Figure out which rings have points contributing to the ground plane
  bool calibrate_ring[velodyne_rawdata::N_LASERS];
  for (int i = 0; i < velodyne_rawdata::N_LASERS; i++) {
    calibrate_ring[i] = false;
  }

  // Compute ground plane parameters that fit these boundaries
  pcl::PointIndices inliers;
  pcl::PointCloud<pcl::PointXYZ> plane_cloud;
  pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> estimator;

  int point_count = 0;
  BOOST_FOREACH(const velodyne_rawdata::VPoint& point, cloud.points) {
    if (point.x <= x_high && point.x >= x_low &&
        point.y <= y_high && point.y >= y_low &&
        fabs(point.z - ground_height_) < 0.25) {
      pcl::PointXYZ xyz_point;
      xyz_point.x = point.x;
      xyz_point.y = point.y;
      xyz_point.z = point.z;
      inliers.indices.push_back(point_count++);
      plane_cloud.push_back(xyz_point);
      calibrate_ring[point.ring] = true;
    }
  }
  Eigen::Vector4f params;
  float curvature;
  estimator.computePointNormal(plane_cloud, inliers.indices, params, curvature);

  // TODO print out ground plane parameters here, and the contributing rings

  float sum_sq_coeff = params.x() * params.x() + 
      params.y() * params.y() + params.z() * params.z();

  // Now lets work to match each ring to the plane
  current_calibration_ = default_calibration_;
  ros::param::set("~calibration", output_file_);
  for (int ring = 0; ring < velodyne_rawdata::N_LASERS; ++ring) {

    if (!calibrate_ring[ring])
      continue;

    ROS_INFO_STREAM("Calibrating ring " << ring);

    // Search in the vicinity of the current pitch
    float error_metric[NUM_ROUNDS];
    for (int round = 0 ; round < NUM_ROUNDS; ++round) {

      // Write new calibration
      velodyne_pointcloud::Calibration calib = current_calibration_;
      calib.laser_corrections[velodyne_rawdata::LASER_SEQUENCE[ring]].vert_correction =
        current_calibration_.laser_corrections[velodyne_rawdata::LASER_SEQUENCE[ring]].vert_correction +
        (round - NUM_ROUNDS / 2) * 0.001;

      // Get the new calibration
      calib.write(output_file_);
      calibrated_data_.reset(new velodyne_rawdata::RawData());
      ros::NodeHandle private_nh("~");
      calibrated_data_->setup(private_nh);

      // Get the new pointcloud
      velodyne_rawdata::VPointCloud calibrated_cloud;
      for (size_t i = 0; i < scan_msg->packets.size(); ++i) {
        calibrated_data_->unpack(scan_msg->packets[i], calibrated_cloud);
      }

      // Compute the error metric
      int count = 0;
      float error = 0;
      BOOST_FOREACH(const velodyne_rawdata::VPoint& point, calibrated_cloud.points) {
        if (point.ring != ring)
          continue;
        if (point.x <= x_high && point.x >= x_low &&
            point.y <= y_high && point.y >= y_low) {
          ++count;
          float dist_numerator = point.x * params.x() + 
              point.y * params.y() + point.z * params.z() + params.w();
          error += dist_numerator * dist_numerator / sum_sq_coeff;
        }
      }
      if (count != 0) {
        error_metric[round] = error / count;
      } else {
        error_metric[round] = 9000.0;
      }
      ROS_INFO_STREAM("  round " << round << ": " << error_metric);
    }

    // Compute the pitch with the lowest error
    int best_round = 0;
    float lowest_error = error_metric[0];
    for (int i = 1; i < NUM_ROUNDS; i++) {
      if (error_metric[i] < lowest_error) {
        best_round = i;
        lowest_error = error_metric[i];
      }
    }
    ROS_INFO_STREAM("  best round: " << best_round);

    // Set the current calibration to the best pitch
    current_calibration_.laser_corrections[velodyne_rawdata::LASER_SEQUENCE[ring]].vert_correction +=
      (best_round - NUM_ROUNDS / 2) * 0.001;

  }
  current_calibration_.write(output_file_);

  // Reset the current calibraion so that the image display gets updated
  data_.reset(new velodyne_rawdata::RawData());
  ros::NodeHandle private_nh("~");
  data_->setup(private_nh);

  calibrate_ = false;
}

void drawRectangle(cv::Mat image) {
  if (rectangle_available_ || rectangle_in_process_) {
    cv::rectangle(image, cv::Point(rect_x1_, rect_y1_), 
        cv::Point(rect_x2_, rect_y2_), cv::Scalar(255));
  }
}

void processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scan_msg) {

  // Obtain pointcloud from raw packets 
  velodyne_rawdata::VPointCloud::Ptr
    cloud(new velodyne_rawdata::VPointCloud());
  for (size_t i = 0; i < scan_msg->packets.size(); ++i) {
    data_->unpack(scan_msg->packets[i], *cloud);
  }

  m_calib.lock();
  if (calibrate_) {
    performCalibration(*cloud, scan_msg);
  }
  m_calib.unlock();

  // Obtain images from pointcloud
  cv::Mat height_image;
  cv::Mat intensity_image;
  generator_.getOverheadImages(*cloud, height_image, intensity_image);
  drawRectangle(height_image);
  cv::imshow("Height Image", height_image);
}

void mouseHandler(int event, int x, int y, int flags, void *param) {
  m_calib.lock();
  switch(event) {
    /* left button down */
    case CV_EVENT_LBUTTONDOWN:
      //ROS_INFO("Left button down (%d, %d).\n", x,y);
      rectangle_available_ = false;
      rectangle_in_process_ = true;
      rect_x1_ = x;
      rect_y1_ = y;
      rect_x2_ = x;
      rect_y2_ = y;
      break;
   
    /* left button down */
    case CV_EVENT_LBUTTONUP:
      if (x != rect_x1_ && y != rect_y1_) {
        rect_x2_ = x;
        rect_y2_ = y;
        rectangle_available_ = true;
      }
      rectangle_in_process_ = false;
      break;

    /* right button down */
    case CV_EVENT_RBUTTONDOWN:
      if (rectangle_available_) {
        calibrate_ = true;
      }
      break;
   
    /* mouse move */
    case CV_EVENT_MOUSEMOVE:
      if (rectangle_in_process_) {
        rect_x2_ = x;
        rect_y2_ = y; 
      }
      break;
  }
  m_calib.unlock();
}

/**
 * \brief Receives reconfiguration requests from the dynamic reconfigure
 *        server and updates the generator
 */
void reconfigure(const velodyne_image::OverheadImageConfig& new_config, 
    uint32_t level) {
  ROS_INFO_STREAM(NODE ": Received reconfigure request level " << level);
  image_size_ = new_config.image_size;
  distance_per_pixel_ = new_config.distance_per_pixel;
  ground_height_ = new_config.mean;
  generator_.reconfigure(new_config);
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle node, private_nh("~");

  // Read default calibration 
  std::string default_calibration_file = 
    ros::package::getPath("velodyne_pointcloud") + "tests/angles.yaml";
  ros::param::get("~calibration", default_calibration_file);
  default_calibration_.read(default_calibration_file);

  // Setup the rawdata class
  data_.reset(new velodyne_rawdata::RawData());
  data_->setup(private_nh); 

  // Subscribe to VelodyneScan packets
  velodyne_scan_ =
    node.subscribe("velodyne_packets", 10, &processScan,
                   ros::TransportHints().tcpNoDelay(true));
 
  // Create the display window
  cv::namedWindow("Height Image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO |
                                  CV_GUI_NORMAL);
  cvResizeWindow("Height Image", 400, 400);
  cvSetMouseCallback("Height Image", mouseHandler, NULL);
  cvStartWindowThread();

  // Setup a dynamic reconfigure server and setup the callback
  dynamic_reconfigure::Server<velodyne_image::OverheadImageConfig> srv;
  dynamic_reconfigure::Server<velodyne_image::OverheadImageConfig>::
    CallbackType cb = boost::bind(&reconfigure, _1, _2);
  srv.setCallback(cb);

  ROS_INFO_STREAM(NODE ": starting up");
  ros::spin();                          // handle incoming data
  ROS_INFO_STREAM(NODE ": shutting down");

  // Shutdown display on exit
  cvDestroyWindow("Height Image");

  return 0;
}
