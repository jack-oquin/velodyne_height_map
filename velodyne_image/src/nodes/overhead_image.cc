/**
 * \file  overhead_image.cc
 * \brief node for converting a PointCloud message into overhead images 
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2011, UT Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/22/2011 04:55:11 PM piyushk $
 */

#include <iomanip>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>

#include <velodyne_image/overhead_image_generator.h>
#include <velodyne_image/common.h>
#include <velodyne_image/OverheadImageConfig.h>

#define NODE "velodyne_overhead_image"

namespace {

  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  int q_depth_ = 1;
  bool display_ = false;

  // The 2 outgoing images
  cv::Mat height_image_;
  cv::Mat intensity_image_;
  cv::Mat disparity_image_;

  velodyne_image::OverheadImageGenerator generator_;
  image_transport::Publisher height_publisher_;
  image_transport::Publisher intensity_publisher_;

  // Variables used to keep track of odometry updates
  bool first_odom_msg_ = true;
  double prev_odom_time_;
  double x_diff_ = 0;
  double orientation_diff_ = 0;
  cv::Mat transformation_(3, 3, CV_64F);   ///< transformation matrix for odom

  int image_size_;                         ///< current size of image in pixels
  float distance_per_pixel_;               ///< current resolution of the image

  float mean_;
  float sigma_;
  float disparity_mean_;
  float disparity_sigma_;

  bool generate_snapshots_ = false;
  std::string snapshot_prefix_;

}

/**
 * \brief   Compute transformation for the image given the change in position
 *          and orientation. This transformation formulation does not work 
 *          extremely well while turning, but other alternatives are unavailable
 *
 */
cv::Mat getOdometryTransformation(double &x_diff, double &orientation_diff, 
    int rows, int cols) {

  cv::Point2f center(rows / 2, cols / 2);

  // Convert meters into pixels, and only remove as many pixels
  int x_pxl = floor(x_diff / distance_per_pixel_);
  x_diff -= x_pxl * distance_per_pixel_;

  // Calculate Affine Transformation
  cv::Mat translation = cv::Mat::eye(3, 3, CV_64F);
  translation.at<double>(1,2) = x_pxl;

  cv::Mat rotation = cv::getRotationMatrix2D(center, -orientation_diff, 1);
  orientation_diff = 0;
  rotation.resize(3);
  rotation.at<double>(2,0) = 0;
  rotation.at<double>(2,1) = 0;
  rotation.at<double>(2,2) = 1;

  cv::Mat transformation = rotation * translation;

  return transformation;
}

/**
 * \brief  Apply affine transformation on given image
 */
void shiftImageByOdometry(cv::Mat &image, const cv::Mat &transformation) {
  cv::Mat temp_image;
  image.copyTo(temp_image);
  cv::warpAffine(temp_image, image, transformation, image.size()); 
}

/**
 * \brief  Process PointCloud2 message to generate the images
 */
void processPointCloud(const VPointCloud::ConstPtr& cloud) {

  // Shift the image based on the affine transformation
  transformation_.resize(2);
  if (height_image_.data != NULL)
    shiftImageByOdometry(height_image_, transformation_);
  // if (intensity_image_.data != NULL)
  //   shiftImageByOdometry(intensity_image_, transformation_);
  if (disparity_image_.data != NULL)
    shiftImageByOdometry(disparity_image_, transformation_);
  transformation_ = cv::Mat::eye(3,3,CV_64F);

  // Shift the image based on the affine transformation
  // Check if snapshots need to be saved
  if (generate_snapshots_) {
    generator_.generateSnapshots(snapshot_prefix_, *cloud, height_image_);
    generate_snapshots_ = false;
  }
  
  // Generate new images (using the old ones)
  generator_.getOverheadImages(*cloud, height_image_, disparity_image_, 
      intensity_image_);

  if (display_) {
    cv::Mat display_height;
    velodyne_image::enhanceContrast(height_image_, display_height, 
        mean_, sigma_, false);
    cv::imshow("Height Image", display_height);
    //cv::imshow("Intensity Image", intensity_image_);
    cv::imshow("Disparity Image", disparity_image_);
  }

  ROS_DEBUG_STREAM(NODE ": Publishing Images");
  // Publish height image
  cv_bridge::CvImage height_msg;
  height_msg.header = cloud->header;
  height_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  //height_msg.image = height_image_; // Does not copy image data
  height_msg.image = disparity_image_;
  height_publisher_.publish(height_msg.toImageMsg());

  // Publish intensity image
  // cv_bridge::CvImage intensity_msg;
  // intensity_msg.header = cloud->header;
  // intensity_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  // intensity_msg.image = intensity_image_; // Does not copy image data
  // intensity_publisher_.publish(intensity_msg.toImageMsg());

}

/**
 * \brief Process odometry to compute affine transformation in previously
 *        calculated images
 */
void processOdom(const nav_msgs::Odometry::ConstPtr &odom) {

  if (first_odom_msg_) {
    ros::Time time = odom->header.stamp;
    prev_odom_time_ = time.toSec();
    if (prev_odom_time_ == 0.0) { // Some times we get a bad timestamp
      return;
    }
    first_odom_msg_ = false;
    return;
  }

  ros::Time time = odom->header.stamp;
  double curr_time = time.toSec();
  if (curr_time == 0.0) {
    return;
  }
  double time_diff = curr_time - prev_odom_time_;

  x_diff_ += odom->twist.twist.linear.x * time_diff; 
  orientation_diff_ += odom->twist.twist.angular.z * time_diff;

  // Compute new transformation and multiply it into existing transformation
  transformation_ = getOdometryTransformation(x_diff_, orientation_diff_, 
      image_size_, image_size_) * transformation_; 
  
  prev_odom_time_ = curr_time;
}

void processSnapshotRequest(const std_msgs::String::ConstPtr &snapshot_req) {
  generate_snapshots_ = true;
  snapshot_prefix_ = snapshot_req->data; 
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
  mean_ = new_config.mean;
  sigma_ = new_config.sigma;
  disparity_mean_ = new_config.disparity_mean;
  disparity_sigma_ = new_config.disparity_sigma;
  generator_.reconfigure(new_config);
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  ros::param::get("~display", display_);
  ros::param::get("~q_depth", q_depth_);
    
  // Initialize display if required
  if (display_) {
    cv::namedWindow("Height Image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO |
                                    CV_GUI_NORMAL);
    cvResizeWindow("Height Image", 400, 400);
    // cv::namedWindow("Intensity Image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO |
    //                                    CV_GUI_NORMAL);
    // cvResizeWindow("Intensity Image", 400, 400);
    cv::namedWindow("Disparity Image", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO |
                                       CV_GUI_NORMAL);
    cvResizeWindow("Disparity Image", 400, 400);
    cvStartWindowThread();    
  }

  // Setup a dynamic reconfigure server and setup the callback
  dynamic_reconfigure::Server<velodyne_image::OverheadImageConfig> srv;
  dynamic_reconfigure::Server<velodyne_image::OverheadImageConfig>::
    CallbackType cb = boost::bind(&reconfigure, _1, _2);
  srv.setCallback(cb);

  // Subscribe to point cloud
  ros::Subscriber cloud_subscriber = 
    node.subscribe<VPointCloud>("velodyne_points", q_depth_, processPointCloud);

  // Subscribe to snapshot generation request
  ros::Subscriber snapshot_request_subscriber = 
    node.subscribe("generate_snapshots", q_depth_, processSnapshotRequest);

  // Subscribe to odometry (if available) to update image
  ros::Subscriber odom_subscriber = 
    node.subscribe("odom", q_depth_, processOdom);

  // Advertise images
  image_transport::ImageTransport it = image_transport::ImageTransport(node);
  height_publisher_ = 
    it.advertise("velodyne_height/overhead/image_raw", q_depth_);
  // intensity_publisher_ = 
  //   it.advertise("velodyne_intensity/overhead/image_raw", q_depth_);

  ROS_INFO_STREAM(NODE ": starting up");
  ros::spin();                          // handle incoming data
  ROS_INFO_STREAM(NODE ": shutting down");

  // Shutdown displays on exit
  if (display_) {
    cvDestroyWindow("Height Image");
    //cvDestroyWindow("Intensity Image");
    cvDestroyWindow("Disparity Image");
  }

  return 0;
}
