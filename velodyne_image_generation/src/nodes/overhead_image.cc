/**
 * \file  overhead_image.cc
 * \brief node for converting a PointCloud2 message into overhead images 
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/22/2011 04:55:11 PM piyushk $
 *
 * \todo modify odometry subscription to be optional
 * \todo paremterize the odometry topic name to be passed in as an argument
 */

#include <iomanip>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>
#include <boost/thread/mutex.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include "velodyne_image_generation/OverheadImageConfig.h"
#include "velodyne_image_generation/OverheadImageGenerator.h"
#include "velodyne_image_generation/ImageRef.h"

#define NODE "velodyne_image_overhead"

using namespace velodyne_image_generation;

namespace {

  int qDepth = 1;
  bool display = false;

  // The 2 outgoing images
  cv_bridge::CvImage heightImage;
  cv_bridge::CvImage intensityImage;

  OverheadImageGenerator image_;
  image_transport::Publisher outputHeight_;
  image_transport::Publisher outputIntensity_;

  boost::mutex mGenerator;
  boost::mutex mOdom;

  const float pixelResolution = 0.25;      ///< meters per pixel

  bool firstOdom_ = true;
  double prevOdomTime_;

  double xDiff_ = 0;
  double orientationDiff_ = 0;
  cv::Mat transformation_(3, 3, CV_64F);   ///< transformation matrix used to keep changes in odometry

  int pixelSize_;                          ///< current size of image in pixels

}

/**
 * \brief   Compute transformation for the image given the change in position
 *          and orientation
 */
cv::Mat getOdometryTransformation(double &xDiff, double &orientationDiff, int rows, int cols) {

  cv::Point2f center(rows / 2, cols / 2);

  // convert meters into pixels
  int xPxl = floor(xDiff / pixelResolution);
  xDiff -= xPxl * pixelResolution;

  // Calculate Affine Transformation
  cv::Mat translation = cv::Mat::eye(3, 3, CV_64F);
  translation.at<double>(1,2) = xPxl;

  cv::Mat rotation = cv::getRotationMatrix2D(center, -orientationDiff, 1);
  orientationDiff = 0;
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

  cv::Mat tempImage;
  image.copyTo(tempImage);
  cv::Size imageSize(image.rows, image.cols);
  cv::warpAffine(tempImage, image, transformation, imageSize); 

}

/**
 * \brief  Process PointCloud2 message to generate the images
 */
void processPointCloud(const sensor_msgs::PointCloud2 &cloud2) {

  heightImage.header = cloud2.header;
  intensityImage.header = cloud2.header;

  mGenerator.lock();

  mOdom.lock();
  // Shift the image based on the affine transformation
  transformation_.resize(2);
  shiftImageByOdometry(heightImage.image, transformation_);
  shiftImageByOdometry(intensityImage.image, transformation_);
  transformation_ = cv::Mat::eye(3,3,CV_64F);
  mOdom.unlock();
  
  // Generate new images (using the old ones)
  image_.getOverheadImage(cloud2, heightImage.image, intensityImage.image, pixelResolution);
  mGenerator.unlock();

  heightImage.encoding = intensityImage.encoding = "mono8";

  if (display) {
    cv::imshow("HeightImage", heightImage.image);
    cv::imshow("IntensityImage", intensityImage.image);
  }

  ROS_DEBUG(NODE ": Publishing Images");
  outputHeight_.publish(heightImage.toImageMsg());
  outputIntensity_.publish(intensityImage.toImageMsg());
}

/**
 * \brief  Reinitialize images to given size 
 */
void createImages(int pixelSize) {
  heightImage.image = cv::Mat::zeros(pixelSize, pixelSize, CV_8U);
  intensityImage.image = cv::Mat::zeros(pixelSize, pixelSize, CV_8U);
}

/**
 * \brief Process odometry to compute affine transformation in previously
 *        calculated images
 */
void processOdom(const nav_msgs::Odometry::ConstPtr &odomIn) {

  if (firstOdom_) {
    ros::Time time = odomIn->header.stamp;
    prevOdomTime_ = time.toSec();
    if (prevOdomTime_ == 0.0) {    // Some times we get a bad timestamp
      return;
    }
    firstOdom_ = false;
    return;
  }

  ros::Time time = odomIn->header.stamp;
  double currTime = time.toSec();
  if (currTime == 0.0) {
    return;
  }

  mOdom.lock();
  xDiff_ += odomIn->twist.twist.linear.x * (currTime - prevOdomTime_); 
  orientationDiff_ += odomIn->twist.twist.angular.z * (currTime - prevOdomTime_);

  // Compute new transformation and multiply it into existing transformation
  transformation_ = getOdometryTransformation(xDiff_, orientationDiff_, pixelSize_, pixelSize_) * transformation_; 
  mOdom.unlock();
  
  prevOdomTime_ = currTime;

}

/** 
 * \brief handle dynamic reconfigure service request
 *
 * \param newConfig new configuration from dynamic reconfigure client,
 *        becomes the service reply message as updated here.
 * \param level SensorLevels value (0xffffffff on initial call)
 *
 */
void reconfigure(velodyne_image_generation::OverheadImageConfig &newConfig, uint32_t level) {
  ROS_INFO(NODE ": Dynamic reconfigure, level 0x%x", level);

  bool recreateImages = true;
  if (image_.getConfig()->pixelSize == newConfig.pixelSize) 
    recreateImages = false;
  image_.reconfigure(newConfig);

  if (recreateImages) {
    mGenerator.lock();
    createImages(newConfig.pixelSize);
    pixelSize_ = newConfig.pixelSize;
    mGenerator.unlock();
  }

}

/**
 * \brief   Use getopt to parse command line flags
 */
int getParameters(int argc, char *argv[]) {
  // use getopt to parse the flags
  char ch;
  const char* optflags = "q:d";
  while(-1 != (ch = getopt(argc, argv, optflags))) {
    switch(ch) {

      case 'q':
        qDepth = atoi(optarg);
        if (qDepth < 1) {
          qDepth = 1;
        }
        break;

      case 'd':
        display = true;
        break;
    }
  }

  return 1;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  if (!getParameters(argc, argv))
    return -1; 
    
  // Initialize Display if required
  if (display) {

    cv::namedWindow("HeightImage", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    cvResizeWindow("HeightImage", 400, 400);
    cvMoveWindow("HeightImage", 0, 25);

    cv::namedWindow("IntensityImage", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL);
    cvResizeWindow("IntensityImage", 400, 400);
    cvMoveWindow("IntensityImage", 500, 25);

    cvStartWindowThread();    
  }

  // Declare dynamic reconfigure callback
  dynamic_reconfigure::Server<velodyne_image_generation::OverheadImageConfig> srv;
  dynamic_reconfigure::Server<velodyne_image_generation::OverheadImageConfig>::CallbackType cb =
    boost::bind(&reconfigure, _1, _2);
  srv.setCallback(cb);

  // Subscribe to point cloud, and get ready to publish images
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber velodyneScan =
    node.subscribe("velodyne/pointcloud2", qDepth,
                   &processPointCloud, noDelay);

  ros::Subscriber odomState = node.subscribe("odom", qDepth, processOdom, noDelay);

  image_transport::ImageTransport it = image_transport::ImageTransport(node);
  outputHeight_ = it.advertise("velodyne/heightImage", qDepth);
  outputIntensity_ = it.advertise("velodyne/intensityImage", qDepth);

  // Create the images before starting ros spin
  createImages(image_.getConfig()->pixelSize);

  ROS_INFO(NODE ": starting up");
  
  ros::spin();                          // handle incoming data

  ROS_INFO(NODE ": shutting down");

  if (display) {
    cvDestroyWindow("HeightImage");
    cvDestroyWindow("IntensityImage");
  }

  return 0;
}
