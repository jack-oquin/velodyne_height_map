/**
 * \file  circular_image.cc
 * \brief node for converting a PointCloud message from the velodyne to 
 *        circular images
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/22/2011 04:46:00 PM piyushk $
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <velodyne_image_generation/CircularImageConfig.h>
#include <velodyne_image_generation/CircularImageGenerator.h>
#include <velodyne_image_generation/ImageRef.h>

#define NODE "velodyne_image_circular"

namespace {

  int qDepth = 1;
  bool display = false;

  // The 3 outgoing images
  IplImage *heightImage = NULL;
  IplImage *intensityImage = NULL;
  IplImage *distanceImage = NULL;

  IplImage *indexImage = NULL;     ///< Temporary image to hold indices in pc

  velodyne_image_generation::CircularImageGenerator image_;

  sensor_msgs::CvBridge bridge_;

  image_transport::Publisher outputHeight_;
  image_transport::Publisher outputIntensity_;
  image_transport::Publisher outputDistance_;

}

/**
 * \brief   Processes the PointCloud message pc to generate the images
 */
void processPointCloud(const sensor_msgs::PointCloud &pc) {

  image_.calculateIndices(pc, indexImage);
  image_.getIntensityImage(pc, indexImage, intensityImage);
  image_.getGaussianHeightImage(pc, indexImage, heightImage);
  image_.getDistanceImage(pc, indexImage, distanceImage);

  if (display) {
    cvShowImage("HeightImage", heightImage);
    cvShowImage("IntensityImage", intensityImage);
    cvShowImage("DistanceImage", distanceImage);
  }

  ROS_DEBUG(NODE ": Publishing Images");
  outputHeight_.publish(bridge_.cvToImgMsg(heightImage));
  outputIntensity_.publish(bridge_.cvToImgMsg(intensityImage));
  outputDistance_.publish(bridge_.cvToImgMsg(distanceImage));
}

/** 
 * /brief handle dynamic reconfigure service request
 *
 * /param newConfig new configuration from dynamic reconfigure client,
 *        becomes the service reply message as updated here.
 * /param level SensorLevels value (0xffffffff on initial call)
 *
 * /todo don't recreate images if size does not change
 * /todo fix seg fault on changes to size
 *
 */
void reconfigure(velodyne_image_generation::CircularImageConfig &newConfig, uint32_t level) {
  ROS_INFO(NODE ": Dynamic reconfigure, level 0x%x", level);
  image_.reconfigure(newConfig);

  // Recreate Images
  if (heightImage) {
    cvReleaseImage(&heightImage);
    heightImage = NULL;
  } 
  heightImage = cvCreateImage(
      cvSize(newConfig.pointsPerLaser, newConfig.numLasers),
      IPL_DEPTH_8U, 1);

  if (intensityImage) {
    cvReleaseImage(&intensityImage);
    intensityImage = NULL;
  } 
  intensityImage = cvCreateImage(
      cvSize(newConfig.pointsPerLaser, newConfig.numLasers),
      IPL_DEPTH_8U, 1);

  if (distanceImage) {
    cvReleaseImage(&distanceImage);
    distanceImage = NULL;
  } 
  distanceImage = cvCreateImage(
      cvSize(newConfig.pointsPerLaser, newConfig.numLasers),
      IPL_DEPTH_8U, 1);

  if (indexImage) {
    cvReleaseImage(&indexImage);
    indexImage = NULL;
  } 
  indexImage = cvCreateImage(
      cvSize(newConfig.pointsPerLaser, newConfig.numLasers),
      IPL_DEPTH_32S, 1);
}

/**
 * \brief   Use getopt to parse command line flags
 */
int getParameters(int argc, char *argv[]) {
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
    return 22; 
    
  // Initialize Display if required
  if (display) {
    cvNamedWindow("HeightImage", 0);
    cvResizeWindow("HeightImage", 1024, 128);
    cvMoveWindow("HeightImage", 0, 25);
    cvNamedWindow("IntensityImage", 0);
    cvResizeWindow("IntensityImage", 1024, 128);
    cvMoveWindow("IntensityImage", 0, 185);
    cvNamedWindow("DistanceImage", 0);
    cvResizeWindow("DistanceImage", 1024, 128);
    cvMoveWindow("DistanceImage", 0, 345);
    cvStartWindowThread();    
  }

  // Declare dynamic reconfigure callback
  dynamic_reconfigure::Server<velodyne_image_generation::CircularImageConfig> srv;
  dynamic_reconfigure::Server<velodyne_image_generation::CircularImageConfig>::CallbackType cb =
    boost::bind(&reconfigure, _1, _2);
  srv.setCallback(cb);

  // Subscribe to point cloud, and get ready to publish images
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber velodyne_scan =
    node.subscribe("velodyne/pointcloud", qDepth,
                   &processPointCloud, noDelay);

  image_transport::ImageTransport it = image_transport::ImageTransport(node);
  outputHeight_ = it.advertise("velodyne/heightImage", qDepth);
  outputIntensity_ = it.advertise("velodyne/intensityImage", qDepth);
  outputDistance_ = it.advertise("velodyne/distanceImage", qDepth);

  ROS_INFO(NODE ": starting up");

  ros::spin();                          // handle incoming data

  ROS_INFO(NODE ": shutting down");

  // Cleanup
  cvReleaseImage(&heightImage);
  cvReleaseImage(&intensityImage);
  cvReleaseImage(&indexImage);

  if (display) {
    cvDestroyWindow("HeightImage");
    cvDestroyWindow("IntensityImage");
  }

  return 0;
}
