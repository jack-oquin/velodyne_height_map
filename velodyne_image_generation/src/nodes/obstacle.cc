/**
 * \file  obstacle.cc
 * \brief Uses the distance circular image to detect obstacles around the car 
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Piyush Khandelwal
 *
 * License: Modified BSD License
 *
 * $ Id: 08/22/2011 05:36:18 PM piyushk $
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

#include "velodyne_image_generation/ImageGeneratorConfig.h"
#include "velodyne_image_generation/VelodyneImageGenerator.h"
#include "velodyne_image_generation/ImageRef.h"


#define NODE "velodyne_image"

namespace {

  int qDepth = 1;
  bool display = false;

  IplImage *gradientImage = NULL;

  // A temporary image containing the mapping of points in the
  // cloud onto the image
  IplImage *indexImage = NULL;

  velodyne_image_generation::VelodyneImageGenerator image_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher publisher_image_;
   
  sensor_msgs::PointCloud obsCloud;             // outgoing PointCloud message
  static ros::Publisher publisher_obstacle;
}

void getDistanceImage(const sensor_msgs::PointCloud &pc, IplImage *indexImage,  IplImage *img) {
  BwImage distanceImage(img);
  BwImageInt index(indexImage);

  
  size_t npoints = pc.points.size();
  obsCloud.points.resize(npoints);

  // pass along original time stamp and frame ID
  obsCloud.header.stamp = pc.header.stamp;
  obsCloud.header.frame_id = pc.header.frame_id;
  
  size_t count=0;
  
  for (unsigned int i = (unsigned int) image_.getConfig()->numLasers-1; i>1; i--) {
    if (i==51 || i==52) continue;
    for (unsigned int j = 0; j < (unsigned int) image_.getConfig()->pointsPerLaser; j++) {
      
      bool pointAvailable = 
          index[i][j] != MISSING_POINT && index[i][j] != FAILED_POINT && index[i][j] != OUT_OF_RANGE_POINT;
      bool usePreviousPoint =
          (image_.getConfig()->correctGaps && index[i][j] == MISSING_POINT) ||
          (image_.getConfig()->correctFailures && index[i][j] == FAILED_POINT);

      bool prevPointAvailable = 
          index[i-1][j] != MISSING_POINT && index[i-1][j] != FAILED_POINT && index[i-1][j] != OUT_OF_RANGE_POINT;
      if (pointAvailable && prevPointAvailable) {
        int scaledZ;
        if (i==0) {
          scaledZ=0;
        } else {
          float newX = sqrtf(pow(pc.points[index[i][j]].x,2)+pow(pc.points[index[i][j]].y,2));
          float newZ = pc.points[index[i][j]].z;
          float prevX = sqrtf(pow(pc.points[index[i-1][j]].x,2)+pow(pc.points[index[i-1][j]].y,2)); //pc.points[index[i-1][j]].x;
          float prevZ = pc.points[index[i-1][j]].z;
          float rise = (newZ - prevZ);
          float run = (newX - prevX);
          if (run==0) run = 0.000001;
          float grad = rise/run;
          if (fabs(grad)>1.0) {
            scaledZ = 255;
            obsCloud.points[count].x = pc.points[index[i][j]].x;
            obsCloud.points[count].y = pc.points[index[i][j]].y;
            obsCloud.points[count].z = pc.points[index[i][j]].z;
            count++;
          } else {
            scaledZ = 0;
          }
        }
        float z = pc.points[index[i][j]].x;;
        
        distanceImage[i][j] = scaledZ;
      } else if (!usePreviousPoint) {
        distanceImage[i][j] = 0;
      }
    }    
  }
  
  obsCloud.points.resize(count);
  
  publisher_obstacle.publish(obsCloud);
}


void processPointCloud(const sensor_msgs::PointCloud &pc) {

  image_.calculateIndices(pc, indexImage);
  getDistanceImage(pc, indexImage, gradientImage);
  if (display) {
    cvShowImage("GradientImage", gradientImage);
  }

  ROS_DEBUG(NODE ": Publishing Images");
  publisher_image_.publish(bridge_.cvToImgMsg(gradientImage));
}

/** handle dynamic reconfigure service request
 *
 * @param newConfig new configuration from dynamic reconfigure client,
 *        becomes the service reply message as updated here.
 * @param level SensorLevels value (0xffffffff on initial call)
 *
 * This is done without any locking because it is called in the same
 * main thread as ros::spinOnce() and all the topic subscription
 * call-backs. If not, we would need a lock.
 */
void reconfigure(velodyne_image_generation::ImageGeneratorConfig &newConfig, uint32_t level) {
  ROS_INFO(NODE ": Dynamic reconfigure, level 0x%x", level);
  image_.reconfigure(newConfig);

  // TODO: dont re-create if size does not change

  // Recreate Images
  if (gradientImage) {
    cvReleaseImage(&gradientImage);
    gradientImage = NULL;
  } 
  gradientImage = cvCreateImage(
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
    return 22; 
    
  // Initialize Display if required
  if (display) {
    cvNamedWindow("GradientImage", 0);
    cvResizeWindow("GradientImage", 1024, 128);
    cvStartWindowThread();    
  }

  // Declare dynamic reconfigure callback
  dynamic_reconfigure::Server<velodyne_image_generation::ImageGeneratorConfig> srv;
  dynamic_reconfigure::Server<velodyne_image_generation::ImageGeneratorConfig>::CallbackType cb =
    boost::bind(&reconfigure, _1, _2);
  srv.setCallback(cb);

  obsCloud.points.resize(99840); //velodyne::SCANS_PER_REV);
  
  // Subscribe to point cloud, and get ready to publish images
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber velodyne_scan =
    node.subscribe("velodyne/pointcloud", qDepth,
                   &processPointCloud, noDelay);

  image_transport::ImageTransport it = image_transport::ImageTransport(node);
  publisher_image_ = it.advertise("velodyne/gradientImage", qDepth);

  publisher_obstacle = node.advertise<sensor_msgs::PointCloud>("velodyne/obstacles",
                                                   qDepth);
  
  
  ROS_INFO(NODE ": starting up");

  ros::spin();                          // handle incoming data

  ROS_INFO(NODE ": shutting down");

  // Cleanup
  cvReleaseImage(&gradientImage);

  if (display) {
    cvDestroyWindow("GradientImage");
  }

  return 0;
}
