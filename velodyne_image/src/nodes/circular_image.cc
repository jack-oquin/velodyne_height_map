/**
 * \file  circular_image.cc
 * \brief Node for converting a PointCloud message from the velodyne to 
 *        circular images
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2011, UT Austin, Austin Robot Technology
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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>

#include <velodyne_image/circular_image_generator.h>
#include <velodyne_image/CircularImageConfig.h>

#define NODE "velodyne_circular_image"

namespace {

  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  int q_depth_ = 1;
  bool display_ = false;

  velodyne_image::CircularImageGenerator generator_;

  cv::Mat height_image_;
  cv::Mat intensity_image_;
  image_transport::Publisher height_publisher_;
  image_transport::Publisher intensity_publisher_;

}

/**
 * \brief Processes the PointCloud message to generate images
 */
void processPointCloud(const VPointCloud::ConstPtr& cloud) {

  generator_.getCircularImages(*cloud, height_image_, intensity_image_);

  if (display_) {
    cv::imshow("Height Image", height_image_);
    cv::imshow("Intensity Image", intensity_image_);
  }

  ROS_DEBUG_STREAM(NODE ": Publishing Images");

  // Publish height image
  cv_bridge::CvImage height_msg;
  height_msg.header = cloud->header;
  height_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  height_msg.image = height_image_; // Does not copy image data
  height_publisher_.publish(height_msg.toImageMsg());

  // Publish intensity image
  cv_bridge::CvImage intensity_msg;
  intensity_msg.header = cloud->header;
  intensity_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  intensity_msg.image = intensity_image_; // Does not copy image data
  intensity_publisher_.publish(intensity_msg.toImageMsg());

}

/**
 * \brief Receives reconfiguration requests from the dynamic reconfigure
 *        server and updates the generator
 */
void reconfigure(const velodyne_image::CircularImageConfig& new_config, 
    uint32_t level) {
  ROS_INFO_STREAM("Received reconfigure request: " << level);
  generator_.reconfigure(new_config);
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  ros::param::get("~display", display_);

  // Initialize display if required
  if (display_) {
    cv::namedWindow("Height Image", 0);
    cvResizeWindow("Height Image", 512, 64);
    cv::namedWindow("Intensity Image", 0);
    cvResizeWindow("Intensity Image", 512, 64);
    cvStartWindowThread();    
  }

  // Setup a dynamic reconfigure server and setup the callback
  dynamic_reconfigure::Server<velodyne_image::CircularImageConfig> srv;
  dynamic_reconfigure::Server<velodyne_image::CircularImageConfig>::
    CallbackType cb = boost::bind(&reconfigure, _1, _2);
  srv.setCallback(cb);

  // Subscribe to point cloud
  ros::Subscriber cloud_subscriber = 
    node.subscribe<VPointCloud>("velodyne_points", q_depth_, processPointCloud);

  // Advertise images
  image_transport::ImageTransport it = image_transport::ImageTransport(node);
  height_publisher_ = 
    it.advertise("velodyne_height/circular/image_raw", q_depth_);
  intensity_publisher_ = 
    it.advertise("velodyne_intensity/circular/image_raw", q_depth_);

  ROS_INFO_STREAM(NODE ": starting up");
  ros::spin();                          // handle incoming data
  ROS_INFO_STREAM(NODE ": shutting down");

  // Shutdown displays on exit
  if (display_) {
    cvDestroyWindow("Height Image");
    cvDestroyWindow("Intensity Image");
  }

  return 0;
}
