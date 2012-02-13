/*
 *  Copyright (C) 2010 Austin Robot Technology & UT Austin
 *  License: Modified BSD Software License Agreement
 */

/** \file

    This ROS node converts raw Velodyne HDL-64E 3D LIDAR data to a 
    PointCloud. If provided with calibration information, will 
    calibrate the data as well

*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/SensorLevels.h>

#include <velodyne/data.h>

#include "velodyne_calibrate/VelodyneFilterConfig.h"
#include "velodyne_calibrate/VelodyneUtil.h"

#define NODE "velodyne_calibrate_cloud"

using namespace velodyne_common;

namespace {

  // command options
  int qDepth = 1;                      // ROS topic queue size

  // local static data
  velodyne::DataXYZ *data = NULL;
  ros::Publisher output;

  sensor_msgs::PointCloud pc;           // outgoing PointCloud message

  velodyne_calibrate::VelodyneUtil velodyne_;

}

/** \brief callback for XYZ points
 *
 * publishes Velodyne data points as a point cloud
 */
void processXYZ(const std::vector<velodyne::laserscan_xyz_t> &scan) {

  // pass along original time stamp and frame ID
  data->getMsgHeaderFields(pc.header.stamp, pc.header.frame_id);

  size_t numPoints = scan.size();
  velodyne_.getPointCloud(scan, &pc);
  ROS_DEBUG(NODE ": Publishing %u Velodyne points.", numPoints);
  output.publish(pc);
}

/** handle dynamic reconfigure service request
 *
 * @param newconfig new configuration from dynamic reconfigure client,
 *        becomes the service reply message as updated here.
 * @param level SensorLevels value (0xffffffff on initial call)
 *
 * This is done without any locking because it is called in the same
 * main thread as ros::spinOnce() and all the topic subscription
 * call-backs. If not, we would need a lock.
 */
void reconfigure(velodyne_calibrate::VelodyneFilterConfig &newConfig, uint32_t level) {
  ROS_INFO(NODE ": Dynamic reconfigure, level 0x%x", level);
  velodyne_calibrate::ReconfigureStatus status = velodyne_.reconfigure(newConfig);
  if (status == velodyne_calibrate::RECONFIGURE_FILE_ERROR) {
    ROS_ERROR(NODE ": Unable to find calibration file: %s", newConfig.calibrationFile.c_str());
  }
}

int getParameters(const ros::NodeHandle &node) {
  node.param("qDepth", qDepth, 1);
  return 1;
}

int main(int argc, char *argv[]) {

  ros::init(argc, argv, NODE);
  ros::NodeHandle node;

  if (!getParameters(node))
    return -1; 

  // Declare dynamic reconfigure callback
  dynamic_reconfigure::Server<velodyne_calibrate::VelodyneFilterConfig> srv;
  dynamic_reconfigure::Server<velodyne_calibrate::VelodyneFilterConfig>::CallbackType cb =
      boost::bind(&reconfigure, _1, _2);
  srv.setCallback(cb);

  // setup data
  data = new velodyne::DataXYZ();
  data->getParams();
  data->subscribeXYZ(processXYZ);
  if (0 != data->setup())
    return 2;

  // Subscribe to velodyne input.
  ros::TransportHints noDelay = ros::TransportHints().tcpNoDelay(true);
  ros::Subscriber velodyne_scan = node.subscribe("velodyne/rawscan", qDepth,
          &velodyne::Data::processRawScan, (velodyne::Data *) data, noDelay);

  output = node.advertise<sensor_msgs::PointCloud>("velodyne/pointcloud", qDepth);

  velodyne_.preAllocatePointCloud(&pc);

  ros::spin();                          // handle incoming data

  data->shutdown();
  delete data;

  return 0;
}
