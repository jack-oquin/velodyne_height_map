/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    @brief ROS node for detecting obstacles in a point cloud.

*/

#include <ros/ros.h>
#include <velodyne_height_map/heightmap.h>

/** Main entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "heightmap_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create height map class, which subscribes to velodyne_points
  velodyne_height_map::HeightMap hm(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
