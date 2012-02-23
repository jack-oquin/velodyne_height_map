/*
 *  Copyright (C) 2010 UT-Austin & Austin Robot Technology,
 *  David Claridge, Michael Quinlan, Jack O'Quin
 *  Copyright (C) 2012 Jack O'Quin
 * 
 *  License: Modified BSD Software License 
 */

/** @file

   This ROS nodelet produces a point cloud containing all points that
   lie on an obstacle larger than HEIGHT_DIFF_THRESHOLD in height.

Subscribes:

- @b velodyne/pointcloud [sensor_msgs/PointCloud] raw data from one
     revolution of the Velodyne transformed to /odom frame of reference

Publishes:

- @b veloydne/obstacles [sensor_msgs::PointCloud] points that lie on an obstacle
- @b veloydne/clear [sensor_msgs::PointCloud] points are clear of obstackes


@author David Claridge, Michael Quinlan, Jack O'Quin
*/

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <velodyne_height_map/heightmap.h>

namespace velodyne_height_map {

  class HeightMapNodelet: public nodelet::Nodelet
  {
  public:

    HeightMapNodelet() {}
    ~HeightMapNodelet() {}

    void onInit(void)
    {
      heightmap_.reset(new HeightMap(getNodeHandle(), getPrivateNodeHandle()));
    }

  private:

    boost::shared_ptr<HeightMap> heightmap_;
  };

} // namespace velodyne_height_map

// Register this plugin with pluginlib.  Names must match height_map_nodelet.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_height_map, HeightMapNodelet,
                        velodyne_height_map::HeightMapNodelet, nodelet::Nodelet);
