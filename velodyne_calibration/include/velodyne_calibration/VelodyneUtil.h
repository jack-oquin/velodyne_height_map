/*
 *  Copyright (C) 2010 UT Austin & Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 */

/**
 * \file
 *
 * C++ interface for converting rawscan data into a pointcloud and pointcloud2 message.
 * Also allows for calibrating the pointcloud.
 *
 * \author Piyush Khandelwal
 */

#ifndef __VELODYNE_UTIL_H__
#define __VELODYNE_UTIL_H__

#include <velodyne/data.h>
#include <velodyne/ring_sequence.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_calibrate/VelodyneFilterConfig.h>

namespace velodyne_calibrate {

  const std::string NAME_X = "x";
  const std::string NAME_Y = "y";
  const std::string NAME_Z = "z";
  const std::string NAME_RING = "ring";
  const std::string NAME_HEADING = "heading";
  const std::string NAME_INTENSITY = "intensity";
  const std::string NAME_AVAILABLE = "available";

  enum ReconfigureStatus {
    RECONFIGURE_COMPLETE = 0,
    RECONFIGURE_FILE_ERROR = 1
  };

  struct IntensityCalib {
    float a;
    float b;
    float c;
    float d;
  };

  typedef float HeightCalib;

  class VelodyneUtil {

    private:
      VelodyneFilterConfig config_;
      IntensityCalib iCalib_[velodyne::N_LASERS];
      HeightCalib hCalib_[velodyne::N_LASERS];
      float pitchCalib_;

      ReconfigureStatus getCalibrationInfo();

      void correctVelodynePitch(sensor_msgs::PointCloud * pc);
      void correctHeight(sensor_msgs::PointCloud *pc);
      void correctIntensity(sensor_msgs::PointCloud *pc);

      /* The new and improved point cloud 2 correction. Yay! */
      void calibrateVelodyneData(sensor_msgs::PointCloud2 &cloud2);

    public:

      void getPointCloud(
        const std::vector<velodyne::laserscan_xyz_t> & scan, 
        sensor_msgs::PointCloud *pc);

      void getPointCloud2(
        const std::vector<velodyne::laserscan_xyz_t> & scan,
        sensor_msgs::PointCloud2 &cloud2);

      ReconfigureStatus reconfigure(VelodyneFilterConfig &newConfig);

      void preAllocatePointCloud(sensor_msgs::PointCloud *pc);
      void preAllocatePointCloud2(sensor_msgs::PointCloud2 &cloud2);

      

  };
}

#endif
