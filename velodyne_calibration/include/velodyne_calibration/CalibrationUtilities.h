/**
 * \file  CalibrationUtilities.h
 * \brief  
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:23:39 AM piyushk $
 */


#ifndef CALIBRATIONUTILITIES_PQBF1LWD
#define CALIBRATIONUTILITIES_PQBF1LWD

#include <string>
#include <pcl/pcl.h>

#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_calibration/Calibration.h>

namespace velodyne {

  template <typename PointT>
  void scanToPointCloud(const velodyne_msgs::VelodyneScan& velodyne_scan, 
      pcl::PointCloud<PointT>& cloud);

  template <typename PointT>
  void scanToPointCloud(const velodyne_msgs::VelodyneScan& velodyne_scan, 
      const velodyne::Calibration& calibration, pcl::PointCloud<PointT>& cloud);

  void getDefaultCalibration(velodyne::Calibration& calibration);
  void readCalibrationFromFile(const std::string& calibration_file, 
      velodyne::Calibration& calibration);
  void writeCalibrationToFile(const std::string& calibration_file, 
      const velodyne::Calibration& calibration);

}

#endif /* end of include guard: CALIBRATIONUTILITIES_PQBF1LWD */
