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
#include <pcl/point_cloud.h>

#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_calibration/Calibration.h>

namespace velodyne {

  /**
   * Raw Velodyne packet constants and structures.
   */
  static const int SIZE_BLOCK = 100;
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

  static const float ROTATION_RESOLUTION = 0.01f; /**< degrees */
  static const float ROTATION_MAX_UNITS = 36000; /**< hundredths of degrees */

  /** According to Bruce Hall DISTANCE_MAX is 65.0, but we noticed
   *  valid packets with readings up to 130.0. */
  static const float DISTANCE_MAX = 130.0f;        /**< meters */
  static const float DISTANCE_RESOLUTION = 0.002f; /**< meters */
  static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX
                                           / DISTANCE_RESOLUTION + 1.0);
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;

  /** \brief Raw Velodyne data block.
   *
   *  Each block contains data from either the upper or lower laser
   *  bank.  The device returns three times as many upper bank blocks.
   *
   *  use stdint.h types, so things work with both 64 and 32-bit machines
   */
  typedef struct raw_block
  {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE];
  } raw_block_t;

  // /** used for unpacking the first two data bytes in a block
  //  *
  //  *  They are packed into the actual data stream misaligned.  I doubt
  //  *  this works on big endian machines.
  //  */
  // union two_bytes
  // {
  //   uint16_t uint;
  //   uint8_t  bytes[2];
  // };

  static const int PACKET_SIZE = 1206;
  static const int BLOCKS_PER_PACKET = 12;
  static const int PACKET_STATUS_SIZE = 4;
  static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);
  static const int PACKETS_PER_REV = 260;
  static const int SCANS_PER_REV = (SCANS_PER_PACKET * PACKETS_PER_REV);

  /** \brief Raw Velodyne packet.
   *
   *  revolution is described in the device manual as incrementing
   *    (mod 65536) for each physical turn of the device.  Our device
   *    seems to alternate between two different values every third
   *    packet.  One value increases, the other decreases.
   *
   *  \todo figure out if revolution is only present for one of the
   *  two types of status fields
   *
   *  status has either a temperature encoding or the microcode level
   */
  typedef struct raw_packet
  {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE]; 
  } raw_packet_t;

  /** \brief A single laser scan in Velodyne's frame of reference.
   *
   *   pitch is relative to the plane of the unit (in its frame of
   *   reference): positive is above, negative is below
   *
   *   heading is relative to the front of the unit (the outlet is the back):
   *   positive is clockwise  (yaw)
   */
  typedef struct laserscan
  {
    //float range;                        ///< in meters
    //float heading;                      ///< in radians
    //float pitch;                        ///< in radians
    float x;
    float y;
    float z;
    uint16_t revolution;
    uint8_t  laser_number;
    uint8_t  intensity;
  } laserscan_t;

  template <typename PointT>
  void scanToPointCloud(const velodyne_msgs::VelodyneScan& velodyne_scan, 
      const velodyne::Calibration& calibration, pcl::PointCloud<PointT>& cloud);

  void readCalibrationFromFile(const std::string& calibration_file, 
      velodyne::Calibration& calibration);
  void writeCalibrationToFile(const std::string& calibration_file, 
      const velodyne::Calibration& calibration);

  void writeTestCalibrationFile(const std::string& test_calibration_file);

}

#endif /* end of include guard: CALIBRATIONUTILITIES_PQBF1LWD */
