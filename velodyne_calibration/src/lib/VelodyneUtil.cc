/*
 *  Copyright (C) 2010 UT Austin & Austin Robot Technology
 *  License: Modified BSD Software License Agreement
 */

/**  
 *  \file
 *
 *  C++ class for converting rawscan data into a pointcloud.
 *  If the class is instantiated with a file containing calibration parameters,
 *  can be used to generate the calibrated scan as well
 *
 *  \author Piyush Khandelwal
 *
 */

#include <fstream>
#include <velodyne_calibrate/VelodyneUtil.h>

#define JUNK_DISTANCE 2.0

namespace velodyne_calibrate {

  /** 
   * Converts Velodyne data points as a point cloud
   * Assumes point cloud is of the correct size for efficiency
   */
  void VelodyneUtil::getPointCloud(
      const std::vector<velodyne::laserscan_xyz_t> &scan,
      sensor_msgs::PointCloud *pc) {

    size_t numPoints = scan.size();

    for (unsigned i = 0; i < numPoints; i++) {
    
      geometry_msgs::Point32 pt;      
      pt.x = scan[i].x;
      pt.y = scan[i].y;
      pt.z = scan[i].z;

      // Mark all failed points
      bool available = true;
      if (fabsf(pt.x) < JUNK_DISTANCE && fabsf(pt.y) < JUNK_DISTANCE) {
        available = false;
      } 
      int intensity = scan[i].intensity;
      int ring = velodyne::LASER_RING[scan[i].laser_number];
      float heading = scan[i].heading;

      // Add this point to the output
      pc->points[i] = pt;

      // Publish ring number for data analysis
      pc->channels[0].values[i] = (float) ring;
      pc->channels[1].values[i] = heading;
      pc->channels[2].values[i] = (float) intensity;
      pc->channels[3].values[i] = (float) available;
    }

    // Do calibration if necessary
    if (config_.calibrateCloud) {
      correctVelodynePitch(pc);
      correctHeight(pc);
      correctIntensity(pc);
    }

  }

  /** 
   * Converts Velodyne data points as a point cloud
   * Assumes point cloud is of the correct size for efficiency
   */
  void VelodyneUtil::getPointCloud2(
      const std::vector<velodyne::laserscan_xyz_t> &scan,
      sensor_msgs::PointCloud2 &cloud2) {

    for (unsigned i = 0; i < scan.size() && i < cloud2.width * cloud2.height; i++) {

      int offsetToInt = 4 * 4;
      float* ptDataFloat = reinterpret_cast<float*>(&(cloud2.data[0]) + i * cloud2.point_step);
      uint8_t* ptDataInt = reinterpret_cast<uint8_t*>(&(cloud2.data[0]) + i * cloud2.point_step + offsetToInt);

      ptDataFloat[0] = scan[i].x;
      ptDataFloat[1] = scan[i].y;
      ptDataFloat[2] = scan[i].z;
    
      // Mark all failed points 
      bool available = true;
      if (fabsf(scan[i].x) < JUNK_DISTANCE && fabsf(scan[i].y) < JUNK_DISTANCE) {
        available = false;
      }
      
      uint8_t intensity = scan[i].intensity;
      uint8_t ring = velodyne::LASER_RING[scan[i].laser_number];
      float heading = scan[i].heading;

      ptDataFloat[3] = heading;

      ptDataInt[0] = ring;
      ptDataInt[1] = intensity;
      ptDataInt[2] = (uint8_t) available;

    }

    // Do calibration if necessary
    if (config_.calibrateCloud) {
      calibrateVelodyneData(cloud2);
    }

  }

  void VelodyneUtil::preAllocatePointCloud(sensor_msgs::PointCloud *pc) {

    // preallocate the anticipated amount of space for the point cloud
    pc->points.resize(velodyne::SCANS_PER_REV);
    pc->channels.resize(4);
    pc->channels[0].name = NAME_RING;
    pc->channels[0].values.resize(velodyne::SCANS_PER_REV);
    pc->channels[1].name = NAME_HEADING;
    pc->channels[1].values.resize(velodyne::SCANS_PER_REV);
    pc->channels[2].name = NAME_INTENSITY;
    pc->channels[2].values.resize(velodyne::SCANS_PER_REV);
    pc->channels[3].name = NAME_AVAILABLE;
    pc->channels[3].values.resize(velodyne::SCANS_PER_REV);

  }

  void VelodyneUtil::preAllocatePointCloud2(sensor_msgs::PointCloud2 &cloud2) {

    cloud2.height = 1;
    cloud2.width = velodyne::SCANS_PER_REV; // Unordered point cloud

    cloud2.fields.resize (7);
    cloud2.fields[0].name = NAME_X;
    cloud2.fields[1].name = NAME_Y;
    cloud2.fields[2].name = NAME_Z;
    cloud2.fields[3].name = NAME_HEADING;
    cloud2.fields[4].name = NAME_RING;
    cloud2.fields[5].name = NAME_INTENSITY;
    cloud2.fields[6].name = NAME_AVAILABLE;

    // Set all the fields types accordingly
    int offset = 0;

    // float values
    for (size_t s = 0; s < 4; s++, offset += 4) {
      cloud2.fields[s].offset   = offset;
      cloud2.fields[s].count    = 1;
      cloud2.fields[s].datatype = sensor_msgs::PointField::FLOAT32;
    }
    // uchar values
    for (size_t s = 4; s < 7; s++, offset += 1) {
      cloud2.fields[s].offset   = offset;
      cloud2.fields[s].count    = 1;
      cloud2.fields[s].datatype = sensor_msgs::PointField::UINT8;
    }
    cloud2.point_step = offset;
    cloud2.row_step   = cloud2.point_step * cloud2.width;
    cloud2.data.resize (cloud2.row_step * cloud2.height);
    cloud2.is_dense = true;  

  }

  ReconfigureStatus VelodyneUtil::reconfigure(VelodyneFilterConfig &newConfig) {
    config_ = newConfig;
    ReconfigureStatus status = getCalibrationInfo();
    return status;
  }

  ReconfigureStatus VelodyneUtil::getCalibrationInfo() {

    if (config_.calibrateCloud) {

      // TODO(piyushk): eventually move this to a calibration file handler class

      // Intensity Calibration

      std::ifstream calibFile;
      calibFile.open(config_.calibrationFile.c_str());
      if (!calibFile) {
        config_.calibrateCloud = false;
        return RECONFIGURE_FILE_ERROR;
      }

      for (int i = 0; i < velodyne::N_LASERS; i++) {
        iCalib_[i].a = iCalib_[i].c = iCalib_[i].d = 0;
        iCalib_[i].b = 1;
      }

      int id;
      double a, b, c, d;
      calibFile >> id >> a >> b >> c >> d;
      while(!calibFile.eof() && id != -1) {
        iCalib_[id].a = a;
        iCalib_[id].b = b;
        iCalib_[id].c = c;
        iCalib_[id].d = d;
        calibFile >> id >> a >> b >> c >> d;
      }

      for (int i = 0; i < velodyne::N_LASERS; i++) {
        hCalib_[i] = 0;
      }

      calibFile >> pitchCalib_;

      calibFile >> id >> a;
      while(!calibFile.eof() && id != -1) {
        hCalib_[id] = a;
        calibFile >> id >> a;
      }

      calibFile.close();
    }

    return RECONFIGURE_COMPLETE;

  }

  void VelodyneUtil::correctVelodynePitch (
      sensor_msgs::PointCloud *pc) {

    float sinTheta = sinf(pitchCalib_);
    float cosTheta = cosf(pitchCalib_);

    for (unsigned int n = 0; n < pc->points.size(); n++) {

      float x = pc->points[n].x;
      float y = pc->points[n].y;
      float z = pc->points[n].z;

      float newZ = z * cosTheta - x * sinTheta;
      float newX = z * sinTheta + x * cosTheta;
      float newY = y;

      pc->points[n].x = newX;
      pc->points[n].y = newY;
      pc->points[n].z = newZ;

    }
  }

  void VelodyneUtil::correctHeight(
      sensor_msgs::PointCloud *pc) {

    for (unsigned int n = 0; n < pc->points.size(); n++) {

      int laserNum = pc->channels[0].values[n];

      float x = pc->points[n].x;
      float y = pc->points[n].y;
      float z = pc->points[n].z;

      float r = sqrt(x * x + y * y + z * z);
      float thetaAct = asin (z / r);
      float thetaCorrected = thetaAct + hCalib_[laserNum];

      float newZ = r * sin(thetaCorrected);
      float newX = x; //TODO: These still need to be corrected
      float newY = y;

      pc->points[n].x = newX;
      pc->points[n].y = newY;
      pc->points[n].z = newZ;

    }
  }

  void VelodyneUtil::correctIntensity(
      sensor_msgs::PointCloud *pc) {

    for (unsigned int n = 0; n < pc->points.size(); n++) {
      int laserNum = pc->channels[0].values[n];
      int origI = pc->channels[2].values[n];
      int newI = 
          iCalib_[laserNum].a +
          iCalib_[laserNum].b * origI +
          iCalib_[laserNum].c * origI * origI +
          iCalib_[laserNum].d * origI * origI * origI;

      if (newI < 0) {
        newI = 0;
      } else if (newI > 255) {
        newI = 255;
      }

      pc->channels[2].values[n] = newI;
    }
  }

  void VelodyneUtil::calibrateVelodyneData (
      sensor_msgs::PointCloud2 &cloud2) {

    float sinTheta = sinf(pitchCalib_);
    float cosTheta = cosf(pitchCalib_);

    for (unsigned int i = 0; i < cloud2.height * cloud2.width; i++) {
      
      unsigned int offsetToInt = 4 * 4;
      float* ptDataFloat = reinterpret_cast<float*>(&(cloud2.data[0]) + i * cloud2.point_step);
      uint8_t* ptDataInt = reinterpret_cast<uint8_t*>(&(cloud2.data[0]) + i * cloud2.point_step + offsetToInt);
      uint8_t laserNum = ptDataInt[0];

      // Perform pitch correction

      float x = ptDataFloat[0];
      float y = ptDataFloat[1];
      float z = ptDataFloat[2];

      float xTemp = z * sinTheta + x * cosTheta;
      float yTemp = y;
      float zTemp = z * cosTheta - x * sinTheta;

      // Perform individual laser angle correction
          
      float r = sqrtf(xTemp * xTemp + yTemp * yTemp + zTemp * zTemp);
      float thetaAct = asinf (zTemp / r);
      float thetaCorrected = thetaAct + hCalib_[laserNum];

      // Replace values in cloud

      ptDataFloat[0] = xTemp; //TODO: x & y are currently not corrected
      ptDataFloat[1] = yTemp;
      ptDataFloat[2] = r * sinf(thetaCorrected);

      // Calculate shift in intensity values
      int origI = ptDataInt[1];
      int newI = 
          iCalib_[laserNum].a +
          iCalib_[laserNum].b * origI +
          iCalib_[laserNum].c * origI * origI +
          iCalib_[laserNum].d * origI * origI * origI;

      if (newI < 0) {
        newI = 0;
      } else if (newI > 255) {
        newI = 255;
      }
      ptDataInt[1] = newI;
    }
  }

}
