/**
 * \file  calibration_test.cpp
 * \brief  
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, The University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:25:49 PM piyushk $
 */

#include <velodyne_calibration/CalibrationUtilities.h>

int main(int argc, const char *argv[]) {
  velodyne::writeTestCalibrationFile("test.yaml");
  velodyne::Calibration calibration;
  velodyne::readCalibrationFromFile("test.yaml", calibration);
  return 0;
}
