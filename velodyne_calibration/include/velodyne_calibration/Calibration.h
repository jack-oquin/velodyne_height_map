/**
 * \file  Calibration.h
 * \brief  
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:25:34 AM piyushk $
 */

#ifndef CALIBRATION_CQUVIJWI
#define CALIBRATION_CQUVIJWI

namespace velodyne {

  /* degree 3 polynomial transformation applied to intensity values */
  struct IntensityCorrection {
    float a;
    float b;
    float c;
    float d;
  };

  /* calibration values for a single laser */
  struct LaserCorrection {
    /* new parameters in db.xml provided by Velodyne S2.1 Revision D 2011*/
    /* http://velodynelidar.com/lidar/products/manual/63-HDL64E%20S2%20Manual_Rev%20D_2011_web.pdf */
    float rot_correction;
    float cos_rot_correction; // cached value
    float sin_rot_correction; // cached value
    float vert_correction;
    float cos_vert_correction; // cached value
    float sin_vert_correction; // cached value
    float dist_correction;
    float vert_offset_correction;
    float horiz_offset_correction;
    int max_intensity;
    int min_intensity;
    float focal_distance;
    float focal_slope;
    //IntensityCorrection intensity_correction;
  };

  struct Calibration {
    int num_lasers;
    float pitch;
    float roll;
    std::map<int, LaserCorrection> laser_corrections;
  };
  `
} /* velodyne */


#endif /* end of include guard: CALIBRATION_CQUVIJWI */

