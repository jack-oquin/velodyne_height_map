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
    /* new parameters in db.xml provided by Velodyne */
    float rot_correction;
    float vert_correction;
    float dist_correction;
    float vert_offset_correction;
    float horiz_offset_correction;
    IntensityCorrection intensity_correction;
  };

  struct Calibration {
    int num_lasers;
    float pitch;
    float roll;
    std::map<int, LaserCorrection> laser_corrections;
  };
  
} /* velodyne */


#endif /* end of include guard: CALIBRATION_CQUVIJWI */

