/**
 * \file  common.h
 * \brief Contains common functionality between both types of image generators
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, UT Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 03/01/2012 01:17:12 PM piyushk $
 */

#ifndef COMMON_NENROON3
#define COMMON_NENROON3

#include <opencv/cv.h>

namespace velodyne_image {

  /**
   * \brief Calculates a scaled height value between 0 and 1 that tries to
   *        better exhibit features around the mean height 
   */
  float enhanceContrast(float z, float mean, float sigma);

  /**
   * \brief Takes the temp images and fills in the actual images using 
   *        scaling. This function does no error checking, and assumes all the
   *        images to have the correct size, type and allocated data.
   */
  void fillImageValues(cv::Mat& avg_image, bool use_old_image, 
      cv::Mat& temp_height, cv::Mat& temp_intensity, 
      cv::Mat& height_image, cv::Mat& intensity_image,
      float mean, float sigma);

} /* velodyne_image */

#endif /* end of include guard: COMMON_NENROON3 */

