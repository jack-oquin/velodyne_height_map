/**
 * \file  common.cpp
 * \brief Implementations for the common functions in image generation
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, UT Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 03/01/2012 04:02:39 PM piyushk $
 */

#include <velodyne_image/common.h>
#include <cmath>

namespace velodyne_image {

  /**
   * \brief Calculates a scaled height value between 0 and 1 that tries to
   *        better exhibit features around the mean height 
   */
  float enhanceContrast(float z, float mean, float sigma) {
    float scaled_z = 
      1.0 - powf(M_E, - (z - mean)*(z - mean) / (2 * sigma));
    if (z < mean)
      scaled_z *= -1;
    scaled_z = 0.5 * (1 + scaled_z);
    return scaled_z;
  }

  float enhanceDisparityContrast(float z, float mean, float sigma) {
    float scaled_z = 
      1.0 - powf(M_E, - (z - mean)*(z - mean) / (2 * sigma));
    return scaled_z;
  }

  /**
   * \brief Takes the temp images and fills in the actual images using 
   *        scaling. This function does no error checking, and assumes all the
   *        images to have the correct size, type and allocated data.
   */
  void fillImageValues(cv::Mat& avg_image, bool use_old_image, 
      cv::Mat& temp_height, cv::Mat& temp_intensity, 
      cv::Mat& height_image, cv::Mat& intensity_image,
      float mean, float sigma, bool scale_values) {

    // Let's fill in the actual images that are going to be returned
    for (int y = 0; y < avg_image.rows; ++y) {

      // Get temp image pointers
      unsigned char* avg_image_row = avg_image.ptr<unsigned char>(y);
      int* temp_intensity_row = temp_intensity.ptr<int>(y);
      float* temp_height_row = temp_height.ptr<float>(y);

      // Get actual image pointers
      unsigned char* intensity_row = intensity_image.ptr<unsigned char>(y);
      float* height_row = height_image.ptr<float>(y);

      for (int x = 0; x < avg_image.cols; ++x) {

        if (avg_image_row[x] == 0) {
          if (!use_old_image) {
            height_row[x] = 0;
            intensity_row[x] = 0;
          }
          continue;
        }

        // Height values
        float z = temp_height_row[x] / avg_image_row[x];
        if (scale_values) {
          float scaled_z = enhanceContrast(z, mean, sigma); 
          height_row[x] = scaled_z;
        } else {
          height_row[x] = z;
        }

        // Intensity values
        unsigned char intensity = (unsigned char) (temp_intensity_row[x] /
                                                   avg_image_row[x]);
        intensity_row[x] = intensity;

      } /* end x */
    } /* end y */
  }

  void enhanceContrast(const cv::Mat& src, cv::Mat& dst, 
      float mean, float sigma, bool use_disparity_formula) {

    if (dst.size() != src.size() || 
        dst.type() != CV_32F ||
        dst.data == NULL) {
      dst = cv::Mat::zeros(src.size(), CV_32F);
    }

    for (int y = 0; y < src.rows; ++y) {
      const float* src_row = src.ptr<float>(y);
      float* dst_row = dst.ptr<float>(y);
      for (int x = 0; x < src.cols; ++x) {
        if (use_disparity_formula) {
          dst_row[x] = enhanceDisparityContrast(src_row[x], mean, sigma);
        } else {
          dst_row[x] = enhanceContrast(src_row[x], mean, sigma);
        }
      }
    }

  }
} /* velodyne_image */
