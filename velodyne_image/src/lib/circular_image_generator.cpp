/**
 * \file  circular_image_generator.cpp
 * \brief  Definitions for the CircularImageGenerator class
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2011, UT Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/16/2011 02:35:44 PM piyushk $
 */

#include <opencv/cv.h>
#include <boost/foreach.hpp>

#include <velodyne_pointcloud/ring_sequence.h>
#include <velodyne_image/circular_image_generator.h>

namespace velodyne_image {

  void CircularImageGenerator::getCircularImages(const VPointCloud& cloud, 
      cv::Mat& height_image, cv::Mat& intensity_image) {

    // Create some temp intermediate images
    cv::Mat avg_image = cv::Mat::zeros(velodyne_rawdata::N_LASERS, 
      config_.points_per_laser, CV_32F);
    cv::Mat temp_intensity = cv::Mat::zeros(velodyne_rawdata::N_LASERS, 
      config_.points_per_laser, CV_32S);
    cv::Mat temp_height = cv::Mat::zeros(velodyne_rawdata::N_LASERS, 
      config_.points_per_laser, CV_32F);

    // Recreate images if necessary - 
    // good for first creation and when dynamic reconfigure changes size
    if (height_image.size() != avg_image.size() || 
        height_image.type() != CV_32F) {
      height_image.create(avg_image.size(), CV_32F);
    }
    if (intensity_image.size() != avg_image.size() || 
        intensity_image.type() != CV_8U) {
      intensity_image.create(avg_image.size(), CV_8U);
    }

    // Iterate over all the points and fill in the temp images
    BOOST_FOREACH(const VPoint& point, cloud.points) {

      // Calculate image indices
      float heading(atan2(point.y, point.x));
      heading = heading + M_PI;
      int heading_idx = heading / (2 * M_PI) * config_.points_per_laser;
      int ring_number = velodyne_rawdata::N_LASERS - point.ring - 1;

      avg_image.at<unsigned char>(ring_number, heading_idx) += 1;
      temp_intensity.at<unsigned char>(ring_number, heading_idx) += 
        point.intensity;
      avg_image.at<unsigned char>(ring_number, heading_idx) += point.z;
    } 

    for (int y = 0; y < velodyne_rawdata::N_LASERS; ++y) {

      // Get temp image poiinters
      unsigned char* avg_image_row = avg_image.ptr<unsigned char>(y);
      int* temp_intensity_row = temp_intensity.ptr<int>(y);
      float* temp_height_row = temp_height.ptr<float>(y);

      // Get actual image pointers
      unsigned char* intensity_row = intensity_image.ptr<unsigned char>(y);
      float* height_row = height_image.ptr<float>(y);

      for (int x = 0; x < config_.points_per_laser; ++x) {

        // Height values
        float z = temp_height_row[x] / avg_image_row[x];
        float scaled_z = 
          1.0 - powf(M_E, - (z - config_.mean)*(z - config_.mean) /
                            (2 * config_.sigma));
        if (z < config_.mean)
          scaled_z *= -1;
        scaled_z = 0.5 * (1 + scaled_z);
        height_row[x] = scaled_z;

        // Intensity values
        unsigned char intensity = (unsigned char) (temp_intensity_row[x] /
                                                   avg_image_row[x]);
        intensity_row[x] = intensity;

      } /* end x */
    } /* end y */

  }
}
