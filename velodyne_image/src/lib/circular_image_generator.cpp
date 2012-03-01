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
#include <velodyne_image/common.h>

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
        height_image.type() != CV_32F ||
        height_image.data == NULL) {
      height_image.create(avg_image.size(), CV_32F);
    }
    if (intensity_image.size() != avg_image.size() || 
        intensity_image.type() != CV_8U ||
        intensity_image.data == NULL) {
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

    // Assign values to the actual images
    fillImageValues(avg_image, config_.use_old_image, 
        temp_height, temp_intensity, height_image, intensity_image,
        config_.mean, config_.sigma);

  }
}
