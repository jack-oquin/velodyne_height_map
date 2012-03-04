/**
 * \file  overhead_image_generator.cpp
 * \brief Definitions for the OverheadImageGenerator class 
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2011, UT Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/16/2011 02:38:38 PM piyushk $
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/foreach.hpp>

#include <velodyne_image/overhead_image_generator.h>
#include <velodyne_image/common.h>

namespace velodyne_image {

  /**
   * \brief Takes a pcl::PointCloud and obtains a height and 
   *        intensity image from it.
   */
  void OverheadImageGenerator::getOverheadImages(
      const VPointCloud& cloud, cv::Mat& height_image,
      cv::Mat& intensity_image) {

    int dim_size = config_.image_size;

    cv::Mat available_image = cv::Mat::zeros(dim_size, dim_size, CV_8U);
    cv::Mat avg_image = cv::Mat::zeros(dim_size, dim_size, CV_8U);
    cv::Mat temp_height = cv::Mat::zeros(dim_size, dim_size, CV_32F);
    cv::Mat temp_intensity = cv::Mat::zeros(dim_size, dim_size, CV_32S);

    // Recreate images if necessary - 
    // good for first creation and when dynamic reconfigure changes size
    if (height_image.size() != avg_image.size() || 
        height_image.type() != CV_32F ||
        height_image.data == NULL) {
      height_image = cv::Mat::zeros(avg_image.size(), CV_32F);
    }
    if (intensity_image.size() != avg_image.size() || 
        intensity_image.type() != CV_8U ||
        intensity_image.data == NULL) {
      intensity_image = cv::Mat::zeros(avg_image.size(), CV_8U);
    }
    // Store points across radial lines
    int prev_x_pxl[NUM_HEADINGS]; 
    int prev_y_pxl[NUM_HEADINGS];
    for (unsigned int i = 0; i < NUM_HEADINGS; i++) {
      prev_x_pxl[i] = prev_y_pxl[i] = dim_size / 2;
    }

    for (int current_ring = 0; 
        current_ring < velodyne_rawdata::N_LASERS; ++current_ring) {

      BOOST_FOREACH(const VPoint& point, cloud.points) {

        // Only process points for the current ring
        if (point.ring != current_ring)
          continue;

        float heading = atan2(point.y, point.x);
        int heading_idx = NUM_HEADINGS / 2 +
          heading / (2.0 * M_PI) * NUM_HEADINGS;

        // Check if image boundary has already been reached for this radial line
        if ((prev_x_pxl[heading_idx] == 0 || 
             prev_x_pxl[heading_idx] == dim_size - 1) ||
            (prev_y_pxl[heading_idx] == 0 || 
             prev_y_pxl[heading_idx] == dim_size - 1)) {
          continue;
        }

        int y_pxl = -point.x / config_.distance_per_pixel + dim_size / 2;
        int x_pxl = -point.y / config_.distance_per_pixel + dim_size / 2;
        
        // Time to interpolate - calculate parameters for Bresenham's
        int dx = abs(x_pxl - prev_x_pxl[heading_idx]);
        int dy = abs(y_pxl - prev_y_pxl[heading_idx]);
        int sx = -1 + 2 * (prev_x_pxl[heading_idx] < x_pxl);
        int sy = -1 + 2 * (prev_y_pxl[heading_idx] < y_pxl);
        int err = dx - dy;

        // Interpolation that is too long typically means an obstacle, throw out
        // those lines. Ideally this should be a function of the ring, but a 
        // fixed value works fine in practice
        int squared_interpolation_distance = dx * dx + dy * dy;
        float pixel_noise_distance = 
          config_.noise_distance / config_.distance_per_pixel;
        float squared_pixel_noise_distance = 
          pixel_noise_distance * pixel_noise_distance;
        bool is_noise =
          squared_interpolation_distance > squared_pixel_noise_distance;

        bool not_reached_end_point = 
          prev_x_pxl[heading_idx] != x_pxl || prev_y_pxl[heading_idx] != y_pxl;
        bool not_reached_boundaries = 
          prev_x_pxl[heading_idx] >= 0 && prev_x_pxl[heading_idx] < dim_size &&
          prev_y_pxl[heading_idx] >= 0 && prev_y_pxl[heading_idx] < dim_size;

        while (not_reached_end_point && not_reached_boundaries) {

          // Store point values if not noise
          if (!is_noise) {
            avg_image.at<unsigned char>(prev_y_pxl[heading_idx], 
                prev_x_pxl[heading_idx]) += 1;
            temp_height.at<float>(prev_y_pxl[heading_idx], 
                prev_x_pxl[heading_idx]) += point.z;
            temp_intensity.at<int>(prev_y_pxl[heading_idx], 
                prev_x_pxl[heading_idx]) += point.intensity;
          }
          // Mark pixel as processed
          available_image.at<unsigned char>(prev_y_pxl[heading_idx], 
              prev_x_pxl[heading_idx]) = true;

          // Optimized Bresenham's code from wikipedia
          // http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
          int e2 = 2 * err;
          if (e2 > -dy) {
            err = err - dy;
            prev_x_pxl[heading_idx] += sx;
          }
          if (e2 < dx) {
            err = err + dx;
            prev_y_pxl[heading_idx] += sy;
          }

          // Recompute terminal conditions
          not_reached_end_point = (prev_x_pxl[heading_idx] != x_pxl || 
                                   prev_y_pxl[heading_idx] != y_pxl);
          not_reached_boundaries = (prev_x_pxl[heading_idx] >= 0 && 
                                    prev_x_pxl[heading_idx] < dim_size &&
                                    prev_y_pxl[heading_idx] >= 0 && 
                                    prev_y_pxl[heading_idx] < dim_size);
        }
      } /* end point */

      // Now that interpolation has been performed, perform the following edge
      // preserving measure
      if (config_.prevent_bleeding) {
        
        // Move the radial lines outwards if the pixels they were going to
        // change have already been filled
        for (unsigned int heading_idx = 0; 
             heading_idx < NUM_HEADINGS; ++heading_idx) {

          // Find far external points on these radial lines
          float heading = 
            ((float) heading_idx) / NUM_HEADINGS * 
            (2 * M_PI) - M_PI;
          float max_distance = 100;

          // TODO: these values can be cached as the headings/radials are fixed
          float x = max_distance * cosf(heading);
          float y = max_distance * sinf(heading);

          int y_pxl = -x / config_.distance_per_pixel + dim_size / 2;
          int x_pxl = -y / config_.distance_per_pixel + dim_size / 2;

          int x_orig = prev_x_pxl[heading_idx];
          int y_orig = prev_y_pxl[heading_idx];
        
          // Bresenham's again, except this stops when it finds points that have
          // not been reached
          int dx = abs(x_pxl - x_orig);
          int dy = abs(y_pxl - y_orig);
          int sx = -1 + 2 * (x_orig < x_pxl);
          int sy = -1 + 2 * (y_orig < y_pxl);
          int err = dx - dy;

          bool not_reached_end_point = 
            x_orig != x_pxl || y_orig != y_pxl;
          bool not_reached_boundaries = 
            x_orig < dim_size && x_orig >= 0 && 
            y_orig < dim_size && y_orig >= 0;
          bool image_pixel_filled = 
            available_image.at<unsigned char>(y_orig, x_orig); 

          while (not_reached_end_point && not_reached_boundaries && 
                 image_pixel_filled) {

            // Calculate magnitudes to add
            prev_x_pxl[heading_idx] = x_orig;
            prev_y_pxl[heading_idx] = y_orig;

            int e2 = 2 * err;
            if (e2 > -dy) {
              err = err - dy;
              x_orig += sx;
            }
            if (e2 < dx) {
              err = err + dx;
              y_orig += sy;
            }

            // Recompute termination conditions
            not_reached_end_point = 
              x_orig != x_pxl || y_orig != y_pxl;
            not_reached_boundaries = 
              x_orig < dim_size && x_orig >= 0 && 
              y_orig < dim_size && y_orig >= 0;
            image_pixel_filled = 
              available_image.at<unsigned char>(y_orig, x_orig); 

          }
        } /* end heading_idx */
      } /* end prevent_bleeding */
    } /* end current_ring */

    // Assign values to the actual images
    fillImageValues(avg_image, config_.use_old_image, 
        temp_height, temp_intensity, height_image, intensity_image,
        config_.mean, config_.sigma);

  }

  /** 
   * \brief For visualization only - do not use in production code
   */
  void OverheadImageGenerator::generateSnapshots(const std::string& prefix,
          const VPointCloud& cloud, const cv::Mat& prev_height_image) {
    
    bool config_use_old_image = config_.use_old_image;
    bool config_bleeding = config_.prevent_bleeding;
    cv::Mat temp_intensity = 
      cv::Mat::zeros(prev_height_image.size(), prev_height_image.type());
   
    cv::Mat naive_image = 
      cv::Mat::zeros(prev_height_image.size(), prev_height_image.type());
    getNaiveImages(cloud, naive_image, temp_intensity);

    config_.use_old_image = false;
    config_.prevent_bleeding = false;
    cv::Mat overhead_image = 
      cv::Mat::zeros(prev_height_image.size(), prev_height_image.type());
    getOverheadImages(cloud, overhead_image, temp_intensity);

    config_.prevent_bleeding = true;
    cv::Mat noise_free_image = 
      cv::Mat::zeros(prev_height_image.size(), prev_height_image.type());
    getOverheadImages(cloud, noise_free_image, temp_intensity);

    config_.use_old_image = true;
    cv::Mat odom_image;
    prev_height_image.copyTo(odom_image);
    getOverheadImages(cloud, odom_image, temp_intensity);

    config_.use_old_image = config_use_old_image;
    config_.prevent_bleeding = config_bleeding;

    // Save images to file
    cv::imwrite(prefix + "_1.jpg", naive_image);
    cv::imwrite(prefix + "_2.jpg", overhead_image);
    cv::imwrite(prefix + "_3.jpg", noise_free_image);
    cv::imwrite(prefix + "_4.jpg", odom_image);

  }

  /** 
   * \brief For visualization only - do not use in production code
   */
  void OverheadImageGenerator::getNaiveImages(
      const VPointCloud& cloud, cv::Mat& height_image,
      cv::Mat& intensity_image) {
    
    int dim_size = config_.image_size;

    cv::Mat avg_image = cv::Mat::zeros(dim_size, dim_size, CV_8U);
    cv::Mat temp_height = cv::Mat::zeros(dim_size, dim_size, CV_32F);
    cv::Mat temp_intensity = cv::Mat::zeros(dim_size, dim_size, CV_32S);

    // Recreate images if necessary 
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

    BOOST_FOREACH(const VPoint& point, cloud.points) {
      int y_pxl = -point.x / config_.distance_per_pixel + dim_size / 2;
      int x_pxl = -point.y / config_.distance_per_pixel + dim_size / 2;
        
      if (x_pxl >= 0 && x_pxl < dim_size && y_pxl >=0 && y_pxl < dim_size) {
        avg_image.at<float>(y_pxl, x_pxl) += 1;
        temp_height.at<float>(y_pxl, x_pxl) += point.z;
        temp_intensity.at<float>(y_pxl, x_pxl) += point.intensity;
      }
    }

    // Assign values to the actual images
    fillImageValues(avg_image, false, 
        temp_height, temp_intensity, height_image, intensity_image,
        config_.mean, config_.sigma);
  }
}
