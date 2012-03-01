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

#include <boost/foreach.hpp>
#include <velodyne_image/overhead_image_generator.h>

namespace velodyne_image {

  /**
   * \brief Takes a pcl::PointCloud and obtains a height and 
   *        intensity image from it.
   */
  void OverheadImageGenerator::getOverheadImages(
      const VPointCloud& cloud, cv::Mat& height_image,
      cv::Mat& intensity_image) {

    int dim_size = height_image.rows;

    Mat availableImage = Mat::zeros(dim_size, dim_size, CV_8U);
    Mat avg_image = Mat::zeros(dim_size, dim_size, CV_8U);
    Mat temp_height = Mat::zeros(dim_size, dim_size, CV_32F);
    Mat temp_intensity = Mat::zeros(dim_size, dim_size, CV_32S);

    // Store points across radial lines
    int prev_x_pxl[NUM_HEADINGS]; 
    int prev_y_pxl[NUM_HEADINGS];
    for (int i = 0; i < NUM_HEADINGS; i++) {
      prev_x_pxl[i] = prev_y_pxl[i] = dim_size / 2;
    }

    float x, y, z, heading;
    int ring;
    unsigned int intensity;
    int heading_idx, x_pxl, y_pxl;

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

        int y_pxl = -x / config_.distance_per_pixel + dim_size / 2;
        int x_pxl = -y / config_.distance_per_pixel + dim_size / 2;
        
        // Time to interpolate - calculate parameters for Bresenham's
        int dx = abs(x_pxl - prev_x_pxl[heading_idx]);
        int dy = abs(y_pxl - prev_y_pxl[heading_idx]);
        int sx = -1 + 2 * (prev_x_pxl[heading_idx] < x_pxl);
        int sy = -1 + 2 * (prev_y_pxl[heading_idx] < y_pxl);
        int err = dx-dy;

        // Interpolation that is too long typically means an obstacle, throw out
        // those lines. Ideally this should be a function of the ring, but a 
        // fixed value works fine in practice
        bool is_noise = dx * dx + dy * dy > 
          config_.pixel_noise_distance * config_.pixel_noise_distance;

        bool not_reached_end_point = 
          prev_x_pxl[heading_idx] != x_pxl || prev_y_pxl[heading_idx] != y_pxl;
        bool not_reached_boundaries = 
          prev_x_pxl[heading_idx] >= 0 && prev_x_pxl[heading_idx] < dim_size &&
          prev_y_pxl[heading_idx] >= 0 && prev_y_pxl[heading_idx] < dim_size;

        while (not_reached_end_point && not_reached_boundaries) {
          if (!isNoise) {
            avg_image.at<float>(prev_y_pxl[heading_idx], prev_x_pxl[heading_idx]) += 1;
            temp_height.at<float>(prev_y_pxl[heading_idx], prev_x_pxl[heading_idx]) += z;
            temp_intensity.at<float>(prev_y_pxl[heading_idx], prev_x_pxl[heading_idx]) += intensity;
          }
        }

          availableImage.at<unsigned char>(prev_y_pxl[heading_idx], prev_x_pxl[heading_idx]) = true;

          int e2 = 2 * err;
          if (e2 > -dy) {
            err = err - dy;
            prev_x_pxl[heading_idx] += sx;
          }
          if (e2 < dx) {
            err = err + dx;
            prev_y_pxl[heading_idx] += sy;
          }
        }
      } /* end point */

      if (config_.preventBleeding) {
        // After each ring, move the prevPxl points to the outermost points available
        // This prevents bleeding in those cases where points had failed for one particular heading
        for (int heading = 0; heading < num_headings; heading++) {
          float headingAngle = ((heading - num_headings / 2) * 2 * M_PI) / num_headings;
          float maxDistance = 100;

          x = maxDistance * cosf(headingAngle);
          y = maxDistance * sinf(headingAngle);

          y_pxl = -x / config_.distance_per_pixel + dim_size / 2;
          x_pxl = -y / config_.distance_per_pixel + dim_size / 2;

          int x_orig = dim_size / 2;
          int y_orig = dim_size / 2;
        
          // Bresenhams again, this time to find the outermost available pixel on this heading
          int dx = abs(x_pxl - x_orig);
          int dy = abs(y_pxl - y_orig);
          int sx = -1 + 2 * (x_orig < x_pxl);
          int sy = -1 + 2 * (y_orig < y_pxl);
          int err = dx-dy;

          while ((x_orig != x_pxl || y_orig != y_pxl) && (x_orig < dim_size && x_orig >= 0 && y_orig < dim_size && y_orig >=0) && availableImage.at<unsigned char>(y_orig, x_orig) == true) {
           //std::cout << current_ring << ": " << heading << ": " << x_orig << " " << y_orig << std::endl;

            // Calculate magnitudes to add
            prev_x_pxl[heading] = x_orig;
            prev_y_pxl[heading] = y_orig;

            int e2 = 2 * err;
            if (e2 > -dy) {
              err = err - dy;
              x_orig += sx;
            }
            if (e2 < dx) {
              err = err + dx;
              y_orig += sy;
            }
          }
        }

      }
    } /* end current_ring */

    for (int i = 0; i < dim_size; i++) {

      float* temp_height_row = temp_height.ptr<float>(i);
      float* temp_intensity_row = temp_intensity.ptr<float>(i);
      float* avg_image_row = avg_image.ptr<float>(i);

      unsigned char* height_row = height_image.ptr<unsigned char>(i);
      unsigned char* intensity_row = intensity_image.ptr<unsigned char>(i);

      for (int j = 0; j < dim_size; j++) {

        if (avg_image_row[j] == 0) {
          if (!config_.useOldImage) {
            height_row[j] = 0;
            intensity_row[j] = 0;
          }
          continue;
        }

        // Height
        float z = temp_height_row[j] / avg_image_row[j];
        int scaledZ = 127 * (1.0 - pow (M_E, -(z - config_.mean) * (z - config_.mean) / (2 * config_.sigma)));
        if (z < config_.mean) scaledZ *= -1;
        scaledZ += 128;
        height_row[j] = scaledZ;

        // Intensity
        float intensity = temp_intensity_row[j] / avg_image_row[j];
        intensity_row[j] = (unsigned char) intensity;

      }
    }
  }

  void OverheadImageGenerator::generateSnapshots(
          const std::string prefix,
          const sensor_msgs::PointCloud2 &cloud,
          Mat &oldHeightImage,
          float config_.distance_per_pixel) {
    
    bool config_use_old_image = config_.useOldImage;
    bool config_bleeding = config_.preventBleeding;
    Mat temp_intensityImage = cv::Mat::zeros(oldHeightImage.size(), oldHeightImage.type());
   
    Mat naiveImage = cv::Mat::zeros(oldHeightImage.size(), oldHeightImage.type());
    generateNaiveImage(cloud, naiveImage, config_.distance_per_pixel);

    config_.useOldImage = false;
    config_.preventBleeding = false;
    Mat overheadImage = cv::Mat::zeros(oldHeightImage.size(), oldHeightImage.type());
    getOverheadImage(cloud, overheadImage, temp_intensityImage, config_.distance_per_pixel);

    config_.preventBleeding = true;
    Mat noiseFreeImage = cv::Mat::zeros(oldHeightImage.size(), oldHeightImage.type());
    getOverheadImage(cloud, noiseFreeImage, temp_intensityImage, config_.distance_per_pixel);

    config_.useOldImage = true;
    Mat odomImage;
    oldHeightImage.copyTo(odomImage);
    getOverheadImage(cloud, odomImage, temp_intensityImage, config_.distance_per_pixel);

    config_.useOldImage = config_use_old_image;
    config_.preventBleeding = config_bleeding;

    // Save images to file
    imwrite(prefix + "_1.jpg", naiveImage);
    imwrite(prefix + "_2.jpg", overheadImage);
    imwrite(prefix + "_3.jpg", noiseFreeImage);
    imwrite(prefix + "_4.jpg", odomImage);

  }

  void OverheadImageGenerator::generateNaiveImage(
          const sensor_msgs::PointCloud2 &cloud,
          Mat &height_image,
          float config_.distance_per_pixel) {
    
    int dim_size = height_image.rows;

    Mat avg_image = Mat::zeros(dim_size, dim_size, CV_32F);
    Mat temp_height = Mat::zeros(dim_size, dim_size, CV_32F);

    int num_headings = velodyne::SCANS_PER_REV / velodyne_rawdata::N_LASERS;
    int offsetToInt = 4 * 4;
    float x, y, z, heading;
    int ring;
    unsigned int intensity;
    bool available;

    int x_pxl, y_pxl;

    for (int current_ring = 1; current_ring < velodyne_rawdata::N_LASERS; current_ring++) {
      for (unsigned int i = 0; i < cloud.width * cloud.height; i++) {

        const uint8_t* ptDataInt = reinterpret_cast<const uint8_t*>(&(cloud.data[0]) + i * cloud.point_step + offsetToInt);
        ring = ptDataInt[0];
        intensity = ptDataInt[1];
        available = (bool) ptDataInt[2];

        if (ring == current_ring && available) {
          const float* ptDataFloat = reinterpret_cast<const float*>(&(cloud.data[0]) + i * cloud.point_step);
          x = ptDataFloat[0];
          y = ptDataFloat[1];
          z = ptDataFloat[2];
          heading = atan2(y,x);
          //heading = ptDataFloat[3];

          y_pxl = -x / config_.distance_per_pixel + dim_size / 2;
          x_pxl = -y / config_.distance_per_pixel + dim_size / 2;

          if (x_pxl >= 0 && x_pxl < dim_size && y_pxl >=0 && y_pxl < dim_size) {
            avg_image.at<float>(y_pxl, x_pxl) += 1;
            temp_height.at<float>(y_pxl, x_pxl) += z;
          }
        }
      }
    }

    for (int i = 0; i < dim_size; i++) {

      float* temp_height_row = temp_height.ptr<float>(i);
      float* avg_image_row = avg_image.ptr<float>(i);

      unsigned char* height_row = height_image.ptr<unsigned char>(i);

      for (int j = 0; j < dim_size; j++) {

        if (avg_image_row[j] == 0) {
          if (!config_.useOldImage) {
            height_row[j] = 0;
          }
          continue;
        }

        // Height
        float z = temp_height_row[j] / avg_image_row[j];
        int scaledZ = 127 * (1.0 - pow (M_E, -(z - config_.mean) * (z - config_.mean) / (2 * config_.sigma)));
        if (z < config_.mean) scaledZ *= -1;
        scaledZ += 128;
        height_row[j] = scaledZ;

      }
    }
  }
}
