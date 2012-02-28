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

#include <velodyne_image/overhead_image_generator.h>

namespace velodyne_image_generation {

  /**
   * \brief   Takes a PointCloud2 message and obtains a height and 
   *          intensity image from it.
   */
  void OverheadImageGenerator::getOverheadImage(
          const sensor_msgs::PointCloud2 &cloud2,
          cv::Mat& heightImage,
          cv::Mat& intensityImage,
      ) {

    int size = heightImage.rows;

    Mat availableImage = Mat::zeros(size, size, CV_8U);
    Mat avgImage = Mat::zeros(size, size, CV_32F);
    Mat tempHeight = Mat::zeros(size, size, CV_32F);
    Mat tempIntensity = Mat::zeros(size, size, CV_32F);

    int numHeadings = velodyne::SCANS_PER_REV / velodyne::N_LASERS;
    int prevXPxl[numHeadings]; 
    int prevYPxl[numHeadings];
    for (int i = 0; i < numHeadings; i++) {
      prevXPxl[i] = prevYPxl[i] = size / 2;
    }
    int offsetToInt = 4 * 4;
    float x, y, z, heading;
    int ring;
    unsigned int intensity;
    bool available;

    int headingIdx, xPxl, yPxl;

    for (int currentRing = 1; currentRing < velodyne::N_LASERS; currentRing++) {
      for (unsigned int i = 0; i < cloud2.width * cloud2.height; i++) {

        const uint8_t* ptDataInt = reinterpret_cast<const uint8_t*>(&(cloud2.data[0]) + i * cloud2.point_step + offsetToInt);
        ring = ptDataInt[0];
        intensity = ptDataInt[1];
        available = (bool) ptDataInt[2];

        if (ring == currentRing && available) {
          const float* ptDataFloat = reinterpret_cast<const float*>(&(cloud2.data[0]) + i * cloud2.point_step);
          x = ptDataFloat[0];
          y = ptDataFloat[1];
          z = ptDataFloat[2];
          heading = atan2(y,x);
          //heading = ptDataFloat[3];

          yPxl = -x / distancePerPixel + size / 2;
          xPxl = -y / distancePerPixel + size / 2;
          
          headingIdx = heading * numHeadings / (2.0 * M_PI) + numHeadings / 2;
          headingIdx -= numHeadings * (heading == numHeadings);
          
          // Dont do anything if boundary has already been reached
          if (prevXPxl[headingIdx] == 0 || prevXPxl[headingIdx] == size - 1 ||
              prevYPxl[headingIdx] == 0 || prevYPxl[headingIdx] == size - 1) {
            continue;
          }

          // Bresenhams
          int dx = abs(xPxl - prevXPxl[headingIdx]);
          int dy = abs(yPxl - prevYPxl[headingIdx]);
          int sx = -1 + 2 * (prevXPxl[headingIdx] < xPxl);
          int sy = -1 + 2 * (prevYPxl[headingIdx] < yPxl);
          int err = dx-dy;
          bool isNoise = dx * dx + dy * dy > 225;

          while ((prevXPxl[headingIdx] != xPxl || prevYPxl[headingIdx] != yPxl) && prevXPxl[headingIdx] >= 0 && prevXPxl[headingIdx] < size && prevYPxl[headingIdx] >= 0 && prevYPxl[headingIdx] < size) {

            // Calculate magnitudes to add
            //if (!config_.useOldImage || !isNoise) {
            if (!isNoise) {
              avgImage.at<float>(prevYPxl[headingIdx], prevXPxl[headingIdx]) += 1;
              tempHeight.at<float>(prevYPxl[headingIdx], prevXPxl[headingIdx]) += z;
              tempIntensity.at<float>(prevYPxl[headingIdx], prevXPxl[headingIdx]) += intensity;
            }
            availableImage.at<unsigned char>(prevYPxl[headingIdx], prevXPxl[headingIdx]) = true;

            int e2 = 2 * err;
            if (e2 > -dy) {
              err = err - dy;
              prevXPxl[headingIdx] += sx;
            }
            if (e2 < dx) {
              err = err + dx;
              prevYPxl[headingIdx] += sy;
            }
          }
        }
      }

      if (config_.preventBleeding) {
        // After each ring, move the prevPxl points to the outermost points available
        // This prevents bleeding in those cases where points had failed for one particular heading
        for (int heading = 0; heading < numHeadings; heading++) {
          float headingAngle = ((heading - numHeadings / 2) * 2 * M_PI) / numHeadings;
          float maxDistance = 100;

          x = maxDistance * cosf(headingAngle);
          y = maxDistance * sinf(headingAngle);

          yPxl = -x / distancePerPixel + size / 2;
          xPxl = -y / distancePerPixel + size / 2;

          int xOrig = size / 2;
          int yOrig = size / 2;
        
          // Bresenhams again, this time to find the outermost available pixel on this heading
          int dx = abs(xPxl - xOrig);
          int dy = abs(yPxl - yOrig);
          int sx = -1 + 2 * (xOrig < xPxl);
          int sy = -1 + 2 * (yOrig < yPxl);
          int err = dx-dy;

          while ((xOrig != xPxl || yOrig != yPxl) && (xOrig < size && xOrig >= 0 && yOrig < size && yOrig >=0) && availableImage.at<unsigned char>(yOrig, xOrig) == true) {
           //std::cout << currentRing << ": " << heading << ": " << xOrig << " " << yOrig << std::endl;

            // Calculate magnitudes to add
            prevXPxl[heading] = xOrig;
            prevYPxl[heading] = yOrig;

            int e2 = 2 * err;
            if (e2 > -dy) {
              err = err - dy;
              xOrig += sx;
            }
            if (e2 < dx) {
              err = err + dx;
              yOrig += sy;
            }
          }
        }

      }
    }

    for (int i = 0; i < size; i++) {

      float* tempHeightRow = tempHeight.ptr<float>(i);
      float* tempIntensityRow = tempIntensity.ptr<float>(i);
      float* avgImageRow = avgImage.ptr<float>(i);

      unsigned char* heightRow = heightImage.ptr<unsigned char>(i);
      unsigned char* intensityRow = intensityImage.ptr<unsigned char>(i);

      for (int j = 0; j < size; j++) {

        if (avgImageRow[j] == 0) {
          if (!config_.useOldImage) {
            heightRow[j] = 0;
            intensityRow[j] = 0;
          }
          continue;
        }

        // Height
        float z = tempHeightRow[j] / avgImageRow[j];
        int scaledZ = 127 * (1.0 - pow (M_E, -(z - config_.mean) * (z - config_.mean) / (2 * config_.sigma)));
        if (z < config_.mean) scaledZ *= -1;
        scaledZ += 128;
        heightRow[j] = scaledZ;

        // Intensity
        float intensity = tempIntensityRow[j] / avgImageRow[j];
        intensityRow[j] = (unsigned char) intensity;

      }
    }
  }

  void OverheadImageGenerator::generateSnapshots(
          const std::string prefix,
          const sensor_msgs::PointCloud2 &cloud2,
          Mat &oldHeightImage,
          float distancePerPixel) {
    
    bool config_use_old_image = config_.useOldImage;
    bool config_bleeding = config_.preventBleeding;
    Mat tempIntensityImage = cv::Mat::zeros(oldHeightImage.size(), oldHeightImage.type());
   
    Mat naiveImage = cv::Mat::zeros(oldHeightImage.size(), oldHeightImage.type());
    generateNaiveImage(cloud2, naiveImage, distancePerPixel);

    config_.useOldImage = false;
    config_.preventBleeding = false;
    Mat overheadImage = cv::Mat::zeros(oldHeightImage.size(), oldHeightImage.type());
    getOverheadImage(cloud2, overheadImage, tempIntensityImage, distancePerPixel);

    config_.preventBleeding = true;
    Mat noiseFreeImage = cv::Mat::zeros(oldHeightImage.size(), oldHeightImage.type());
    getOverheadImage(cloud2, noiseFreeImage, tempIntensityImage, distancePerPixel);

    config_.useOldImage = true;
    Mat odomImage;
    oldHeightImage.copyTo(odomImage);
    getOverheadImage(cloud2, odomImage, tempIntensityImage, distancePerPixel);

    config_.useOldImage = config_use_old_image;
    config_.preventBleeding = config_bleeding;

    // Save images to file
    imwrite(prefix + "_1.jpg", naiveImage);
    imwrite(prefix + "_2.jpg", overheadImage);
    imwrite(prefix + "_3.jpg", noiseFreeImage);
    imwrite(prefix + "_4.jpg", odomImage);

  }

  void OverheadImageGenerator::generateNaiveImage(
          const sensor_msgs::PointCloud2 &cloud2,
          Mat &heightImage,
          float distancePerPixel) {
    
    int size = heightImage.rows;

    Mat avgImage = Mat::zeros(size, size, CV_32F);
    Mat tempHeight = Mat::zeros(size, size, CV_32F);

    int numHeadings = velodyne::SCANS_PER_REV / velodyne::N_LASERS;
    int offsetToInt = 4 * 4;
    float x, y, z, heading;
    int ring;
    unsigned int intensity;
    bool available;

    int xPxl, yPxl;

    for (int currentRing = 1; currentRing < velodyne::N_LASERS; currentRing++) {
      for (unsigned int i = 0; i < cloud2.width * cloud2.height; i++) {

        const uint8_t* ptDataInt = reinterpret_cast<const uint8_t*>(&(cloud2.data[0]) + i * cloud2.point_step + offsetToInt);
        ring = ptDataInt[0];
        intensity = ptDataInt[1];
        available = (bool) ptDataInt[2];

        if (ring == currentRing && available) {
          const float* ptDataFloat = reinterpret_cast<const float*>(&(cloud2.data[0]) + i * cloud2.point_step);
          x = ptDataFloat[0];
          y = ptDataFloat[1];
          z = ptDataFloat[2];
          heading = atan2(y,x);
          //heading = ptDataFloat[3];

          yPxl = -x / distancePerPixel + size / 2;
          xPxl = -y / distancePerPixel + size / 2;

          if (xPxl >= 0 && xPxl < size && yPxl >=0 && yPxl < size) {
            avgImage.at<float>(yPxl, xPxl) += 1;
            tempHeight.at<float>(yPxl, xPxl) += z;
          }
        }
      }
    }

    for (int i = 0; i < size; i++) {

      float* tempHeightRow = tempHeight.ptr<float>(i);
      float* avgImageRow = avgImage.ptr<float>(i);

      unsigned char* heightRow = heightImage.ptr<unsigned char>(i);

      for (int j = 0; j < size; j++) {

        if (avgImageRow[j] == 0) {
          if (!config_.useOldImage) {
            heightRow[j] = 0;
          }
          continue;
        }

        // Height
        float z = tempHeightRow[j] / avgImageRow[j];
        int scaledZ = 127 * (1.0 - pow (M_E, -(z - config_.mean) * (z - config_.mean) / (2 * config_.sigma)));
        if (z < config_.mean) scaledZ *= -1;
        scaledZ += 128;
        heightRow[j] = scaledZ;

      }
    }
  }
}
