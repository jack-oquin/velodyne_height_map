/**
 * \file  OverheadImageGenerator.cc
 * \brief Definitions for the OverheadImageGenerator class 
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/16/2011 02:38:38 PM piyushk $
 */

#include <velodyne/data.h>
#include <velodyne/ring_sequence.h>

#include <velodyne_image_generation/OverheadImageGenerator.h>
#include <velodyne_image_generation/ImageRef.h>

using namespace cv;

namespace velodyne_image_generation {

  /**
   * \brief   Takes a PointCloud2 message and obtains a height and 
   *          intensity image from it.
   */
  void OverheadImageGenerator::getOverheadImage(
          const sensor_msgs::PointCloud2 &cloud2,
          Mat &heightImage,
          Mat &intensityImage,
          float distancePerPixel
      ) {

    int size = heightImage.rows;

    Mat availableImage = Mat::zeros(size, size, CV_8U);
    Mat avgImage = Mat::zeros(size, size, CV_32F);
    Mat avgHeightImage = Mat::zeros(size, size, CV_32F);
    Mat tempHeight = Mat::zeros(size, size, CV_32F);
    Mat tempHeightDiff = Mat::zeros(size, size, CV_32F);
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
            if (!config_.useOldImage || !isNoise) {
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

    // Compute the differences before the compression loses the image quality
    for (int heading = 0; heading < numHeadings; heading++) {
      float headingAngle = ((heading - numHeadings / 2) * 2 * M_PI) / numHeadings;
      float maxDistance = 100;
      float prevZ = 0;
      bool prevZAvailable = false;

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

        if (prevZAvailable) {
          if (avgImage.at<float>(yOrig,xOrig) != 0) { // current z available
            float zDiff = tempHeight.at<float>(yOrig,xOrig) / avgImage.at<float>(yOrig,xOrig) - prevZ;
            tempHeightDiff.at<float>(yOrig,xOrig) += zDiff;
            avgHeightImage.at<float>(yOrig,xOrig) += 1;
          }
        }

        if (avgImage.at<float>(yOrig,xOrig) != 0) {
          prevZ = tempHeight.at<float>(yOrig,xOrig) / avgImage.at<float>(yOrig,xOrig);
          prevZAvailable = true;
        } else {
          prevZAvailable = false;
        }

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

      tempHeightDiff.at<float>(size/2,size/2) = 0;
      avgHeightImage.at<float>(size/2,size/2) = 1;
    }


    for (int i = 0; i < size; i++) {

      //float* tempHeightRow = tempHeight.ptr<float>(i);
      float* tempHeightDiffRow = tempHeightDiff.ptr<float>(i);
      float* tempIntensityRow = tempIntensity.ptr<float>(i);
      float* avgImageRow = avgImage.ptr<float>(i);
      float* avgHeightImageRow = avgHeightImage.ptr<float>(i);

      unsigned char* heightRow = heightImage.ptr<unsigned char>(i);
      unsigned char* intensityRow = intensityImage.ptr<unsigned char>(i);

      for (int j = 0; j < size; j++) {

        if (avgImageRow[j] == 0) {
          if (!config_.useOldImage) {
            intensityRow[j] = 0;
          }
          continue;
        }
        if (avgHeightImageRow[j] == 0) {
          if (!config_.useOldImage) {
            heightRow[j] = 0;
          }
          continue;
        }

        // Height
        /*
        float z = tempHeightRow[j] / avgImageRow[j];
        int scaledZ = 127 * (1.0 - pow (M_E, -(z - config_.mean) * (z - config_.mean) / (2 * config_.sigma)));
        if (z < config_.mean) scaledZ *= -1;
        scaledZ += 128;
        */
        float z = tempHeightDiffRow[j] / avgHeightImageRow[j];
        int scaledZ = 255 * (1.0 - pow (M_E, -z*z / (2 * config_.sigma)));
        heightRow[j] = scaledZ;

        // Intensity
        float intensity = tempIntensityRow[j] / avgImageRow[j];
        intensityRow[j] = (unsigned char) intensity;

      }
    }
  }
}
