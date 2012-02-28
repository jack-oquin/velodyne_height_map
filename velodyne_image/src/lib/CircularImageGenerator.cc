/**
 * \file  CircularImageGenerator.cc
 * \brief  Definitions for the CircularImageGenerator class
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/16/2011 02:35:44 PM piyushk $
 */

#include <velodyne/data.h>
#include <velodyne/ring_sequence.h>

#include <velodyne_image_generation/CircularImageGenerator.h>
#include <velodyne_image_generation/ImageRef.h>

namespace velodyne_image_generation {

  /**
   * \brief   Corrects the image for those points that have failed.
   */
  void CircularImageGenerator::correctFailedPoints(IplImage * indexImage, IplImage * img) {

    BwImage valueImage(img);
    BwImageInt index(indexImage);

    bool atTop, atBottom, atLeft, atRight;
    for (int i = 0; i < config_.numLasers; i++) {
      atTop = i == 0;
      atBottom = i == config_.numLasers - 1;
      for (int j = 0; j < config_.pointsPerLaser; j++) {

        if (index[i][j] != FAILED_POINT)
          continue;

        atLeft = j == 0;
        atRight = j == config_.pointsPerLaser - 1;

        int count = 0, value = 0;

        if (!atTop && !atLeft && index[i - 1][j - 1] != FAILED_POINT)
          value += valueImage[i - 1][j - 1], count++;
        if (!atTop && index[i - 1][j] != FAILED_POINT)
          value += valueImage[i - 1][j], count++;
        if (!atTop && !atRight && index[i - 1][j + 1] != FAILED_POINT)
          value += valueImage[i - 1][j + 1], count++;
        if (!atLeft && index[i][j - 1] != FAILED_POINT)          
          value += valueImage[i][j - 1], count++;
        if (!atRight && index[i][j + 1] != FAILED_POINT) 
          value += valueImage[i][j + 1], count++;
        if (!atBottom && !atLeft && index[i + 1][j - 1] != FAILED_POINT)
          value += valueImage[i + 1][j - 1], count++;
        if (!atBottom && index[i + 1][j] != FAILED_POINT)
          value += valueImage[i + 1][j], count++;
        if (!atBottom && !atRight && index[i + 1][j + 1] != FAILED_POINT)
          value += valueImage[i + 1][j + 1], count++;

        if (count > config_.missingSurroundPoints) {
          valueImage[i][j] = value / count;
        }
        
      }
    }

  }

  /**
   * \brief   calculates the index of the point in the point cloud
   *          corresponding to each image pixel
   */
  void CircularImageGenerator::calculateIndices(
      const sensor_msgs::PointCloud &pc,
      IplImage * indexImage) {

    BwImageInt index(indexImage);

    for (int i = 0; i < config_.numLasers; i++) {
      for (int j = 0; j < config_.pointsPerLaser; j++) {
        index[i][j] = MISSING_POINT; // Initialize all points as missing
      }
    }

    for (unsigned int i = 0; i < pc.points.size(); i++) {
      
      float heading = pc.channels[1].values[i];
      int hzIndex = 
          (int) (heading / (M_PI * 2.0f) * config_.pointsPerLaser) + config_.pointsPerLaser / 2;
      if (hzIndex == config_.pointsPerLaser) {
        hzIndex = 0;
      }
      hzIndex = config_.pointsPerLaser - 1 - hzIndex; // Flip horizontal
      int vtIndex = pc.channels[0].values[i];
      vtIndex = config_.numLasers - 1 - vtIndex; // Flip Vertical

      // If the point is missing, introduce the failed point, but if it is failed try to write over with a valid point
      unsigned int idx = i; 
      // Check if the point had failed
      if (!(pc.channels[3].values[i])) {
        idx = FAILED_POINT;
      }

      if (index[vtIndex][hzIndex] == MISSING_POINT || index[vtIndex][hzIndex] == FAILED_POINT) {
        index[vtIndex][hzIndex] = idx;
      }

    }

    bool atTop, atBottom, atLeft, atRight;
    for (int i = 0; i < config_.numLasers; i++) {
      atTop = i == 0;
      atBottom = i == config_.numLasers - 1;
      for (int j = 0; j < config_.pointsPerLaser; j++) {

        if (index[i][j] != FAILED_POINT)
          continue;

        atLeft = j == 0;
        atRight = j == config_.pointsPerLaser - 1;

        int count = 0;

        if (!atTop && !atLeft && index[i - 1][j - 1] > FAILED_POINT)
          count++;
        if (!atTop && index[i - 1][j] > FAILED_POINT)
          count++;
        if (!atTop && !atRight && index[i - 1][j + 1] > FAILED_POINT)
          count++;
        if (!atLeft && index[i][j - 1] > FAILED_POINT)          
          count++;
        if (!atRight && index[i][j + 1] > FAILED_POINT) 
          count++;
        if (!atBottom && !atLeft && index[i + 1][j - 1] > FAILED_POINT)
          count++;
        if (!atBottom && index[i + 1][j] > FAILED_POINT)
          count++;
        if (!atBottom && !atRight && index[i + 1][j + 1] > FAILED_POINT)
          count++;

        if (count <= config_.missingSurroundPoints) {
          index[i][j] = OUT_OF_RANGE_POINT;
        }
        
      }
    }
  }

  /**
   * \brief   obtains the intensity value from the pc and fills the img
   *          by referring the indexImage
   */
  void CircularImageGenerator::getIntensityImage (
      const sensor_msgs::PointCloud &pc,
      IplImage * indexImage,
      IplImage * img) {

    BwImage intensityImage(img);
    BwImageInt index(indexImage);

    for (int i = 0; i < config_.numLasers; i++) {
      for (int j = 0; j < config_.pointsPerLaser; j++) {

        bool pointAvailable = 
            index[i][j] != MISSING_POINT && index[i][j] != FAILED_POINT && index[i][j] != OUT_OF_RANGE_POINT;
        bool usePreviousPoint =
            (config_.correctGaps && index[i][j] == MISSING_POINT) ||
            (config_.correctFailures && index[i][j] == FAILED_POINT);

        if (pointAvailable) {
          intensityImage[i][j] = (unsigned char) pc.channels[2].values[index[i][j]];
        } else if (!usePreviousPoint) {
          intensityImage[i][j] = 0;
        }
      }
    }

  }

  /**
   * \brief   obtains the height value from the pc and fills the img
   *          by referring the indexImage
   */
  void CircularImageGenerator::getLinearHeightImage(
      const sensor_msgs::PointCloud &pc,
      IplImage *indexImage,
      IplImage *img) {

    BwImage heightImage(img);
    BwImageInt index(indexImage);

    float maxHeight = 0;
    float minHeight = 0;

    // Decide linear scaling
    if (config_.useConstantScale) {
      maxHeight = config_.maxHeight;
      minHeight = config_.minHeight;
    } else {
      maxHeight = -1000.0f;
      minHeight = 1000.0f;
      for (unsigned int i = 0; i < pc.points.size(); i++) {
        float z = pc.points[i].z;
        maxHeight = (z > maxHeight) ? z : maxHeight;
        minHeight = (z < minHeight) ? z : minHeight;
      }
    }

    for (unsigned int i = 0; i < (unsigned int) config_.numLasers; i++) {
      for (unsigned int j = 0; j < (unsigned int) config_.pointsPerLaser; j++) {

        bool pointAvailable = 
            index[i][j] != MISSING_POINT && index[i][j] != FAILED_POINT && index[i][j] != OUT_OF_RANGE_POINT;
        bool usePreviousPoint =
            (config_.correctGaps && index[i][j] == MISSING_POINT) ||
            (config_.correctFailures && index[i][j] == FAILED_POINT);

        if (pointAvailable) {
          float z = pc.points[index[i][j]].z;
          int scaledZ = ((z - minHeight) * 255) / (maxHeight - minHeight);
          scaledZ = (scaledZ < 0) ? 0 : scaledZ;
          scaledZ = (scaledZ > 255) ? 255 : scaledZ;
          heightImage[i][j] = scaledZ;
        } else if (!usePreviousPoint) {
          heightImage[i][j] = 0;
        }

      }
    }
    
  }

  /**
   * \brief   obtains the height value from the pc and fills the img
   *          by referring the indexImage (contrast enhanced)
   */
  void CircularImageGenerator::getGaussianHeightImage(
      const sensor_msgs::PointCloud &pc,
      IplImage *indexImage,
      IplImage *img) {

    BwImage heightImage(img);
    BwImageInt index(indexImage);

    for (unsigned int i = 0; i < (unsigned int) config_.numLasers; i++) {
      for (unsigned int j = 0; j < (unsigned int) config_.pointsPerLaser; j++) {
        
        bool pointAvailable = 
            index[i][j] != MISSING_POINT && index[i][j] != FAILED_POINT && index[i][j] != OUT_OF_RANGE_POINT;
        bool usePreviousPoint =
            (config_.correctGaps && index[i][j] == MISSING_POINT) ||
            (config_.correctFailures && index[i][j] == FAILED_POINT);

        if (pointAvailable) {
          float z = pc.points[index[i][j]].z;
          int scaledZ = 127 * (1.0 - pow (M_E, -(z - config_.mean) * (z - config_.mean) / (2 * config_.sigma)));
          if (z < config_.mean) scaledZ *= -1;
          scaledZ += 128;
          heightImage[i][j] = scaledZ;
        } else if (!usePreviousPoint) {
          heightImage[i][j] = 0;
        }
      }
    }

  }

  /**
   * \brief   obtains the distance value from the pc and fills the img
   *          by referring the indexImage
   */
  void CircularImageGenerator::getDistanceImage(
      const sensor_msgs::PointCloud &pc,
      IplImage *indexImage,
      IplImage *img) {

    BwImage distanceImage(img);
    BwImageInt index(indexImage);

    for (unsigned int i = 0; i < (unsigned int) config_.numLasers; i++) {
      for (unsigned int j = 0; j < (unsigned int) config_.pointsPerLaser; j++) {
        
        bool pointAvailable = 
            index[i][j] != MISSING_POINT && index[i][j] != FAILED_POINT && index[i][j] != OUT_OF_RANGE_POINT;
        bool usePreviousPoint =
            (config_.correctGaps && index[i][j] == MISSING_POINT) ||
            (config_.correctFailures && index[i][j] == FAILED_POINT);

        if (pointAvailable) {
          float x = pc.points[index[i][j]].x;
          float y = pc.points[index[i][j]].y;
          float z = pc.points[index[i][j]].z;
          float distance = sqrtf(x*x + y*y + z*z);
          int scaledDistance = 255 * (1.0 - pow (M_E, -(distance * distance) / (2 * 50.0)));
          distanceImage[i][j] = scaledDistance;
        } else if (!usePreviousPoint){
          distanceImage[i][j] = 0;
        }
      }
    }
  }

}
