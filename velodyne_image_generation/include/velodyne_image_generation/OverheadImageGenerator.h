/**
 * \file  OverheadImageGenerator.h
 * \brief Header for converting a PointCloud2 message into overhead images 
 *
 * This class converts velodyne data into overhead images using bresehams to 
 * fill in the discontinuities. This is useful as the 3d data is in an easy to
 * understand perspective
 *
 * \author  Piyush Khandelwal (piyushk), piyushk@cs.utexas.edu
 * Copyright (C) 2011, The University of Texas at Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/16/2011 02:29:46 PM piyushk $
 */

#ifndef OVERHEADIMAGEGENERATOR_QAUQWXVA
#define OVERHEADIMAGEGENERATOR_QAUQWXVA

#include <opencv/cv.h>
#include <sensor_msgs/PointCloud2.h>

#include <velodyne_image_generation/OverheadImageConfig.h>

namespace velodyne_image_generation {

  class OverheadImageGenerator {

    private:

      OverheadImageConfig config_;  ///< configuration file (used in conjunction with dynamic reconfigure)
     
    public:

      /**
       * \brief   Takes a PointCloud2 message and obtains a height and 
       *          intensity image from it.
       *
       * \param   cloud2 The source PointCloud2 message
       * \param   heightImage The input image on which all changes are made.
       *          Depending on the configuration, some portions of the old 
       *          image can be retained.
       * \param   intensityImage Similar to heightImage, but corresponding to
       *          intensity values
       * \param   distancePerPixel Defines the resolution of the image
       *
       * \todo distancePerPixel needs to be a parameter in config
       */
      void getOverheadImage(
          const sensor_msgs::PointCloud2 &cloud2,
          cv::Mat &heightImage,
          cv::Mat &intensityImage,
          float distancePerPixel
      );

      /**
       * \brief   Accept reconfiguration request from dynamic reconfigure
       */
      void reconfigure(OverheadImageConfig newConfig) {
        config_ = newConfig;
      }

      /**
       * \brief   Returns current configuration settings
       */
      OverheadImageConfig* getConfig() {
        return &config_;
      }
  };
}

#endif /* end of include guard: OVERHEADIMAGEGENERATOR_QAUQWXVA */
