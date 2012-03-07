/**
 * \file  overhead_image_generator.h
 * \brief Header for converting a PointCloud into overhead images 
 *
 * This class converts velodyne data into orthographic "overhead" images. 
 * Missing data is interpolated using Breseham's line algorithm 
 * 
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2011, UT Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/16/2011 02:29:46 PM piyushk $
 */

#ifndef OVERHEAD_IMAGE_GENERATOR_QAUQWXVA
#define OVERHEAD_IMAGE_GENERATOR_QAUQWXVA

#include <opencv/cv.h>
#include <pcl/point_cloud.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/ring_sequence.h>
#include <velodyne_image/OverheadImageConfig.h>

namespace velodyne_image {

  // Shorthand typedefs for point cloud representations
  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  const unsigned int NUM_HEADINGS = 
    velodyne_rawdata::SCANS_PER_REV / velodyne_rawdata::N_LASERS;

  class OverheadImageGenerator {

    private:

      OverheadImageConfig config_;  ///< configuration file 
     
    public:

      /**
       * \brief Takes a pcl::PointCloud and obtains a height and 
       *        intensity image from it.
       *
       * \param cloud The source pcl::PointCloud message
       * \param height_image The input image on which all changes are made.
       *        Depending on the configuration, some portions of the old 
       *        image can be retained.
       * \param intensity_image Similar to height_image, but corresponding to
       *        intensity values
       */
      void getOverheadImages(const VPointCloud& cloud, cv::Mat& height_image,
          cv::Mat& disparity_image, cv::Mat& intensity_image);

      /**
       * \brief Gets an edge intensity image from the height image, and at the
       *        same time avoiding low level noise. This method is used as a
       *        replacement for Canny, as the version of Canny inside OpenCV
       *        requires 8bit images
       */
      void getDisparityImage(const cv::Mat& height_image, 
          const cv::Mat& avg_image, cv::Mat& dst, float threshold);

      /** 
       * \brief For visualization only - do not use in production code
       */
      void generateSnapshots(const std::string& prefix, 
          const VPointCloud& cloud, const cv::Mat& prev_height_image);

      /** 
       * \brief For visualization only - do not use in production code
       */
      void getNaiveImages(const VPointCloud& cloud, cv::Mat& height_image,
          cv::Mat& intensity_image);

      /**
       * \brief Accept reconfiguration request from dynamic reconfigure
       */
      void reconfigure(const OverheadImageConfig& new_config) {
        config_ = new_config;
      }

      /**
       * \brief   Returns current configuration settings
       */
      OverheadImageConfig getConfig() {
        return config_;
      }
  };
}

#endif /* end of include guard: OVERHEAD_IMAGE_GENERATOR_QAUQWXVA */
