/**
 * \file  circular_image_generator.h
 * \brief Header for converting a PointCloud into circular images
 *
 * This class converts velodyne data into a circular image - somewhat akin to
 * range images.
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2011, UT Austin, Austin Robot Technology
 *
 * License: Modified BSD License
 *
 * $ Id: 08/16/2011 02:04:00 PM piyushk $
 */

#ifndef CIRCULAR_IMAGE_GENERATOR_XB6OIK9U
#define CIRCULAR_IMAGE_GENERATOR_XB6OIK9U

#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_image_generation/circular_image_config.h>

namespace velodyne_image {

  // Shorthand typedefs for point cloud representations
  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  /**
   * \class CircularImageGenerator
   * \brief Contains class declarations to generate circular images from
   *        velodyne clouds
   */
  class CircularImageGenerator {

    private:

      CircularImageConfig config_;  ///< configuration file 

    public:

      /**
       * \brief   obtains the intensity value from the pc and fills the img
       *          by referring the indexImage
       */
      void getCircularImages(const VPointCloud& cloud, cv::Mat& height_image,
          cv::Mat& intensity_image);
      
      /**
       * \brief   Accept reconfiguration request from dynamic reconfigure
       */
      void reconfigure(const CircularImageConfig& new_config) {
        config_ = new_config;
      }

      /**
       * \brief   Returns current configuration settings
       */
      CircularImageConfig getConfig() {
        return config_;
      }
  };
}

#endif /* end of include guard: CIRCULAR_IMAGE_GENERATOR_XB6OIK9U */
