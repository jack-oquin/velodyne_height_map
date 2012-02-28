#include <fstream>
#include <iostream>
#include <cstring>
#include <velodyne_pointcloud/calibration.h>
#include <angles/angles.h>

// Dimension scaling to convert cm to meters
#define VLS_DIM_SCALE 0.01

int readAnglesFile(const std::string angles_file, velodyne_pointcloud::Calibration& calibration) {

  // angles file does not support pitch/roll
  calibration.pitch = 0;
  calibration.roll = 0; 

  // read angles correction file for this specific unit
  std::ifstream config(angles_file.c_str());
  if (!config) {
    std::cerr << "Failure opening angles.config file: " << angles_file << std::endl;
    return -1;
  }

  int index = 0;
  float rotational = 0;
  float vertical = 0;
  int enabled = 0;
  float offset1 = 0;
  float offset2 = 0;
  float offset3 = 0;
  float horzCorr = 0;
  float vertCorr = 0;

  char buffer[256];
  while(config.getline(buffer, sizeof(buffer))) {
    if (buffer[0] == '#') { 
      continue;
    } else if (strcmp(buffer, "upper") == 0) {
      continue;
    } else if (strcmp(buffer, "lower") == 0) {
      continue;
    } else {

      // scan old angles file
      int len = sscanf(buffer,"%d %f %f %f %f %f %d", &index, &rotational, 
          &vertical, &offset1, &offset2, &offset3, &enabled);
      
      if (len != 7) { // not old angles file, scan for new one
        len = sscanf(buffer,"%d %f %f %f %f %f %f %f %d", &index,
                        &rotational, &vertical, &offset1, &offset2,
                        &offset3, &vertCorr, &horzCorr, &enabled);
        if (len != 9) {
          // not angles file
          continue;
        }
      } else { // old angles file, these parameters unavailable
        horzCorr = 0;
        vertCorr = 0;
      }

      std::pair<int, velodyne_pointcloud::LaserCorrection> correction;

      correction.first = index;

      velodyne_pointcloud::LaserCorrection& laser_correction = correction.second;
      laser_correction.rot_correction = angles::from_degrees(rotational);
      laser_correction.vert_correction = angles::from_degrees(vertical);
      laser_correction.dist_correction = offset3 * VLS_DIM_SCALE;  // convert to meters from centimeters
      laser_correction.dist_correction_x = 0; // unsupported
      laser_correction.dist_correction_y = 0; // unsupported
      laser_correction.vert_offset_correction = vertCorr * VLS_DIM_SCALE; // convert to meters
      laser_correction.horiz_offset_correction = horzCorr * VLS_DIM_SCALE; // convert to meters

      laser_correction.max_intensity = 255;
      laser_correction.min_intensity = 0;

      laser_correction.focal_distance = 0; // unsupported
      laser_correction.focal_slope = 0; // unsupported
      
      calibration.laser_corrections.insert(correction);
    }
  }

  config.close();
  return 0;
}

int main(int argc, const char *argv[]) {
  
  if (argc != 3) {
    std::cerr << "USAGE: angles2yaml angles.config out.yaml" << std::endl;
    return -1;
  }

  std::string angles_file(argv[1]);
  std::string yaml_file(argv[2]);

  velodyne_pointcloud::Calibration calibration;
  int result = readAnglesFile(angles_file, calibration);
  if (result != 0) {
    std::cerr << "Error reading angles file: " << angles_file << std::endl;
    return result;
  }

  calibration.write(yaml_file);

  return 0;
}
