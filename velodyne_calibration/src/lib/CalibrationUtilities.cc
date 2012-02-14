/**
 * \file  CalibrationUtilities.cc
 * \brief  
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, The University of Texas at Austin
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:36:36 AM piyushk $
 */

namespace velodyne {

  const std::string NUM_LASERS = "num_lasers";
  const std::string PITCH = "pitch";
  const std::string ROLL = "roll";
  const std::string LASERS = "lasers";
  const std::string LASER_ID = "laser_id";
  const std::string ROT_CORRECTION = "rot_correction";
  const std::string VERT_CORRECTION = "vert_correction";
  const std::string DIST_CORRECTION = "dist_correction";
  const std::string VERT_OFFSET_CORRECTION = "vert_offset_correction";
  const std::string HORIZ_OFFSET_CORRECTION = "vert_offset_correction";
  const std::string INTENSITY = "intensity";
  const std::string A = "a";
  const std::string B = "b";
  const std::string C = "c";
  const std::string D = "d";

  void operator >> (const YAML::Node& node, IntensityCorrection& intensity_correction) {
    node[A] >> intensity_correction.a;
    node[B] >> intensity_correction.b;
    node[C] >> intensity_correction.c;
    node[D] >> intensity_correction.d;
  }

  void operator >> (const YAML::Node& node, std::pair<int, LaserCorrection>& correction) {
    node[LASER_ID] >> correction.first;
    node[ROT_CORRECTION] >> correction.second.rot_correction;
    node[VERT_CORRECTION] >> correction.second.vert_correction;
    node[DIST_CORRECTION] >> correction.second.dist_correction;
    node[VERT_OFFSET_CORRECTION] >> correction.second.vert_offset_correction;
    node[HORIZ_OFFSET_CORRECTION] >> correction.second.horiz_offset_correction;
    node[INTENSITY] >> correction.second.intensity_correction;
  }

  void operator >> (const YAML::Node& node, Calibration& calibration) {
    node[NUM_LASERS] >> calibration.num_lasers;
    node[PITCH] >> calibration.pitch;
    node[ROLL] >> calibration.roll;

    calibration.laser_corrections.clear();
    for (int i = 0; i < calibration.num_lasers; i++) {
      std::pair<int, LaserCorrection> correction;
      node[LASERS] >> correction;
      calibration.laser_corrections.insert(correction);     
    }
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const std::pair<int, LaserCorrection> correction) {
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
    out << YAML::Key << LASER_ID << YAML::Value << correction.first;
  }

  YAML::Emitter& operator << (YAML::Emitter& out, const Calibration& calibration) {
    out << YAML::Key << NUM_LASERS << YAML::Value << calibration.num_lasers;
    out << YAML::Key << PITCH << YAML::Value << calibration.pitch;
    out << YAML::Key << ROLL << YAML::Value << calibration.roll;
    out << YAML::Key << LASERS << YAML::BeginSeq;
    for (std::map<int, LaserCorrection>::Iterator it = calibration.laser_corrections.begin();
        it != calibration.laser_correction.end(); it++) {
      out << *it; 
    }
    out << YAML::EndMap;
  }

  void readCalibrationFromFile(const std::string& calibration_file, 
      Calibration& calibration);

  void writeCalibrationToFile(const std::string& calibration_file, 
      const Calibration& calibration);
  
} /* velodyne */
