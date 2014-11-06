/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

#include <string>

class SLAMConfig 
{
public:
   //SLAM
   double data_association_threshold_new_feature;
   int min_existence_estimate_map_feature;
   int particle_count;
   int max_particle_copies;
   double threshold_effective_sample_size;
   // Robot & System environment
   bool use_points;
   bool use_lines;
   bool use_corners;   
   double odo_var_factor_distance;
   double odo_var_factor_orientation;
   // Files
   std::string best_map_file_name;
   std::string best_map_file_path;   
};

#endif // SLAM_CONFIG_H