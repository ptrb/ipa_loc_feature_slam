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
   double data_association_threshold_new_feature;
   int min_existence_estimate_map_feature;
   std::string best_map_file_name;
   std::string best_map_file_path;
};

#endif // SLAM_CONFIG_H