/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#ifndef SLAM_LOCAL_MAP_H
#define SLAM_LOCAL_MAP_H

#include "slam_landmark.h"

struct map_element
{
   std::vector<point2d> v_points_;
   std::vector<double> v_angles_;
   bool visibility_;
   SLAMLandmark * landmark_;
};

class SLAMLocalMap
{
private:
   std::vector<map_element> v_map_elements_;
public:
   void pushElement( std::vector<point2d> , SLAMLandmark * );
   void setVisibilities( int );
};

#endif // SLAM_LOCAL_MAP_H
