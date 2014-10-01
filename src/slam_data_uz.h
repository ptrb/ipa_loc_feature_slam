#ifndef SLAM_DATA_UZ_H
#define SLAM_DATA_UZ_H

#define _USE_MATH_DEFINES
// Standard C++
#include <sstream>
#include <queue>
#include <math.h>
#include <numeric>
// ROS 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
// ROS - IPA
#include "ipa_navigation_msgs/FeatureList.h"
#include "ipa_navigation_msgs/PointFeature.h"
// BOOST
#include "boost/foreach.hpp"
#include "boost/random.hpp"
#include "boost/random/normal_distribution.hpp"
#include "boost/random/uniform_real.hpp"
#include <boost/random/mersenne_twister.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
// EIGEN
#include <eigen3/Eigen/Dense>
using namespace Eigen;
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
// IPA LOC FEATURE SLAM

#include "slam_observation.h"

/*! \brief SLAMData contains one or more observation and a timestamp reference.
 * 
 */
class SLAMData
{
   private:
      std::vector<SLAMObservation *> v_observations_; //!< All observations for this instance, which can be of different types.  
      double timestamp_; //!< The timestamp will serve to determine odometry parameters.
   public:
      SLAMData( const ipa_navigation_msgs::FeatureList::ConstPtr& , tf::TransformListener * );
      ~SLAMData();
      SLAMData( const SLAMData& );
      void pushObservations( const ipa_navigation_msgs::FeatureList::ConstPtr& , tf::TransformListener * );
      std::vector<SLAMObservation *> getObservations() const; //!< Get all observations, as vector of Pointers.
      
      double getTimestamp(){ return timestamp_; }; //!< Get the timestamp.
      
      int sensor;
};

#endif