/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#ifndef SLAM_OBSERVATION_H
#define SLAM_OBSERVATION_H

#define _USE_MATH_DEFINES
// Standard C++
#include <sstream>
#include <queue>
#include <math.h>
#include <numeric>
#include <tr1/memory>
// ROS 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
// ROS - IPA
#include "ipa_navigation_msgs/FeatureList.h"
#include "ipa_navigation_msgs/PointFeature.h"
// EIGEN
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include "slam_landmark.h"

class SLAMObservation;
class SLAMLandmark;

bool transformPoint( const geometry_msgs::Point& point_in , Vector2d& point_out , const std_msgs::Header& header_in );

class SLAMObservation
{
protected:
   SLAMObservation() {};
   SLAMObservation( const SLAMObservation& ) {};
   MatrixXd cov_;
public:
   MatrixXd getCovar() { return cov_; }
   virtual SLAMLandmark * createFeature( Vector3d , double ) = 0;
   virtual SLAMObservation * getCopy() = 0;
   virtual bool suitableInitObs() { return true; } //!< Is this observation suitable for Feature creation?
};

class SLAMPointObservation : public SLAMObservation
{
private:
   Vector2d point_;
public:
   SLAMPointObservation( const ipa_navigation_msgs::PointFeature& , const std_msgs::Header& , tf::TransformListener * );
   SLAMPointObservation( const SLAMPointObservation& old );
//    ~SLAMPointObservation() { cov_.resize(0,0); }
   SLAMLandmark * createFeature( Vector3d , double );
   Vector2d getPoint() const;
   SLAMPointObservation * getCopy();
   bool suitableInitObs();
};

class SLAMLineObservation : public SLAMObservation
{
private:
   Vector2d start_;
   Vector2d end_;
public:
   SLAMLineObservation( const ipa_navigation_msgs::LineFeature& , const std_msgs::Header& , tf::TransformListener * );
   SLAMLineObservation( const SLAMLineObservation& old );
//    ~SLAMLineObservation() { cov_.resize(0,0); }
   SLAMLandmark * createFeature( Vector3d , double );
   bool suitableInitObs();
   Vector2d getStart() const;
   Vector2d getEnd() const;
   SLAMLineObservation * getCopy();
};

#endif // SLAM_OBSERVATION_H
