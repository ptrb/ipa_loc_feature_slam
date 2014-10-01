/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#ifndef SLAM_PROCESSOR_H
#define SLAM_PROCESSOR_H

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
// EIGEN
#include <eigen3/Eigen/Dense>
using namespace Eigen;
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
// IPA LOC FEATURE SLAM
#include "slam_particle.h"
#include "slam_config.h"
#include "slam_data_uz.h"

/*! \brief Class SLAMProcessor maintains the input data, coordainte frame stuff, and manages all particles.
 *  
 * 
 */
class SLAMProcessor
{
private:
   std::queue<SLAMData> q_observations_; //!< All observations are stored in this queue. One element will be processed per FastSLAM-iteration ( update() )
   std::vector<SLAMParticle> v_particles_; //!< The FastSLAM-particles.
   const static int count_particles_ = 100; //!< The count of particles.
   
   static tf::TransformListener * tf_listener_; //!< Pointer to a TransformListener that will be used for all transformations.
   bool init_; //!< true if the SLAMProcessor is initialized, what means that transformation parameters are known (base_link <-> odom_combined). This can not be guranteed by the constructor, because the necessary transform is may not possible.
   
   static double latest_odo_pose_[3]; //!< Pose in /fast_slam-frame. Will be used for visualization of the plain odometry position.
   static tf::Stamped<tf::Pose> tf_global_odo_transformation_; //!< Transformation between the global frame as used by all particles & landmarks and the current robot position.
   
   std::vector<double> v_weights_; //!< Accumulated weights, one per particle.
   visualization_msgs::MarkerArray best_slam_map_; //!< The best map currently available. Chosen by particle-weights.
   
   void resampleParticles( std::vector<double> ); //!< Resample particles according to their weights.
   bool getLocalOdom( const ros::Time& , std::vector<double>& , tf::Stamped<tf::Pose>& ) const; //!< Calculate odometry from latest pose to the robot's position at the given instant.
   bool deadReckoning( const ros::Time& , std::vector<double>& , tf::Stamped<tf::Pose>& , double [3] ) const; //!< Get odometry for given instant and do the dead reckoning for the frame transformations.
   int updateBestMap( SLAMConfig& ); //!< Get the highest-rated particle's map.   
   
   // Devlopment
   void trackBestMap( int );
   
public:
   SLAMProcessor( tf::TransformListener * ); //!< Constructor needs an existing TransformListener
   bool update( SLAMConfig& ); //!< Process one FastSLAM-step. 
   bool getInitStatus() {return init_; } //!< Get the initalization status.
   void pushNewData( const ipa_navigation_msgs::FeatureList::ConstPtr& ); //!< Push data that will be stored as SLAMData and later be processed through update()
   void configure( SLAMConfig& );
   tf::StampedTransform getSlamMapTransform(); //!< Returns the transformation parameters between the self defined slam-map and current robot position.
   
   // Visualization
   geometry_msgs::PoseStamped visualizePoseFromOdometry(); //!< Visualize all particle-poses
   visualization_msgs::MarkerArray getFeatureMarkers(); //!< Visualize all maps
   visualization_msgs::MarkerArray getFeatureMarkersCandidates(); //!< Visualize all maps
   visualization_msgs::Marker getParticleMarkers() const;   //!< Visualize particles
   visualization_msgs::MarkerArray getBestMap() const { return best_slam_map_; } //!< Visualize the best_slam_map   
   

};

#endif // SLAM_PROCESSOR_H
