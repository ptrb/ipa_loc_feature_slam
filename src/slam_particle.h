/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#ifndef SLAM_PARTICLE_H
#define SLAM_PARTICLE_H


// BOOST
#include "boost/foreach.hpp"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
// EIGEN
#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include "slam_sensor_data.h"
#include "slam_landmark.h"
#include "slam_config.h"

/*! \brief A FastSLAM-particle with a pose and landmarks of different types.
 * 
 */
class SLAMParticle
{
private:
   Vector3d pose_; //!< The pose
   Vector3d sampled_pose_; //!< Sampled pose
   Vector3d calculated_pose_; //!< Calculated pose
   std::vector<SLAMLandmark *> v_landmarks_; //!< The landmarks are stored as pointers. All derived SLAMLandmark-classes can be used.
   Matrix<double, 3, 3> covariance_odo_; //!< Covariance of the odometry. Calculated with the Particle's orientation and cached in this variable.
   
   double mergeSimultaneousFeatures(); //!< Merge features that are quite similar. 
   void samplePose( Vector3d& ); //!< Sample the pose with the given odometry parameters.
   void calcPose( Vector3d& ); //!< Calculate pose with the given odometry parameters without noise.   
   void setPose( Vector3d in ) { pose_ = in; calculated_pose_ = in; sampled_pose_ = in; }   
   void setCovarianceOdo( SLAMSensorData& );   
public:
   SLAMParticle( const Vector3d& );
   SLAMParticle( const SLAMParticle & old );
   ~SLAMParticle();
   static boost::mt19937 rng; //!< Random generator, used for pose sampling.   
   double updateParticle( SLAMSensorData , SLAMConfig& ); //!< Run FastSLAM iteration step for this particle's map according to the given observation data.

   Vector3d getPose() const; //!< Returns the current pose.
   Vector3d getCalculatedPose() { return calculated_pose_; }
   Vector3d getSampledPose() { return sampled_pose_; }
   long getLandmarkCount(); //!< Get current count of landmark
   std::vector<visualization_msgs::Marker> getFeatureMarkers( ros::Time& , SLAMConfig& ); //!< Markers for the various landmark types.
   std::vector<visualization_msgs::Marker> getFeatureMarkersCandidates( ros::Time& , SLAMConfig& ); //!< Markers for the various landmark types.
};

#endif // SLAM_PARTICLE_H
