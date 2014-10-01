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

#include "slam_data_uz.h"
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
   std::vector<SLAMLandmark *> v_landmark_candidates_; //!< Landmark candidates
   Matrix<double, 3, 3> covariance_odo_; //!< Covariance of the odometry. Calculated with the Particle's orientation and cached in this variable.
   
   double pickCandidates( SLAMConfig& ); //!< Pick candidates and add or merge them to map.
   double mergeSimultaneousFeatures(); //!< Merge features that are quite similar. 
public:
   static boost::mt19937 rng; //!< Random generator, used for pose sampling.
   SLAMParticle( const Vector3d& );
   SLAMParticle( const SLAMParticle & old );
   ~SLAMParticle();
   std::vector<double> updateParticle( SLAMData , SLAMConfig& ); //!< Run FastSLAM iteration step for this particle's map according to the given observation data.
   void samplePose( double [4] ); //!< Sample the pose with the given odometry parameters.
   void calcPose( double [4] ); //!< Calculate pose with the given odometry parameters without noise.
   Vector3d getPose() const; //!< Returns the current pose.
   Vector3d getCalculatedPose() { return calculated_pose_; }
   Vector3d getSampledPose() { return sampled_pose_; }
   void setPose( Vector3d in ) { pose_ = in; calculated_pose_ = in; sampled_pose_ = in; }
   
   
   long getLandmarkCount(); //!< Get current count of landmark
   long getCandidateCount() { return v_landmark_candidates_.size(); }

   void mergeSimultaneousCandidates();
   void setCovarianceOdo( double [4] );
   
   std::vector<visualization_msgs::Marker> getFeatureMarkers( ros::Time& ); //!< Markers for the various landmark types.
   std::vector<visualization_msgs::Marker> getFeatureMarkersCandidates( ros::Time& );
};

#endif // SLAM_PARTICLE_H
