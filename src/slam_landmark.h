/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#ifndef SLAM_LANDMARK_H
#define SLAM_LANDMARK_H

#include "slam_observation.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <tr1/memory>

class SLAMLandmark;
class SLAMPointLandmark;
class SLAMLineLandmark;
class SLAMRfidLandmark;

class SLAMObservation;
class SLAMPointObservation;
class SLAMLineObservation;
class SLAMRfidObservation;

struct point2d {
   float x;
   float y;
};

/*! \brief This abstract class is the generalization of landmarks which can be implemented by the different concrete types.
 * 
 * A landmark has to provide mainly three things.
 * First, the constructor that should compute all class variables from a given observation and a robot pose.
 * Second, a function measureLikelihood which calculates the possiblity that a given observation was caused by this landmark.
 * And a function that updates the landmark according to an observation. This is the Kalman update.
 *
 * Moreover a few parameters and functions are necesseray for the integration in the object network, as defined by this 
 * abstract class. 
 */
class SLAMLandmark
{
protected:
   long existence_estimate_; //!< Increasing with each observation, decreasing if feature should have been seen.
   bool should_have_been_seen_; //!< True if that feature should have been seen from the current robot pose.
   Vector3d pose_; //!< The pose is sampled according to the associated observation and robot pose.
   VectorXd state_; //!< The Kalman-filter state of the feature.
   MatrixXd cov_; //!< The covariance matrix of the state.
   
   // Data structures storing cached values
      // Depend on state and the robot's pose
   MatrixXd jacobian_pose_; //!< Cached partial derivate (robot pose) of the measurement model.
   MatrixXd jacobian_map_; //!< Cached partial derivate (feature state) of the measurement model.
   VectorXd prediction_; //!< The predicted observation of this feature.
   
   Vector3d mean_pose_; //!< Cached mean pose of the proposal distribution. Updated by measureLikelihood.
   Matrix3d mean_pose_variance_; //!< Cached mean pose covariance matrix of the proposal distribution. Updated by measureLikelihood.
   MatrixXd meas_cov_q_; //!< The measurement covariance matrix.
   VectorXd z_diff_; //!< Cached difference of the prediction and the observation.
   
   virtual void predictMeasurement( const Vector3d& ) = 0; //!< Predict the observation, depending on the given robot pose.
   virtual void calculateJacobianMap( const Vector3d& ) = 0; //!< Set the partial derivate of the observation model with respect to the state variables.
   virtual void calculateJacobianPose( const Vector3d& ) = 0; //!< Set the partial derivate of the observation model with respect to the robot pose variables.   
   virtual void typeSpecializedUpdate( SLAMObservation * ) = 0;
   virtual void setObservationDifference( SLAMObservation * ) = 0;
   virtual double getFactor( SLAMObservation * ) { return 1.0; }   
   virtual bool shouldExamine( SLAMObservation * , const Vector3d& ) = 0; //!< 
public:
   SLAMLandmark(); 
   SLAMLandmark( const SLAMLandmark& feature );
   
   // DP: Strategy
   virtual SLAMLandmark * getCopy() = 0; //!< Returns a pointer to a new object that is a copy of this.
   virtual bool shouldMerge( SLAMLandmark * ) = 0; //!< decide if two features should be merged
   virtual double merge( SLAMLandmark * ) = 0; //!< update this, merge with param object   

   // DP: Template method
   double measureLikelihood( SLAMObservation* obs, Vector3d& pose_robot, Matrix3d& cov_odo ); 
   double updateKalman( SLAMObservation * , Vector3d& , double , Matrix3d& );  
   void refreshLandmarkForIteration( const Vector3d& );
   
   // Getters and Setters
   virtual std::vector<geometry_msgs::Point> getVisualizationPoints() = 0;
   bool shouldHaveBeenSeen( Vector3d ) { return should_have_been_seen_; } //!< Is used to delete erroneous landmarks which should have been seen from the current robot pose.
   double getExistenceEstimate();
   double decreaseExistenceEstimate();
   void setVisibility( bool in ){ should_have_been_seen_ = in; }
   Vector3d getPose() { return pose_; }
};

class SLAMPointLandmark : public SLAMLandmark
{
private:
   void calculateJacobianPose( const Vector3d& );
   void calculateJacobianMap( const Vector3d& );
   void predictMeasurement( const Vector3d& );
   bool shouldExamine( SLAMObservation * , const Vector3d& );
   bool shouldMerge( SLAMLandmark * );
   double merge( SLAMLandmark * );
   void typeSpecializedUpdate( SLAMObservation* obs );
   void setObservationDifference( SLAMObservation * );
public:
   SLAMPointLandmark( SLAMPointObservation& , Vector3d , double );
   SLAMPointLandmark( const SLAMPointLandmark& );
   SLAMPointLandmark * getCopy();  
   std::vector<geometry_msgs::Point> getVisualizationPoints();
};

class SLAMLineLandmark : public SLAMLandmark
{
private:
   Vector2d p1_;
   Vector2d p2_;
   bool facing_origin_;
   bool intersect_;
   void calculateJacobianPose( const Vector3d& );
   void calculateJacobianMap( const Vector3d& );
   void predictMeasurement( const Vector3d& );
   bool shouldExamine( SLAMObservation * , const Vector3d& );
   bool shouldMerge( SLAMLandmark * );
   double merge( SLAMLandmark * );
   void typeSpecializedUpdate( SLAMObservation * );
   void setObservationDifference( SLAMObservation * );
   double getFactor( SLAMObservation * );   
public:
   SLAMLineLandmark( SLAMLineObservation& , Vector3d , double );
   SLAMLineLandmark( const SLAMLineLandmark& );
   SLAMLineLandmark * getCopy();

   Vector2d getStart() const;
   Vector2d getEnd() const;
   std::vector<geometry_msgs::Point> getVisualizationPoints();
};

#endif // SLAM_LANDMARK_H
