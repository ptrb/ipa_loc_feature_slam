/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

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
// ROS - IPA
#include "ipa_navigation_msgs/FeatureList.h"
#include "ipa_navigation_msgs/PointFeature.h"

#include "slam_particle.h"
#include "slam_local_map.h"

boost::mt19937 SLAMParticle::rng = boost::mt19937();

double normRandom( const double& mean , const double& stddev ){
   static double seed;
   SLAMParticle::rng.seed((++seed) + time(NULL));
   boost::normal_distribution<double> nd(mean, stddev);
   boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > var_nor(SLAMParticle::rng, nd);
   double ret = var_nor();
   return ret;
}

SLAMParticle::SLAMParticle( const Vector3d& pose )
{
   pose_ = pose;
   calculated_pose_ = pose;
   sampled_pose_ = pose;
}

SLAMParticle::SLAMParticle( const SLAMParticle & old ) {
   try {
      v_landmarks_.reserve( old.v_landmarks_.size() );
      v_landmark_candidates_.reserve( old.v_landmark_candidates_.size() );
   }
   catch( std::length_error ) {
      ROS_ERROR("Failed to reserve memory for landmarks during particle copying");
   }
   for( int i = 0 ; i < old.v_landmarks_.size() ; i++ ){
      v_landmarks_.push_back( old.v_landmarks_[i]->getCopy() );
   }   

   for( int i = 0 ; i < old.v_landmark_candidates_.size() ; i++ ){
      v_landmark_candidates_.push_back( old.v_landmark_candidates_[i]->getCopy() );
   }   

   pose_ = old.pose_;
   sampled_pose_ = old.sampled_pose_;
   calculated_pose_ = old.calculated_pose_;
}

SLAMParticle::~SLAMParticle()
{
   std::vector<SLAMLandmark *>::iterator it;
   
   for( it = v_landmarks_.begin() ; it != v_landmarks_.end() ; ++it ){
      delete *it;
   }
   for( it = v_landmark_candidates_.begin() ; it != v_landmark_candidates_.end() ; ++it ){
      delete *it;
   }  
}

void SLAMParticle::samplePose( double odo[4] )
{
   double delta1[3];
   double delta2[3];
   double ret[3];
   double dx, dy;
   double a1, a2, a3, a4;

   a1 = 0.0002;
   a2 = 0.0002;
   a3 = 0.00015;
   a4 = 0.00015;
   dx = odo[0];
   dy = odo[1];
   
   if( fabs(dx) < 0.0001 && fabs(dy) < 0.0001 && fabs(odo[3]) < 0.00001 ) return;
   
   delta1[0] = atan2( dy , dx ) - (odo[3]-odo[2]) ;
   delta1[1] = sqrt( dx*dx + dy*dy );
   delta1[2] = odo[2] - delta1[0];
   delta2[0] = delta1[0] - normRandom(0, a1*fabs(delta1[0])+a2*delta1[1]);
   delta2[1] = delta1[1] - normRandom(0, a3*delta1[1]+a4*( fabs(delta1[0]) + fabs(delta1[2]) ));
   delta2[2] = delta1[2] - normRandom(0, a1*fabs(delta1[2])+a2*delta1[1]);
   ret[0] = sampled_pose_(0) + delta2[1] * cos( sampled_pose_(2)+delta2[0] );
   ret[1] = sampled_pose_(1) + delta2[1] * sin( sampled_pose_(2)+delta2[0] );
   ret[2] = sampled_pose_(2) + delta2[0] + delta2[2];

   sampled_pose_(0) = ret[0];
   sampled_pose_(1) = ret[1];
   sampled_pose_(2) = ret[2];
}

void SLAMParticle::calcPose( double odo[4] )
{
   if( fabs(odo[0]) < 0.0001 && fabs(odo[1]) < 0.0001 && fabs(odo[3]) < 0.00001 ) return;
   double cos_v_o, sin_v_o, s, angle; 
   angle = calculated_pose_[2] + atan2( odo[1], odo[0] ) - (odo[3]-odo[2]);
   cos_v_o = cos( angle );
   sin_v_o = sin( angle );   
   s = sqrt( odo[0]*odo[0] + odo[1]*odo[1] );
   calculated_pose_(0) += s * cos_v_o;
   calculated_pose_(1) += s * sin_v_o;
   calculated_pose_(2) += odo[2];
}

Vector3d SLAMParticle::getPose() const
{
   return pose_;
}

std::vector<double> SLAMParticle::updateParticle( SLAMData data_uz , SLAMConfig& config )
{
   std::vector<SLAMObservation *> v_obs = data_uz.getObservations();
   std::vector<SLAMObservation *>::iterator it_obs;
   std::vector<SLAMLandmark *>::iterator it_land;
   std::vector<SLAMLandmark *>::iterator it_land_cand;
   std::vector<double> max_w_all_obs; // one max_w for each observation
   
   // multimap matches stores weights, landmarks and observations for reobserved landmarks. Sorted by weights.
   std::multimap< double , std::pair<SLAMLandmark * , SLAMObservation *> > mm_matches;
//    std::vector<SLAMLandmark *> found_match;
//    std::vector<SLAMObservation *> found_match_observation;
   std::vector<SLAMObservation *> no_match_observation;   
   
   for ( it_land = v_landmarks_.begin() ; it_land != v_landmarks_.end() ; ++it_land ){
      (*it_land)->refreshLandmarkForIteration( calculated_pose_ );
      
// 	 l_map.pushElement( local_points , *it_land );
   }
   for ( it_land_cand = v_landmark_candidates_.begin() ; it_land_cand != v_landmark_candidates_.end() ; ++it_land_cand ){
      (*it_land_cand)->refreshLandmarkForIteration( calculated_pose_ );
   }   
   
   // Phase 1
   // Try to associate each observation with an existing feature. Separate observations
   // without an association. 
   for( it_obs = v_obs.begin() ; it_obs != v_obs.end() ; ++it_obs ) {  
      
//       SLAMLocalMap l_map; // TODO improve combination with observations - less processing
//       ROS_INFO("\nNEW OBSERVATION Landmarks: %d\n",v_landmarks_.size() + v_landmark_candidates_.size() );
      double max_w = 0;
      double max_w_cand = 0;
      unsigned long landmark_index = 0;
      unsigned long landmark_index_cand = 0;
      std::vector<double> v_weights;
      std::vector<double> v_weights_cand;

      
      // Get likelihood for each landmark
      for ( it_land = v_landmarks_.begin() ; it_land != v_landmarks_.end() ; ++it_land ){
	 std::vector<point2d> local_points;
	 v_weights.push_back( (*it_land)->measureLikelihood( *it_obs , calculated_pose_ , covariance_odo_ ) );
// 	 l_map.pushElement( local_points , *it_land );
      }
      for ( it_land_cand = v_landmark_candidates_.begin() ; it_land_cand != v_landmark_candidates_.end() ; ++it_land_cand ){
	 std::vector<point2d> local_points;
	 
	 v_weights_cand.push_back( (*it_land_cand)->measureLikelihood( *it_obs , calculated_pose_ , covariance_odo_ ) );
// 	 l_map.pushElement( local_points , *it_land_cand );
      }
//      l_map.setVisibilities( data_uz.sensor );
      // Determine max likelihood -> value and index
      for( int j = 0 ; j < v_weights.size() ; j++ ){
	    if( max_w < v_weights[j] ){
	       max_w = v_weights[j];
	       landmark_index = j;
	    }
      }
      for( int j = 0 ; j < v_weights_cand.size() ; j++ ){
	    if( max_w_cand < v_weights_cand[j] ){
	       max_w_cand = v_weights_cand[j];
	       landmark_index_cand = j;
	    }
      }
      // Max w below threshold? Add new feature.
      if( max_w < config.data_association_threshold_new_feature 
	 && max_w_cand < config.data_association_threshold_new_feature ) 
      { 
	 landmark_index_cand = v_landmark_candidates_.size();
	 no_match_observation.push_back( *it_obs );
      }
      else {
	 if( max_w > max_w_cand ) 
	 {
// 	    found_match.push_back( v_landmarks_.at(landmark_index) );
// 	    found_match_observation.push_back( *it_obs );
	    mm_matches.insert( std::make_pair( max_w , std::make_pair( v_landmarks_.at(landmark_index) , *it_obs ) ) );
	 }
	 else 
	 {
// 	    found_match.push_back( v_landmark_candidates_.at(landmark_index_cand) );
// 	    found_match_observation.push_back( *it_obs );
	    
	    mm_matches.insert( std::make_pair( max_w_cand , std::make_pair(v_landmark_candidates_.at(landmark_index_cand) , *it_obs ) ) );
	 }
      }
      
      // Update existance-estimate for all other features
      for( it_land = v_landmarks_.begin() ; it_land != v_landmarks_.end() ; ){
	 bool erased = false;
	 if( landmark_index != it_land - v_landmarks_.begin() ) {
	    bool observable = (*it_land)->shouldHaveBeenSeen( pose_ );
	    if( observable ){ // feature should have been seen

	       // TODO may only on candidates???
// 	       ROS_INFO("SHOULD HAVE BEEN SEEN");
	       double estimate = (*it_land)->decreaseExistenceEstimate();
	       if(estimate < 0) {
		  delete *it_land;
		  it_land = v_landmarks_.erase( it_land );
		  erased = true;
		  ROS_INFO("E R A S E D");
		  --landmark_index;
	       }
	    }
	 }
	 if( !erased ) ++it_land;
      }
      
      for( it_land_cand = v_landmark_candidates_.begin() ; it_land_cand != v_landmark_candidates_.end() ; ){
	 bool erased = false;
	 if( landmark_index_cand != it_land_cand - v_landmark_candidates_.begin() ) {
	    bool observable = (*it_land_cand)->shouldHaveBeenSeen( pose_ );
	    if( observable ){ // feature should have been seen

// 	       ROS_INFO("SHOULD HAVE BEEN SEEN");
	       double estimate = (*it_land_cand)->decreaseExistenceEstimate();
	       if(estimate < 0) {
		  delete *it_land_cand;
		  it_land_cand = v_landmark_candidates_.erase( it_land_cand );
		  erased = true;
		  ROS_INFO("E R A S E D");
		  --landmark_index_cand;
	       }
	    }
	 }
	 if( !erased ) ++it_land_cand;
      }
      // repeat for next observati1on...
   }
   
   // Phase 2
   // Update existing Features and create new Features.
   Vector3d new_pose(0.0,0.0,0.0);
   Matrix3d proposal_cov = covariance_odo_;
   Vector3d proposal_mean = calculated_pose_;
   std::multimap< double , std::pair< SLAMLandmark*,SLAMObservation* > >::reverse_iterator map_rev_itr;
   double weight, weight_sum = 0;
   int debug_cntr = 0;
   for( map_rev_itr = mm_matches.rbegin() ; map_rev_itr != mm_matches.rend() ; ++map_rev_itr ){
      debug_cntr++;
//        mm_matches.lower_bound( map_itr->first );
//       ROS_INFO("w was:\t%f",map_rev_itr->first);
      SLAMLandmark * landmark = map_rev_itr->second.first;
      SLAMObservation * observation = map_rev_itr->second.second;
      Vector3d old_pose = landmark->getPose();
      weight = landmark->updateKalman( observation , proposal_mean , data_uz.getTimestamp() , proposal_cov );
//       ROS_INFO("w is:\t%f",weight);
      if( weight < 1e-14 ){
	 ROS_INFO("debug_cntr: %d",debug_cntr);
	 ROS_ERROR("w likelihood was: %f",map_rev_itr->first);
	 ROS_INFO_STREAM("prop mean\n"<<proposal_mean);
	 ROS_INFO_STREAM("prop var\n"<<proposal_cov);
	 ROS_INFO_STREAM("calculated_pose\n"<<calculated_pose_);
	 ROS_INFO_STREAM("covariance_odo\n"<<covariance_odo_);
	 ROS_INFO_STREAM("old pose\n"<<old_pose);
	 ROS_INFO_STREAM("new pose\n"<<landmark->getPose());
	 ROS_INFO_STREAM("landmark:: "<< landmark->getVisualizationPoints()[0] );
      }
      max_w_all_obs.push_back( weight );   
//       landmark->setMeanPose( proposal_mean );
//       landmark->setMeanPoseVariance( proposal_cov );
      new_pose += weight * landmark->getPose();
      weight_sum += weight;
   }
   
   
   if( max_w_all_obs.empty() ){ // No match
      for( int i = 0 ; i < no_match_observation.size() ; i++){
	 max_w_all_obs.push_back( 4 );
	 if( (no_match_observation[i])->suitableInitObs() ){
// 	    ROS_INFO("Add feature - w_max: %f\tw_max_cand: %f", max_w, max_w_cand);
	    v_landmark_candidates_.push_back( (no_match_observation[i])->createFeature( sampled_pose_ , data_uz.getTimestamp()  ) ); 
	 }
      }
      new_pose = sampled_pose_;
   }
   else { 
      new_pose = new_pose / weight_sum;
//       new_pose = (*(found_match.end()-1))->getPose();
      for( int i = 0 ; i < no_match_observation.size() ; i++){
	 max_w_all_obs.push_back( 4 );
	 if( (no_match_observation[i])->suitableInitObs() ){
	    v_landmark_candidates_.push_back( (no_match_observation[i])->createFeature( new_pose , data_uz.getTimestamp() ) );
	 }
      }
   }   
   
   ROS_ERROR_COND( isnan(new_pose(0)) || isnan(new_pose(1)) || isnan(new_pose(2)) , "New pose is NAN!" );
   new_pose(2) = new_pose(2) - floor( new_pose(2) / (2 * M_PI)) * 2 * M_PI;
   if( new_pose(2) > M_PI ) new_pose(2) -= 2*M_PI;
   setPose( new_pose );
//    mergeSimultaneousCandidates();
   max_w_all_obs.push_back( pickCandidates( config ) ); // weight
   max_w_all_obs.push_back( mergeSimultaneousFeatures() ); // weight
   return max_w_all_obs;
}

long int SLAMParticle::getLandmarkCount()
{
   return v_landmarks_.size();
}



double SLAMParticle::pickCandidates( SLAMConfig& config )
{
   std::vector<SLAMLandmark *>::iterator it, it_cand;
   double ret = 1.0;
   for( it_cand = v_landmark_candidates_.begin() ; it_cand != v_landmark_candidates_.end() ; ){
      if( (*it_cand)->getExistenceEstimate() > config.min_existence_estimate_map_feature ){ // Trust this feature and add or merge it to map.

	 bool is_done = false;
	 for( it = v_landmarks_.begin() ; it != v_landmarks_.end() ; ++it ){
	    if( (*it)->shouldMerge( *it_cand ) ) {
	       ret *= (*it)->merge( *it_cand );
	       is_done = true;
	       break;
	    }
	 }  
	 if( !is_done ) v_landmarks_.push_back( (*it_cand)->getCopy() );
	 delete *it_cand;
	 it_cand = v_landmark_candidates_.erase( it_cand );
      }
      else {
	 ++it_cand;
      }
   }   
   return ret;
}

double SLAMParticle::mergeSimultaneousFeatures()
{
   std::vector<SLAMLandmark *>::iterator it_out, it_in;
   double ret = 1.0;
   if( v_landmarks_.empty() ) return ret;
//    int i = 0;
   for( it_out = v_landmarks_.begin() ; it_out <= v_landmarks_.end()-1 ; ++it_out ){

	 for( it_in = it_out+1 ; it_in != v_landmarks_.end() ; ){
	    if( (*it_out)->shouldMerge( *it_in ) ) {
	       ret *= (*it_out)->merge( *it_in );
	       delete *it_in;
	       it_in = v_landmarks_.erase( it_in ); 
	    }
	    else {
	       ++it_in;
	    }
	 }  
   }
   return ret;
//    ROS_INFO("compared %d pairs",i);
}

void SLAMParticle::mergeSimultaneousCandidates()
{
   std::vector<SLAMLandmark *>::iterator it_out, it_in;
   if( v_landmark_candidates_.empty() ) return;  // it_out != v_landmark_candidates_.end()-1
   for( it_out = v_landmark_candidates_.begin() ; it_out <= v_landmark_candidates_.end()-1 ; ++it_out ){
	 
// 	 ROS_INFO(" it_in is: %p ",it_in);
	 for( it_in = it_out+1 ; it_in != v_landmark_candidates_.end() ; ){
	    if( (*it_out)->shouldMerge( *it_in ) ) {
	       (*it_out)->merge( *it_in );
// 	       ROS_INFO("merged simultaneous features");
	       delete *it_in;
	       it_in = v_landmark_candidates_.erase( it_in ); 
	    }
	    else {
	       ++it_in;
	    }
	 }  
   }   
}

void SLAMParticle::setCovarianceOdo( double odo[4] ) {
      Matrix<double, 3, 3> covariance_odo = Matrix3d::Zero(); // cov in global frame
      double odo_x, odo_y, odo_o;
      if( odo[0] == 0 && odo[1] == 0 && odo[2] == 0 )
      {
	 odo_x = 1e-8;
	 odo_y = 1e-8;
	 odo_o = 1e-8;
      }
      else
      {
	 odo_x = odo[0];
	 odo_y = odo[1];
	 odo_o = odo[2];
      }
//       covariance_odo(0,0) = fabs(0.005 * odo_x);
//       covariance_odo(1,1) = fabs(0.005 * odo_y);
//       covariance_odo(2,2) = fabs(0.009 * odo_o);
      covariance_odo(0,0) = fabs(0.01 * odo_x);
      covariance_odo(1,1) = fabs(0.01 * odo_y);
      covariance_odo(2,2) = fabs(0.009 * odo_o);
      Matrix3d rot;
      rot << cos(-odo[3]), -sin(-odo[3]), 0,  // /slam_map to /base_link
      sin(-odo[3]), cos(-odo[3]), 0,
      0 , 0, 1;
      covariance_odo_ = rot * covariance_odo * rot.transpose();
      ROS_ERROR_COND( covariance_odo_.determinant() == 0 , "cov odo with det = 0");
}

std::vector<visualization_msgs::Marker> SLAMParticle::getFeatureMarkers( ros::Time& ref_time )
{
   visualization_msgs::Marker new_marker;
   static double seed;

   new_marker.type = visualization_msgs::Marker::POINTS;
   new_marker.action = visualization_msgs::Marker::ADD;
   new_marker.header.frame_id = "/slam_map";
   new_marker.header.stamp = ref_time;
   new_marker.scale.x = 0.1;
   new_marker.scale.y = 0.1;
   new_marker.color.a = 1.0;
   new_marker.color.r = 0.58;
   new_marker.color.g = 0.91;
   new_marker.color.b = 0.90;
   
   visualization_msgs::Marker new_marker_line;
   
   new_marker_line.type = visualization_msgs::Marker::LINE_LIST;
   new_marker_line.action = visualization_msgs::Marker::ADD;
   new_marker_line.header.frame_id = "/slam_map";
   new_marker_line.header.stamp = ref_time;
   new_marker_line.scale.x = 0.03;
   new_marker_line.scale.y = 0.03;
   new_marker_line.color.a = 1.0;
   new_marker_line.color.r = 0.10;
   new_marker_line.color.g = 0.91;
   new_marker_line.color.b = 0.90;
   
   for( int i = 0 ; i < v_landmarks_.size() ; i++ ){
      std::vector<geometry_msgs::Point> geom = v_landmarks_[i]->getVisualizationPoints();
      if( geom.size() == 1 ) new_marker.points.push_back( geom[0] );  
      else if( geom.size() == 2 ){
	 new_marker_line.points.push_back( geom[0] );
	 new_marker_line.points.push_back( geom[1] );
      }
   }   
   std::vector<visualization_msgs::Marker> ret;
   ret.push_back( new_marker );
   ret.push_back( new_marker_line );
   return ret;
}

std::vector<visualization_msgs::Marker> SLAMParticle::getFeatureMarkersCandidates( ros::Time& ref_time )
{
   visualization_msgs::Marker new_marker;
   static double seed;

   new_marker.type = visualization_msgs::Marker::POINTS;
   new_marker.action = visualization_msgs::Marker::ADD;
   new_marker.header.frame_id = "/slam_map";
   new_marker.header.stamp = ref_time;
   new_marker.scale.x = 0.1;
   new_marker.scale.y = 0.1;
   new_marker.color.a = 1.0;
   new_marker.color.r = 0.58;
   new_marker.color.g = 0.91;
   new_marker.color.b = 0.90;
   
   visualization_msgs::Marker new_marker_line;
   
   new_marker_line.type = visualization_msgs::Marker::LINE_LIST;
   new_marker_line.action = visualization_msgs::Marker::ADD;
   new_marker_line.header.frame_id = "/slam_map";
   new_marker_line.header.stamp = ref_time;
   new_marker_line.scale.x = 0.03;
   new_marker_line.scale.y = 0.03;
   new_marker_line.color.a = 1.0;
   new_marker_line.color.r = 0.10;
   new_marker_line.color.g = 0.91;
   new_marker_line.color.b = 0.90;
   
   for( int i = 0 ; i < v_landmark_candidates_.size() ; i++ ){
      std::vector<geometry_msgs::Point> geom = v_landmark_candidates_[i]->getVisualizationPoints();
      if( geom.size() == 1 ) new_marker.points.push_back( geom[0] );  
      else if( geom.size() == 2 ){
	 new_marker_line.points.push_back( geom[0] );
	 new_marker_line.points.push_back( geom[1] );
      }
   }   
   std::vector<visualization_msgs::Marker> ret;
   ret.push_back( new_marker );
   ret.push_back( new_marker_line );
   return ret;
}