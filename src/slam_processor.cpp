/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#include "slam_processor.h"
#include <fstream>

static boost::mt19937 rng = boost::mt19937();
tf::TransformListener * SLAMProcessor::tf_listener_; 
double SLAMProcessor::latest_odo_pose_[3]; // Pose in /fast_slam frame
tf::Stamped<tf::Pose> SLAMProcessor::tf_global_odo_transformation_; 

SLAMProcessor::SLAMProcessor( tf::TransformListener * tf )
{   
   tf_listener_ = tf;
   init_ = false;
   v_weights_.assign(count_particles_ , 1.0);
}

/*!\brief Get translation and rotation relative to the last tracked pose in a local robot reference frame.
 * The odometry is calculated via a coordinate transformation from base_link to odometry_combined.
 * @param[in] reference_instant The current timestamp.
 * @param[out] odo_diff_out The resulting local odometry params: [0]:= x [1]:= y [2]:= orientation.
 * @param[in,out] tf_global_odo_transformation This is the last known pose in the global frame. Will be updated.
 */
bool SLAMProcessor::getLocalOdom( const ros::Time& reference_instant , std::vector<double>& odo_diff_out , tf::Stamped<tf::Pose>& tf_global_odo_transformation ) const 
{
   double dx, dy, theta_old;

   tf::Stamped<tf::Pose> odom_pose;
   tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), reference_instant, "/base_link");
   
   std::string * error_msg = new std::string;
   const ros::Duration timeout(0.2);
   const ros::Duration polling_sleep_duration(0.005);
   if( tf_listener_->waitForTransform( "/odom_combined", "/base_link", reference_instant, timeout, polling_sleep_duration, error_msg ) )
   {
      tf_listener_->transformPose("/odom_combined", ident, odom_pose);
      delete error_msg;
   }
   else 
   {
      ROS_ERROR("getOdom-transform not possible. %s", error_msg->c_str() );
      delete error_msg;
      return false;
   }	
   // Calculate the difference in the global odometry frame
   dx = odom_pose.getOrigin().x() - tf_global_odo_transformation.getOrigin().x();
   dy = odom_pose.getOrigin().y() - tf_global_odo_transformation.getOrigin().y();
   theta_old = tf::getYaw(tf_global_odo_transformation.getRotation());
   theta_old= theta_old - floor( theta_old / (2 * M_PI)) * 2 * M_PI;
   // Transform in local frame
   odo_diff_out[0] = cos(theta_old)*dx + sin(theta_old)*dy;
   odo_diff_out[1] = -sin(theta_old)*dx + cos(theta_old)*dy;
   double theta_odom_pose = tf::getYaw(odom_pose.getRotation());
   theta_odom_pose= theta_odom_pose - floor( theta_odom_pose / (2 * M_PI)) * 2 * M_PI;
   odo_diff_out[2] = theta_odom_pose - theta_old;
   if( odo_diff_out[2] < - M_PI ) odo_diff_out[2] += (M_PI * 2);
   if( odo_diff_out[2] > M_PI ) odo_diff_out[2] -= (M_PI * 2);

   if( fabs(odo_diff_out[2]) > 0.5 ) {
      ROS_INFO("odo diff is to large!!! ");
      ROS_INFO("odo diff: %f %f %f\t\todom pose: %f %f %f",odo_diff_out[0],odo_diff_out[1],odo_diff_out[2],odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),theta_odom_pose);
      ROS_INFO("stamp ref: %f\t stamp global: %f",reference_instant.toSec(),tf_global_odo_transformation.stamp_.toSec());
   }
   // Update global position
   tf_global_odo_transformation = odom_pose;
  
   return true;
}

/*!Get odometry in slam_map system.
 * @param[in] reference_instant The current timestamp.
 * @param[out] odo_diff_out The resulting odometry params in the slam_map system: [0]:= x [1]:= y [2]:= orientation [3]:= absolute orientation.
 * @param[in,out] tf_global_odo_transformation This is the last known pose in the global frame. Will be updated.
 * @param[in,out] latest_odo_pose The position in the slam_map system: [0]:= x [1]:= y [2]:= orientation.
 */
bool SLAMProcessor::deadReckoning( const ros::Time& reference_instant , std::vector<double>& odo_diff_out , tf::Stamped<tf::Pose>& tf_global_odo_transformation , double latest_odo_pose[3] ) const
{
   double d_x, d_y, cos_v_o, sin_v_o;
   std::vector<double> new_pose_diff;
   new_pose_diff.resize(4);
   if( getLocalOdom( reference_instant , new_pose_diff , tf_global_odo_transformation ) )
   {
      cos_v_o = cos( latest_odo_pose[2] + new_pose_diff[2] );
      sin_v_o = sin( latest_odo_pose[2] + new_pose_diff[2] );   
      d_x = new_pose_diff[0] * cos_v_o - new_pose_diff[1] * sin_v_o;
      d_y = new_pose_diff[1] * cos_v_o + new_pose_diff[0] * sin_v_o;
      new_pose_diff[0] = d_x;
      new_pose_diff[1] = d_y;
      new_pose_diff[3] = latest_odo_pose[2] + new_pose_diff[2];
      
      if( fabs(new_pose_diff[0]) < 0.0001 && fabs(new_pose_diff[1]) < 0.0001 && fabs(new_pose_diff[2]) < 0.0001 )
      {
	 odo_diff_out[0] = 0.0;
	 odo_diff_out[1] = 0.0;
	 odo_diff_out[2] = 0.0;
      }
      else 
      {
	 odo_diff_out[0] = new_pose_diff[0];
	 odo_diff_out[1] = new_pose_diff[1];
	 odo_diff_out[2] = new_pose_diff[2];
	 odo_diff_out[3] = new_pose_diff[3];
	 latest_odo_pose[0] += d_x;
	 latest_odo_pose[1] += d_y;
	 latest_odo_pose[2] += new_pose_diff[2];
	 latest_odo_pose[2] = latest_odo_pose[2] - std::floor( latest_odo_pose[2] / (2 * M_PI)) * 2 * M_PI;
      }
      return true;
   }
   else {
      return false;
   }
}

/*!Get odometry in slam_map system.
 * @param[in] config The configuration parameters.
 */
bool SLAMProcessor::update( SLAMConfig& config )
{
   // Check if data is available
   if( q_observations_.size() < 5 ) return false; 

   static int count; // Count of iterations until resampling
   count++;
   
   SLAMData current_data = q_observations_.front();
   std::vector<SLAMParticle>::iterator it_particles;
   
   tf::Stamped<tf::Pose> tf_global_odo_transformation_tmp = tf_global_odo_transformation_;
   double latest_odo_pose_tmp[3] = { latest_odo_pose_[0],latest_odo_pose_[1],latest_odo_pose_[2] };
   ros::Time instant;
   double timestep_duration = current_data.getTimestamp() - tf_global_odo_transformation_tmp.stamp_.toSec();
   std::vector<std::vector<double> > v_odom;
   bool success = true;
   double nt = 0;
   while( timestep_duration > 0.1 && success )
   {
      instant = instant.fromSec( tf_global_odo_transformation_tmp.stamp_.toSec() + 0.1 );
      timestep_duration -= 0.1;
      std::vector<double> odom;
      odom.resize(4);
      if( !deadReckoning( instant , odom , tf_global_odo_transformation_tmp , latest_odo_pose_tmp ) ) {
	 success = false;
	 break;
      }
      v_odom.push_back( odom );
   }
   
   std::vector<double> odom;
   odom.resize(4);
   instant = instant.fromSec( current_data.getTimestamp() );
   if( !deadReckoning( instant, odom , tf_global_odo_transformation_tmp , latest_odo_pose_tmp ) ) success = false;
   if( success )
   { 
      v_odom.push_back( odom ); 
//       ROS_INFO("new tf global stamp: %f\t last stamp: %f",tf_global_odo_transformation_tmp.stamp_.toSec(),tf_global_odo_transformation_.stamp_.toSec());     
      tf_global_odo_transformation_ = tf_global_odo_transformation_tmp;
      latest_odo_pose_[0] = latest_odo_pose_tmp[0];
      latest_odo_pose_[1] = latest_odo_pose_tmp[1];
      latest_odo_pose_[2] = latest_odo_pose_tmp[2];

      std::vector<std::vector<double> > v_weights_current_iteration;

      for( it_particles = v_particles_.begin() ; it_particles != v_particles_.end() ; ++it_particles ){
	 // try odometry update. sample minimum rate 5Hz

	    std::vector<std::vector<double> >::iterator it_odom;
	    double odo_total[4] = {0,0,0,0};
	    for( it_odom = v_odom.begin() ; it_odom != v_odom.end() ; ++it_odom ){
	       double dom_arr[4] = { it_odom->at(0),it_odom->at(1),it_odom->at(2),it_odom->at(3) };
	       it_particles->samplePose( dom_arr ); 
	       it_particles->calcPose( dom_arr );
	       odo_total[0] += dom_arr[0];
	       odo_total[1] += dom_arr[1];
	       odo_total[2] += dom_arr[2];
	       odo_total[3] = dom_arr[3];
	    }
	    it_particles->setCovarianceOdo( odo_total );
	 
	 v_weights_current_iteration.push_back( it_particles->updateParticle( current_data , config ) );
      }

      std::vector<double> acc_weights_particle;
      acc_weights_particle.assign( count_particles_ , 1.0 );
      for( int i = 0 ; i < count_particles_ ; i++ )
      {
	 for( int j = 0 ; j < v_weights_current_iteration[i].size() ; j++ )
	 {
	    acc_weights_particle[i] *= v_weights_current_iteration[i][j];
	    ROS_ERROR_COND( isnan(acc_weights_particle[i]) , "acc_weights_particle IS NAN");
	    ROS_ERROR_COND( isinf(acc_weights_particle[i]) , "acc_weights_particle IS INF");
	 }
      }
      
      // Normalize factors for this round
      double acc_weights = std::accumulate( acc_weights_particle.begin() , acc_weights_particle.end() , 0.0 );
      
      if( acc_weights <= 1e-40 ) {
	 acc_weights = 1;
      }
      for( int i = 0 ; i < v_weights_.size() ; i++ )
      {
	 v_weights_[i] *= ( acc_weights_particle[i] / acc_weights );
      }
      
      acc_weights = std::accumulate( v_weights_.begin() , v_weights_.end() , 0.0 );
      if( acc_weights <= 1e-40 ) {
	 acc_weights = 1;
      }
      
      for( int i = 0 ; i < v_weights_.size() ; i++ )
      {
	 v_weights_[i] = ( v_weights_[i] / acc_weights ) * count;
	 nt += v_weights_[i] * v_weights_[i];
	 ROS_ERROR_COND( isnan(v_weights_[i]) , "v_weights is NAN. Greetings from SLAMProcessor's update function." );
	 ROS_ERROR_COND( isinf(v_weights_[i]) , "v_weights is INF. Greetings from SLAMProcessor's update function." );
      }
   }
   else {
      ROS_INFO("DEADRECKONING not possible.");
   }

   static int best_map_index = 0;   
   
   if( nt != 0){
      if( (count*count / nt) < ((double)count_particles_ / (double)2)){
	 
	 resampleParticles( v_weights_ );
	 best_map_index = updateBestMap( config );
	 count = 0;
	 v_weights_.assign(count_particles_, 1.0);  
	 
// 	 Vector3d pose_error = true_pose_ - v_particles_[best_map_index].getPose();
// 	 if( pose_error(2) < - M_PI ) pose_error(2) += (M_PI * 2);
// 	 if( pose_error(2) > M_PI ) pose_error(2) -= (M_PI * 2);
// 	 ROS_INFO_STREAM("Pose error is:" << pose_error);
// 	 
// 	 // app = append, ate = position to end, trunc = delete everything
// 	 std::ofstream file("pose_error.txt", std::ios::app);
// 	 file << pose_error(0)<<"\t"<<pose_error(1)<<"\t"<<pose_error(2)<<std::endl;
// 	 file.close();
	 
      }
   }
//    else {
//       trackBestMap( best_map_index );
//    }
//    ROS_INFO("Timestamp data: %f\ttrue pose: %f",tsa.toSec(),true_pose_stamp_.toSec());
//    ROS_INFO_STREAM("TRUE Pose:"<<true_pose_);
//    ROS_INFO_STREAM("EST Pose:"<< v_particles_[best_map_index].getPose());
   q_observations_.pop();
     
   return true;
}

void SLAMProcessor::trackBestMap( int index ){
 
   best_slam_map_.markers.clear();
   
   ros::Time ref_time = ros::Time::now();
   std::vector<visualization_msgs::Marker> v_markers = v_particles_[index].getFeatureMarkers( ref_time );
   std::vector<visualization_msgs::Marker>::iterator it;
   int i = 0;
   for( it = v_markers.begin() ; it != v_markers.end() ; ++it ){
      best_slam_map_.markers.push_back( *it );
      best_slam_map_.markers[i].id = i;
      i++;
   }
}

int SLAMProcessor::updateBestMap( SLAMConfig& config ){
   int best_index = 0;
   int max_weight = 0;
   for( int i = 0 ; i < v_particles_.size() ; i++ ){
      if( v_weights_[i] > max_weight ){
	 max_weight = v_weights_[i];
	 best_index = i;
      }
   }   
   
   best_slam_map_.markers.clear();
   
   ros::Time ref_time = ros::Time::now();
   std::vector<visualization_msgs::Marker> v_markers = v_particles_[best_index].getFeatureMarkers( ref_time );
   std::vector<visualization_msgs::Marker>::iterator it;
   int i = 0;
   for( it = v_markers.begin() ; it != v_markers.end() ; ++it ){
      if( it->type == visualization_msgs::Marker::POINTS ){
	 it->color.r = 1.0;
	 it->color.g = 1.0;
	 it->color.b = 0.33;
      }
      else {
	 it->color.r = 0.87;
	 it->color.g = 0.24;
	 it->color.b = 0.11;	 
      }
      best_slam_map_.markers.push_back( *it );
      best_slam_map_.markers[i].id = i;
      i++;
   }
   return best_index;
}

inline void SLAMProcessor::resampleParticles( std::vector<double> v_weights ){
   // normalize weights.
   std::vector<SLAMParticle> new_particles;
   std::vector<double>::iterator it;
   double sum = std::accumulate( v_weights.begin() , v_weights.end() , 0.0 );
   ROS_ERROR_COND( sum == 0 , "All weights are zero during resampling" );
   
   for( it = v_weights.begin() ; it != v_weights.end() ; ++it )
   {
      *it = *it / sum;
   }
      
   // build array with accumulated weights.
   std::vector<double> v_acc;
   double acc = 0;
   for( it = v_weights.begin() ; it != v_weights.end() ; ++it ){
      acc += *it;
      v_acc.push_back(acc);
   }  

   boost::uniform_real<double> dist(0, 1);
   
   double random;
   
   int count [count_particles_] = {}; // Debug
   
//    for( int i = 0 ; i < count_particles_ ; i++ ){
//       random = dist(rng);
//       int index;
//       // find index
//       for( int j = 0 ; j < v_acc.size() ; j++ ){
// 	 if( v_acc[j] >= random ) { 
// 	    index = j;
//             count[index]++; //Debug
// 	    break;
// 	 }
//       }
//       new_particles.push_back( v_particles_[index] );
//    }
   random = dist(rng);
   double uk;
   for( int i = 0 ; i < count_particles_ ; i++ ){
      uk = ( i + dist(rng) ) / count_particles_;
      int index;
      // find index
      for( int j = 0 ; j < v_acc.size() ; j++ ){
	 if( v_acc[j] >= uk ) { 
	    index = j;
            count[index]++; //Debug
	    break;
	 }
      }
      new_particles.push_back( v_particles_[index] );
   }
   
   for( int j = 0 ; j < count_particles_ ; j++){ //Debug
      std::string stars = "";
      for( int u = 0 ; u < count[j] ; u++ ) {
	 stars += "*";
      }
//       ROS_INFO("old particle[%d]   %s\t\tfeatures: %d\tcandidates: %d",j,stars.c_str(),v_particles_[j].getLandmarkCount(),v_particles_[j].getCandidateCount() );
   }
   
   v_particles_.clear();
   
   for( int i = 0 ; i < new_particles.size() ; i++ ){
      v_particles_.push_back( new_particles[i] );
   }
   
//    ROS_INFO("End resample\n");
   return;
   
}



void SLAMProcessor::pushNewData( const ipa_navigation_msgs::FeatureList::ConstPtr& msg_featr )
{
   // initialize frame transformation and related timestamp as well as particles, so that they have coordinates 0,0,0 
   // right at that timestamp.
   if( !init_ ){
         tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0,0,0)), msg_featr->header.stamp, "/base_link");
	 std::string * error_msg = new std::string;
	 const ros::Duration timeout(0.01);
	 const ros::Duration polling_sleep_duration(0.005);
	 if( tf_listener_->waitForTransform( "/odom_combined", "/base_link", msg_featr->header.stamp, timeout, polling_sleep_duration, error_msg ) ){
	    tf_listener_->transformPose("/odom_combined", ident, tf_global_odo_transformation_);
	       for( int i = 0 ; i < count_particles_ ; i++){
		  v_particles_.push_back( SLAMParticle( Vector3d(0.0,0.0,0.0 ) ) );
	       }
	       latest_odo_pose_[0] = 0.0;
	       latest_odo_pose_[1] = 0.0;
	       latest_odo_pose_[2] = 0.0;
	       init_ = true;
	       ROS_INFO("Initialization was successfull!");
	       
	 }
      delete error_msg;
      return;
   }
   try {
      if( !q_observations_.empty() && fabs(q_observations_.back().getTimestamp() - msg_featr->header.stamp.toSec()) < 0.005 ) q_observations_.back().pushObservations(msg_featr , tf_listener_);
      else {
	 SLAMData new_data( msg_featr , tf_listener_ );
	 if( new_data.getObservations().size() != 0 );
	 q_observations_.push( new_data );
      }
      
   }
   catch( int n ){   
      if(n == 13) ROS_INFO("Catched exception following impossible transform. Elements: %d", q_observations_.size());
   }
}

tf::StampedTransform SLAMProcessor::getSlamMapTransform(){
   tf::Transform transform;
   
   double x_trans, y_trans;   
   double angle = latest_odo_pose_[2] - std::floor( latest_odo_pose_[2] / (2 * M_PI)) * 2 * M_PI;
   double cos_o = cos( -angle );
   double sin_o = sin( -angle ); 
   double dx = - latest_odo_pose_[0];
   double dy = - latest_odo_pose_[1]; 
   x_trans = dx * cos_o - dy * sin_o;
   y_trans = dy * cos_o + dx * sin_o;
    
   transform.setOrigin( tf::Vector3( x_trans , y_trans , 0.0) );
   tf::Quaternion q;
   q.setRPY(0,0, -angle);
   transform.setRotation( q );
   return tf::StampedTransform(transform, tf_global_odo_transformation_.stamp_, "/base_link", "/slam_map");
}

visualization_msgs::Marker SLAMProcessor::getParticleMarkers() const
{
   visualization_msgs::Marker new_marker;
   new_marker.type = visualization_msgs::Marker::POINTS;
   new_marker.action = visualization_msgs::Marker::ADD;
   new_marker.header.frame_id = "/slam_map";
   new_marker.header.stamp = ros::Time::now();
   new_marker.scale.x = 0.1;
   new_marker.scale.y = 0.1;
   new_marker.color.a = 1.0;
   new_marker.color.r = 0.6;
   new_marker.color.g = 1.0;
   new_marker.color.b = 0.6;
   for( int i = 0 ; i < v_particles_.size() ; i++ )
   {
      geometry_msgs::Point point;
      point.x = v_particles_[i].getPose()[0];
      point.y = v_particles_[i].getPose()[1];
      point.z = 0.0;
      new_marker.points.push_back(point);   
   }   
   return new_marker;
}

visualization_msgs::MarkerArray SLAMProcessor::getFeatureMarkers()
{
   visualization_msgs::MarkerArray new_markers;
   ros::Time ref_time = ros::Time::now();
   for( int i = 0 ; i < v_particles_.size() ; i++ ){
      std::vector<visualization_msgs::Marker> markers = v_particles_[i].getFeatureMarkers( ref_time );
      markers[0].id = i;
      markers[1].id = i + count_particles_;
      if( !markers[0].points.empty() ) new_markers.markers.push_back( markers[0] );
      if( !markers[1].points.empty() ) new_markers.markers.push_back( markers[1] );
   }
   return new_markers;
}

visualization_msgs::MarkerArray SLAMProcessor::getFeatureMarkersCandidates()
{
   visualization_msgs::MarkerArray new_markers;
   ros::Time ref_time = ros::Time::now();
   for( int i = 0 ; i < v_particles_.size() ; i++ ){
      std::vector<visualization_msgs::Marker> markers = v_particles_[i].getFeatureMarkersCandidates( ref_time );
      markers[0].id = i;
      markers[1].id = i + count_particles_;
      if( !markers[0].points.empty() ) new_markers.markers.push_back( markers[0] );
      if( !markers[1].points.empty() ) new_markers.markers.push_back( markers[1] );
   }
   return new_markers;
}

geometry_msgs::PoseStamped SLAMProcessor::visualizePoseFromOdometry(){        
   geometry_msgs::PoseStamped out_visualize_pose;
   out_visualize_pose.pose.position.x = latest_odo_pose_[0];
   out_visualize_pose.pose.position.y = latest_odo_pose_[1];
   out_visualize_pose.pose.position.z = 0;
   Vector3d zaxis;
   zaxis = Vector3d::Zero();
   zaxis(2) += 1;
   AngleAxis<double> aa(latest_odo_pose_[2], zaxis);
   Quaternion<double> q(aa);

   out_visualize_pose.pose.orientation.x = q.x();
   out_visualize_pose.pose.orientation.y = q.y();
   out_visualize_pose.pose.orientation.z = q.z();
   out_visualize_pose.pose.orientation.w = q.w();
   out_visualize_pose.header.frame_id="/slam_map";
   out_visualize_pose.header.stamp = tf_global_odo_transformation_.stamp_;
   return out_visualize_pose;
}

void SLAMProcessor::configure(SLAMConfig& config )
{

}
