/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#include <sstream>
// ROS 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
// ROS - IPA
#include "ipa_navigation_msgs/FeatureList.h"
#include <ipa_navigation_msgs/success.h>
// This package
#include "slam_processor.h"
#include "slam_sensor_data.h"
#include "slam_config.h"

#include "message_filters/subscriber.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "boost/bind.hpp"

#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <ipa_loc_feature_slam_development/ipa_loc_feature_slam_developmentConfig.h>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <pwd.h>
#include <dirent.h>
DIR *dpdf;
struct dirent *epdf;

class IpaLocFeatureSLAMRos
{
public:
   ros::NodeHandle n_;
   
   dynamic_reconfigure::Server<ipa_loc_feature_slam_development::ipa_loc_feature_slam_developmentConfig> server;
   dynamic_reconfigure::Server<ipa_loc_feature_slam_development::ipa_loc_feature_slam_developmentConfig>::CallbackType f;
   
   SLAMProcessor prog_;
   SLAMConfig config_;
   
   ros::Subscriber sub_feature_list_;
   
   ros::Publisher pub_current_pose_;
   ros::Publisher pub_sample_poses_;
   ros::Publisher pub_best_map_;
   ros::Publisher pub_best_pose_;
   ros::Publisher pub_feature_markers_;
   ros::Publisher pub_candidate_markers_;
   
   tf::TransformListener tf_listener_;
   tf::TransformBroadcaster tf_broadcaster_;
   
   ros::ServiceServer write_best_map_to_file_;
   
   IpaLocFeatureSLAMRos() : tf_listener_( ros::Duration(1000), true ) , prog_( &tf_listener_ )
   {
      write_best_map_to_file_ = n_.advertiseService("/ipa_loc_feature_slam_development/write_best_map_to_file", &IpaLocFeatureSLAMRos::service_callback_write_best_map, this);
	    
      sub_feature_list_ = n_.subscribe<ipa_navigation_msgs::FeatureList>( "/featList_meas", 1000, &IpaLocFeatureSLAMRos::topic_callback_features ,this );
   
      pub_best_pose_ = n_.advertise<geometry_msgs::PoseStamped>("/pub_best_pose",10);
      pub_current_pose_  = n_.advertise<geometry_msgs::PoseStamped>("/pub_current_pose",10);
      pub_sample_poses_ = n_.advertise<visualization_msgs::Marker>( "/visualization_particle_poses" , 10 );         
      pub_feature_markers_ = n_.advertise<visualization_msgs::MarkerArray>( "/visualization_feature_markers" , 10 );
      pub_candidate_markers_ = n_.advertise<visualization_msgs::MarkerArray>( "/visualization_candidate_markers" , 10 );
      pub_best_map_ = n_.advertise<visualization_msgs::MarkerArray>( "/visualization_best_map" , 10 );
            
      f = boost::bind(&IpaLocFeatureSLAMRos::configure_callback, this, _1, _2);
      server.setCallback(f);
      
      n_.param("/ipa_loc_feature_slam_development/data_association_threshold_new_feature" , config_.data_association_threshold_new_feature , (double)0.01);
      n_.param("/ipa_loc_feature_slam_development/min_existence_estimate_map_feature" , config_.min_existence_estimate_map_feature , (int) 40 );
      n_.param("/ipa_loc_feature_slam_development/particle_count" , config_.particle_count , (int) 200 );
      n_.param("/ipa_loc_feature_slam_development/max_particle_copies" , config_.max_particle_copies , (int) 10 );
      n_.param("/ipa_loc_feature_slam_development/threshold_effective_sample_size" , config_.threshold_effective_sample_size , (double)100);
      
      n_.param("/ipa_loc_feature_slam_development/use_points" , config_.use_points , (bool) true );
      n_.param("/ipa_loc_feature_slam_development/use_lines" , config_.use_lines , (bool) true );
      n_.param("/ipa_loc_feature_slam_development/use_corners" , config_.use_corners , (bool) true );
      n_.param("/ipa_loc_feature_slam_development/odo_var_factor_distance" , config_.odo_var_factor_distance , (double)0.005);
      n_.param("/ipa_loc_feature_slam_development/odo_var_factor_orientation" , config_.odo_var_factor_orientation , (double)0.05);
      
      n_.param<std::string>("/ipa_loc_feature_slam_development/best_map_file_name" , config_.best_map_file_name , "best_map");
      n_.param<std::string>("/ipa_loc_feature_slam_development/best_map_file_path" , config_.best_map_file_path , ".");
   }
      
   void update(){
      static bool initialized;
      if( !initialized )
      {
	 initialized = prog_.getInitStatus( config_ ); 
      }
      else 
      {
         prog_.update( config_ );
	 
	 tf_broadcaster_.sendTransform( prog_.getSlamMapTransform() );
	 
         pub_current_pose_.publish( prog_.visualizePoseFromOdometry() );
         pub_best_pose_.publish( prog_.visualizeBestPose() );
	 pub_sample_poses_.publish( prog_.getParticleMarkers() );
	 pub_best_map_.publish( prog_.getBestMap() );
	 pub_feature_markers_.publish( prog_.getFeatureMarkersCandidates( config_ ) );
// 	 pub_candidate_markers_.publish( prog_.getFeatureMarkersCandidates() );
      }
   }

   void topic_callback_features( const ipa_navigation_msgs::FeatureList::ConstPtr& msg_featr )
   {
       if(msg_featr->source_id != "rfid" && ( !msg_featr->lines.empty() || !msg_featr->points.empty() ) )
       {
	 prog_.pushNewData( msg_featr , config_ );
       }
   }   
    
   void configure_callback(ipa_loc_feature_slam_development::ipa_loc_feature_slam_developmentConfig &config, uint32_t level)
   {
      config_.data_association_threshold_new_feature = config.data_association_threshold_new_feature;
      config_.min_existence_estimate_map_feature = config.min_existence_estimate_map_feature;
      config_.use_corners = config.use_corners;
      config_.use_lines = config.use_lines;
      config_.use_points = config.use_points;
      config_.odo_var_factor_distance = config.odo_var_factor_distance;
      config_.odo_var_factor_orientation = config.odo_var_factor_orientation;
      config_.best_map_file_name = config.best_map_file_name;
      config_.best_map_file_path = config.best_map_file_path;
      config_.max_particle_copies = config.max_particle_copies;
      config_.threshold_effective_sample_size = config.threshold_effective_sample_size;
   }
   
   void configure()
   {
      prog_.configure( config_ );
   }
   
   bool service_callback_write_best_map( ipa_navigation_msgs::success::Request &req , ipa_navigation_msgs::success::Response &res )
   {
      visualization_msgs::MarkerArray best_map = prog_.getBestMap();
      res.success = false;
      std::ofstream state_file_;
      std::string temp = config_.best_map_file_path + "/" + config_.best_map_file_name;
      ROS_INFO("hello");
      try
      {
	 state_file_.open(temp.c_str(), std::ios::out | std::ios::trunc);
	 ROS_INFO("state file is: %d",state_file_.is_open());
	 int id = 1;
	 if (state_file_.is_open())
	 {
	    state_file_ <<"source_id: \"fast slam processed\""<<std::endl;
	    state_file_ <<"features:"<<std::endl;
	    for( int i = 0 ; i < best_map.markers.size() ; i++ ){
	       if( best_map.markers[i].type == visualization_msgs::Marker::POINTS ){
		  for( int j = 0 ; j < best_map.markers[i].points.size() ; j++){
		     state_file_<<"  - id: "<<id<<std::endl;
		     state_file_<<"    point: ["<<best_map.markers[i].points[j].x<<", "<<best_map.markers[i].points[j].y<<", 0.0]"<<std::endl;
		     id++;
		  }
	       }
	       else {
		  for( int a = 0 ; a < best_map.markers[i].points.size() ; a++){
		     state_file_<<"  - id: "<<id<<std::endl;
		     state_file_<<"    source: ["<<best_map.markers[i].points[a].x<<", "<<best_map.markers[i].points[a].y<<", 0.0]"<<std::endl;
		     a++;
		     state_file_<<"    target: ["<<best_map.markers[i].points[a].x<<", "<<best_map.markers[i].points[a].y<<", 0.0]"<<std::endl;
		     id++;
		  }
	       }
	 }

	 state_file_.close();
	 res.success = true;
	 }
	 else
	 {
	 ROS_ERROR("File %s not open.", temp.c_str());
	 res.success = false;
	 }
      }
      catch (std::fstream::failure& e)
      {
	 ROS_ERROR("Caught exception while trying to open and write to file %s.", temp.c_str());
	 res.success = false;
      }
      return res.success;
   }

};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ipa_loc_feature_slam_development");
   IpaLocFeatureSLAMRos node;
   node.configure();
   ros::Rate loop_rate(50); // Hz
   
   while(node.n_.ok())
   {
          node.update();         
          loop_rate.sleep();
          ros::spinOnce();
   }
   return 0;
}
