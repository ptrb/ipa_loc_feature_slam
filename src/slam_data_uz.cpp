/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#include "slam_data_uz.h"
#include <set>

/*! The constructor transforms all objects into the /base_link frame and stores 
 * them as a SLAMObservation implementation. 
 * 
 */
SLAMData::SLAMData( const ipa_navigation_msgs::FeatureList::ConstPtr& msg_featr , tf::TransformListener * tf_listener)
{
   // Add features to data queue
   try {
      for( int i = 0 ; i < msg_featr->points.size() ; i++ ){
	 if( msg_featr->source_id != "rfid") {
	    v_observations_.push_back( new SLAMPointObservation( msg_featr->points[i] , msg_featr->header , tf_listener  ) );
	 }
      }
      for( int i = 0 ; i < msg_featr->lines.size() ; i++ ){
	 double squa_length, dy, dx;
	 dx = (msg_featr->lines[i].source.x*msg_featr->lines[i].source.x) + (msg_featr->lines[i].source.y*msg_featr->lines[i].source.y);
	 dy = (msg_featr->lines[i].target.x*msg_featr->lines[i].target.x) + (msg_featr->lines[i].target.y*msg_featr->lines[i].target.y);
	 squa_length = dx*dx + dy*dy;
// 	 dist_end = sqrt( msg_featr->lines[i].target.x*msg_featr->lines[i].target.x+msg_featr->lines[i].target.y*msg_featr->lines[i].target.y );

	 if( dy < 100.0 || dx < 100.0 ){
	    v_observations_.push_back( new SLAMLineObservation( msg_featr->lines[i] , msg_featr->header , tf_listener ) );}
	 }
   }
   catch (int n) {
      ROS_INFO("Catched exception");
      throw 13;
   }
    
   timestamp_ = msg_featr->header.stamp.toSec();
   if( msg_featr->header.frame_id == "/base_laser_rear_link" ) sensor = 2;
   else sensor = 1;
}

void SLAMData::pushObservations(const ipa_navigation_msgs::FeatureList::ConstPtr& msg_featr, tf::TransformListener* tf_listener)
{
   try {
      for( int i = 0 ; i < msg_featr->points.size() ; i++ ){
	 if( msg_featr->source_id != "rfid") {
	    v_observations_.push_back( new SLAMPointObservation( msg_featr->points[i] , msg_featr->header , tf_listener  ) );
	 }
      }
      for( int i = 0 ; i < msg_featr->lines.size() ; i++ ){
	 double squa_length, dy, dx;
	 dx = (msg_featr->lines[i].source.x*msg_featr->lines[i].source.x) + (msg_featr->lines[i].source.y*msg_featr->lines[i].source.y);
	 dy = (msg_featr->lines[i].target.x*msg_featr->lines[i].target.x) + (msg_featr->lines[i].target.y*msg_featr->lines[i].target.y);
	 squa_length = dx*dx + dy*dy;
// 	 dist_end = sqrt( msg_featr->lines[i].target.x*msg_featr->lines[i].target.x+msg_featr->lines[i].target.y*msg_featr->lines[i].target.y );

	 if( dy < 100.0 || dx < 100.0 ){
	    v_observations_.push_back( new SLAMLineObservation( msg_featr->lines[i] , msg_featr->header , tf_listener ) );}
	 }
   }
   catch (int n) {
      ROS_INFO("Catched exception");
      throw 13;
   }
}


/*! Retrieve the object's observations */

std::vector<SLAMObservation *> SLAMData::getObservations() const
{
   return v_observations_;
}

/*! Destructor will delete all observation objects */
SLAMData::~SLAMData()
{
   std::vector<SLAMObservation *>::iterator it;
   for( it = v_observations_.begin() ; it != v_observations_.end() ; ++it ){
        delete *it;
   }
}

/*! Copying an instance and the complete set of observation pointers. */
SLAMData::SLAMData( const SLAMData& old ) {
   v_observations_.reserve( old.v_observations_.size() );
   for( int i = 0 ; i < old.v_observations_.size() ; i++ ){
      v_observations_.push_back( old.v_observations_[i]->getCopy() );
   }   
   timestamp_ = old.timestamp_;

   sensor = old.sensor;
}
