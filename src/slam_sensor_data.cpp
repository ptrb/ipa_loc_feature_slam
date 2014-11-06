/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#include "slam_sensor_data.h"
#include <set>

/*! The constructor transforms all objects into the /base_link frame and stores 
 * them as a SLAMObservation implementation. 
 * 
 */
SLAMSensorData::SLAMSensorData( const ipa_navigation_msgs::FeatureList::ConstPtr& msg_featr , tf::TransformListener * tf_listener , Vector3d& odom , SLAMConfig& config )
{
   // Add features to data queue
   try {
      if( config.use_points ){
         for( int i = 0 ; i < msg_featr->points.size() ; i++ ){
            if( msg_featr->source_id != "rfid") {
               v_observations_.push_back( new SLAMPointObservation( msg_featr->points[i] , msg_featr->header , tf_listener  ) );
            }
         }
      }
      if( config.use_lines ){
         for( int i = 0 ; i < msg_featr->lines.size() ; i++ ){
            double dy, dx;
            dx = (msg_featr->lines[i].source.x*msg_featr->lines[i].source.x) + (msg_featr->lines[i].source.y*msg_featr->lines[i].source.y);
            dy = (msg_featr->lines[i].target.x*msg_featr->lines[i].target.x) + (msg_featr->lines[i].target.y*msg_featr->lines[i].target.y);

            if( dy < 100.0 || dx < 100.0 ){
               v_observations_.push_back( new SLAMLineObservation( msg_featr->lines[i] , msg_featr->header , tf_listener ) );}
            }
      }
      if( config.use_corners ){
         // Placeholder
      }
   }
   catch (int n) {
      ROS_INFO("Catched exception");
      throw 13;
   }
   
   // Odom
   odom_ = odom;
   
   // Covariance Odom
   double odo_x, odo_y, odo_o;   
   if( odom[0] < 1e-20 && odom[1] < 1e-20 && odom[2] < 1e-20 )
   {
      odo_x = 1e-8;
      odo_y = 1e-8;
      odo_o = 1e-8;
   }
   else
   {
      odo_x = odom[0];
      odo_y = odom[1];
      odo_o = odom[2];
   }   
   cov_odom_(0,0) = fabs(config.odo_var_factor_distance * odo_x);
   cov_odom_(1,1) = fabs(config.odo_var_factor_distance * odo_y);
   cov_odom_(2,2) = fabs(config.odo_var_factor_orientation * odo_o);   
   
   timestamp_ = msg_featr->header.stamp.toSec();
   if( msg_featr->header.frame_id == "/base_laser_rear_link" ) sensor = 2;
   else sensor = 1;
}

/*! Retrieve the object's observations */

std::vector<SLAMObservation *> SLAMSensorData::getObservations() const
{
   return v_observations_;
}

/*! Destructor will delete all observation objects */
SLAMSensorData::~SLAMSensorData()
{
   std::vector<SLAMObservation *>::iterator it;
   for( it = v_observations_.begin() ; it != v_observations_.end() ; ++it ){
        delete *it;
   }
}

/*! Copying an instance and the complete set of observation pointers. */
SLAMSensorData::SLAMSensorData( const SLAMSensorData& old ) {
   v_observations_.reserve( old.v_observations_.size() );
   for( int i = 0 ; i < old.v_observations_.size() ; i++ ){
      v_observations_.push_back( old.v_observations_[i]->getCopy() );
   }   
   timestamp_ = old.timestamp_;
   odom_ = old.odom_;
   cov_odom_ = old.cov_odom_;
   sensor = old.sensor;
}
