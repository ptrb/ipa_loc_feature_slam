/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#include "slam_observation.h"
#include <string>

bool transformPoint( const geometry_msgs::Point& point_in , Vector2d& point_out , const std_msgs::Header& header_in , tf::TransformListener * tf_listener ){

   geometry_msgs::PointStamped point_old_frame;
   geometry_msgs::PointStamped point_new_frame;
   std::string * error_msg = new std::string;
   ros::Duration du_wait(2);
   ros::Duration du_interval(0.005);
   
   point_old_frame.header = header_in;
   point_old_frame.point = point_in;
   
   if( tf_listener->canTransform( "/base_link", header_in.frame_id, header_in.stamp, error_msg ) ){
      tf_listener->transformPoint( "/base_link" , point_old_frame , point_new_frame );
      point_out[0] = point_new_frame.point.x;
      point_out[1] = point_new_frame.point.y;
   }
   else {
      ROS_WARN("Transform not possible. %s", error_msg->c_str() );
      delete error_msg;
      return false;
   }
   delete error_msg;
   return true;
}
      
SLAMPointObservation::SLAMPointObservation( const ipa_navigation_msgs::PointFeature& ipa_point , const std_msgs::Header& header , tf::TransformListener * tf_listener )
{
   if ( !transformPoint( ipa_point.point , point_ , header , tf_listener ) ) throw 13;
   cov_ = MatrixXd::Zero(2,2);
   tf::StampedTransform transform_to_base_link;
   tf_listener->lookupTransform( "base_link", header.frame_id , header.stamp , transform_to_base_link );
   const double angle = tf::getYaw( transform_to_base_link.getRotation() );
   Matrix2d rot;
   rot << cos(angle), sin(angle), 
   -sin(angle), cos(angle);
   MatrixXd cov_all = MatrixXd::Zero(2,2);
   cov_all << ipa_point.covariance[0], ipa_point.covariance[1],
   ipa_point.covariance[2], ipa_point.covariance[3];
//    cov_all = cov_all * 10;
   cov_all = 0.5 * ( cov_all + cov_all.transpose() );
   
   cov_ = rot * cov_all * rot.transpose();
   cov_ = cov_ * 20.0;
}

Vector2d SLAMPointObservation::getPoint() const
{
   return point_;
}

SLAMLandmark* SLAMPointObservation::createFeature( Vector3d pose_robot , double timestamp )
{
   return (new SLAMPointLandmark( *this , pose_robot , timestamp ));
}

SLAMLineObservation::SLAMLineObservation( const ipa_navigation_msgs::LineFeature& line , const std_msgs::Header& header,tf::TransformListener * tf_listener )
{
   if( !transformPoint( line.source , start_ , header , tf_listener ) ) throw 13;
   if( !transformPoint( line.target , end_ , header , tf_listener) ) throw 13;
   
   cov_ = MatrixXd::Zero(2,2);
   
   tf::StampedTransform transform_to_base_link;
   tf_listener->lookupTransform( "base_link", header.frame_id , header.stamp , transform_to_base_link );
   
   const double angle = tf::getYaw( transform_to_base_link.getRotation() );
   // getAngle-function ignores sign! bad. 
   Matrix2d rot;
   rot << cos(angle), sin(angle), 
   -sin(angle), cos(angle);
   MatrixXd cov_all, rot_all;
   
   rot_all = MatrixXd::Zero(4,4);
   rot_all.block(0,0,2,2) = rot;
   rot_all.block(2,2,2,2) = rot;
   
   cov_all = MatrixXd::Zero(4, 4);
   int index = 0;
   for( int i = 0 ; i < 4 ; i++ ){
      for( int j = 0 ; j < 4 ; j++ ){
	 cov_all(i,j) = line.covariance[index];
	 index++;
      }     
   }
   
   cov_all = 0.5 * ( cov_all + cov_all.transpose() );
   cov_all = rot_all * cov_all * rot_all.transpose();
   
   // Calculate covariance for distance-angle parametrization.
   const double x = start_(0);
   const double y = start_(1);
   const double a = end_(0);
   const double b = end_(1);
   MatrixXd A;
   A = MatrixXd::Zero(2,4); // jacobian, functional model
   
   const double a_m_x = a-x;
   const double b_m_y = b-y;
   const double t = sqrt((a_m_x*a_m_x)/(b_m_y*b_m_y)+1)*( a*a-2*a*x+b*b-2*b*y+x*x+y*y );
   const double reci_t = 1/ t;
   const double reci_z = 1 / (a_m_x*a_m_x+b_m_y*b_m_y);
   const double reci_t_b_m_y = 1 / (b_m_y * t);
  
   A(0,0) = (a*a-a*x+b*b_m_y) * reci_t;
   A(0,1) = - (a_m_x*(a*a-a*x+b*b_m_y)) * reci_t_b_m_y;
   A(0,2) = (x*x-a*x+y*(y-b)) * reci_t;
   A(0,3) = (a_m_x*(a*x+y*b_m_y-x*x)) * reci_t_b_m_y;
   A(1,0) = b_m_y * reci_z;
   A(1,1) = a_m_x * reci_z;
   A(1,2) = - A(1,0);
   A(1,3) = - A(1,1);
   
   cov_ = A * cov_all * A.transpose() ;
   cov_ = cov_ * 16.0;
}

Vector2d SLAMLineObservation::getEnd() const
{
   return end_;
}

Vector2d SLAMLineObservation::getStart() const
{
   return start_;
}

SLAMLandmark* SLAMLineObservation::createFeature(Vector3d pose_robot , double timestamp  )
{
   return (new SLAMLineLandmark( *this , pose_robot , timestamp ));
}


SLAMPointObservation::SLAMPointObservation(const SLAMPointObservation& feature ) : SLAMObservation(feature)
{
   point_ = feature.point_;
   cov_ = feature.cov_;
}

SLAMPointObservation* SLAMPointObservation::getCopy()
{
   return (new SLAMPointObservation(*this));
}



SLAMLineObservation::SLAMLineObservation(const SLAMLineObservation& feature ) : SLAMObservation(feature)
{
   start_ = feature.start_;
   end_ = feature.end_;
   cov_ = feature.cov_;
}

SLAMLineObservation* SLAMLineObservation::getCopy()
{
   return (new SLAMLineObservation(*this));
}

bool SLAMLineObservation::suitableInitObs()
{
   if( ( end_ - start_ ).norm() < 0.3 ){
      return false;
   }
   else return true;
}

bool SLAMPointObservation::suitableInitObs()
{
   if( point_.norm() > 12 ) return false;
   else return true;
}

