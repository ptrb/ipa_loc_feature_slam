/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#include "slam_landmark.h"
#include <math.h>
#include <string>
#include "slam_particle.h"
#include <eigen3/Eigen/Cholesky>

boost::mt19937 rng_ = boost::mt19937();
boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > var_ = boost::variate_generator<boost::mt19937&,boost::normal_distribution<> >(rng_, boost::normal_distribution<>(0.0, 1.0));

Vector3d getRandomVector()
{
   Vector3d random_vector;
   random_vector(0) = var_();
   random_vector(1) = var_();
   random_vector(2) = var_();
   return random_vector;
}

// normalizes angle to range [0;2*pi]
double normalize2Pi(double angle)
{
   return angle - floor( angle / (2 * M_PI)) * 2 * M_PI;
}

   // normalizes angle to range [-pi;pi]
double normalizePi(double angle)
{
   double ret = normalize2Pi(angle);
   if (ret>M_PI)
	    ret -= 2 * M_PI;
   return ret;
}
    
inline Vector2d projectPointOnLine( const Vector2d& start , const Vector2d& end , const Vector2d& point ){
   Vector2d line_orientation, projected_point, start_to_point;
   
   start_to_point[0] = point[0] - start[0];
   start_to_point[1] = point[1] - start[1];
   line_orientation[0] = end[0] - start[0];
   line_orientation[1] = end[1] - start[1];
   line_orientation.normalize();
   projected_point = start_to_point.dot( line_orientation ) * line_orientation;
   projected_point[0] = projected_point[0] + start[0];
   projected_point[1] = projected_point[1] + start[1];
   
   return projected_point;
}    

// Calculate the minimum distance between a point and a line segment.
inline double minimum_distance_line_and_point( const Vector2d& start , const Vector2d& end , const Vector2d& point ) {
   double t, dist;
   Vector2d line_orientation, projected_point, start_to_point;
   
   start_to_point = point - start;
   line_orientation = end - start;
   line_orientation.normalize();
   t = start_to_point.dot( line_orientation );
   
   if( t < 0.0 ) dist = start_to_point.norm();
   else if( t > (end-start).norm() ) dist = (point-end).norm();
   else {
      projected_point = t * line_orientation + start;
      dist = (point-projected_point).norm();
   }
   return dist;
}
    
inline bool linesIntersect( const Vector2d& start_1 , const Vector2d& end_1 , const Vector2d& start_2 , const Vector2d& end_2 ){
   Vector2d cmp, r, s;
   double cmp_r, cmp_s, rxs;
   
   cmp[0] = start_2[0] - start_1[0];
   cmp[1] = start_2[1] - start_1[1];
   r[0] = end_1[0] - start_1[0];
   r[1] = end_1[1] - start_1[1];
   s[0] = end_2[0] - start_2[0];
   s[1] = end_2[1] - start_2[1];
   
   cmp_r = cmp[0]*r[1] - cmp[1]*r[0];
   cmp_s = cmp[0]*s[1] - cmp[1]*s[0];
   rxs = r[0]*s[1] - r[1]*s[0];
   
   if( cmp_r == 0 ) return ( 
      ( ( start_2[0] - start_1[0] < 0 ) != ( start_2[0] - end_1[0] < 0 ) )
      || ( ( start_2[1] - start_1[1] < 0 ) != ( start_2[1] - end_1[1] < 0 ) ));
   if( rxs == 0 ) return false;
   
   double rxsr = 1 / rxs;
   double t = cmp_s * rxsr;
   double u = cmp_r * rxsr;
   
   return (t >= 0) && (t <= 1) && (u >= 0) && (u <= 1);
}    

inline bool linesOverlap( const double& threshold , const Vector2d& start_1 , const Vector2d& end_1 , const Vector2d& start_2 , const Vector2d& end_2 ){
   double length_1, length_2;
   length_1 = (end_1 - start_1).norm();
   length_2 = (end_2 - start_2).norm();
   bool match_1_s, match_1_e, match_2_s, match_2_e;
   match_2_s = (minimum_distance_line_and_point( start_1,end_1,start_2 ) < threshold);
   match_2_e = (minimum_distance_line_and_point( start_1,end_1,end_2 ) < threshold );
   match_1_s = (minimum_distance_line_and_point( start_2,end_2,start_1 ) < threshold ); 
   match_1_e = (minimum_distance_line_and_point( start_2,end_2,end_1 ) < threshold );
   int matches = (int)match_1_s + (int)match_1_e + (int)match_2_s + (int)match_2_e;
   if( matches > 2 ) return true;     
   else {
      // one line fully overlaps the other
      if( ( match_2_s && match_2_e ) || ( match_1_s && match_1_e ) ) return true;
      else if( ( match_2_s || match_2_e ) && ( match_1_s || match_1_e ) ){
	 Vector2d start , end , point;
	 if( match_1_s && match_2_s ){
	    start = start_1;
	    end = end_1;
	    point = start_2;
	 }
	 else if( match_1_e && match_2_s ){
	    start = end_1;
	    end = start_1;
	    point = start_2;	       
	 }
	 else if( match_1_s && match_2_e ){
	    start = start_1;
	    end = end_1;
	    point = end_2;
	 }
	 else if( match_1_e && match_2_e ){
	    start = end_1;
	    end = start_1;
	    point = end_2;	       
	 }
	 Vector2d projected = projectPointOnLine( start,end,point );
	 double overlap = (projected - start).norm();
         double line_length = (end-start).norm();
         double remain = (projected - end).norm();
         if( remain > line_length ) return false;
	 if( overlap > 0.5 || overlap >= length_1*0.9 || overlap >= length_2*0.9  ) {
	    return true;
	 }
      }
      
   }
   return false;
}    

Vector2d linePointsToRohTheta( double a_x , double a_y , double e_x , double e_y )
{
   Vector2d ret;
   ret(1) = atan2( e_x - a_x , e_y - a_y )* -1;
   ret(0) = ( 0.5 * ( (a_x+e_x)*cos(ret(1)) + (a_y+e_y)*sin(ret(1)) ));
   if(ret(0) < 0) {
      ret(0) = fabs(ret(0));
      ret(1) += M_PI;
   }
   ret(1) = normalizePi( ret(1) );
   return ret;
}

// h Point
inline Vector2d transformPointGlobalToLocal( double x_in , double y_in , const Vector3d& pose_robot )
{
   Vector2d ret;   
   double angle = normalize2Pi( pose_robot(2) );
   double cos_o = cos( -angle );
   double sin_o = sin( -angle ); 
   double dx = x_in - pose_robot(0);
   double dy = y_in - pose_robot(1); 
   ret(0) = dx * cos_o - dy * sin_o;
   ret(1) = dy * cos_o + dx * sin_o;
   return ret;
}

// h-1 Point
inline Vector2d transformPointLocalToGlobal( double x_in , double y_in , const Vector3d& pose_robot )
{
   Vector2d ret;   
   double angle = normalize2Pi( pose_robot(2) );
   double cos_o = cos( angle );
   double sin_o = sin( angle );
   ret(0) = x_in * cos_o - y_in * sin_o + pose_robot(0);
   ret(1) = y_in * cos_o + x_in * sin_o + pose_robot(1);
   return ret;
}

void SLAMLandmark::refreshLandmarkForIteration( const Vector3d& pose_robot )
{
   calculateJacobianMap( pose_robot );
   calculateJacobianPose( pose_robot );
   should_have_been_seen_ = false;
}

double SLAMLandmark::measureLikelihood( SLAMObservation* obs, Vector3d& pose_robot , Matrix3d& cov_odo )
{
   if( !shouldExamine( obs , pose_robot ) ) return 0.0;
   predictMeasurement( pose_robot );
   setObservationDifference( obs );
   
   meas_cov_q_.noalias() = jacobian_map_ * cov_ * jacobian_map_.transpose() + obs->getCovar();

   mean_pose_variance_ = (jacobian_pose_.transpose() * meas_cov_q_.inverse() * jacobian_pose_ + cov_odo.inverse()).inverse();
   
   mean_pose_.noalias() = mean_pose_variance_ * jacobian_pose_.transpose() * meas_cov_q_.inverse() * z_diff_ + pose_robot;
//    mean_pose_(2) = normalizePi( mean_pose_(2) );
   
   Matrix3d cholesky = mean_pose_variance_.llt().matrixL();   
   // Cholesky 
   pose_ = mean_pose_ + cholesky * getRandomVector(); 
   
   predictMeasurement( pose_ );
   setObservationDifference( obs );

   if( z_diff_.norm() > 1.3 ) return 0.0;
   
   Matrix<double, 1, 1> mahalonobis_distance = z_diff_.transpose() * meas_cov_q_.inverse() * z_diff_;

   double weight = 1/(2*M_PI*sqrt(fabs((meas_cov_q_).determinant()))) * exp( -0.5 * mahalonobis_distance(0,0) ); 
   ROS_ERROR_COND( isnan(weight) , "Landmark::measureLikelihood returns NAN");
   return weight;
}

double SLAMLandmark::updateKalman( SLAMObservation * obs , Vector3d& mean_pose , double timestamp , Matrix3d& proposal_cov  )
{    
   calculateJacobianMap( mean_pose );
   calculateJacobianPose( mean_pose );
   measureLikelihood( obs , mean_pose , proposal_cov );
   
   MatrixXd kalman_gain = cov_ * jacobian_map_.transpose() * meas_cov_q_.inverse();
     
   state_ = state_ + kalman_gain * z_diff_ * getFactor( obs );

   MatrixXd I = MatrixXd::Identity( cov_.rows() , cov_.cols() );
   MatrixXd cov_old;
   cov_old = cov_;
   cov_ = (I - ( kalman_gain * jacobian_map_ ) ) * cov_;
   typeSpecializedUpdate( obs );
   Matrix2d L;
   L.noalias() = jacobian_pose_ * proposal_cov * jacobian_pose_.transpose() + jacobian_map_*cov_old*jacobian_map_.transpose()+obs->getCovar();
   Matrix<double, 1, 1> mahalonobis_distance = z_diff_.transpose() * L.inverse() * z_diff_;
   double weight = 1/(2*M_PI*sqrt(fabs((L).determinant()))) * exp( -0.5 * mahalonobis_distance(0,0) );
   if( weight < 1e-14 ) {
      if( dynamic_cast<SLAMPointObservation *>(obs) ){
	 ROS_INFO("POINT");
      }
      else {
	 ROS_INFO_STREAM("z_diff\n"<<z_diff_);
	 
      }
   }
   if( isnan(weight) ){
      ROS_INFO("++++++++++++++++++++++++++ updateKalman Point\n");
      ROS_INFO_STREAM("pose:\n"<<pose_);
      ROS_INFO_STREAM("L:\n"<<L);
      ROS_INFO_STREAM("z diff:\n"<<z_diff_);
      ROS_INFO_STREAM("tmp:\n"<<mahalonobis_distance);
      ROS_INFO_STREAM("L inverse: "<< L.inverse() );
      ROS_INFO_STREAM("L determinant: "<< L.determinant() );
      ROS_INFO_STREAM("cov old: "<<cov_old);
   }
   
   existence_estimate_++;
   proposal_cov = mean_pose_variance_;
   mean_pose = mean_pose_;
   return weight;
}

SLAMPointLandmark::SLAMPointLandmark( SLAMPointObservation& o1 , Vector3d pose_robot , double timestamp )
{
   jacobian_pose_ = MatrixXd::Zero(2,3);
   jacobian_map_ = MatrixXd::Zero(2,2);
   prediction_ = VectorXd::Zero(2);
   meas_cov_q_ = MatrixXd::Zero(2,2);
   z_diff_ = VectorXd::Zero(2);
   
   state_ = transformPointLocalToGlobal( o1.getPoint()[0] , o1.getPoint()[1] , pose_robot );
   
   calculateJacobianMap( pose_robot );
   Matrix2d H_inverse = jacobian_map_.inverse();
      
   cov_.noalias() = H_inverse * o1.getCovar() * H_inverse.transpose();

   existence_estimate_ = 5;  
}

void SLAMPointLandmark::calculateJacobianMap( const Vector3d& pose_robot )
{
   double angle = pose_robot(2) - std::floor( pose_robot(2) / (2 * M_PI)) * 2 * M_PI;
   double cos_o = cos(-angle);
   double sin_o = sin(-angle);
   jacobian_map_(0,0) = cos_o;
   jacobian_map_(0,1) = -sin_o;
   jacobian_map_(1,0) = sin_o;
   jacobian_map_(1,1) = cos_o;
}

void SLAMPointLandmark::calculateJacobianPose( const Vector3d& pose_robot )
{
   double angle = pose_robot(2) - std::floor( pose_robot(2) / (2 * M_PI)) * 2 * M_PI;
   double cos_o = cos(-angle);
   double sin_o = sin(-angle);
   double dx = state_(0) - pose_robot(0);
   double dy = state_(1) - pose_robot(1); 
   jacobian_pose_(0,0) = -cos_o;
   jacobian_pose_(0,1) = sin_o;
   jacobian_pose_(0,2) = -dx*sin_o-dy*cos_o;
   jacobian_pose_(1,0) = -sin_o;
   jacobian_pose_(1,1) = -cos_o;
   jacobian_pose_(1,2) = -dy*sin_o+dx*cos_o;
}

void SLAMPointLandmark::predictMeasurement( const Vector3d& pose_robot )
{
   prediction_ = transformPointGlobalToLocal( state_(0) , state_(1) , pose_robot );
}

bool SLAMPointLandmark::shouldExamine( SLAMObservation* obs , const Vector3d& pose_robot )
{
   SLAMPointObservation * point_obs = dynamic_cast<SLAMPointObservation *>( obs );
   if( point_obs ) return true;
   else return false;
}

void SLAMPointLandmark::typeSpecializedUpdate( SLAMObservation * obs )
{
   if( cov_(0,0) < 1e-15 ) cov_(0,0) = 0.000005;
   while( cov_(0,0) < 0.00001 ){
      cov_(0,0) *= 2;
      cov_(0,1) *= sqrt(2);
      cov_(1,0) *= sqrt(2); 
   }
   if( cov_(1,1) < 1e-15 ) cov_(1,1) = 0.000005;
   while( cov_(1,1) < 0.00001 ){
      cov_(1,1) *= 2;
      cov_(0,1) *= sqrt(2);
      cov_(1,0) *= sqrt(2); 
   }
}

void SLAMPointLandmark::setObservationDifference( SLAMObservation * obs )
{
   SLAMPointObservation * point_obs = dynamic_cast<SLAMPointObservation *>( obs );
   z_diff_.noalias() = point_obs->getPoint() - prediction_;
}


/////////////////////////////-    L I N E    -\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

void SLAMLineLandmark::calculateJacobianMap(const Vector3d& pose_robot )
{
      intersect_ = false;
      double alpha = state_(1) - normalizePi( atan2( pose_robot(1) , pose_robot(0) ) );
      if( alpha < - M_PI ) alpha += (M_PI * 2);
      if( alpha > M_PI ) alpha -= (M_PI * 2);
      
      if( fabs(alpha) < ( M_PI / 2 ) ){
	 double a = state_(0) / cos(alpha);
	 double origin_r = sqrt( pose_robot(0)*pose_robot(0) + pose_robot(1)*pose_robot(1) );
	 if( fabs( a ) < origin_r ) intersect_ = true;
      }   
   if( intersect_ ){
      jacobian_map_(0,0) = -1;
      jacobian_map_(0,1) = -sin(state_(1))*pose_robot[0]+cos(state_(1))*pose_robot[1];
   }
   else {
      jacobian_map_(0,0) = 1;
      jacobian_map_(0,1) = sin(state_(1))*pose_robot[0]-cos(state_(1))*pose_robot[1];      
   } 
   jacobian_map_(1,0) = 0;
   jacobian_map_(1,1) = 1;
}

void SLAMLineLandmark::calculateJacobianPose(const Vector3d& pose_robot )
{
   if( intersect_ ){
      jacobian_pose_(0,0) = cos(state_(1));
      jacobian_pose_(0,1) = sin(state_(1));
   }
   else {
      jacobian_pose_(0,0) = -cos(state_(1));
      jacobian_pose_(0,1) = -sin(state_(1));     
   }
   jacobian_pose_(0,2) = 0;
   jacobian_pose_(1,0) = 0;
   jacobian_pose_(1,1) = 0;
   jacobian_pose_(1,2) = -1;
}

void SLAMLineLandmark::predictMeasurement(const Vector3d& pose_robot )
{   
      prediction_(0) = fabs( state_(0) - pose_robot(0)*cos(state_(1)) - pose_robot(1)*sin(state_(1)) );      
      if( intersect_ ){
	 prediction_(1) = normalizePi( state_(1) - normalizePi( pose_robot(2) ) - M_PI );
      }
      else {
	 prediction_(1) = normalizePi( state_(1) - normalizePi( pose_robot(2) ) );
      }
}

bool SLAMLineLandmark::shouldExamine( SLAMObservation* obs , const Vector3d& pose_robot )
{
   SLAMLineObservation * line_obs = dynamic_cast<SLAMLineObservation *>( obs );
   if( line_obs )
   {  
      // Line was only seen from other side
      if( intersect_ == facing_origin_ ) return false;
      
      // Check for overlap
      Vector2d start = line_obs->getStart();
      Vector2d end = line_obs->getEnd();
      Vector2d start_global, end_global;

      start_global = transformPointLocalToGlobal( start[0] , start[1] , pose_robot );
      end_global = transformPointLocalToGlobal( end[0] , end[1] , pose_robot );  
      
      if( linesIntersect( p1_ , p2_ , start_global , end_global ) ) return true;
      if( linesOverlap( 0.2 , p1_ , p2_ , start_global , end_global ) ) return true;
      return false;
   }
   else return false;
}

SLAMLineLandmark::SLAMLineLandmark( SLAMLineObservation& obs , Vector3d pose_robot , double timestamp )
{
   jacobian_pose_ = MatrixXd::Zero(2,3);
   jacobian_map_ = MatrixXd::Zero(2,2);
   prediction_ = VectorXd::Zero(2);
   meas_cov_q_ = MatrixXd::Zero(2,2);
   z_diff_ = VectorXd::Zero(2);
   
   Vector2d point_begin, point_end, roh_theta;

   p1_ = transformPointLocalToGlobal( obs.getStart()[0] , obs.getStart()[1] , pose_robot );
   p2_ = transformPointLocalToGlobal( obs.getEnd()[0] , obs.getEnd()[1] , pose_robot );
   
   state_ = linePointsToRohTheta( p1_(0) , p1_(1) , p2_(0) , p2_(1) );

   intersect_ = false;
   double alpha = state_(1) - normalizePi( atan2( pose_robot(1) , pose_robot(0) ) );
   if( alpha < - M_PI ) alpha += (M_PI * 2);
   if( alpha > M_PI ) alpha -= (M_PI * 2);
   
   if( fabs(alpha) < ( M_PI / 2 ) ){
      double a = state_(0) / cos(alpha);
      double origin_r = sqrt( pose_robot(0)*pose_robot(0) + pose_robot(1)*pose_robot(1) );
      if( fabs( a ) < origin_r ) intersect_ = true;
   }
   facing_origin_ = !intersect_;
   calculateJacobianMap( pose_robot );
   Matrix2d H_inverse = jacobian_map_.inverse();
      
   cov_.noalias() = H_inverse * obs.getCovar() * H_inverse.transpose();
   existence_estimate_ = 1;    
}

void SLAMLineLandmark::typeSpecializedUpdate( SLAMObservation * obs )
{
   // Ensure lower limit for covariance matrice
   state_(1) = normalizePi( state_(1) );
   
   if( cov_(0,0) < 1e-15 ) cov_(0,0) = 0.000005;
   while( cov_(0,0) < 0.00001 ){
      cov_(0,0) *= 2;
      cov_(0,1) *= sqrt(2);
      cov_(1,0) *= sqrt(2); 
   }
   if( cov_(1,1) < 1e-15 ) cov_(1,1) = 0.000002;
   while( cov_(1,1) < 0.000003 ){
      cov_(1,1) *= 2;
      cov_(0,1) *= sqrt(2);
      cov_(1,0) *= sqrt(2); 
   }   
   // Update end points
   Vector2d line_start, line_end, start, end;
   Vector2d point_candidates[4];
   SLAMLineObservation * line_obs = dynamic_cast<SLAMLineObservation *>(obs);
   start = line_obs->getStart();
   end = line_obs->getEnd();
   line_start[0] = 0;
   line_start[1] = state_(0) / sin( state_(1) );
   line_end[0] = state_(0) / cos( state_(1) );
   line_end[1] = 0;
   point_candidates[0] = p1_;
   point_candidates[1] = p2_;
   point_candidates[2] = transformPointLocalToGlobal( start[0] , start[1] , pose_ );
   point_candidates[3] = transformPointLocalToGlobal( end[0] , end[1] , pose_ );

   for( int i = 0 ; i < 4 ; i++ ){
      point_candidates[i] = projectPointOnLine( line_start , line_end , point_candidates[i] );
   }
   int index_max_dist_origin = 0;
   double max_dist_origin = point_candidates[0][0]*point_candidates[0][0] + point_candidates[0][1]*point_candidates[0][1];
   for( int i = 1 ; i < 4 ; i++ ){
      double dist = (point_candidates[i][0]*point_candidates[i][0] + point_candidates[i][1]*point_candidates[i][1]);
      if( max_dist_origin < dist ){
	 max_dist_origin = dist;
	 index_max_dist_origin = i;
      }
   }      
   int index_max_dist_start = 0;
   double max_dist_start = 0;
   for( int i = 0 ; i < 4 ; i++ ){
      double dist = (point_candidates[index_max_dist_origin] - point_candidates[i]).norm();
      if( dist > max_dist_start ) {
	 max_dist_start = dist;
	 index_max_dist_start = i;
      }
   }
   p1_ = point_candidates[index_max_dist_origin];
   p2_ = point_candidates[index_max_dist_start];
}

void SLAMLineLandmark::setObservationDifference( SLAMObservation * obs )
{
   SLAMLineObservation * line_obs = dynamic_cast<SLAMLineObservation *>( obs );
   Vector2d start = line_obs->getStart();
   Vector2d end = line_obs->getEnd();
   z_diff_.noalias() = linePointsToRohTheta( start(0) , start(1) , end(0) , end(1) ) - prediction_;
   if( z_diff_(1) < - M_PI ) z_diff_(1) += (M_PI * 2);
   if( z_diff_(1) > M_PI ) z_diff_(1) -= (M_PI * 2);
}

double SLAMLineLandmark::getFactor(SLAMObservation* obs )
{
   SLAMLineObservation * line_obs = dynamic_cast<SLAMLineObservation *>( obs );
      Vector2d start = line_obs->getStart();
      Vector2d end = line_obs->getEnd();
      double length_state, length_observation;
      length_observation = (end-start).norm();
      length_state = (p2_ - p1_).norm();
      double factor = length_observation / length_state;
      if( factor > 1.0 ) factor = 1.0;
      return factor;
}


double SLAMLandmark::getExistenceEstimate()
{
   return existence_estimate_;
}

double SLAMLandmark::decreaseExistenceEstimate()
{
   return --existence_estimate_;
}

Vector2d SLAMLineLandmark::getEnd() const
{
   return p2_;
}

Vector2d SLAMLineLandmark::getStart() const
{
   return p1_;
}

SLAMLandmark::SLAMLandmark() 
{ 
   should_have_been_seen_ = false; 
   existence_estimate_ = 0;
};

SLAMLandmark::SLAMLandmark( const SLAMLandmark& feature ) 
{ 
   should_have_been_seen_ = feature.should_have_been_seen_;
   existence_estimate_ = feature.existence_estimate_;   
   pose_ = feature.pose_;
   
   state_ = feature.state_;
   cov_ = feature.cov_;
   
   jacobian_pose_ = feature.jacobian_pose_;
   jacobian_map_ = feature.jacobian_map_;
   prediction_ = feature.prediction_;
   meas_cov_q_ = feature.meas_cov_q_;
   z_diff_ = feature.z_diff_;         
}

SLAMPointLandmark::SLAMPointLandmark(const SLAMPointLandmark& feature ) : SLAMLandmark( feature )
{

}

SLAMPointLandmark* SLAMPointLandmark::getCopy()
{
   return (new SLAMPointLandmark(*this));
}

SLAMLineLandmark::SLAMLineLandmark(const SLAMLineLandmark& feature ) : SLAMLandmark( feature )
{
   p1_ = feature.p1_;
   p2_ = feature.p2_;
   intersect_ = feature.intersect_;
   facing_origin_ = feature.facing_origin_;
   
}

SLAMLineLandmark* SLAMLineLandmark::getCopy()
{
   return (new SLAMLineLandmark(*this));
}

bool SLAMPointLandmark::shouldMerge(SLAMLandmark* other_in)
{
   SLAMPointLandmark * other = dynamic_cast<SLAMPointLandmark *>( other_in );
      
   if( other )
   {  
      double dx, dy, dist;
      
      dx = state_(0) - other->state_(0);
      dy = state_(1) - other->state_(1); 
      dist = dx*dx + dy*dy;
      
      if( dist < 0.0001 ) return true;
      else return false;
   }
   return false;
}

double SLAMPointLandmark::merge(SLAMLandmark* other_in )
{
   SLAMPointLandmark * other = dynamic_cast<SLAMPointLandmark *>( other_in );
      
   if( other )
   {  
      state_(0) = ( state_(0) + other->state_(0) ) * 0.5;
      state_(1) = ( state_(1) + other->state_(1) ) * 0.5;
      cov_(0,0) = 0.25 * (cov_(0,0)+other->cov_(0,0));
      cov_(0,1) = 0.25 * (cov_(0,1)+other->cov_(0,1));
      cov_(1,0) = 0.25 * (cov_(1,0)+other->cov_(1,0));
      cov_(1,1) = 0.25 * (cov_(1,1)+other->cov_(1,1));

      if( cov_(0,0) < 1e-15 ) cov_(0,0) = 0.000005;
      while( cov_(0,0) < 0.00001 ){
	 cov_(0,0) *= 2;
	 cov_(0,1) *= sqrt(2);
	 cov_(1,0) *= sqrt(2); 
      }
      if( cov_(1,1) < 1e-15 ) cov_(1,1) = 0.000005;    
      while( cov_(1,1) < 0.00001 ){
	 cov_(1,1) *= 2;
	 cov_(0,1) *= sqrt(2);
	 cov_(1,0) *= sqrt(2); 
      }
   }
   // Todo calc weight
   return 1.0;
}

bool SLAMLineLandmark::shouldMerge(SLAMLandmark* other_in)
{
   
   SLAMLineLandmark * other = dynamic_cast<SLAMLineLandmark *>( other_in );
      
   if( other )
   {  
      
      bool examine = false;
      if( facing_origin_ != other->facing_origin_ ) return false;  
      
      if( linesIntersect( p1_ , p2_ , other->p1_ , other->p2_ ) ) examine = true;
      if( linesOverlap( 0.04 , p1_ , p2_ , other->p1_ , other->p2_ ) ) examine = true;

      if( !examine ) return false;
	
	 double diff_theta = state_(1) - other->state_(1);
	 if( diff_theta < - M_PI ) diff_theta += (M_PI * 2);
	 if( diff_theta > M_PI ) diff_theta -= (M_PI * 2);
	 const Vector2d diff( state_(0) - other->state_(0) , diff_theta );
	 const Matrix2d cov_mean_ = (cov_ + other->cov_) * 0.5;
	 double mahalonobis_distance = diff.transpose() * cov_mean_.inverse() * diff;
	 if( fabs(state_(0) - other->state_(0)) < 0.1 && fabs(diff_theta) < 0.1 ) { 
	    return true;
	 }
	 else return false;
   }
   return false;
}

double SLAMLineLandmark::merge(SLAMLandmark* other_in)
{
   SLAMLineLandmark * other = dynamic_cast<SLAMLineLandmark *>( other_in );
   double w = 1.0;
   if( other )
   {  
      cov_(0,0) = 0.25 * (cov_(0,0)+other->cov_(0,0));
      cov_(0,1) = 0.25 * (cov_(0,1)+other->cov_(0,1));
      cov_(1,0) = 0.25 * (cov_(1,0)+other->cov_(1,0));
      cov_(1,1) = 0.25 * (cov_(1,1)+other->cov_(1,1));
                     
      if( cov_(0,0) < 1e-15 ) cov_(0,0) = 0.000005;
      while( cov_(0,0) < 0.00001 ){
	 cov_(0,0) *= 2;
	 cov_(0,1) *= sqrt(2);
	 cov_(1,0) *= sqrt(2); 
      }
      if( cov_(1,1) < 1e-15 ) cov_(1,1) = 0.000001;   
      while( cov_(1,1) < 0.000003 ){
	 cov_(1,1) *= 2;
	 cov_(0,1) *= sqrt(2);
	 cov_(1,0) *= sqrt(2); 
      }      
      double length1, length2;
      length1 = ( p2_-p1_ ).norm();
      length2 = (other->p2_ -other->p1_).norm();
      
      double diff_theta = state_(1) - other->state_(1);
      if( diff_theta < - M_PI ) diff_theta += (M_PI * 2);
      if( diff_theta > M_PI ) diff_theta -= (M_PI * 2);
      const Vector2d diff( state_(0) - other->state_(0) , diff_theta );
      const Matrix2d cov_mean_ = (cov_ + other->cov_) * 0.5;
      double mahalonobis_distance = diff.transpose() * cov_mean_.inverse() * diff;
      w = 1/(2*M_PI*sqrt(fabs((cov_).determinant()))) * exp( -0.5 * mahalonobis_distance );      
      
      int test = other->getExistenceEstimate();
      Vector2d point_candidates[4];
      point_candidates[0] = p1_;
      point_candidates[1] = p2_;
      point_candidates[2] = other->p1_;
      point_candidates[3] = other->p2_;

      double sin_sum,cos_sum;
      sin_sum = sin(state_(1)) * length1 + sin(other->state_(1)) * length2;
      cos_sum = cos(state_(1)) * length1 + cos(other->state_(1)) * length2;

      state_(0) = (state_(0) * length1 + other->state_(0) * length2) / ( length1 + length2 );
      state_(1) = atan2( sin_sum , cos_sum );
      state_(1) = normalizePi( state_(1) );

      // Update end points
      Vector2d line_start, line_end;
      
      line_start[0] = 0;
      line_start[1] = state_(0) / sin( state_(1) );
      line_end[0] = state_(0) / cos( state_(1) );
      line_end[1] = 0;

      for( int i = 0 ; i < 4 ; i++ ){
	 point_candidates[i] = projectPointOnLine( line_start , line_end , point_candidates[i] );
      }
      int index_max_dist_origin = 0;
      double max_dist_origin = point_candidates[0][0]*point_candidates[0][0] + point_candidates[0][1]*point_candidates[0][1];
      for( int i = 1 ; i < 4 ; i++ ){
	 double dist = (point_candidates[i][0]*point_candidates[i][0] + point_candidates[i][1]*point_candidates[i][1]);
	 if( max_dist_origin < dist ){
	    max_dist_origin = dist;
	    index_max_dist_origin = i;
	 }
      }      
      int index_max_dist_start = 0;
      double max_dist_start = 0;
      for( int i = 0 ; i < 4 ; i++ ){
	 double dist = (point_candidates[index_max_dist_origin] - point_candidates[i]).norm();
	 if( dist > max_dist_start ) {
	    max_dist_start = dist;
	    index_max_dist_start = i;
	 }
      }
      p1_ = point_candidates[index_max_dist_origin];
      p2_ = point_candidates[index_max_dist_start];
      
   }
   return w;
}

std::vector<geometry_msgs::Point> SLAMLineLandmark::getVisualizationPoints()
{
   std::vector<geometry_msgs::Point> ret;
   geometry_msgs::Point new_point_start;
   geometry_msgs::Point new_point_end;
   new_point_start.x = p1_(0);
   new_point_start.y = p1_(1);
   new_point_start.z = 0;
   new_point_end.x = p2_(0);
   new_point_end.y = p2_(1);
   new_point_end.z = 0;
   ret.push_back( new_point_start );
   ret.push_back( new_point_end );
   return ret;
}

std::vector<geometry_msgs::Point> SLAMPointLandmark::getVisualizationPoints()
{
   std::vector<geometry_msgs::Point> ret;
   geometry_msgs::Point new_point;
   new_point.x = state_(0);
   new_point.y = state_(1);
   new_point.z = 0;
   ret.push_back( new_point );
   return ret;
}
