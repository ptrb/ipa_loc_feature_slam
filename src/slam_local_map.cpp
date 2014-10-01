/*!
 *****************************************************************
 * \author
 *   Author: Peter Broßeit - peter.brosseit@gmail.com
 ****************************************************************/

#include "slam_local_map.h"

void SLAMLocalMap::pushElement( std::vector<point2d> prediction, SLAMLandmark* landmark_in )
{
   map_element new_element;
   
   new_element.landmark_ = landmark_in;
   
   std::vector<point2d>::iterator it = prediction.begin();

   bool is_in_range = false;
   while(  !is_in_range && it != prediction.end() ){
      float s = it->x * it->x + it->y * it->y;
      if( s < 6*6 ) is_in_range = true;      
      ++it;
   }
   if( !is_in_range ) {
      new_element.visibility_ = false;
      v_map_elements_.push_back( new_element );
      return;
   }
   for( it = prediction.begin() ; it != prediction.end() ; ++it ){
      double angle = atan2( it->y , it->x ); //-pi .. pi
      angle -= floor( angle / (2 * M_PI)) * 2 * M_PI; // 0..2pi
      new_element.v_points_.push_back( *it );
      new_element.v_angles_.push_back(angle);
   }
   
   new_element.visibility_ = true;
     
   v_map_elements_.push_back( new_element );
}

void SLAMLocalMap::setVisibilities( int sensor )
{
   // sensor 1:= rear, 2:= front
//    float upper_bound, lower_bound, upper_bound_2, lower_bound_2;
//    if( sensor == 2 ){
//       upper_bound = (3/4)*M_PI;
//       lower_bound = 0;
//       upper_bound_2 = 2*M_PI;
//       lower_bound_2 = (7/4)*M_PI;
//    }
//    else if( sensor == 1 ){
//       upper_bound = (7/4)*M_PI;
//       lower_bound = (3/4)*M_PI;
//       upper_bound_2 = 0;
//       lower_bound_2 = 0;
//    }
//    for( int a = 0 ; a < v_map_elements_.size() ; a++ ){
//       bool in_sensor_viewfield = true;
//       for( int i = 0 ; i < v_map_elements_[a].v_angles.size() ; i++ ){
// 	 if( !((v_map_elements_[a].v_angles[i] < upper_bound && v_map_elements_[a].v_angles[i] > lower_bound) 
// 	   || (v_map_elements_[a].v_angles[i] < upper_bound_2 && v_map_elements_[a].v_angles[i] > lower_bound_2) )){
// 	    in_sensor_viewfield = false;
// 	 }
//       }
//       if( in_sensor_viewfield && v_map_elements_[a].visibility ){
// 
// 	    // Determine visibility
// 	 std::vector<bool> point_visibility;
// 	 point_visibility.assign( v_map_elements_[a].v_angles.size() , true );
// 	 
// 	 for( int i = 0 ; i < v_map_elements_.size() ; i++ ){
// 	    if( v_map_elements_[i].visibility && v_map_elements_[i].v_points.size() == 2 ) { // Only visible line landmarks matter
// 	       // check if map landmark hides this landmark
// 	       // begin with angle criterium
// 	       float diff_map, diff_1, diff_2;
// 	       diff_map = M_PI - fabs( fabs( v_map_elements_[i].v_angles[0] - v_map_elements_[i].v_angles[1] ) - M_PI );
// 	       // exclude steep lines
// 	       if( diff_map < 0.34 ){
// 		  for( int j = 0 ; j < v_map_elements_[a].v_points.size() ; j++ ){
// 		     point_visibility[j] = false;
// 		  }
// 	       }
// 	       else {
// 		  for( int j = 0 ; j < v_map_elements_[a].v_points.size() ; j++ ){
// 		     diff_1 = M_PI - fabs( fabs( v_map_elements_[i].v_angles[0] - v_map_elements_[a].v_angles[j] ) - M_PI );
// 		     diff_2 = M_PI - fabs( fabs( v_map_elements_[i].v_angles[1] - v_map_elements_[a].v_angles[j] ) - M_PI );
// 		     if( diff_1 < diff_map && diff_2 && diff_map ){ // sight to point intersects with line. 
// 			// Is point hidden by line? - use dot product for left or right.
// 			bool origin, point; // is left of line?
// 			point = ((v_map_elements_[i].v_points[1].x-v_map_elements_[i].v_points[0].x)*(v_map_elements_[a].v_points[j].y-v_map_elements_[i].v_points[0].y)-(v_map_elements_[i].v_points[1].y-v_map_elements_[i].v_points[0].y)*(v_map_elements_[a].v_points[j].x-v_map_elements_[i].v_points[0].x)) > 0;
// 			origin = ((v_map_elements_[i].v_points[1].x-v_map_elements_[i].v_points[0].x)*(-v_map_elements_[i].v_points[0].y)-(v_map_elements_[i].v_points[1].y-v_map_elements_[i].v_points[0].y)*(-v_map_elements_[i].v_points[0].x)) > 0;
// 			if( point != origin ){ // not visible
// 			   point_visibility[j] = false;
// 			   
// 			   // TODO some break conditions..
// 			}
// 		     }
// 		  }
// 	       }
// 	    }
// 	 }
// 	 v_map_elements_[a].visibility = false; // If at least one point is visible, assume landmark visibility
// 	 for( int i = 0 ; i < point_visibility.size() ; i++ ){
// 	    v_map_elements_[a].visibility = v_map_elements_[a].visibility || point_visibility[i];
// 	 }
//       }
//       else {
// 	  v_map_elements_[a].visibility = false; 
//       }
//    }
   
   for( int i = 0 ; i < v_map_elements_.size() ; i++ ){
//       v_map_elements_[i].landmark->setVisibility( v_map_elements_[i].visibility );
      v_map_elements_[i].landmark_->setVisibility( false );
   }
   
   
}
