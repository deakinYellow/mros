#ifndef VISUALIZATION_MARKER_PUBLISHER_HPP
#define VISUALIZATION_MARKER_PUBLISHER_HPP

#include <istream>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "data_type.h"
#include "node_handle.hpp"

namespace mros {


class VisualizationMarkerPublisher{

public:

  explicit VisualizationMarkerPublisher( const ros::NodeHandlePtr nh ) : nh_( nh ) {
    ;
  }
  //points_max：轨迹允许最大点数，设置为负数则不做限制
  void Init( const VisualizationMarkerPubParametersT& params ){
    //ros timer, auto publish
    timer_ = nh_->createTimer( ros::Duration( 1.0 / double(params.fq) ), &mros::VisualizationMarkerPublisher::TimerCallBack, this );
    publisher_ = nh_->advertise<visualization_msgs::Marker>( params.topic , 10 );

    //marker default setting, must
    MarkerDefaultSetting();
    points_max_ = params.points_max;
    //get color value
    ColorT color = GetColorRbgValue( params.color );
    //get scale value
    double scale = GetScaleValue( params.scale );
    //ROS_DEBUG("frame_id: %s id: %d color:(%f %f %f) scale: %f",
             //params.frame_id.c_str(), params.id,
             //color.r, color.g, color.b, scale );
    SetMarker( params.frame_id, params.id, scale, color.r, color.g, color.b, color.a );
  }


  void SetMarker( const std::string frame_id, const int id ,
                  const double scale,
                  const double r, const double g, const double b, const double a ){
    marker_points_->header.frame_id = frame_id;
    marker_points_->type = visualization_msgs::Marker::POINTS;
    marker_points_->action = visualization_msgs::Marker::ADD;
    marker_points_->id = id;
    marker_points_->lifetime = ros::Duration();

    marker_points_->scale.x = scale;
    marker_points_->scale.y = scale;
    marker_points_->scale.z = 0;

    marker_points_->color.r = float(r);
    marker_points_->color.g = float(g);
    marker_points_->color.b = float(b);
    marker_points_->color.a = float(a);
  }


#if 0
  void SetMarkerColor( const VisualizationMarkerColorsL& color_opition ){
    //get color value
    ColorT color = GetColorRbgValue( color_opition );
    marker_points_->color.r = float(color.r);
    marker_points_->color.g = float(color.g);
    marker_points_->color.b = float(color.b);
    marker_points_->color.a = float(color.a);
  }
#endif

  void AddPoint( const geometry_msgs::Point  p ){
    marker_points_->points.push_back( p );
    if( points_max_ >= 0 &&
        marker_points_->points.size() > ulong( points_max_ ) ){
      marker_points_->points.erase( marker_points_->points.begin() );
    }
  }

private:

  ros::NodeHandlePtr nh_;
  ros::Timer timer_;
  ros::Publisher publisher_;

  visualization_msgs::Marker::Ptr marker_points_ =
      visualization_msgs::Marker::Ptr( new visualization_msgs::Marker );
  int points_max_;

  void TimerCallBack( const ros::TimerEvent &event ){
    //ROS_DEBUG("visualization timer callback!");
    marker_points_->header.stamp = ros::Time::now();
    publisher_.publish( marker_points_ );
  }
  void MarkerDefaultSetting( void ){
    marker_points_->header.frame_id = "map";
    marker_points_->type = visualization_msgs::Marker::POINTS;
    marker_points_->action = visualization_msgs::Marker::ADD;
    marker_points_->lifetime = ros::Duration();
#if 0
    marker_points_->id = 1;
    marker_points_->scale.x = 0.02;
    marker_points_->scale.y = 0.02;
    marker_points_->scale.z = 0.02;
    marker_points_->color.r = 0.9f;
    marker_points_->color.g = 0.9f;
    marker_points_->color.b = 0.9f;
    marker_points_->color.a = 1.0;
#endif
  }

  double GetScaleValue( const VisualizationMarkerScaleL& opition ){
    double scale;
    double min_scale = 0.02;
    switch ( opition ) {
      case VISUALIZATION_MARKER_SCALE_01 :
        scale = min_scale;
        break;
      case VISUALIZATION_MARKER_SCALE_02 :
        scale = min_scale * 2;
        break;
      case VISUALIZATION_MARKER_SCALE_03 :
        scale = min_scale * 3;
        break;
      case VISUALIZATION_MARKER_SCALE_04 :
        scale = min_scale * 4;
        break;
      case VISUALIZATION_MARKER_SCALE_05 :
        scale = min_scale * 5;
        break;
    }
    return scale;
  }


  ColorT GetColorRbgValue( const VisualizationMarkerColorsL& opition ){
    ColorT color = Color(1,1,1);
    switch ( opition ) {
      case VISUALIZATION_MARKER_COLOR_WHITE :
        color.r = 1.0;
        color.g = 1.0;
        color.b = 1.0;
        break;
      case VISUALIZATION_MARKER_COLOR_BLACK :
        color.r = 0.0;
        color.g = 0.0;
        color.b = 0.0;
        break;
      case VISUALIZATION_MARKER_COLOR_RED :
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        break;
      case VISUALIZATION_MARKER_COLOR_ORANGE :
        color.r = 255/255.0;
        color.g = 97/255.0;
        color.b = 0.0;
        break;
      case VISUALIZATION_MARKER_COLOR_YELLOW :
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
      case VISUALIZATION_MARKER_COLOR_GREEN :
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        break;
      case VISUALIZATION_MARKER_COLOR_BLUE :
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        break;
      case VISUALIZATION_MARKER_COLOR_CYAN :
        color.r = 34/255.0;
        color.g = 139/255.0;
        color.b = 34/255.0;
        break;
      case VISUALIZATION_MARKER_COLOR_VIOLET :
        color.r = 160/255.0;
        color.g = 32/255.0;
        color.b = 240/255.0;
        break;
    }
    return color;
  }



};  //end of VisualizationMarkerPublisher


}


#endif // VISUALIZATION_MARKER_PUBLISHER_HPP
