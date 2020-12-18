#ifndef VISUALIZATION_MARKER_PUBLISHER_HPP
#define VISUALIZATION_MARKER_PUBLISHER_HPP

#include <istream>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "node_handle.hpp"

namespace mros {

class VisualizationMarkerPublisher{

public:
    explicit VisualizationMarkerPublisher( const ros::NodeHandlePtr nh ) : nh_( nh ) {
        //marker default setting
        defaultSetting();
    }

    void Start( uint fq, std::string topic ){
        timer_ = nh_->createTimer( ros::Duration( 1.0 / double(fq) ), &mros::VisualizationMarkerPublisher::timerCallBack, this );
        publisher_ = nh_->advertise<visualization_msgs::Marker>( topic , 10 );
    }

    void SetMarker( std::string frame_id, int id , double scale, float r, float g, float b, float a ){
        marker_points_->header.frame_id = frame_id;
        marker_points_->type = visualization_msgs::Marker::POINTS;
        marker_points_->action = visualization_msgs::Marker::ADD;
        marker_points_->id = id;
        marker_points_->lifetime = ros::Duration();

        marker_points_->scale.x = scale;
        marker_points_->scale.y = scale;
        //points.scale.z = 0.02;

        marker_points_->color.r = r;
        marker_points_->color.g = g;
        marker_points_->color.b = b;
        marker_points_->color.a = a;
    }

    void AddPoint( const geometry_msgs::Point  p ){
        marker_points_->points.push_back( p );
    }
private:

    ros::NodeHandlePtr nh_;
    ros::Timer timer_;
    ros::Publisher publisher_;

    visualization_msgs::Marker::Ptr marker_points_ =
        visualization_msgs::Marker::Ptr( new visualization_msgs::Marker );

    void timerCallBack( const ros::TimerEvent &event ){
        //ROS_INFO("visualization timer callback!");
        marker_points_->header.stamp = ros::Time::now();
        publisher_.publish( marker_points_ );
    }
    void defaultSetting( void ){
        marker_points_->header.frame_id = "map";
        marker_points_->type = visualization_msgs::Marker::POINTS;
        marker_points_->action = visualization_msgs::Marker::ADD;
        marker_points_->id = 1;
        marker_points_->lifetime = ros::Duration();
        marker_points_->scale.x = 0.02;
        marker_points_->scale.y = 0.02;
        //points.scale.z = 0.02;
        marker_points_->color.g = 0.7f;
        marker_points_->color.a = 1.0;
    }

};


}

#endif // VISUALIZATION_MARKER_PUBLISHER_HPP
