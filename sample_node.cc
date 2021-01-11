
#include <iostream>
#include <memory>
#include <ros/ros.h>

#define ENABLE_MLOGD
#include "parameter.hpp"
#include "node_handle.hpp"

#include "visualization_marker_publisher.hpp"


/**
 * @brief
 * @param  [in]
 * @param  [out]
 * @retval
 * @note
 **/
int main(int argc, char **argv){

  //initialize ros
  std::string node_name="mros_sample";
  printf("node name: %s \n", node_name.c_str() );
  ros::init( argc, argv, node_name );
  ros::NodeHandle nh( node_name );
  ros::NodeHandlePtr nh_p = ros::NodeHandlePtr( &nh );
  ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info );

  int markers_max = -1;
  mros::VisualizationMarkerPublisher marker_publisher( nh_p );


  std::string pub_topic = "/" + node_name +"/marker";
  ROS_INFO("pub topic: %s ", pub_topic.c_str() );
  mros::VisualizationMarkerPubParametersT marker_params =
    mros::VisualizationMarkerPubParameters( pub_topic,30,"map",1,
                                            mros::VISUALIZATION_MARKER_COLOR_YELLOW,
                                            mros::VISUALIZATION_MARKER_SCALE_01,
                                            markers_max );
  marker_publisher.Init( marker_params );

  geometry_msgs::Point point;
  point.z = 0;
  //draw a cricle.
  double x0 = 0.0;
  double y0 = 0.0;
  double A = 1.0;
  for( double t = 0; t < 2 * M_PI; t+= 0.01 ){
    point.x = x0 + A*cos( t );
    point.y = y0 + A*sin( t );
    marker_publisher.AddPoint( point );
  }

  ros::Rate loop_rate(1000);
  while( ros::ok() ){
    //ROS_INFO("mros sample loop.");
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}


