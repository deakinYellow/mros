#ifndef NODE_HANDLE_HPP
#define NODE_HANDLE_HPP

#include <iostream>
#include <ros/ros.h>

namespace mros {

class NodeHandle{

public:
    explicit NodeHandle( const std::string name_space ) : nh_( name_space ) {
        nh =  ros::NodeHandlePtr( &nh_ );
    }
    ~NodeHandle( void ){
        nh->shutdown();
    }
    ros::NodeHandlePtr nh;
private:
    ros::NodeHandle nh_;
};

}

#endif // ROS_NH_HPP

