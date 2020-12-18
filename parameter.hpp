#ifndef PARAMETER_HPP
#define PARAMETER_HPP

#include "ros/ros.h"

namespace mros {

class Parameter{

public:
    explicit Parameter( const ros::NodeHandlePtr nh ) : nh_( nh ) {
        ;
    }

    template<typename T>
    bool get(const std::string& param_name , T& param )  {
        return nh_->getParam( param_name, param );
    }

    template<typename T>
    void get(const std::string& param_name , T& param, T param_default_val )  {
        param = nh_->param<T>( param_name, param_default_val );
    }

    template<typename T>
    void set( const std::string& param_name , const T& set_val  )  {
        nh_->setParam( param_name, set_val );
    }

    ~Parameter(){
        ;
    }

private:
    ros::NodeHandlePtr nh_;

};


}

#endif // PARAM_HPP
