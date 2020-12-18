#ifndef TF_HPP
#define TF_HPP

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

namespace mros {

class TF{

public:
    //根据当前姿态发布tf
    void SendTransform( const std::string& frame_id,
                        const std::string& tf_id,
                        const geometry_msgs::Pose& pose ){

        transform_->setOrigin( tf::Vector3( pose.position.x , pose.position.y, pose.position.z ) );
        tf::Quaternion q;
        q.setW( pose.orientation.w );
        q.setX( pose.orientation.x );
        q.setY( pose.orientation.y );
        q.setZ( pose.orientation.z );
        q.normalize();
        transform_->setRotation( q );
        broadcaster_->sendTransform( tf::StampedTransform( *transform_, ros::Time::now(), frame_id, tf_id ) );
    }

    ~TF(){
        delete  broadcaster_;
        delete  transform_;
    }
private:
    //tf广播对象
    tf::TransformBroadcaster* broadcaster_ = new tf::TransformBroadcaster;
    //tf变换对象
    tf::Transform* transform_ = new tf::Transform;

};  //class TF


} //namespace mros


#endif // TF_HPP

